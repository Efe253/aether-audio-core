// ============================================================================
//  aurora/core/memory_pool.h
//
//  Real-time-safe fixed-block memory pool.
//
//  Rationale
//  ---------
//  Dynamic allocation (new/delete/malloc/free) is forbidden on the audio
//  thread because its worst-case latency is unbounded. Instead, all memory the
//  render path could ever need is pre-allocated up front (on the control
//  thread). The pool then hands out and reclaims fixed-size blocks with a
//  lock-free free-list, which is wait-free and never touches the system
//  allocator.
//
//  Design
//  ------
//  * The pool owns one big contiguous slab, carved into `block_count` blocks of
//    `block_size` bytes each.
//  * Free blocks are threaded into an intrusive singly-linked "free list"
//    (each free block stores the index of the next free block in its first
//    bytes).
//  * allocate()/deallocate() use a lock-free CAS loop on an atomic head with a
//    tagged pointer (ABA counter) so concurrent producers/consumers are safe.
// ============================================================================
#ifndef AURORA_CORE_MEMORY_POOL_H
#define AURORA_CORE_MEMORY_POOL_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>

#include "aurora/core/ring_buffer.h" // for kCacheLineSize

namespace aurora {

// ---------------------------------------------------------------------------
//  MemoryPool
//
//  A lock-free pool of equally-sized blocks. Real-time safe: allocate() and
//  deallocate() perform no system calls and no locking.
// ---------------------------------------------------------------------------
class MemoryPool {
public:
    /// Sentinel meaning "end of free list".
    static constexpr std::uint32_t kNull = 0xFFFFFFFFu;

    /// Construct a pool of `block_count` blocks, each at least `block_size`
    /// bytes and aligned to `alignment`. All allocation happens HERE, on the
    /// (non-real-time) constructing thread.
    MemoryPool(std::size_t block_size,
               std::size_t block_count,
               std::size_t alignment = alignof(std::max_align_t))
        : block_size_(align_up(block_size < sizeof(std::uint32_t)
                                   ? sizeof(std::uint32_t)
                                   : block_size,
                               alignment)),
          block_count_(block_count),
          alignment_(alignment) {
        // Over-allocate so we can align the slab base ourselves (portable).
        raw_ = ::operator new(block_size_ * block_count_ + alignment_);
        auto base = reinterpret_cast<std::uintptr_t>(raw_);
        auto aligned = align_up(base, alignment_);
        slab_ = reinterpret_cast<std::byte*>(aligned);

        // Thread every block onto the free list: 0 -> 1 -> 2 -> ... -> null.
        for (std::uint32_t i = 0; i < block_count_; ++i) {
            std::uint32_t next = (i + 1 < block_count_) ? (i + 1) : kNull;
            *next_index_ptr(i) = next;
        }
        head_.store(pack(0, 0), std::memory_order_relaxed);
        free_count_.store(block_count_, std::memory_order_relaxed);
    }

    ~MemoryPool() {
        ::operator delete(raw_);
    }

    MemoryPool(const MemoryPool&)            = delete;
    MemoryPool& operator=(const MemoryPool&) = delete;

    /// Grab one block. Returns nullptr when exhausted. Wait-free.
    [[nodiscard]] void* allocate() noexcept {
        std::uint64_t old_head = head_.load(std::memory_order_acquire);
        for (;;) {
            const std::uint32_t index = unpack_index(old_head);
            if (index == kNull) return nullptr; // pool empty
            const std::uint32_t next = *next_index_ptr(index);
            const std::uint64_t new_head = pack(next, unpack_tag(old_head) + 1);
            if (head_.compare_exchange_weak(old_head, new_head,
                                            std::memory_order_acq_rel,
                                            std::memory_order_acquire)) {
                free_count_.fetch_sub(1, std::memory_order_relaxed);
                return block_ptr(index);
            }
            // CAS failed: old_head refreshed, retry.
        }
    }

    /// Return a block previously obtained from allocate(). Wait-free.
    void deallocate(void* ptr) noexcept {
        if (ptr == nullptr) return;
        const std::uint32_t index = index_of(ptr);
        std::uint64_t old_head = head_.load(std::memory_order_acquire);
        for (;;) {
            *next_index_ptr(index) = unpack_index(old_head);
            const std::uint64_t new_head = pack(index, unpack_tag(old_head) + 1);
            if (head_.compare_exchange_weak(old_head, new_head,
                                            std::memory_order_acq_rel,
                                            std::memory_order_acquire)) {
                free_count_.fetch_add(1, std::memory_order_relaxed);
                return;
            }
        }
    }

    [[nodiscard]] std::size_t block_size()  const noexcept { return block_size_; }
    [[nodiscard]] std::size_t block_count() const noexcept { return block_count_; }
    [[nodiscard]] std::size_t free_count()  const noexcept {
        return free_count_.load(std::memory_order_relaxed);
    }

private:
    static constexpr std::uintptr_t align_up(std::uintptr_t v, std::size_t a) noexcept {
        return (v + (a - 1)) & ~(static_cast<std::uintptr_t>(a) - 1);
    }

    std::byte* block_ptr(std::uint32_t index) noexcept {
        return slab_ + static_cast<std::size_t>(index) * block_size_;
    }

    std::uint32_t* next_index_ptr(std::uint32_t index) noexcept {
        // A free block reuses its own storage to hold the next-free index.
        return reinterpret_cast<std::uint32_t*>(block_ptr(index));
    }

    std::uint32_t index_of(void* ptr) const noexcept {
        const auto diff = reinterpret_cast<std::byte*>(ptr) - slab_;
        return static_cast<std::uint32_t>(static_cast<std::size_t>(diff) / block_size_);
    }

    // Tagged head: low 32 bits = block index, high 32 bits = ABA counter.
    static constexpr std::uint64_t pack(std::uint32_t index, std::uint32_t tag) noexcept {
        return (static_cast<std::uint64_t>(tag) << 32) | index;
    }
    static constexpr std::uint32_t unpack_index(std::uint64_t v) noexcept {
        return static_cast<std::uint32_t>(v & 0xFFFFFFFFu);
    }
    static constexpr std::uint32_t unpack_tag(std::uint64_t v) noexcept {
        return static_cast<std::uint32_t>(v >> 32);
    }

    const std::size_t block_size_;
    const std::size_t block_count_;
    const std::size_t alignment_;

    void*      raw_  = nullptr; // original allocation (for delete)
    std::byte* slab_ = nullptr; // aligned base of the block array

    alignas(kCacheLineSize) std::atomic<std::uint64_t> head_{0};
    alignas(kCacheLineSize) std::atomic<std::size_t>   free_count_{0};
};

// ---------------------------------------------------------------------------
//  TypedPool<T>
//
//  Thin type-safe wrapper: construct/destroy T objects backed by MemoryPool
//  storage. create()/destroy() are real-time safe (no system allocation).
// ---------------------------------------------------------------------------
template <typename T>
class TypedPool {
public:
    explicit TypedPool(std::size_t count)
        : pool_(sizeof(T), count, alignof(T)) {}

    template <typename... Args>
    [[nodiscard]] T* create(Args&&... args) noexcept {
        void* mem = pool_.allocate();
        if (!mem) return nullptr;
        return ::new (mem) T(std::forward<Args>(args)...);
    }

    void destroy(T* obj) noexcept {
        if (!obj) return;
        obj->~T();
        pool_.deallocate(obj);
    }

    [[nodiscard]] std::size_t free_count() const noexcept { return pool_.free_count(); }

private:
    MemoryPool pool_;
};

} // namespace aurora

#endif // AURORA_CORE_MEMORY_POOL_H
