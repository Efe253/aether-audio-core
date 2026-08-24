// ============================================================================
//  aurora/core/ring_buffer.h
//
//  Lock-free Single-Producer / Single-Consumer (SPSC) ring buffer.
//
//  Purpose
//  -------
//  The audio (render) thread must never block. Communication between the
//  control thread and the render thread — parameter changes, telemetry, event
//  streams — flows through wait-free SPSC queues built on std::atomic. This is
//  the canonical real-time-safe hand-off primitive from the research report.
//
//  Correctness
//  -----------
//  * Exactly ONE producer thread may call push()/write().
//  * Exactly ONE consumer thread may call pop()/read().
//  * The head (write) and tail (read) indices are separated onto their own
//    cache lines (padding) to avoid false sharing between the two threads.
//  * Capacity is rounded up to a power of two so index wrapping is a cheap
//    bit-mask instead of a modulo.
//  * Acquire/Release memory ordering guarantees that data written before the
//    producer publishes the new head is visible to the consumer.
// ============================================================================
#ifndef AURORA_CORE_RING_BUFFER_H
#define AURORA_CORE_RING_BUFFER_H

#include <atomic>
#include <cstddef>
#include <new>
#include <span>
#include <type_traits>
#include <vector>

namespace aurora {

// Cache-line size used for padding. 64 bytes is correct for virtually all
// mainstream x86-64 and Arm64 CPUs. std::hardware_destructive_interference_size
// would be ideal but is not portable across all toolchains, so we fix it.
inline constexpr std::size_t kCacheLineSize = 64;

/// Round up to the next power of two (>= 1).
constexpr std::size_t next_power_of_two(std::size_t v) noexcept {
    if (v < 2) return 1;
    --v;
    for (std::size_t i = 1; i < sizeof(std::size_t) * 8; i <<= 1) {
        v |= v >> i;
    }
    return v + 1;
}

// ---------------------------------------------------------------------------
//  SpscRingBuffer<T>
//
//  A bounded, lock-free FIFO for trivially-copyable payloads. Designed so that
//  the consumer side (typically the audio thread) never allocates or blocks.
// ---------------------------------------------------------------------------
template <typename T>
class SpscRingBuffer {
    static_assert(std::is_trivially_copyable_v<T>,
                  "SpscRingBuffer requires a trivially-copyable element type "
                  "so that reads/writes are real-time safe.");

public:
    /// `capacity` is rounded up to a power of two. Usable slots = capacity - 1
    /// (one slot is kept empty to distinguish full from empty).
    explicit SpscRingBuffer(std::size_t capacity)
        : capacity_(next_power_of_two(capacity)),
          mask_(capacity_ - 1),
          storage_(capacity_) {}

    // Non-copyable, non-movable: the atomics pin the object in place.
    SpscRingBuffer(const SpscRingBuffer&)            = delete;
    SpscRingBuffer& operator=(const SpscRingBuffer&) = delete;

    /// Maximum number of elements that can be stored simultaneously.
    [[nodiscard]] std::size_t capacity() const noexcept { return capacity_ - 1; }

    // ---- Producer side --------------------------------------------------

    /// Push one element. Returns false if the buffer is full (no overwrite).
    [[nodiscard]] bool push(const T& item) noexcept {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t next = (head + 1) & mask_;
        // If advancing head would collide with the consumer's tail, we're full.
        if (next == tail_.load(std::memory_order_acquire)) {
            return false;
        }
        storage_[head] = item;
        head_.store(next, std::memory_order_release);
        return true;
    }

    // ---- Consumer side --------------------------------------------------

    /// Pop one element into `out`. Returns false if the buffer is empty.
    [[nodiscard]] bool pop(T& out) noexcept {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) {
            return false; // empty
        }
        out = storage_[tail];
        tail_.store((tail + 1) & mask_, std::memory_order_release);
        return true;
    }

    // ---- Bulk transfer (block DSP data efficiently) --------------------

    /// Write up to src.size() elements; returns the number actually written.
    [[nodiscard]] std::size_t write(std::span<const T> src) noexcept {
        std::size_t written = 0;
        std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t tail = tail_.load(std::memory_order_acquire);
        while (written < src.size()) {
            const std::size_t next = (head + 1) & mask_;
            if (next == tail) break; // full
            storage_[head] = src[written++];
            head = next;
        }
        head_.store(head, std::memory_order_release);
        return written;
    }

    /// Read up to dst.size() elements; returns the number actually read.
    [[nodiscard]] std::size_t read(std::span<T> dst) noexcept {
        std::size_t got = 0;
        std::size_t tail = tail_.load(std::memory_order_relaxed);
        const std::size_t head = head_.load(std::memory_order_acquire);
        while (got < dst.size() && tail != head) {
            dst[got++] = storage_[tail];
            tail = (tail + 1) & mask_;
        }
        tail_.store(tail, std::memory_order_release);
        return got;
    }

    // ---- Observers (approximate; safe to call from either side) ---------

    [[nodiscard]] bool empty() const noexcept {
        return head_.load(std::memory_order_acquire) ==
               tail_.load(std::memory_order_acquire);
    }

    [[nodiscard]] std::size_t size_approx() const noexcept {
        const std::size_t h = head_.load(std::memory_order_acquire);
        const std::size_t t = tail_.load(std::memory_order_acquire);
        return (h - t) & mask_;
    }

private:
    const std::size_t capacity_;
    const std::size_t mask_;

    // Head and tail live on separate cache lines to prevent false sharing:
    // the producer only writes head_, the consumer only writes tail_.
    alignas(kCacheLineSize) std::atomic<std::size_t> head_{0};
    alignas(kCacheLineSize) std::atomic<std::size_t> tail_{0};
    alignas(kCacheLineSize) std::vector<T> storage_;
};

} // namespace aurora

#endif // AURORA_CORE_RING_BUFFER_H
