// ============================================================================
//  src/hal/null_backend.cpp
//
//  NullAudioBackend - a silent audio device.
//
//  It drives the render callback exactly like a real backend would, but instead
//  of sending audio to hardware it simply discards it (or lets the caller pull
//  blocks manually for offline render). This makes it perfect for:
//    * unit / integration testing of the DSP graph,
//    * deterministic offline rendering (e.g. rendering to a WAV file),
//    * development on machines without an audio device.
//
//  When started, it spawns a std::jthread that repeatedly invokes the render
//  callback at the wall-clock cadence implied by the block size and sample
//  rate, mimicking a hardware clock.
// ============================================================================
#include "aurora/hal/audio_backend.h"
#include "aurora/hal/audio_device.h"

#include <atomic>
#include <chrono>
#include <thread>

namespace aurora::hal {

class NullAudioDevice final : public AudioDevice {
public:
    NullAudioDevice() = default;
    ~NullAudioDevice() override { stop(); }

    [[nodiscard]] DeviceInfo info() const override {
        return DeviceInfo{
            .name         = "Null Output (silent)",
            .max_outputs  = 2,
            .max_inputs   = 0,
            .default_rate = 48000,
            .is_default   = true,
        };
    }

    bool open(AudioFormat& fmt, FrameCount preferred_block) override {
        // The null device accepts any float/planar request as-is. It coerces
        // to the engine-native representation for consistency.
        fmt.sample_type = SampleType::Float32;
        fmt.layout      = SampleLayout::Planar;
        if (fmt.channels == 0)     fmt.channels = 2;
        if (fmt.sample_rate == 0)  fmt.sample_rate = 48000;
        format_     = fmt;
        block_size_ = preferred_block ? preferred_block : 512;
        opened_     = true;
        return true;
    }

    void set_render_callback(RenderCallback cb) override {
        callback_ = std::move(cb);
    }

    bool start() override {
        if (!opened_ || !callback_ || running_.load()) return false;
        running_.store(true);
        stream_time_ = 0;
        // std::jthread auto-joins on destruction and supports stop_token,
        // giving us clean, C++20-idiomatic thread lifetime management.
        worker_ = std::jthread([this](std::stop_token st) { run(st); });
        return true;
    }

    void stop() override {
        if (!running_.exchange(false)) return;
        if (worker_.joinable()) {
            worker_.request_stop();
            worker_.join();
        }
    }

    [[nodiscard]] bool is_running() const override { return running_.load(); }
    [[nodiscard]] AudioFormat format() const override { return format_; }
    [[nodiscard]] FrameCount block_size() const override { return block_size_; }

private:
    void run(std::stop_token st) {
        const ChannelCount ch = format_.channels;
        const FrameCount   nf = block_size_;

        // Pre-allocate the planar scratch buffer ONCE, before the loop, so the
        // pseudo-hardware callback never allocates (mirrors real backends).
        std::vector<Sample>  storage(static_cast<std::size_t>(ch) * nf, 0.0f);
        std::vector<Sample*> ptrs(ch);
        for (ChannelCount c = 0; c < ch; ++c) {
            ptrs[c] = storage.data() + static_cast<std::size_t>(c) * nf;
        }
        AudioBuffer buffer(ptrs.data(), ch, nf);

        // Period the "hardware" would take to consume one block.
        const auto period = std::chrono::duration<double>(
            static_cast<double>(nf) / static_cast<double>(format_.sample_rate));
        auto next = std::chrono::steady_clock::now();

        while (!st.stop_requested() && running_.load()) {
            buffer.clear();
            ProcessContextBridge ctx{
                .sample_rate = format_.sample_rate,
                .frames      = nf,
                .stream_time = stream_time_,
            };
            callback_(buffer, ctx);   // pull one block from the engine
            stream_time_ += nf;
            // Output is intentionally discarded (silent device).

            next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
            std::this_thread::sleep_until(next);
        }
    }

    AudioFormat       format_{};
    FrameCount        block_size_ = 512;
    bool              opened_     = false;
    std::atomic<bool> running_{false};
    std::uint64_t     stream_time_ = 0;
    RenderCallback    callback_;
    std::jthread      worker_;
};

// ---- Factory glue ---------------------------------------------------------

std::unique_ptr<AudioDevice> make_null_device() {
    return std::make_unique<NullAudioDevice>();
}

std::unique_ptr<AudioDevice> AudioBackendFactory::create(BackendType type) {
    if (type == BackendType::Automatic) {
        type = default_backend();
    }
    switch (type) {
        case BackendType::Null:
            return make_null_device();
        // Real backends are not compiled into this prototype build. A
        // production version would return WASAPI/CoreAudio/ALSA/PipeWire
        // devices here. We fall back to the null device so callers still work.
        case BackendType::WASAPI:
        case BackendType::CoreAudio:
        case BackendType::ALSA:
        case BackendType::PipeWire:
        case BackendType::Automatic:
        default:
            return make_null_device();
    }
}

std::vector<BackendType> AudioBackendFactory::available_backends() {
    // Only the null backend is available in this build.
    return { BackendType::Null };
}

} // namespace aurora::hal
