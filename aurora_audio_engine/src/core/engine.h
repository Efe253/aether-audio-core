// ============================================================================
//  src/core/engine.h
//
//  AuroraEngine - the top-level facade tying the layers together:
//
//      HAL (AudioDevice)  <--render callback-->  DSP (AudioGraph)
//
//  Responsibilities
//  ----------------
//    * Own the audio graph and the output device.
//    * Configure and negotiate the stream format.
//    * Install a real-time-safe render callback that pulls blocks from the
//      graph and hands them to the device.
//    * Manage start/stop lifecycle. The actual audio thread is owned by the
//      backend (NullAudioDevice uses a std::jthread); the engine coordinates
//      it and keeps the render path allocation- and lock-free.
//    * Offer an offline render() helper for deterministic WAV export/tests.
// ============================================================================
#ifndef AURORA_CORE_ENGINE_H
#define AURORA_CORE_ENGINE_H

#include <memory>

#include "aurora/core/types.h"
#include "aurora/dsp/audio_graph.h"
#include "aurora/hal/audio_backend.h"
#include "aurora/hal/audio_device.h"

namespace aurora {

struct EngineConfig {
    SampleRate   sample_rate = 48000;
    ChannelCount channels    = 2;
    FrameCount   block_size  = 512;
    hal::BackendType backend = hal::BackendType::Automatic;
};

class AuroraEngine {
public:
    AuroraEngine() = default;
    ~AuroraEngine();

    AuroraEngine(const AuroraEngine&)            = delete;
    AuroraEngine& operator=(const AuroraEngine&) = delete;

    /// Access the DSP graph to build the signal chain (control thread only,
    /// before start()).
    [[nodiscard]] dsp::AudioGraph& graph() { return graph_; }

    /// Create the device, negotiate format, prepare the graph and install the
    /// render callback. Returns false on failure.
    bool configure(const EngineConfig& cfg);

    /// Start real-time streaming. The device begins pulling blocks.
    bool start();

    /// Stop streaming; after return the render callback is not running.
    void stop();

    [[nodiscard]] bool is_running() const;

    [[nodiscard]] const EngineConfig& config() const { return config_; }
    [[nodiscard]] AudioFormat format() const { return format_; }

    // ---- Offline (non-real-time) render ---------------------------------

    /// Render `total_frames` of the graph into an interleaved float vector,
    /// block by block, WITHOUT a device. Deterministic; used by the demo to
    /// write a WAV file. Runs on the calling thread.
    [[nodiscard]] std::vector<Sample> render_offline(FrameCount total_frames);

private:
    void render_block(AudioBuffer& output, const hal::ProcessContextBridge& ctx);

    EngineConfig                     config_{};
    AudioFormat                      format_{};
    dsp::AudioGraph                  graph_;
    std::unique_ptr<hal::AudioDevice> device_;
    std::uint64_t                    stream_time_ = 0;
    bool                             configured_  = false;
};

} // namespace aurora

#endif // AURORA_CORE_ENGINE_H
