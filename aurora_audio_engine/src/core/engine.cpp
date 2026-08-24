// ============================================================================
//  src/core/engine.cpp
//
//  AuroraEngine implementation.
// ============================================================================
#include "core/engine.h"

#include <vector>

namespace aurora {

AuroraEngine::~AuroraEngine() {
    stop();
}

// ---------------------------------------------------------------------------
//  configure()
// ---------------------------------------------------------------------------
bool AuroraEngine::configure(const EngineConfig& cfg) {
    config_ = cfg;

    device_ = hal::AudioBackendFactory::create(cfg.backend);
    if (!device_) return false;

    // Negotiate the stream format with the device.
    AudioFormat requested{
        .sample_rate = cfg.sample_rate,
        .channels    = cfg.channels,
        .sample_type = SampleType::Float32,
        .layout      = SampleLayout::Planar,
    };
    if (!device_->open(requested, cfg.block_size)) return false;
    format_ = requested; // may have been adjusted by the backend

    // Compile the graph for the granted format and block size.
    if (!graph_.prepare(format_.sample_rate, device_->block_size())) {
        return false; // e.g. a cycle in the graph
    }

    // Install the real-time render callback (a member function bound via
    // lambda). No allocation happens inside once configured.
    device_->set_render_callback(
        [this](AudioBuffer& out, const hal::ProcessContextBridge& ctx) {
            render_block(out, ctx);
        });

    stream_time_ = 0;
    configured_  = true;
    return true;
}

// ---------------------------------------------------------------------------
//  render_block() - the real-time path
//
//  Bridges the HAL callback context to a dsp::ProcessContext and pulls one
//  block from the graph. Allocation-free and lock-free.
// ---------------------------------------------------------------------------
void AuroraEngine::render_block(AudioBuffer& output,
                                const hal::ProcessContextBridge& ctx) {
    dsp::ProcessContext pctx{
        .sample_rate = ctx.sample_rate,
        .frames      = ctx.frames,
        .stream_time = ctx.stream_time,
    };
    graph_.process(pctx, output);
}

// ---------------------------------------------------------------------------
//  Lifecycle
// ---------------------------------------------------------------------------
bool AuroraEngine::start() {
    if (!configured_ || !device_) return false;
    return device_->start();
}

void AuroraEngine::stop() {
    if (device_) device_->stop();
}

bool AuroraEngine::is_running() const {
    return device_ && device_->is_running();
}

// ---------------------------------------------------------------------------
//  render_offline() - deterministic block-by-block render, no device
// ---------------------------------------------------------------------------
std::vector<Sample> AuroraEngine::render_offline(FrameCount total_frames) {
    const ChannelCount ch = format_.channels ? format_.channels : config_.channels;
    const FrameCount   block = device_ ? device_->block_size() : config_.block_size;

    // Interleaved output for easy WAV writing (done OUTSIDE the RT path).
    std::vector<Sample> interleaved(static_cast<std::size_t>(total_frames) * ch, 0.0f);

    // Planar scratch buffer reused for every block (allocated once, here).
    OwnedAudioBuffer scratch(ch, block);

    FrameCount rendered = 0;
    while (rendered < total_frames) {
        const FrameCount n = std::min<FrameCount>(block, total_frames - rendered);
        AudioBuffer buf = scratch.view();
        buf.set_frames(n);
        buf.clear();

        dsp::ProcessContext pctx{
            .sample_rate = format_.sample_rate,
            .frames      = n,
            .stream_time = stream_time_,
        };
        graph_.process(pctx, buf);
        stream_time_ += n;

        // Interleave planar -> interleaved.
        for (FrameCount f = 0; f < n; ++f) {
            for (ChannelCount c = 0; c < ch; ++c) {
                interleaved[static_cast<std::size_t>(rendered + f) * ch + c] =
                    buf.data(c)[f];
            }
        }
        rendered += n;
    }
    return interleaved;
}

} // namespace aurora
