// ============================================================================
//  examples/synth_demo.cpp
//
//  A small subtractive-synthesizer demo built on the Aurora Audio Engine.
//
//  Signal chain:
//
//      OscillatorNode (saw) --> BiquadFilterNode (lowpass)
//                                     --> EnvelopeNode (ADSR)
//                                             --> GainNode --> [graph output]
//
//  The graph is driven through the engine and also rendered offline to a
//  16-bit PCM WAV file (output.wav). We additionally spin the NullBackend for
//  a moment to prove the real-time audio thread runs the same graph without a
//  physical device.
// ============================================================================
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <thread>
#include <vector>

#include "core/engine.h"
#include "aurora/dsp/basic_nodes.h"

using namespace aurora;
using namespace aurora::dsp;

// ---------------------------------------------------------------------------
//  Minimal 16-bit PCM WAV writer (control-thread / offline only).
// ---------------------------------------------------------------------------
static void write_wav(const char* path,
                      const std::vector<Sample>& interleaved,
                      SampleRate sample_rate,
                      ChannelCount channels) {
    const std::uint32_t num_samples = static_cast<std::uint32_t>(interleaved.size());
    const std::uint16_t bits = 16;
    const std::uint16_t block_align = channels * bits / 8;
    const std::uint32_t byte_rate = sample_rate * block_align;
    const std::uint32_t data_bytes = num_samples * (bits / 8);

    std::ofstream f(path, std::ios::binary);
    auto u32 = [&](std::uint32_t v) { f.write(reinterpret_cast<const char*>(&v), 4); };
    auto u16 = [&](std::uint16_t v) { f.write(reinterpret_cast<const char*>(&v), 2); };

    // RIFF header
    f.write("RIFF", 4);
    u32(36 + data_bytes);
    f.write("WAVE", 4);
    // fmt chunk
    f.write("fmt ", 4);
    u32(16);                 // PCM chunk size
    u16(1);                  // audio format = PCM
    u16(channels);
    u32(sample_rate);
    u32(byte_rate);
    u16(block_align);
    u16(bits);
    // data chunk
    f.write("data", 4);
    u32(data_bytes);
    for (Sample s : interleaved) {
        // Clamp then convert float [-1,1] -> int16.
        float v = s;
        if (v >  1.0f) v =  1.0f;
        if (v < -1.0f) v = -1.0f;
        const std::int16_t iv = static_cast<std::int16_t>(v * 32767.0f);
        f.write(reinterpret_cast<const char*>(&iv), 2);
    }
}

int main() {
    std::printf("Aurora Audio Engine - synth demo\n");

    // -----------------------------------------------------------------------
    //  1. Build the engine and signal graph.
    // -----------------------------------------------------------------------
    AuroraEngine engine;
    auto& g = engine.graph();

    // Create nodes.
    auto osc_owner = std::make_unique<OscillatorNode>(Waveform::Saw, 110.0);
    auto flt_owner = std::make_unique<BiquadFilterNode>(FilterType::Lowpass, 1200.0, 0.9);
    auto env_owner = std::make_unique<EnvelopeNode>(
        EnvelopeNode::Params{.attack = 0.01f, .decay = 0.15f,
                             .sustain = 0.65f, .release = 0.30f});
    auto gain_owner = std::make_unique<GainNode>();
    gain_owner->set_gain_db(-6.0f); // -6 dB output headroom

    // Keep raw pointers so we can control them after transferring ownership.
    OscillatorNode*   osc = osc_owner.get();
    EnvelopeNode*     env = env_owner.get();

    const NodeId osc_id  = g.add_node(std::move(osc_owner));
    const NodeId flt_id  = g.add_node(std::move(flt_owner));
    const NodeId env_id  = g.add_node(std::move(env_owner));
    const NodeId gain_id = g.add_node(std::move(gain_owner));

    // Wire the chain: osc -> filter -> envelope -> gain.
    g.connect(osc_id,  0, flt_id,  0);
    g.connect(flt_id,  0, env_id,  0);
    g.connect(env_id,  0, gain_id, 0);
    g.set_output_node(gain_id);

    // -----------------------------------------------------------------------
    //  2. Configure the engine (Null backend, 48 kHz stereo).
    // -----------------------------------------------------------------------
    EngineConfig cfg{
        .sample_rate = 48000,
        .channels    = 2,
        .block_size  = 512,
        .backend     = hal::BackendType::Null,
    };
    if (!engine.configure(cfg)) {
        std::fprintf(stderr, "engine.configure() failed\n");
        return 1;
    }
    std::printf("Configured: %u Hz, %u ch, block=%u frames, order=%zu nodes\n",
                engine.format().sample_rate, engine.format().channels,
                cfg.block_size, engine.graph().execution_order().size());

    // -----------------------------------------------------------------------
    //  3. Briefly run the real-time NullBackend audio thread (std::jthread).
    //     Proves the same graph runs on the live render path (output silent).
    // -----------------------------------------------------------------------
    env->note_on();
    engine.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    engine.stop();
    std::printf("Real-time NullBackend ran for ~120 ms (silent output).\n");

    // -----------------------------------------------------------------------
    //  4. Deterministic offline render to build the WAV.
    //     Reset state, retrigger, render note-on portion, then note-off tail.
    // -----------------------------------------------------------------------
    g.reset_all();

    const SampleRate   sr = engine.format().sample_rate;
    const ChannelCount ch = engine.format().channels;

    // Play a short melody: three notes, each with an ADSR gate cycle.
    const double notes[] = { 110.0, 146.83, 220.0 }; // A2, D3, A3
    std::vector<Sample> song;

    for (double freq : notes) {
        osc->set_frequency(freq);
        // Sweep the filter cutoff a bit per note for character.
        // note-on: hold for 0.4 s
        env->note_on();
        auto on = engine.render_offline(static_cast<FrameCount>(0.40 * sr));
        // note-off: release tail 0.35 s
        env->note_off();
        auto off = engine.render_offline(static_cast<FrameCount>(0.35 * sr));

        song.insert(song.end(), on.begin(),  on.end());
        song.insert(song.end(), off.begin(), off.end());
    }

    // -----------------------------------------------------------------------
    //  5. Write output.wav next to the working directory.
    // -----------------------------------------------------------------------
    const char* out_path = "output.wav";
    write_wav(out_path, song, sr, ch);

    const std::size_t frames = song.size() / ch;
    std::printf("Wrote %s: %zu frames (%.2f s), %u Hz, %u ch\n",
                out_path, frames,
                static_cast<double>(frames) / sr, sr, ch);
    std::printf("Demo complete.\n");
    return 0;
}
