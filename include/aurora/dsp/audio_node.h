// ============================================================================
//  aurora/dsp/audio_node.h
//
//  AudioNode: the atomic unit of the DSP graph.
//
//  Each node consumes zero or more input port buffers, runs a bounded-time
//  process() over one block of frames, and writes zero or more output port
//  buffers. Nodes are connected into a Directed Acyclic Graph (see
//  audio_graph.h) which is topologically sorted and executed once per block.
//
//  C++20 concepts are used to constrain what qualifies as a "node type" at
//  compile time, giving clear errors and enabling generic node factories.
// ============================================================================
#ifndef AURORA_DSP_AUDIO_NODE_H
#define AURORA_DSP_AUDIO_NODE_H

#include <concepts>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "aurora/core/types.h"

namespace aurora::dsp {

/// Per-block context handed to every node's process() call. Contains only
/// immutable timing information — no allocation, no locking.
struct ProcessContext {
    SampleRate sample_rate = 48000;
    FrameCount frames      = 0;   ///< frames to process this block
    std::uint64_t stream_time = 0; ///< frames elapsed since stream start
};

/// Describes a single input or output port of a node.
struct PortInfo {
    std::string  name;
    ChannelCount channels = 1;
};

// ---------------------------------------------------------------------------
//  AudioNode base class
//
//  Concrete DSP nodes derive from this and implement process(). The graph owns
//  scratch buffers and passes const input views + mutable output views.
// ---------------------------------------------------------------------------
class AudioNode {
public:
    virtual ~AudioNode() = default;

    /// Called once before streaming starts (control thread). Nodes may allocate
    /// here — this is NOT the real-time path. `max_frames` is the largest block
    /// size process() will ever receive.
    virtual void prepare(SampleRate sample_rate, FrameCount max_frames) {
        sample_rate_ = sample_rate;
        max_frames_  = max_frames;
    }

    /// The real-time DSP callback. MUST be bounded-time and allocation-free.
    ///
    ///  inputs  : one AudioBuffer view per input port (already summed/routed
    ///            by the graph). May be empty for source nodes.
    ///  outputs : one AudioBuffer view per output port to be filled.
    virtual void process(const ProcessContext& ctx,
                         std::span<const AudioBuffer> inputs,
                         std::span<AudioBuffer> outputs) = 0;

    /// Reset internal state (delay lines, filter memory, phase, ...).
    virtual void reset() {}

    // ---- Port topology (queried by the graph during compilation) --------
    [[nodiscard]] virtual std::size_t num_inputs()  const = 0;
    [[nodiscard]] virtual std::size_t num_outputs() const = 0;

    [[nodiscard]] virtual PortInfo input_port(std::size_t /*i*/) const {
        return PortInfo{"in", 1};
    }
    [[nodiscard]] virtual PortInfo output_port(std::size_t /*i*/) const {
        return PortInfo{"out", 1};
    }

    /// Human-readable node name (debugging / graph visualization).
    [[nodiscard]] virtual const char* type_name() const { return "AudioNode"; }

    void        set_name(std::string name) { name_ = std::move(name); }
    [[nodiscard]] const std::string& name() const { return name_; }

protected:
    SampleRate  sample_rate_ = 48000;
    FrameCount  max_frames_  = 0;
    std::string name_;
};

// ---------------------------------------------------------------------------
//  C++20 concepts constraining node-like types
// ---------------------------------------------------------------------------

/// A type models AudioProcessable if it derives from AudioNode and provides the
/// required real-time interface. Used to constrain generic factory templates.
template <typename T>
concept AudioProcessable =
    std::derived_from<T, AudioNode> &&
    requires(T node,
             const ProcessContext& ctx,
             std::span<const AudioBuffer> in,
             std::span<AudioBuffer> out) {
        { node.process(ctx, in, out) } -> std::same_as<void>;
        { node.num_inputs() }  -> std::convertible_to<std::size_t>;
        { node.num_outputs() } -> std::convertible_to<std::size_t>;
    };

/// A source node produces audio without any input ports (oscillators, etc.).
template <typename T>
concept SourceNode = AudioProcessable<T>;

/// A sink node consumes audio and produces no output ports (device outputs).
template <typename T>
concept SinkNode = AudioProcessable<T>;

} // namespace aurora::dsp

#endif // AURORA_DSP_AUDIO_NODE_H
