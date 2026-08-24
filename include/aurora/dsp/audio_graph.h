// ============================================================================
//  aurora/dsp/audio_graph.h
//
//  DAG-based audio processing graph.
//
//  Model
//  -----
//  * Nodes are owned by the graph (unique_ptr) and referenced by a stable
//    NodeId handle.
//  * A Connection links (source node, source output port) to
//    (destination node, destination input port).
//  * On prepare(), the graph validates acyclicity, computes a topological
//    execution order, and pre-allocates every scratch/output buffer it will
//    need. NOTHING is allocated afterwards.
//  * On process() — the real-time path — nodes run in topological order. For
//    each node, connected upstream outputs are summed into the node's input
//    buffers, then process() is invoked. This is the single-render-thread,
//    pre-compiled-order execution model recommended in the research report.
//
//  Structural mutation (add/remove/connect) is a CONTROL-thread operation and
//  must not be performed concurrently with process().
// ============================================================================
#ifndef AURORA_DSP_AUDIO_GRAPH_H
#define AURORA_DSP_AUDIO_GRAPH_H

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "aurora/core/types.h"
#include "aurora/dsp/audio_node.h"

namespace aurora::dsp {

/// Stable handle to a node inside a graph.
enum class NodeId : std::uint32_t { Invalid = 0xFFFFFFFFu };

/// A directed edge between two node ports.
struct Connection {
    NodeId      src_node;
    std::size_t src_port;
    NodeId      dst_node;
    std::size_t dst_port;
};

// ---------------------------------------------------------------------------
//  AudioGraph
// ---------------------------------------------------------------------------
class AudioGraph {
public:
    AudioGraph() = default;
    ~AudioGraph() = default;

    AudioGraph(const AudioGraph&)            = delete;
    AudioGraph& operator=(const AudioGraph&) = delete;

    // ---- Structural editing (control thread only) -----------------------

    /// Take ownership of `node` and return its handle.
    NodeId add_node(std::unique_ptr<AudioNode> node);

    /// Remove a node and all connections touching it. Returns false if unknown.
    bool remove_node(NodeId id);

    /// Connect src output port -> dst input port. Returns false on bad ports
    /// or if it would introduce a cycle.
    bool connect(NodeId src, std::size_t src_port,
                 NodeId dst, std::size_t dst_port);

    /// Disconnect a specific edge (if present).
    bool disconnect(NodeId src, std::size_t src_port,
                    NodeId dst, std::size_t dst_port);

    /// Designate the node whose (first) output port is the graph's output.
    void set_output_node(NodeId id) { output_node_ = id; }

    [[nodiscard]] NodeId output_node() const { return output_node_; }

    /// Access a node (control thread; e.g. to tweak parameters before start).
    [[nodiscard]] AudioNode* node(NodeId id);

    [[nodiscard]] std::size_t node_count() const { return nodes_.size(); }

    // ---- Compilation ----------------------------------------------------

    /// Validate + topologically sort + pre-allocate all buffers. Must be called
    /// (on the control thread) before process(). Returns false on cycle.
    bool prepare(SampleRate sample_rate, FrameCount max_frames);

    /// Read-only access to the compiled execution order (topological).
    [[nodiscard]] std::span<const NodeId> execution_order() const {
        return execution_order_;
    }

    // ---- Real-time processing (audio thread) ----------------------------

    /// Run one block. `output` receives the graph output node's first port.
    /// Real-time safe: no allocation, no locking. `ctx.frames` <= max_frames.
    void process(const ProcessContext& ctx, AudioBuffer& output);

    /// Reset every node's internal state (control thread).
    void reset_all();

private:
    struct NodeSlot {
        std::unique_ptr<AudioNode>    node;
        bool                          alive = false;
        // One owned buffer per output port (produced this block).
        std::vector<OwnedAudioBuffer> output_buffers;
        // One owned buffer per input port (summed upstream signal).
        std::vector<OwnedAudioBuffer> input_buffers;
    };

    [[nodiscard]] std::size_t index_of(NodeId id) const;
    [[nodiscard]] bool would_create_cycle(NodeId src, NodeId dst) const;
    bool topological_sort(std::vector<NodeId>& out_order) const;

    std::vector<NodeSlot>   nodes_;
    std::vector<Connection> connections_;
    NodeId                  output_node_ = NodeId::Invalid;

    // Compiled state (filled by prepare()).
    std::vector<NodeId>     execution_order_;
    SampleRate              sample_rate_ = 48000;
    FrameCount              max_frames_  = 0;
    bool                    prepared_    = false;

    // Reusable scratch vectors for process() so we never allocate on the RT
    // path. Sized during prepare() to the max port fan-in/out.
    mutable std::vector<AudioBuffer> scratch_inputs_;
    mutable std::vector<AudioBuffer> scratch_outputs_;
};

} // namespace aurora::dsp

#endif // AURORA_DSP_AUDIO_GRAPH_H
