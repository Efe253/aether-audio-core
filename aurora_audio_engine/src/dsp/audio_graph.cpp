// ============================================================================
//  src/dsp/audio_graph.cpp
//
//  Implementation of the DAG audio graph: node ownership, cycle-checked
//  connections, Kahn topological sort, buffer pre-allocation and the real-time
//  process cycle.
// ============================================================================
#include "aurora/dsp/audio_graph.h"

#include <algorithm>
#include <queue>

namespace aurora::dsp {

// ---------------------------------------------------------------------------
//  Helpers
// ---------------------------------------------------------------------------
std::size_t AudioGraph::index_of(NodeId id) const {
    const auto raw = static_cast<std::uint32_t>(id);
    if (raw >= nodes_.size()) return static_cast<std::size_t>(-1);
    return nodes_[raw].alive ? raw : static_cast<std::size_t>(-1);
}

AudioNode* AudioGraph::node(NodeId id) {
    const std::size_t i = index_of(id);
    return i == static_cast<std::size_t>(-1) ? nullptr : nodes_[i].node.get();
}

// ---------------------------------------------------------------------------
//  Structural editing
// ---------------------------------------------------------------------------
NodeId AudioGraph::add_node(std::unique_ptr<AudioNode> node) {
    prepared_ = false;
    // Reuse a dead slot if available (keeps NodeId == index invariant simple).
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
        if (!nodes_[i].alive) {
            nodes_[i].node  = std::move(node);
            nodes_[i].alive = true;
            return static_cast<NodeId>(i);
        }
    }
    nodes_.push_back(NodeSlot{std::move(node), true, {}, {}});
    return static_cast<NodeId>(nodes_.size() - 1);
}

bool AudioGraph::remove_node(NodeId id) {
    const std::size_t i = index_of(id);
    if (i == static_cast<std::size_t>(-1)) return false;
    prepared_ = false;
    nodes_[i].node.reset();
    nodes_[i].alive = false;
    nodes_[i].output_buffers.clear();
    nodes_[i].input_buffers.clear();
    // Drop every connection touching this node.
    connections_.erase(
        std::remove_if(connections_.begin(), connections_.end(),
            [id](const Connection& c) {
                return c.src_node == id || c.dst_node == id;
            }),
        connections_.end());
    if (output_node_ == id) output_node_ = NodeId::Invalid;
    return true;
}

bool AudioGraph::would_create_cycle(NodeId src, NodeId dst) const {
    // Adding edge src->dst creates a cycle iff dst can already reach src.
    // Depth-first search from dst following existing edges.
    if (src == dst) return true;
    std::vector<NodeId> stack{dst};
    std::vector<bool>   visited(nodes_.size(), false);
    while (!stack.empty()) {
        NodeId cur = stack.back();
        stack.pop_back();
        const auto ci = static_cast<std::uint32_t>(cur);
        if (ci < visited.size()) {
            if (visited[ci]) continue;
            visited[ci] = true;
        }
        if (cur == src) return true;
        for (const auto& c : connections_) {
            if (c.src_node == cur) stack.push_back(c.dst_node);
        }
    }
    return false;
}

bool AudioGraph::connect(NodeId src, std::size_t src_port,
                         NodeId dst, std::size_t dst_port) {
    const std::size_t si = index_of(src);
    const std::size_t di = index_of(dst);
    if (si == static_cast<std::size_t>(-1) || di == static_cast<std::size_t>(-1))
        return false;
    if (src_port >= nodes_[si].node->num_outputs()) return false;
    if (dst_port >= nodes_[di].node->num_inputs())  return false;
    if (would_create_cycle(src, dst)) return false;

    connections_.push_back(Connection{src, src_port, dst, dst_port});
    prepared_ = false;
    return true;
}

bool AudioGraph::disconnect(NodeId src, std::size_t src_port,
                            NodeId dst, std::size_t dst_port) {
    const auto before = connections_.size();
    connections_.erase(
        std::remove_if(connections_.begin(), connections_.end(),
            [&](const Connection& c) {
                return c.src_node == src && c.src_port == src_port &&
                       c.dst_node == dst && c.dst_port == dst_port;
            }),
        connections_.end());
    const bool changed = connections_.size() != before;
    if (changed) prepared_ = false;
    return changed;
}

// ---------------------------------------------------------------------------
//  Topological sort (Kahn's algorithm)
// ---------------------------------------------------------------------------
bool AudioGraph::topological_sort(std::vector<NodeId>& out_order) const {
    const std::size_t n = nodes_.size();
    std::vector<int>  in_degree(n, 0);
    std::vector<bool> present(n, false);

    for (std::size_t i = 0; i < n; ++i) present[i] = nodes_[i].alive;

    for (const auto& c : connections_) {
        const auto d = static_cast<std::uint32_t>(c.dst_node);
        if (d < n) in_degree[d]++;
    }

    std::queue<NodeId> ready;
    for (std::size_t i = 0; i < n; ++i) {
        if (present[i] && in_degree[i] == 0) {
            ready.push(static_cast<NodeId>(i));
        }
    }

    out_order.clear();
    while (!ready.empty()) {
        NodeId u = ready.front();
        ready.pop();
        out_order.push_back(u);
        for (const auto& c : connections_) {
            if (c.src_node != u) continue;
            const auto d = static_cast<std::uint32_t>(c.dst_node);
            if (d < n && --in_degree[d] == 0) {
                ready.push(c.dst_node);
            }
        }
    }

    // If we couldn't order every live node, a cycle exists.
    std::size_t live = 0;
    for (std::size_t i = 0; i < n; ++i) if (present[i]) ++live;
    return out_order.size() == live;
}

// ---------------------------------------------------------------------------
//  Compilation: prepare()
// ---------------------------------------------------------------------------
bool AudioGraph::prepare(SampleRate sample_rate, FrameCount max_frames) {
    sample_rate_ = sample_rate;
    max_frames_  = max_frames;

    if (!topological_sort(execution_order_)) {
        prepared_ = false;
        return false; // cycle detected
    }

    std::size_t max_ports = 1;

    // Allocate every node's I/O buffers once. Channels default to 1 per port
    // (mono internal signal) which keeps the prototype simple; PortInfo could
    // widen this per port.
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
        auto& slot = nodes_[i];
        if (!slot.alive) continue;
        slot.node->prepare(sample_rate, max_frames);

        const std::size_t no = slot.node->num_outputs();
        const std::size_t ni = slot.node->num_inputs();
        max_ports = std::max({max_ports, no, ni});

        slot.output_buffers.clear();
        slot.output_buffers.resize(no);
        for (std::size_t p = 0; p < no; ++p) {
            const ChannelCount ch = slot.node->output_port(p).channels;
            slot.output_buffers[p].resize(ch, max_frames);
        }

        slot.input_buffers.clear();
        slot.input_buffers.resize(ni);
        for (std::size_t p = 0; p < ni; ++p) {
            const ChannelCount ch = slot.node->input_port(p).channels;
            slot.input_buffers[p].resize(ch, max_frames);
        }
    }

    // Pre-size the RT scratch view vectors so process() never allocates.
    scratch_inputs_.assign(max_ports, AudioBuffer{});
    scratch_outputs_.assign(max_ports, AudioBuffer{});

    prepared_ = true;
    return true;
}

void AudioGraph::reset_all() {
    for (auto& slot : nodes_) {
        if (slot.alive) slot.node->reset();
    }
}

// ---------------------------------------------------------------------------
//  Real-time process cycle
//
//  For each node, in topological order:
//    1. Zero its input buffers, then sum every connected upstream output into
//       the matching input port (fan-in mixing).
//    2. Build span views over its input and output buffers.
//    3. Invoke node->process().
//  Finally, copy the graph output node's first output port into `output`.
//
//  No allocation, no locking anywhere below.
// ---------------------------------------------------------------------------
void AudioGraph::process(const ProcessContext& ctx, AudioBuffer& output) {
    if (!prepared_) { output.clear(); return; }

    const FrameCount frames = ctx.frames;

    for (NodeId id : execution_order_) {
        const auto ni = static_cast<std::uint32_t>(id);
        NodeSlot& slot = nodes_[ni];
        if (!slot.alive) continue;

        // ---- 1. Gather inputs (sum all incoming connections per port) -----
        for (std::size_t p = 0; p < slot.input_buffers.size(); ++p) {
            AudioBuffer in = slot.input_buffers[p].view();
            in.set_frames(frames);
            in.clear();
            for (const auto& c : connections_) {
                if (!(c.dst_node == id && c.dst_port == p)) continue;
                const auto srci = static_cast<std::uint32_t>(c.src_node);
                NodeSlot& src = nodes_[srci];
                if (!src.alive || c.src_port >= src.output_buffers.size()) continue;
                AudioBuffer up = src.output_buffers[c.src_port].view();
                const ChannelCount ch = std::min(in.channels(), up.channels());
                for (ChannelCount cc = 0; cc < ch; ++cc) {
                    Sample* dst = in.data(cc);
                    const Sample* s = up.data(cc);
                    // Vectorizable accumulation loop.
                    for (FrameCount f = 0; f < frames; ++f) dst[f] += s[f];
                }
            }
            scratch_inputs_[p] = in;
        }

        // ---- 2. Prepare output views --------------------------------------
        for (std::size_t p = 0; p < slot.output_buffers.size(); ++p) {
            AudioBuffer out = slot.output_buffers[p].view();
            out.set_frames(frames);
            out.clear();
            scratch_outputs_[p] = out;
        }

        // ---- 3. Run the node ----------------------------------------------
        std::span<const AudioBuffer> in_span(scratch_inputs_.data(),
                                             slot.input_buffers.size());
        std::span<AudioBuffer> out_span(scratch_outputs_.data(),
                                        slot.output_buffers.size());
        slot.node->process(ctx, in_span, out_span);
    }

    // ---- Copy graph output ------------------------------------------------
    output.clear();
    const std::size_t oi = index_of(output_node_);
    if (oi == static_cast<std::size_t>(-1)) return;
    NodeSlot& out_slot = nodes_[oi];
    if (out_slot.output_buffers.empty()) return;

    AudioBuffer src = out_slot.output_buffers[0].view();
    const ChannelCount ch = std::min(output.channels(), src.channels());
    for (ChannelCount c = 0; c < ch; ++c) {
        const Sample* s = src.data(c);
        Sample* d = output.data(c);
        for (FrameCount f = 0; f < frames; ++f) d[f] = s[f];
    }
    // If the source is mono but output is stereo, fan the single channel out.
    if (src.channels() == 1 && output.channels() > 1) {
        const Sample* s = src.data(0);
        for (ChannelCount c = 1; c < output.channels(); ++c) {
            Sample* d = output.data(c);
            for (FrameCount f = 0; f < frames; ++f) d[f] = s[f];
        }
    }
}

} // namespace aurora::dsp
