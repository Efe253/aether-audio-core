// ============================================================================
//  src/dsp/basic_nodes.cpp
//
//  Non-inline implementations for the basic DSP node library: wavetable
//  generation (additive, band-limited / Nyquist-compliant), RBJ biquad
//  coefficient computation, and per-node process() cycles.
//
//  Every process() below is allocation-free and lock-free. Inner loops are
//  written as plain index loops over contiguous data to encourage the compiler
//  to auto-vectorize (SSE/AVX/Neon) in optimized builds.
// ============================================================================
#include "aurora/dsp/basic_nodes.h"

#include <algorithm>
#include <cmath>

namespace aurora::dsp {

using constants::kPi;
using constants::kTwoPi;

// ===========================================================================
//  GainNode
// ===========================================================================
void GainNode::process(const ProcessContext& ctx,
                       std::span<const AudioBuffer> inputs,
                       std::span<AudioBuffer> outputs) {
    const float g = gain_.load(std::memory_order_relaxed);
    if (inputs.empty() || outputs.empty()) return;
    const AudioBuffer& in = inputs[0];
    AudioBuffer& out = outputs[0];
    const ChannelCount ch = std::min(in.channels(), out.channels());
    for (ChannelCount c = 0; c < ch; ++c) {
        const Sample* s = in.data(c);
        Sample* d = out.data(c);
        for (FrameCount f = 0; f < ctx.frames; ++f) d[f] = s[f] * g; // vectorizable
    }
}

// ===========================================================================
//  MixerNode
// ===========================================================================
void MixerNode::process(const ProcessContext& ctx,
                        std::span<const AudioBuffer> inputs,
                        std::span<AudioBuffer> outputs) {
    if (outputs.empty()) return;
    AudioBuffer& out = outputs[0];
    // Output was already zeroed by the graph; accumulate each input * gain.
    for (std::size_t p = 0; p < inputs.size(); ++p) {
        const AudioBuffer& in = inputs[p];
        const float g = (p < gains_.size()) ? gains_[p] : 1.0f;
        const ChannelCount ch = std::min(in.channels(), out.channels());
        for (ChannelCount c = 0; c < ch; ++c) {
            const Sample* s = in.data(c);
            Sample* d = out.data(c);
            for (FrameCount f = 0; f < ctx.frames; ++f) d[f] += s[f] * g;
        }
    }
}

// ===========================================================================
//  WavetableSet - band-limited additive synthesis
//
//  For each octave table t, the top frequency it will ever play is
//  kLowestFreq * 2^(t+1). We include only the harmonics whose frequency stays
//  below Nyquist (sample_rate / 2) at that top frequency, so playback of any
//  frequency mapped to that table is alias-free.
// ===========================================================================
void WavetableSet::generate(Waveform wave, SampleRate sample_rate) {
    const double nyquist = 0.5 * static_cast<double>(sample_rate);

    for (std::size_t t = 0; t < kNumTables; ++t) {
        // Highest fundamental this table serves (top of its octave).
        const double top_freq = kLowestFreq * std::pow(2.0, static_cast<double>(t) + 1.0);
        // Max harmonic number keeping harmonic*top_freq < Nyquist.
        int max_harmonic = static_cast<int>(std::floor(nyquist / top_freq));
        if (max_harmonic < 1) max_harmonic = 1;

        auto& table = tables_[t];
        // Build one cycle via additive synthesis.
        for (std::size_t i = 0; i < kTableSize; ++i) {
            const double phase = kTwoPi * static_cast<double>(i) /
                                 static_cast<double>(kTableSize);
            double value = 0.0;
            switch (wave) {
                case Waveform::Sine:
                    value = std::sin(phase);
                    max_harmonic = 1; // single harmonic
                    break;
                case Waveform::Saw:
                    // Ideal saw: sum_{k=1..N} (-1)^(k+1) * sin(k*phase) / k
                    // (research-grade band-limited sawtooth)
                    for (int k = 1; k <= max_harmonic; ++k) {
                        const double sign = (k % 2 == 1) ? 1.0 : -1.0;
                        value += sign * std::sin(k * phase) / k;
                    }
                    value *= 2.0 / kPi;
                    break;
                case Waveform::Square:
                    // Odd harmonics only, amplitude 1/k.
                    for (int k = 1; k <= max_harmonic; k += 2) {
                        value += std::sin(k * phase) / k;
                    }
                    value *= 4.0 / kPi;
                    break;
                case Waveform::Triangle:
                    // Odd harmonics, amplitude 1/k^2, alternating sign.
                    for (int k = 1; k <= max_harmonic; k += 2) {
                        const int    n    = (k - 1) / 2;
                        const double sign = (n % 2 == 0) ? 1.0 : -1.0;
                        value += sign * std::sin(k * phase) / (k * k);
                    }
                    value *= 8.0 / (kPi * kPi);
                    break;
            }
            table[i] = static_cast<float>(value);
        }
        // Guard sample enables branchless linear interpolation at the wrap.
        table[kTableSize] = table[0];
    }
}

// ===========================================================================
//  OscillatorNode
// ===========================================================================
void OscillatorNode::prepare(SampleRate sample_rate, FrameCount max_frames) {
    AudioNode::prepare(sample_rate, max_frames);
    tables_.generate(waveform_, sample_rate); // control-thread allocation
    regenerate_ = false;
}

void OscillatorNode::process(const ProcessContext& ctx,
                             std::span<const AudioBuffer> /*inputs*/,
                             std::span<AudioBuffer> outputs) {
    if (outputs.empty()) return;
    AudioBuffer& out = outputs[0];

    const double freq = frequency_.load(std::memory_order_relaxed);
    const float  amp  = amplitude_.load(std::memory_order_relaxed);
    const double inc  = freq / static_cast<double>(ctx.sample_rate); // phase/sample

    Sample* d = out.data(0);
    double phase = phase_;
    for (FrameCount f = 0; f < ctx.frames; ++f) {
        d[f] = amp * tables_.sample(freq, phase);
        phase += inc;
        if (phase >= 1.0) phase -= 1.0;
    }
    phase_ = phase;

    // Fan the mono oscillator out to any extra output channels.
    for (ChannelCount c = 1; c < out.channels(); ++c) {
        Sample* dc = out.data(c);
        for (FrameCount f = 0; f < ctx.frames; ++f) dc[f] = d[f];
    }
}

// ===========================================================================
//  BiquadFilterNode - Robert Bristow-Johnson "Audio EQ Cookbook" formulas
//
//  Intermediate variables (per the cookbook):
//      w0    = 2*pi*f0/Fs
//      alpha = sin(w0) / (2*Q)
//  Coefficients are then normalized by a0.
// ===========================================================================
BiquadCoeffs compute_biquad(FilterType type, double sample_rate,
                            double cutoff_hz, double q) {
    BiquadCoeffs c;

    // Clamp cutoff to a sane range below Nyquist for numerical stability.
    const double nyquist = 0.5 * sample_rate;
    if (cutoff_hz > 0.999 * nyquist) cutoff_hz = 0.999 * nyquist;
    if (cutoff_hz < 1.0)             cutoff_hz = 1.0;
    if (q < 1.0e-4)                  q = 1.0e-4;

    const double w0    = kTwoPi * cutoff_hz / sample_rate;
    const double cosw0 = std::cos(w0);
    const double sinw0 = std::sin(w0);
    const double alpha = sinw0 / (2.0 * q);

    double b0 = 0, b1 = 0, b2 = 0, a0 = 1, a1 = 0, a2 = 0;

    switch (type) {
        case FilterType::Lowpass:
            b0 = (1.0 - cosw0) * 0.5;
            b1 =  1.0 - cosw0;
            b2 = (1.0 - cosw0) * 0.5;
            a0 =  1.0 + alpha;
            a1 = -2.0 * cosw0;
            a2 =  1.0 - alpha;
            break;
        case FilterType::Highpass:
            b0 =  (1.0 + cosw0) * 0.5;
            b1 = -(1.0 + cosw0);
            b2 =  (1.0 + cosw0) * 0.5;
            a0 =   1.0 + alpha;
            a1 =  -2.0 * cosw0;
            a2 =   1.0 - alpha;
            break;
        case FilterType::Bandpass: // constant 0 dB peak gain (uses alpha for b)
            b0 =  alpha;
            b1 =  0.0;
            b2 = -alpha;
            a0 =  1.0 + alpha;
            a1 = -2.0 * cosw0;
            a2 =  1.0 - alpha;
            break;
        case FilterType::Notch:
            b0 =  1.0;
            b1 = -2.0 * cosw0;
            b2 =  1.0;
            a0 =  1.0 + alpha;
            a1 = -2.0 * cosw0;
            a2 =  1.0 - alpha;
            break;
    }

    const double inv_a0 = 1.0 / a0;
    c.b0 = static_cast<float>(b0 * inv_a0);
    c.b1 = static_cast<float>(b1 * inv_a0);
    c.b2 = static_cast<float>(b2 * inv_a0);
    c.a1 = static_cast<float>(a1 * inv_a0);
    c.a2 = static_cast<float>(a2 * inv_a0);
    return c;
}

void BiquadFilterNode::prepare(SampleRate sample_rate, FrameCount max_frames) {
    AudioNode::prepare(sample_rate, max_frames);
    // Allocate per-channel state (mono port here, but keep it general).
    state_.assign(AudioBuffer::kMaxChannels, State{});
    dirty_ = true;
    update_coeffs();
}

void BiquadFilterNode::reset() {
    for (auto& s : state_) { s.z1 = 0; s.z2 = 0; }
}

void BiquadFilterNode::update_coeffs() {
    if (!dirty_) return;
    coeffs_ = compute_biquad(type_, static_cast<double>(sample_rate_), cutoff_, q_);
    dirty_ = false;
}

void BiquadFilterNode::process(const ProcessContext& ctx,
                               std::span<const AudioBuffer> inputs,
                               std::span<AudioBuffer> outputs) {
    if (inputs.empty() || outputs.empty()) return;
    update_coeffs(); // coefficient recompute is cheap & bounded; no allocation

    const AudioBuffer& in = inputs[0];
    AudioBuffer& out = outputs[0];
    const ChannelCount ch = std::min(in.channels(), out.channels());

    const float b0 = coeffs_.b0, b1 = coeffs_.b1, b2 = coeffs_.b2;
    const float a1 = coeffs_.a1, a2 = coeffs_.a2;

    for (ChannelCount c = 0; c < ch; ++c) {
        const Sample* s = in.data(c);
        Sample* d = out.data(c);
        // Direct Form II Transposed: numerically robust, only 2 state vars.
        float z1 = state_[c].z1;
        float z2 = state_[c].z2;
        for (FrameCount f = 0; f < ctx.frames; ++f) {
            const float x = s[f];
            const float y = b0 * x + z1;
            z1 = b1 * x - a1 * y + z2;
            z2 = b2 * x - a2 * y;
            d[f] = y;
        }
        state_[c].z1 = z1;
        state_[c].z2 = z2;
    }
}

// ===========================================================================
//  DelayNode - circular buffer delay line with feedback and wet/dry mix
// ===========================================================================
void DelayNode::prepare(SampleRate sample_rate, FrameCount max_frames) {
    AudioNode::prepare(sample_rate, max_frames);
    dirty_ = true;
    // Size lines for up to 10 seconds of delay so set_delay_seconds() at run
    // time never needs to reallocate.
    const std::size_t max_delay = static_cast<std::size_t>(sample_rate) * 10;
    lines_.assign(AudioBuffer::kMaxChannels, std::vector<float>(max_delay, 0.0f));
    write_pos_.assign(AudioBuffer::kMaxChannels, 0);
    delay_samples_ = std::min<std::size_t>(
        static_cast<std::size_t>(delay_seconds_ * sample_rate), max_delay - 1);
    dirty_ = false;
}

void DelayNode::reset() {
    for (auto& line : lines_) std::fill(line.begin(), line.end(), 0.0f);
    std::fill(write_pos_.begin(), write_pos_.end(), std::size_t{0});
}

void DelayNode::process(const ProcessContext& ctx,
                        std::span<const AudioBuffer> inputs,
                        std::span<AudioBuffer> outputs) {
    if (inputs.empty() || outputs.empty()) return;
    if (dirty_) {
        // Update delay length without reallocating (buffers are max-sized).
        const std::size_t cap = lines_.empty() ? 1 : lines_[0].size();
        delay_samples_ = std::min<std::size_t>(
            static_cast<std::size_t>(delay_seconds_ * sample_rate_), cap - 1);
        dirty_ = false;
    }

    const AudioBuffer& in = inputs[0];
    AudioBuffer& out = outputs[0];
    const ChannelCount ch = std::min(in.channels(), out.channels());
    const float fb  = feedback_.load(std::memory_order_relaxed);
    const float mix = mix_.load(std::memory_order_relaxed);
    const std::size_t D = delay_samples_;

    for (ChannelCount c = 0; c < ch; ++c) {
        auto& line = lines_[c];
        const std::size_t cap = line.size();
        std::size_t w = write_pos_[c];
        const Sample* s = in.data(c);
        Sample* d = out.data(c);
        for (FrameCount f = 0; f < ctx.frames; ++f) {
            const std::size_t r = (w + cap - D) % cap; // read D samples back
            const float delayed = line[r];
            const float x = s[f];
            line[w] = x + delayed * fb;          // write input + feedback
            d[f] = x * (1.0f - mix) + delayed * mix; // wet/dry blend
            w = (w + 1) % cap;
        }
        write_pos_[c] = w;
    }
}

// ===========================================================================
//  EnvelopeNode - linear ADSR applied to the incoming signal
// ===========================================================================
void EnvelopeNode::prepare(SampleRate sample_rate, FrameCount max_frames) {
    AudioNode::prepare(sample_rate, max_frames);
    recompute_rates();
    reset();
}

void EnvelopeNode::reset() {
    stage_ = Stage::Idle;
    level_ = 0.0f;
    last_gate_ = false;
}

void EnvelopeNode::recompute_rates() {
    const float sr = static_cast<float>(sample_rate_);
    // Per-sample linear increments. Guard against zero-length stages.
    attack_inc_  = params_.attack  > 0.0f ? 1.0f / (params_.attack  * sr) : 1.0f;
    decay_inc_   = params_.decay   > 0.0f ? (1.0f - params_.sustain) /
                                            (params_.decay * sr) : 1.0f;
    release_inc_ = params_.release > 0.0f ? params_.sustain /
                                            (params_.release * sr) : 1.0f;
}

void EnvelopeNode::process(const ProcessContext& ctx,
                           std::span<const AudioBuffer> inputs,
                           std::span<AudioBuffer> outputs) {
    if (inputs.empty() || outputs.empty()) return;
    const AudioBuffer& in = inputs[0];
    AudioBuffer& out = outputs[0];
    const ChannelCount ch = std::min(in.channels(), out.channels());

    const bool gate = gate_.load(std::memory_order_relaxed);

    // Compute the envelope for the block into a small stack path per sample,
    // applying it identically across channels. We iterate sample-outer so gate
    // edges are handled sample-accurately.
    float level = level_;
    Stage stage = stage_;
    bool  last_gate = last_gate_;

    for (FrameCount f = 0; f < ctx.frames; ++f) {
        // Detect gate edges.
        if (gate && !last_gate)      stage = Stage::Attack;
        else if (!gate && last_gate) stage = Stage::Release;
        last_gate = gate;

        switch (stage) {
            case Stage::Idle:
                level = 0.0f;
                break;
            case Stage::Attack:
                level += attack_inc_;
                if (level >= 1.0f) { level = 1.0f; stage = Stage::Decay; }
                break;
            case Stage::Decay:
                level -= decay_inc_;
                if (level <= params_.sustain) { level = params_.sustain; stage = Stage::Sustain; }
                break;
            case Stage::Sustain:
                level = params_.sustain;
                break;
            case Stage::Release:
                level -= release_inc_;
                if (level <= 0.0f) { level = 0.0f; stage = Stage::Idle; }
                break;
        }

        for (ChannelCount c = 0; c < ch; ++c) {
            out.data(c)[f] = in.data(c)[f] * level;
        }
    }

    level_ = level;
    stage_ = stage;
    last_gate_ = last_gate;
}

} // namespace aurora::dsp
