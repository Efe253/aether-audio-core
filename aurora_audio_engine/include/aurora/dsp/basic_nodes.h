// ============================================================================
//  aurora/dsp/basic_nodes.h
//
//  A library of fundamental, real-time-safe DSP nodes:
//
//    * GainNode          - volume control with dB<->linear conversion
//    * MixerNode         - sums an arbitrary number of inputs
//    * OscillatorNode    - band-limited wavetable oscillator
//                          (sine / saw / square / triangle)
//    * BiquadFilterNode  - RBJ biquad (lowpass/highpass/bandpass/notch)
//    * DelayNode         - circular-buffer delay line with feedback
//    * EnvelopeNode      - ADSR envelope applied to its input
//
//  Real-time contract
//  ------------------
//  Every process() below is allocation-free, lock-free and bounded-time. All
//  buffers are allocated in prepare() (control thread). Inner loops are written
//  as simple index loops over std::span so the compiler can auto-vectorize.
// ============================================================================
#ifndef AURORA_DSP_BASIC_NODES_H
#define AURORA_DSP_BASIC_NODES_H

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "aurora/core/types.h"
#include "aurora/dsp/audio_node.h"

namespace aurora::dsp {

// ===========================================================================
//  Decibel helpers (science-accurate)
//
//  Amplitude (field quantity) uses 20*log10, since power ~ amplitude^2.
// ===========================================================================
namespace db {
    /// Convert a linear amplitude gain to decibels. gain 1.0 -> 0 dB.
    [[nodiscard]] inline float to_db(float linear) noexcept {
        constexpr float kFloor = 1.0e-9f; // ~ -180 dB, avoids log10(0)
        return 20.0f * std::log10(linear < kFloor ? kFloor : linear);
    }
    /// Convert decibels to a linear amplitude gain. 0 dB -> 1.0.
    [[nodiscard]] inline float from_db(float decibels) noexcept {
        return std::pow(10.0f, decibels * (1.0f / 20.0f));
    }
}

// ===========================================================================
//  GainNode
// ===========================================================================
class GainNode : public AudioNode {
public:
    explicit GainNode(float linear_gain = 1.0f) : gain_(linear_gain) {}

    /// Set gain as a linear amplitude multiplier (1.0 = unity).
    void set_gain_linear(float g) noexcept { gain_.store(g, std::memory_order_relaxed); }
    /// Set gain in decibels (0 dB = unity).
    void set_gain_db(float d) noexcept { gain_.store(db::from_db(d), std::memory_order_relaxed); }
    [[nodiscard]] float gain_linear() const noexcept { return gain_.load(std::memory_order_relaxed); }
    [[nodiscard]] float gain_db()     const noexcept { return db::to_db(gain_linear()); }

    void process(const ProcessContext& ctx,
                 std::span<const AudioBuffer> inputs,
                 std::span<AudioBuffer> outputs) override;

    [[nodiscard]] std::size_t num_inputs()  const override { return 1; }
    [[nodiscard]] std::size_t num_outputs() const override { return 1; }
    [[nodiscard]] const char* type_name()   const override { return "GainNode"; }

private:
    std::atomic<float> gain_; // atomic so control thread can update live
};

// ===========================================================================
//  MixerNode
//
//  Sums `num_inputs` input ports into a single output. Optional per-input gain.
// ===========================================================================
class MixerNode : public AudioNode {
public:
    explicit MixerNode(std::size_t num_inputs = 2)
        : inputs_(num_inputs), gains_(num_inputs, 1.0f) {}

    void set_input_gain(std::size_t port, float linear) {
        if (port < gains_.size()) gains_[port] = linear;
    }

    void process(const ProcessContext& ctx,
                 std::span<const AudioBuffer> inputs,
                 std::span<AudioBuffer> outputs) override;

    [[nodiscard]] std::size_t num_inputs()  const override { return inputs_; }
    [[nodiscard]] std::size_t num_outputs() const override { return 1; }
    [[nodiscard]] const char* type_name()   const override { return "MixerNode"; }

private:
    std::size_t        inputs_;
    std::vector<float> gains_;
};

// ===========================================================================
//  Wavetable oscillator
// ===========================================================================
enum class Waveform : std::uint8_t { Sine, Saw, Square, Triangle };

// A mip-mapped, band-limited wavetable set. One table per octave; each table
// contains only harmonics that stay below Nyquist for that octave's top
// frequency, guaranteeing an alias-free (Nyquist-compliant) oscillator.
// Table generation (additive synthesis) lives in basic_nodes.cpp.
class WavetableSet {
public:
    static constexpr std::size_t kTableSize = 2048;      // samples per table (+guard)
    static constexpr std::size_t kNumTables = 11;        // ~11 octaves of coverage
    static constexpr double      kLowestFreq = 20.0;     // Hz, base of table 0

    /// Build all band-limited tables for `wave` at `sample_rate`. Control-thread
    /// only (allocates). Safe to call in prepare().
    void generate(Waveform wave, SampleRate sample_rate);

    /// Sample the appropriate band-limited table for `freq` at normalized
    /// `phase` in [0,1). Linear interpolation. Real-time safe (read only).
    [[nodiscard]] float sample(double freq, double phase) const noexcept {
        const std::size_t t = select_table(freq);
        const auto& tbl = tables_[t];
        const double x = phase * static_cast<double>(kTableSize);
        const std::size_t i0 = static_cast<std::size_t>(x);
        const double frac = x - static_cast<double>(i0);
        // tables have a guard sample at [kTableSize] == [0] for cheap interp.
        const float a = tbl[i0];
        const float b = tbl[i0 + 1];
        return static_cast<float>(a + (b - a) * frac);
    }

private:
    [[nodiscard]] std::size_t select_table(double freq) const noexcept {
        if (freq <= kLowestFreq) return 0;
        // Each successive table doubles the base frequency (one octave).
        const double octave = std::log2(freq / kLowestFreq);
        std::size_t idx = static_cast<std::size_t>(octave);
        return idx < kNumTables ? idx : (kNumTables - 1);
    }

    // tables_[t] has kTableSize+1 entries (last is a guard copy of the first).
    std::array<std::array<float, kTableSize + 1>, kNumTables> tables_{};
};

class OscillatorNode : public AudioNode {
public:
    explicit OscillatorNode(Waveform wave = Waveform::Sine, double freq_hz = 440.0)
        : waveform_(wave), frequency_(freq_hz) {}

    void set_frequency(double hz) noexcept { frequency_.store(hz, std::memory_order_relaxed); }
    void set_waveform(Waveform w)          { waveform_ = w; regenerate_ = true; }
    void set_amplitude(float a) noexcept   { amplitude_.store(a, std::memory_order_relaxed); }

    [[nodiscard]] double frequency() const noexcept { return frequency_.load(std::memory_order_relaxed); }

    void prepare(SampleRate sample_rate, FrameCount max_frames) override;
    void reset() override { phase_ = 0.0; }

    void process(const ProcessContext& ctx,
                 std::span<const AudioBuffer> inputs,
                 std::span<AudioBuffer> outputs) override;

    // Source node: no inputs, one output.
    [[nodiscard]] std::size_t num_inputs()  const override { return 0; }
    [[nodiscard]] std::size_t num_outputs() const override { return 1; }
    [[nodiscard]] const char* type_name()   const override { return "OscillatorNode"; }

private:
    Waveform            waveform_;
    std::atomic<double> frequency_;
    std::atomic<float>  amplitude_{1.0f};
    double              phase_ = 0.0;     // normalized [0,1)
    bool                regenerate_ = false;
    WavetableSet        tables_;
};

// ===========================================================================
//  BiquadFilterNode  (Robert Bristow-Johnson cookbook formulas)
// ===========================================================================
enum class FilterType : std::uint8_t { Lowpass, Highpass, Bandpass, Notch };

// Transfer function:  H(z) = (b0 + b1 z^-1 + b2 z^-2) / (1 + a1 z^-1 + a2 z^-2)
// Coefficients are already normalized by a0. Computed in basic_nodes.cpp.
struct BiquadCoeffs {
    float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
};

/// Compute normalized RBJ biquad coefficients. Defined in basic_nodes.cpp.
[[nodiscard]] BiquadCoeffs compute_biquad(FilterType type,
                                          double sample_rate,
                                          double cutoff_hz,
                                          double q);

class BiquadFilterNode : public AudioNode {
public:
    BiquadFilterNode(FilterType type = FilterType::Lowpass,
                     double cutoff_hz = 1000.0, double q = 0.70710678)
        : type_(type), cutoff_(cutoff_hz), q_(q) {}

    void set_cutoff(double hz) { cutoff_ = hz; dirty_ = true; }
    void set_q(double q)       { q_ = q;       dirty_ = true; }
    void set_type(FilterType t){ type_ = t;    dirty_ = true; }

    void prepare(SampleRate sample_rate, FrameCount max_frames) override;
    void reset() override;

    void process(const ProcessContext& ctx,
                 std::span<const AudioBuffer> inputs,
                 std::span<AudioBuffer> outputs) override;

    [[nodiscard]] std::size_t num_inputs()  const override { return 1; }
    [[nodiscard]] std::size_t num_outputs() const override { return 1; }
    [[nodiscard]] const char* type_name()   const override { return "BiquadFilterNode"; }

private:
    void update_coeffs();

    FilterType   type_;
    double       cutoff_;
    double       q_;
    bool         dirty_ = true;
    BiquadCoeffs coeffs_;

    // Direct Form II Transposed state, per channel (z1, z2).
    struct State { float z1 = 0, z2 = 0; };
    std::vector<State> state_;
};

// ===========================================================================
//  DelayNode  (circular-buffer delay line)
// ===========================================================================
class DelayNode : public AudioNode {
public:
    explicit DelayNode(float delay_seconds = 0.25f,
                       float feedback = 0.3f,
                       float mix = 0.5f)
        : delay_seconds_(delay_seconds), feedback_(feedback), mix_(mix) {}

    void set_delay_seconds(float s) { delay_seconds_ = s; dirty_ = true; }
    void set_feedback(float f) noexcept { feedback_.store(f, std::memory_order_relaxed); }
    void set_mix(float m)      noexcept { mix_.store(m, std::memory_order_relaxed); }

    void prepare(SampleRate sample_rate, FrameCount max_frames) override;
    void reset() override;

    void process(const ProcessContext& ctx,
                 std::span<const AudioBuffer> inputs,
                 std::span<AudioBuffer> outputs) override;

    [[nodiscard]] std::size_t num_inputs()  const override { return 1; }
    [[nodiscard]] std::size_t num_outputs() const override { return 1; }
    [[nodiscard]] const char* type_name()   const override { return "DelayNode"; }

private:
    float              delay_seconds_;
    std::atomic<float> feedback_;
    std::atomic<float> mix_;
    bool               dirty_ = true;
    std::size_t        delay_samples_ = 0;

    // One circular buffer per channel.
    std::vector<std::vector<float>> lines_;
    std::vector<std::size_t>        write_pos_;
};

// ===========================================================================
//  EnvelopeNode  (ADSR generator applied to its input)
//
//  Gate control: note_on() starts Attack; note_off() starts Release. The
//  envelope multiplies the incoming signal, so it can shape an oscillator's
//  amplitude directly in the demo chain.
// ===========================================================================
class EnvelopeNode : public AudioNode {
public:
    struct Params {
        float attack  = 0.01f; ///< seconds
        float decay   = 0.10f; ///< seconds
        float sustain = 0.7f;  ///< level [0,1]
        float release = 0.20f; ///< seconds
    };

    EnvelopeNode() = default;
    explicit EnvelopeNode(Params p) : params_(p) {}

    void set_params(Params p) { params_ = p; recompute_rates(); }

    /// Gate on/off. Real-time safe (atomic flags read by process()).
    void note_on()  noexcept { gate_.store(true,  std::memory_order_relaxed); }
    void note_off() noexcept { gate_.store(false, std::memory_order_relaxed); }

    void prepare(SampleRate sample_rate, FrameCount max_frames) override;
    void reset() override;

    void process(const ProcessContext& ctx,
                 std::span<const AudioBuffer> inputs,
                 std::span<AudioBuffer> outputs) override;

    [[nodiscard]] std::size_t num_inputs()  const override { return 1; }
    [[nodiscard]] std::size_t num_outputs() const override { return 1; }
    [[nodiscard]] const char* type_name()   const override { return "EnvelopeNode"; }

private:
    enum class Stage : std::uint8_t { Idle, Attack, Decay, Sustain, Release };

    void recompute_rates();

    Params            params_;
    std::atomic<bool> gate_{false};
    bool              last_gate_ = false;
    Stage             stage_ = Stage::Idle;
    float             level_ = 0.0f;     // current envelope value [0,1]

    // Per-sample increments derived from times + sample rate.
    float attack_inc_  = 0.0f;
    float decay_inc_   = 0.0f;
    float release_inc_ = 0.0f;
};

} // namespace aurora::dsp

#endif // AURORA_DSP_BASIC_NODES_H
