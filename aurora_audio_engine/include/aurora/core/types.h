// ============================================================================
//  aurora/core/types.h
//
//  Fundamental value types shared across the whole engine.
//
//  Design notes
//  ------------
//  * The internal sample format is 32-bit float, planar (structure-of-arrays):
//    each channel lives in its own contiguous buffer. This layout is friendly
//    to SIMD auto-vectorization and to cache locality for per-channel DSP
//    loops, and matches the architectural decision in the research report.
//  * Conversion to/from device-native (often interleaved) formats happens only
//    at the HAL boundary.
// ============================================================================
#ifndef AURORA_CORE_TYPES_H
#define AURORA_CORE_TYPES_H

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace aurora {

// ---------------------------------------------------------------------------
//  Scalar aliases
// ---------------------------------------------------------------------------

/// The engine's canonical audio sample type (32-bit float).
using Sample = float;

/// Sampling rate in Hz (e.g. 44100, 48000).
using SampleRate = std::uint32_t;

/// Number of audio channels (1 = mono, 2 = stereo, ...).
using ChannelCount = std::uint16_t;

/// Number of sample frames (a "frame" is one sample per channel).
using FrameCount = std::uint32_t;

// ---------------------------------------------------------------------------
//  AudioFormat
//
//  Describes how samples are laid out at an I/O boundary. Internally the
//  engine always works with Float32 / planar, but a device may want other
//  representations, so the HAL declares what it can consume/produce.
// ---------------------------------------------------------------------------
enum class SampleType : std::uint8_t {
    Int16,      ///< 16-bit signed PCM
    Int24,      ///< 24-bit signed PCM (packed in 3 bytes)
    Int32,      ///< 32-bit signed PCM
    Float32,    ///< 32-bit IEEE-754 float (engine native)
    Float64,    ///< 64-bit IEEE-754 float (offline/analysis paths)
};

enum class SampleLayout : std::uint8_t {
    Interleaved, ///< L R L R ... (array-of-structures)
    Planar,      ///< L L L ... R R R ... (structure-of-arrays, engine native)
};

/// A full description of an audio stream format.
struct AudioFormat {
    SampleRate   sample_rate   = 48000;
    ChannelCount channels      = 2;
    SampleType   sample_type   = SampleType::Float32;
    SampleLayout layout        = SampleLayout::Planar;

    /// Size (in bytes) of a single sample of `sample_type`.
    [[nodiscard]] constexpr std::size_t bytes_per_sample() const noexcept {
        switch (sample_type) {
            case SampleType::Int16:   return 2;
            case SampleType::Int24:   return 3;
            case SampleType::Int32:   return 4;
            case SampleType::Float32: return 4;
            case SampleType::Float64: return 8;
        }
        return 0;
    }
};

// ---------------------------------------------------------------------------
//  AudioBuffer
//
//  A non-owning, planar view over N channels of `Sample` data. Each channel is
//  a std::span<Sample> so DSP code can iterate without pointer arithmetic and
//  the compiler can reason about bounds for vectorization.
//
//  AudioBuffer never allocates; storage is owned elsewhere (memory pool,
//  graph-owned scratch buffers, device buffers). This keeps it real-time safe.
// ---------------------------------------------------------------------------
class AudioBuffer {
public:
    AudioBuffer() = default;

    /// Construct from an array of channel pointers.
    AudioBuffer(Sample* const* channel_ptrs,
                ChannelCount channels,
                FrameCount frames) noexcept
        : channels_(channels), frames_(frames) {
        for (ChannelCount c = 0; c < channels && c < kMaxChannels; ++c) {
            data_[c] = channel_ptrs[c];
        }
    }

    /// Maximum number of channels an AudioBuffer view can address.
    static constexpr ChannelCount kMaxChannels = 32;

    [[nodiscard]] ChannelCount channels() const noexcept { return channels_; }
    [[nodiscard]] FrameCount   frames()   const noexcept { return frames_; }

    /// Mutable span over a single channel.
    [[nodiscard]] std::span<Sample> channel(ChannelCount c) noexcept {
        return {data_[c], frames_};
    }

    /// Read-only span over a single channel.
    [[nodiscard]] std::span<const Sample> channel(ChannelCount c) const noexcept {
        return {data_[c], frames_};
    }

    /// Raw pointer to a channel (for interop / low-level loops).
    [[nodiscard]] Sample*       data(ChannelCount c)       noexcept { return data_[c]; }
    [[nodiscard]] const Sample* data(ChannelCount c) const noexcept { return data_[c]; }

    /// Zero-fill every channel. Real-time safe (no allocation).
    void clear() noexcept {
        for (ChannelCount c = 0; c < channels_; ++c) {
            Sample* p = data_[c];
            for (FrameCount i = 0; i < frames_; ++i) p[i] = Sample{0};
        }
    }

    /// Rebind the view to a different frame count (must not exceed capacity of
    /// the underlying storage — caller's responsibility).
    void set_frames(FrameCount frames) noexcept { frames_ = frames; }

private:
    Sample*      data_[kMaxChannels] = {};
    ChannelCount channels_ = 0;
    FrameCount   frames_   = 0;
};

// ---------------------------------------------------------------------------
//  OwnedAudioBuffer
//
//  A convenience owner for planar sample storage, used OUTSIDE the real-time
//  path (setup, offline render, tests). It hands out AudioBuffer views. Not
//  meant to be allocated on the audio thread.
// ---------------------------------------------------------------------------
class OwnedAudioBuffer {
public:
    OwnedAudioBuffer() = default;

    OwnedAudioBuffer(ChannelCount channels, FrameCount frames)
        : channels_(channels), frames_(frames),
          storage_(static_cast<std::size_t>(channels) * frames, Sample{0}) {
        rebuild_pointers();
    }

    void resize(ChannelCount channels, FrameCount frames) {
        channels_ = channels;
        frames_   = frames;
        storage_.assign(static_cast<std::size_t>(channels) * frames, Sample{0});
        rebuild_pointers();
    }

    [[nodiscard]] AudioBuffer view() noexcept {
        return AudioBuffer(ptrs_.data(), channels_, frames_);
    }

    [[nodiscard]] ChannelCount channels() const noexcept { return channels_; }
    [[nodiscard]] FrameCount   frames()   const noexcept { return frames_; }

private:
    void rebuild_pointers() {
        ptrs_.resize(channels_);
        for (ChannelCount c = 0; c < channels_; ++c) {
            ptrs_[c] = storage_.data() + static_cast<std::size_t>(c) * frames_;
        }
    }

    ChannelCount         channels_ = 0;
    FrameCount           frames_   = 0;
    std::vector<Sample>  storage_;
    std::vector<Sample*> ptrs_;
};

// ---------------------------------------------------------------------------
//  Handy math constants (constexpr, science-accurate)
// ---------------------------------------------------------------------------
namespace constants {
    inline constexpr double kPi     = 3.14159265358979323846;
    inline constexpr double kTwoPi  = 2.0 * kPi;
    inline constexpr double kSqrt2  = 1.41421356237309504880;
    inline constexpr double kLn10   = 2.30258509299404568402;
}

} // namespace aurora

#endif // AURORA_CORE_TYPES_H
