// ============================================================================
//  aurora/hal/audio_device.h
//
//  Platform-agnostic audio device interface (Hardware Abstraction Layer).
//
//  The HAL is the ONLY layer that knows about platform audio APIs (WASAPI on
//  Windows, CoreAudio on macOS, ALSA/PipeWire on Linux). Everything above it
//  works purely in terms of this abstract interface, so DSP and engine code are
//  fully portable.
//
//  Pull model
//  ----------
//  A device drives audio via a periodic *render callback*: the driver asks the
//  engine for the next block of samples. The callback runs on a high-priority
//  audio thread and is bound by the hard real-time contract (no allocation, no
//  locking, no I/O). See the research report, section 1.2.
// ============================================================================
#ifndef AURORA_HAL_AUDIO_DEVICE_H
#define AURORA_HAL_AUDIO_DEVICE_H

#include <functional>
#include <string>
#include <vector>

#include "aurora/core/types.h"

namespace aurora::hal {

// A tiny POD carrying timing info into the callback without pulling in the DSP
// headers here (keeps HAL independent of DSP). Mirrors dsp::ProcessContext.
struct ProcessContextBridge {
    SampleRate    sample_rate = 48000;
    FrameCount    frames      = 0;
    std::uint64_t stream_time = 0;
};

/// The render callback signature. The device passes a planar output buffer to
/// be filled with exactly `output.frames()` frames.
///
/// MUST be real-time safe.
using RenderCallback =
    std::function<void(AudioBuffer& output, const ProcessContextBridge&)>;

/// Description of a physical/logical audio endpoint.
struct DeviceInfo {
    std::string  name        = "Unknown";
    ChannelCount max_outputs = 2;
    ChannelCount max_inputs  = 0;
    SampleRate   default_rate = 48000;
    bool         is_default  = false;
};

// ---------------------------------------------------------------------------
//  AudioDevice (abstract)
//
//  Concrete backends (NullAudioDevice, and later WASAPI/CoreAudio/ALSA)
//  implement this interface.
// ---------------------------------------------------------------------------
class AudioDevice {
public:
    virtual ~AudioDevice() = default;

    /// Human-readable backend/device description.
    [[nodiscard]] virtual DeviceInfo info() const = 0;

    /// Negotiate a stream format. Backends may adjust the requested format to
    /// the nearest supported one; the granted format is written back to `fmt`.
    /// Returns false if no compatible format exists.
    virtual bool open(AudioFormat& fmt, FrameCount preferred_block) = 0;

    /// Install the render callback (control thread; before start()).
    virtual void set_render_callback(RenderCallback cb) = 0;

    /// Begin streaming. The device starts calling the render callback.
    virtual bool start() = 0;

    /// Stop streaming. After return, the callback is guaranteed not running.
    virtual void stop() = 0;

    [[nodiscard]] virtual bool is_running() const = 0;

    /// The granted stream format (valid after open()).
    [[nodiscard]] virtual AudioFormat format() const = 0;

    /// The negotiated block (callback) size in frames.
    [[nodiscard]] virtual FrameCount block_size() const = 0;
};

} // namespace aurora::hal

#endif // AURORA_HAL_AUDIO_DEVICE_H
