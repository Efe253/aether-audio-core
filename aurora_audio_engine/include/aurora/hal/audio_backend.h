// ============================================================================
//  aurora/hal/audio_backend.h
//
//  Backend factory: detects the platform and constructs an appropriate
//  AudioDevice implementation.
//
//  In this prototype only the NullBackend is fully implemented (it runs the
//  graph with no real hardware, ideal for tests and offline render). The real
//  platform backends are declared here and documented as future work so the
//  factory has a single, stable entry point.
// ============================================================================
#ifndef AURORA_HAL_AUDIO_BACKEND_H
#define AURORA_HAL_AUDIO_BACKEND_H

#include <memory>
#include <string_view>
#include <vector>

#include "aurora/hal/audio_device.h"

namespace aurora::hal {

/// Identifies a concrete audio backend implementation.
enum class BackendType {
    Null,       ///< silent backend, no hardware (always available)
    WASAPI,     ///< Windows (future work)
    CoreAudio,  ///< macOS / iOS (future work)
    ALSA,       ///< Linux, direct (future work)
    PipeWire,   ///< Linux, modern (future work)
    Automatic,  ///< pick the best available for the running platform
};

[[nodiscard]] constexpr std::string_view to_string(BackendType t) noexcept {
    switch (t) {
        case BackendType::Null:      return "Null";
        case BackendType::WASAPI:    return "WASAPI";
        case BackendType::CoreAudio: return "CoreAudio";
        case BackendType::ALSA:      return "ALSA";
        case BackendType::PipeWire:  return "PipeWire";
        case BackendType::Automatic: return "Automatic";
    }
    return "Unknown";
}

// ---------------------------------------------------------------------------
//  AudioBackendFactory
// ---------------------------------------------------------------------------
class AudioBackendFactory {
public:
    /// Which backend BackendType::Automatic resolves to on this platform.
    /// Detected at compile time from AURORA_PLATFORM_* macros; falls back to
    /// Null when no real backend is compiled in (as in this prototype).
    [[nodiscard]] static BackendType default_backend() noexcept {
#if defined(AURORA_PLATFORM_WINDOWS)
        // A production build would return BackendType::WASAPI here.
        return BackendType::Null;
#elif defined(AURORA_PLATFORM_MACOS)
        // A production build would return BackendType::CoreAudio here.
        return BackendType::Null;
#elif defined(AURORA_PLATFORM_LINUX)
        // A production build would prefer PipeWire, then ALSA.
        return BackendType::Null;
#else
        return BackendType::Null;
#endif
    }

    /// Create a device for the given backend. Returns nullptr if the requested
    /// backend is not available in this build.
    [[nodiscard]] static std::unique_ptr<AudioDevice> create(
        BackendType type = BackendType::Automatic);

    /// List backends compiled into this build.
    [[nodiscard]] static std::vector<BackendType> available_backends();
};

// Factory function implemented in src/hal/null_backend.cpp for the Null path.
[[nodiscard]] std::unique_ptr<AudioDevice> make_null_device();

} // namespace aurora::hal

#endif // AURORA_HAL_AUDIO_BACKEND_H
