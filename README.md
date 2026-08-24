# Aurora Audio Engine

**Platformdan bağımsız, modüler ve bilime dayalı gerçek zamanlı ses çekirdeği (audio kernel) prototipi.**

Aurora, modern C++20 ile yazılmış, gerçek zamanlı güvenli (real-time safe) bir ses işleme çekirdeğidir. Amaç; donanım ve işletim sistemi farklılıklarını tek bir katmanda soyutlayıp (HAL), zamanlama ile yaşam döngüsünü çekirdekte toplayıp, sinyal işleme algoritmalarını taşınabilir düğümler (node) hâlinde sunmaktır.

> Bu depo bir **prototiptir**. Tam işlevsel gerçek zamanlı ses çıkışı yerine, geliştirme ve test için **NullBackend** (sessiz backend) içerir. Gerçek platform backend'leri (WASAPI / CoreAudio / ALSA / PipeWire) arayüz düzeyinde tanımlıdır ve ileri aşama çalışması olarak işaretlenmiştir.

---

## Öne çıkan özellikler

- **C++20**: `concepts`, `std::jthread`, `std::atomic`, `constexpr`, `std::span`, designated initializers.
- **Katmanlı mimari**: HAL → Core Engine → DSP Graph → High-Level API.
- **Lock-free tasarım**: Ses (render) thread'inde **mutex yok, bellek ayırma yok, sistem çağrısı yok**.
  - SPSC (tek üretici / tek tüketici) lock-free ring buffer, cache-line padding ile false sharing önlenmiş.
  - Gerçek zamanlı güvenli, önceden ayrılmış (pre-allocated) blok tabanlı memory pool.
- **Node tabanlı DAG audio graph**: topolojik sıralama, fan-in mixing, önceden derlenmiş çalıştırma sırası.
- **SIMD-ready DSP**: Tüm iç döngüler bitişik veri üzerinde basit index döngüleri olarak yazıldı; `-O3 -march=native` ile derleyici otomatik vektörizasyonuna (SSE/AVX/Neon) uygun.
- **Bilimsel doğruluk**:
  - dB ↔ lineer dönüşümlerde alan büyüklüğü için `20·log10`.
  - **Nyquist-uyumlu** (alias'sız), oktav başına mip-map'lenmiş band-limited wavetable üretimi (additif sentez).
  - **Robert Bristow-Johnson** ("Audio EQ Cookbook") biquad katsayı formülleri.

---

## Mimari

```
        +-------------------------------------------------------------+
        |                     High-Level API                          |
        |                    (AuroraEngine)                           |
        |   configure() / start() / stop() / render_offline()         |
        +-----------------------------+-------------------------------+
                                      |
                 render callback (real-time, lock-free)
                                      |
        +-----------------------------v-------------------------------+
        |                     DSP Graph (DAG)                         |
        |   AudioGraph: topolojik sıra + buffer yönetimi              |
        |                                                             |
        |   [Oscillator] -> [BiquadFilter] -> [Envelope] -> [Gain]    |
        |     (source)        (lowpass)         (ADSR)      (dB)      |
        +-----------------------------+-------------------------------+
                                      |
                       AudioBuffer (planar, float32)
                                      |
        +-----------------------------v-------------------------------+
        |                  Core (real-time safe)                      |
        |   SpscRingBuffer  |  MemoryPool  |  types (AudioBuffer)     |
        +-----------------------------+-------------------------------+
                                      |
                                pull (callback)
                                      |
        +-----------------------------v-------------------------------+
        |          HAL - Hardware Abstraction Layer                   |
        |   AudioDevice (abstract) + AudioBackendFactory              |
        |                                                             |
        |   NullBackend  |  WASAPI*  |  CoreAudio*  |  ALSA/PipeWire* |
        |   (bu derleme)      (ileri aşama - stub arayüz)            |
        +-------------------------------------------------------------+
              * bu prototipte yalnızca arayüz olarak tanımlıdır
```

### Katmanların sorumlulukları

| Katman | Sorumluluk | Thread |
|---|---|---|
| **HAL** | Cihaz açma/kapama, format pazarlığı, periyodik render callback'i sürme | Ses thread'i (backend'e ait) |
| **Core** | Temel tipler, lock-free ring buffer, memory pool, tampon yönetimi | Her ikisi |
| **DSP Graph** | Node yaşam döngüsü, topolojik sıralama, sinyal yönlendirme, process döngüsü | Ses thread'i |
| **High-Level API** | Grafiği kurma, yapılandırma, başlat/durdur, offline render | Kontrol thread'i |

### Gerçek zamanlı sözleşme (hard real-time contract)

Ses (render) thread'inde **yasaktır**: `new`/`delete`/`malloc`/`free`, dosya/ağ/konsol I/O, `printf`, `std::mutex`/condition variable, grafiği yerinde yeniden kurma, sınırsız döngü.

Tüm bellek `prepare()` içinde (kontrol thread'inde) önceden ayrılır; `process()` yalnızca hazır tamponlar üzerinde sınırlı sürede çalışır.

---

## Dizin yapısı

```
aurora_audio_engine/
├── CMakeLists.txt                 # C++20, platform tespiti, derleme ayarları
├── include/aurora/
│   ├── core/
│   │   ├── types.h                # AudioBuffer, SampleRate, AudioFormat, ...
│   │   ├── ring_buffer.h          # Lock-free SPSC ring buffer (cache-line padded)
│   │   └── memory_pool.h          # Real-time safe blok memory pool + TypedPool
│   ├── dsp/
│   │   ├── audio_node.h           # AudioNode base + C++20 concepts
│   │   ├── audio_graph.h          # DAG audio graph arayüzü
│   │   └── basic_nodes.h          # Gain/Mixer/Oscillator/Biquad/Delay/Envelope
│   └── hal/
│       ├── audio_device.h         # Platform-agnostic AudioDevice arayüzü
│       └── audio_backend.h        # Backend factory (platform tespiti)
├── src/
│   ├── core/
│   │   ├── engine.h               # AuroraEngine ana sınıf
│   │   └── engine.cpp
│   ├── dsp/
│   │   ├── audio_graph.cpp        # Topolojik sıralama + process cycle
│   │   └── basic_nodes.cpp        # Wavetable üretimi, biquad katsayıları, DSP
│   └── hal/
│       └── null_backend.cpp       # NullAudioBackend (std::jthread ile)
├── examples/
│   └── synth_demo.cpp             # Synth demo -> output.wav üretir
└── README.md
```

---

## Derleme

Gereksinimler: **CMake ≥ 3.20** ve **C++20 destekli bir derleyici** (GCC ≥ 11, Clang ≥ 13, MSVC ≥ 19.29).

```bash
cd aurora_audio_engine
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Release derlemesinde `-O3 -ffast-math -march=native` (destekleniyorsa) otomatik açılır; bu, DSP döngülerinin SIMD ile vektörize edilmesini sağlar.

---

## Demo'yu çalıştırma

```bash
./build/synth_demo
```

Beklenen çıktı:

```
Aurora Audio Engine - synth demo
Configured: 48000 Hz, 2 ch, block=512 frames, order=4 nodes
Real-time NullBackend ran for ~120 ms (silent output).
Wrote output.wav: 108000 frames (2.25 s), 48000 Hz, 2 ch
Demo complete.
```

Çalışma dizininde **`output.wav`** oluşur: `Oscillator (saw) → Lowpass Biquad → ADSR Envelope → Gain (-6 dB)` zincirinden üç notalık kısa bir dizi (16-bit PCM, 48 kHz, stereo).

Demo ayrıca **NullBackend**'i gerçek zamanlı olarak `std::jthread` üzerinde ~120 ms çalıştırarak aynı grafiğin canlı render yolunda da (donanım olmadan) çalıştığını gösterir.

---

## Kullanım örneği

```cpp
#include "core/engine.h"
#include "aurora/dsp/basic_nodes.h"

using namespace aurora;
using namespace aurora::dsp;

AuroraEngine engine;
auto& g = engine.graph();

// 1) Node'ları oluştur
auto osc  = std::make_unique<OscillatorNode>(Waveform::Saw, 220.0);
auto flt  = std::make_unique<BiquadFilterNode>(FilterType::Lowpass, 1500.0, 0.707);
auto gain = std::make_unique<GainNode>();
gain->set_gain_db(-6.0f);

// 2) Grafiğe ekle ve bağla
NodeId a = g.add_node(std::move(osc));
NodeId b = g.add_node(std::move(flt));
NodeId c = g.add_node(std::move(gain));
g.connect(a, 0, b, 0);
g.connect(b, 0, c, 0);
g.set_output_node(c);

// 3) Yapılandır ve çalıştır
engine.configure(EngineConfig{
    .sample_rate = 48000, .channels = 2,
    .block_size = 512, .backend = hal::BackendType::Null});
engine.start();
// ... engine.stop();

// veya deterministik offline render:
std::vector<Sample> pcm = engine.render_offline(48000); // 1 sn
```

---

## DSP node kütüphanesi

| Node | Açıklama | Bilimsel temel |
|---|---|---|
| **GainNode** | Ses seviyesi; dB ↔ lineer | `gain = 10^(dB/20)` |
| **MixerNode** | Çoklu girişi (opsiyonel kazançla) toplar | — |
| **OscillatorNode** | sine / saw / square / triangle | Band-limited additif wavetable, oktav mip-map, Nyquist-uyumlu |
| **BiquadFilterNode** | lowpass / highpass / bandpass / notch | Robert Bristow-Johnson formülleri, Direct Form II Transposed |
| **DelayNode** | Circular buffer delay line | feedback + wet/dry mix |
| **EnvelopeNode** | ADSR zarf üreteci (girişe uygular) | Örnek-hassas gate kenarları, lineer segmentler |

---

## Tasarım notları

- **İç örnek biçimi**: 32-bit float, **planar** (structure-of-arrays). SIMD ve önbellek erişimi için AoS'tan (interleaved) daha uygundur; interleaved dönüşüm yalnızca HAL sınırında yapılır.
- **Tek render thread'i**: DSP grafiği tek thread'de, önceden derlenmiş topolojik sıraya göre çalışır (JUCE `AudioProcessorGraph` modeline benzer). Çok çekirdekli scheduler bilinçli olarak ileri aşamaya bırakıldı.
- **Header-only ağırlıklı**: Template-heavy DSP ve lock-free container'lar başlık dosyalarında; yalnızca birkaç çeviri birimi derlenir.
- **Modüller yerine başlık tabanlı API**: C++20 modül desteğinin araç zincirleri arasındaki olgunluk farkı nedeniyle ilk sürüm başlık tabanlıdır (daha düşük risk).

## İleri aşama (roadmap)

- Gerçek platform backend'leri: WASAPI (`IAudioClient3` shared mode), CoreAudio, PipeWire + ALSA fallback.
- Bağımlılık tabanlı çok çekirdekli graph scheduler.
- Polyphase pencere-sinc örnekleme hızı dönüşümü (SRC) ve partitioned convolution.
- Çıkış nicelemesinde TPDF dither.
- FM / additive / granular sentez, HRTF / Ambisonics uzamsal render.
- Sürümlemeli C ABI ile eklenti sistemi.

---

## Lisans

Prototip / eğitim amaçlı örnek kod.
