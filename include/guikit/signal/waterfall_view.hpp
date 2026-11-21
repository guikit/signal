#pragma once

#include "guikit/core/response.hpp"
#include "guikit/core/theme.hpp"
#include "guikit/core/tokens.hpp"
#include "guikit/core/types.hpp"
#include "guikit/graphics/colormap.hpp"
#include "guikit/graphics/icon.hpp"
#include "keel/array.h"
#include "keel/core.h"
#include "keel/optional.h"
#include "keel/ref.h"
#include "keel/string.h"

namespace guikit {

class Context;
class ICanvas;

/// Domain operating mode for the WaterfallView analyzer & tracker.
enum class WaterfallMode : keel::u8 {
    SpectrumRtsa = 0,   ///< Real-Time Spectrum Analyzer (RTSA): Frequency (Hz/kHz/MHz) vs Scrolling Time (s)
    DoaBtr              ///< Bearing-Time Record (BTR) / Direction-Of-Arrival (DOA) Tracker: Azimuth (deg) vs Time (s)
};

/// Panel arrangement within the WaterfallView viewport.
enum class WaterfallLayout : keel::u8 {
    SplitTraceAndWaterfall = 0, ///< Upper instantaneous trace (Live + Peak Hold + Avg + Gate) + Lower scrolling Waterfall
    WaterfallOnly,              ///< Full-height scrolling Waterfall heatmap
    TraceOnly                   ///< Full-height instantaneous Spectrum / DOA Spatial Response trace plot
};

/// Tracking state of an automated DOA contact or frequency-agile emitter track.
enum class WaterfallTrackStatus : keel::u8 {
    Tracking = 0,       ///< Locked active target track with high SNR
    Coasting,           ///< Memory-predicted track during temporary signal fade
    Alarm               ///< Priority alert / sector intrusion / high-SNR emitter threat
};

/// Historical trajectory vertex for a tracked DOA target or drifting spectral emitter.
struct WaterfallTrackPoint {
    keel::f32 timeAgoSec{0.0f}; ///< Relative time in the past [0 .. historyDurationSec]
    keel::f32 xValue{0.0f};     ///< Bearing in degrees (DOA mode) or Frequency in Hz (Spectrum mode)
    keel::f32 snrDb{0.0f};      ///< Instantaneous SNR in dB at that historical timestamp
};

/// Tracked DOA target (BTR contact) or spectral emitter trajectory overlaid on the waterfall & top trace.
struct WaterfallTrack {
    keel::i32 id{0};
    keel::FixedString<64> callsign{};
    WaterfallTrackStatus status{WaterfallTrackStatus::Tracking};
    keel::f32 currentX{0.0f};               ///< Current Bearing (deg) or Frequency (Hz) at t = 0s
    keel::f32 driftRatePerSec{0.0f};        ///< Bearing drift rate d(theta)/dt (deg/s) or Doppler rate df/dt (Hz/s)
    keel::f32 levelDb{-24.0f};              ///< Peak signal level in dB
    keel::f32 snrDb{18.0f};                 ///< Signal-to-Noise Ratio above local noise floor in dB
    keel::f32 gateWidth{8.0f};              ///< Tracking gate / association corridor width in X units (deg or Hz)
    keel::Array<WaterfallTrackPoint> history{}; ///< Trajectory points ordered from newest (0s) to oldest

    [[nodiscard]] WaterfallTrack clone() const {
        WaterfallTrack out{};
        out.id = id;
        out.callsign = callsign;
        out.status = status;
        out.currentX = currentX;
        out.driftRatePerSec = driftRatePerSec;
        out.levelDb = levelDb;
        out.snrDb = snrDb;
        out.gateWidth = gateWidth;
        out.history = history.clone();
        return out;
    }
};

/// Configuration options for the autonomous WaterfallView (RTSA Spectrum & DOA/BTR Tracker) widget.
struct WaterfallViewOptions {
    // --- Domain Mode, Layout & Ring-Buffer Geometry ---
    WaterfallMode mode{WaterfallMode::SpectrumRtsa};
    WaterfallLayout viewLayout{WaterfallLayout::SplitTraceAndWaterfall};
    Colormap colormap{Colormap::Turbo};

    keel::u32 numBins{512};                          ///< Number of X bins per slice (frequency bins or azimuth beams)
    keel::u32 historyRows{300};                      ///< Circular ring-buffer capacity in rows (e.g. 180..1024)
    keel::f32 historyDurationSec{30.0f};                ///< Total historical time span in seconds corresponding to `historyRows`
    keel::f32 minX{0.0f};                               ///< Full-scale minimum X (e.g. 0 Hz or -180 deg)
    keel::f32 maxX{24000.0f};                           ///< Full-scale maximum X (e.g. 24000 Hz or +180 deg)
    keel::f32 topTraceHeightRatio{0.30f};               ///< Height ratio [0.18 .. 0.55] of upper trace panel in Split mode
    bool invertTimeAxis{false};                     ///< false = 0s at top flowing down; true = 0s at bottom flowing up

    // --- Streaming & Snapshot Data Ingestion ---
    keel::u64 sliceSequence{0};                      ///< Monotonic counter; when incremented, `latestSliceDb` is pushed as a new row
    keel::ArrayRef<keel::f32> initialHistoryMatrix{};   ///< Optional [historyRows * numBins] dB matrix (row 0 = newest) for bulk preload

    // --- DSP & Display Conditioning Parameters ---
    keel::f32 dbFloor{-110.0f};                         ///< Display dynamic range floor in dB (-140 .. -20 dB)
    keel::f32 dbCeil{0.0f};                             ///< Display dynamic range ceiling in dB (-60 .. +20 dB)
    keel::f32 gainDb{0.0f};                             ///< Digital display gain offset in dB (-30 .. +30 dB)
    keel::f32 thresholdGateDb{-62.0f};                  ///< Detection / squelch threshold level in dB
    keel::f32 gateSuppression{0.35f};                   ///< GPU noise-floor suppression below `thresholdGateDb` [0.0 .. 1.0]
    keel::f32 gamma{1.0f};                              ///< Colormap contrast gamma exponent [0.4 .. 2.2]
    keel::f32 averageAlpha{0.12f};                      ///< Exponential smoothing factor for the RMS/Average trace [0.01 .. 0.5]
    keel::f32 peakDecayDbPerSec{1.2f};                  ///< Peak-hold decay rate in dB/sec (0.0 = infinite max hold)

    // --- Traces, Target Tracks & Operator Instruments ---
    keel::ArrayRef<WaterfallTrack> tracks{};        ///< Active DOA contacts (BTR) or tracked spectral emitters
    bool showLiveTrace{true};                       ///< Render instantaneous live slice in upper trace panel
    bool showPeakHold{true};                        ///< Render peak-hold envelope trace in upper trace panel
    bool showAverageTrace{true};                    ///< Render exponentially averaged trace in upper trace panel
    bool showThresholdGate{true};                   ///< Render interactive detection threshold line in upper trace panel
    bool showPeakMarkers{true};                     ///< Automatically detect and label dominant peaks / DOA lobes
    bool showTracks{true};                          ///< Render DOA / emitter trajectory curves and vectors over waterfall
    bool showTrackGates{true};                      ///< Render tracking gate corridors and callsign tags
    bool showHarmonicComb{false};                   ///< Render fundamental f0 and harmonic comb markers (2f0..6f0)
    bool showColorbar{true};                        ///< Render right-hand dB intensity colorbar with threshold marker
    bool showHudOverlay{true};                      ///< Render corner operator telemetry HUD cards
    bool showGrid{true};                            ///< Render frequency/bearing and time graticule lines

    // --- Two-Way Interactive State Bindings ---
    keel::i32* selectedTrackId{nullptr};                  ///< Optional external ID of selected track (-1 = none)
    keel::f32* viewMinX{nullptr};                       ///< Optional external visible X start (wheel zoom / drag pan)
    keel::f32* viewMaxX{nullptr};                       ///< Optional external visible X end (wheel zoom / drag pan)
    keel::f32* selectionStartX{nullptr};                ///< Optional external band/sector selection start X
    keel::f32* selectionEndX{nullptr};                  ///< Optional external band/sector selection end X
    keel::f32* harmonicFundamentalX{nullptr};           ///< Optional external fundamental frequency f0 for harmonic comb
    keel::f32* thresholdGateDbPtr{nullptr};             ///< Optional external two-way binding for interactive threshold gate

    // --- Programmatic Actions ---
    bool requestResetZoom{false};
    bool requestResetPeakHold{false};
    bool requestClearHistory{false};

    // --- Sizing & Styling ---
    AxisSize width{};                     ///< Horizontal sizing policy (`Auto` -> `Stretch`, `Fit`, or `Fixed(dp)`)
    AxisSize height{SizeMode::Stretch};   ///< Vertical sizing policy (`Stretch` default, `Fit`, or `Fixed(dp)`)
    StateColor background{};
    Border border{};
    UiSize size{UiSize::Md};
    bool disabled{false};
    UiText tooltip{};
};

/// Rich telemetry & operator interaction state returned by `ui.WaterfallView(...)`.
struct WaterfallViewResponse : public Response {
    WaterfallMode mode{WaterfallMode::SpectrumRtsa};
    keel::f32 visibleMinX{0.0f};
    keel::f32 visibleMaxX{24000.0f};
    keel::f32 zoomFactor{1.0f};

    bool cursorInside{false};
    bool cursorInWaterfall{false};
    bool cursorInTrace{false};
    keel::f32 cursorX{0.0f};                ///< Hovered Frequency (Hz) or Bearing (deg)
    keel::f32 cursorTimeAgoSec{0.0f};       ///< Hovered historical timestamp (-dt in seconds)
    keel::f32 cursorWaterfallDb{-110.0f};   ///< Historical dB value at (cursorX, cursorTimeAgoSec)
    keel::f32 cursorLiveDb{-110.0f};        ///< Current live slice dB at cursorX
    keel::f32 cursorPeakDb{-110.0f};        ///< Peak-hold envelope dB at cursorX
    keel::f32 cursorAvgDb{-110.0f};         ///< Exponentially averaged dB at cursorX

    bool hasSelectionBand{false};
    keel::f32 selectionStartX{0.0f};
    keel::f32 selectionEndX{0.0f};
    keel::f32 selectionCenterX{0.0f};
    keel::f32 selectionSpanX{0.0f};
    keel::f32 selectionBandPowerDb{-110.0f}; ///< Integrated power / peak dB inside selected band or sector

    keel::i32 hoveredTrackId{-1};
    keel::i32 selectedTrackId{-1};
    bool trackSelectionChanged{false};

    keel::f32 dominantPeakX{0.0f};
    keel::f32 dominantPeakDb{-110.0f};
    keel::u32 populatedRows{0};
};

/// Autonomous Hardware-Accelerated Waterfall (RTSA Spectrum Analyzer & DOA / BTR Target Tracker) widget.
class WaterfallView {
public:
    static WaterfallViewResponse Run(
        Context& ctx,
        keel::ArrayRef<keel::f32> latestSliceDb,
        const WaterfallViewOptions& opts = {},
        keel::SourceLocation loc = keel::SourceLocation::current()
    );

    [[nodiscard]] static Vec2 Measure(
        keel::ArrayRef<keel::f32> latestSliceDb,
        const WaterfallViewOptions& opts,
        UiSize size,
        ICanvas* canvas = nullptr,
        const UiTheme& theme = {},
        keel::f32 availableWidth = 0.0f
    );
};

} // namespace guikit
