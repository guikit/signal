#pragma once

// ==============================================================================
// GuiKit Signal Module - DSP Waveform, Spectrogram, Sonar/Radar & Waterfall Views
// ==============================================================================

#include "guikit/context.hpp"
#include "guikit/signal/signal_view.hpp"
#include "guikit/signal/sonar_view.hpp"
#include "guikit/signal/waterfall_view.hpp"

namespace guikit::signal {

/// Renders a synchronized Waveform + STFT Spectrogram analyzer widget.
inline SignalViewResponse SignalView(
    Context& ui,
    keel::ArrayRef<keel::f32> samples,
    const SignalViewOptions& opts = {},
    keel::SourceLocation loc = keel::SourceLocation::current()
) {
    return ui.Run<guikit::SignalView>(samples, opts, loc);
}

/// Renders a Polar Heatmap / Diver Detection Sonar / PPI 360 / Sector / B-Scan Radar widget.
inline SonarViewResponse SonarView(
    Context& ui,
    keel::ArrayRef<keel::f32> polarMatrix,
    const SonarViewOptions& opts = {},
    keel::SourceLocation loc = keel::SourceLocation::current()
) {
    return ui.Run<guikit::SonarView>(polarMatrix, opts, loc);
}

/// Renders an RTSA Spectrum Analyzer & DOA / BTR Waterfall tracker widget.
inline WaterfallViewResponse WaterfallView(
    Context& ui,
    keel::ArrayRef<keel::f32> latestSliceDb,
    const WaterfallViewOptions& opts = {},
    keel::SourceLocation loc = keel::SourceLocation::current()
) {
    return ui.Run<guikit::WaterfallView>(latestSliceDb, opts, loc);
}

} // namespace guikit::signal
