#include "guikit/signal/waterfall_view.hpp"
#include <cstring>
#include "guikit/backend/i_canvas.hpp"
#include "guikit/context.hpp"
#include "guikit/core/control.hpp"
#include "guikit/core/node.hpp"
#include "guikit/graphics/colormap.hpp"
#include "keel/algo.h"
#include "keel/bits.h"
#include "keel/format.h"
#include "keel/math.h"
#include "keel/small_array.h"

namespace guikit {

namespace {

struct WaterfallViewState {
    keel::u32 numBins{0};
    keel::u32 historyRows{0};
    keel::u32 headRow{0};
    keel::u32 validRows{0};

    keel::Array<keel::f32> ringBufferDb{};
    keel::Array<keel::f32> liveSliceDb{};
    keel::Array<keel::f32> peakHoldDb{};
    keel::Array<keel::f32> averageDb{};
    keel::Array<WaterfallTrack> renderedTracks{};

    keel::u64 fullGeneration{1};
    keel::u64 sliceGeneration{1};
    keel::u64 lastSliceSequence{0};
    keel::f64 lastUpdateTimeSec{0.0};
    bool initialized{false};

    // Interactive viewport & operator state
    keel::f32 viewMinX{0.0f};
    keel::f32 viewMaxX{24000.0f};
    keel::f32 lastDomainMinX{0.0f};
    keel::f32 lastDomainMaxX{24000.0f};

    bool hasSelectionBand{false};
    keel::f32 selectionStartX{0.0f};
    keel::f32 selectionEndX{0.0f};
    bool draggingSelection{false};
    keel::f32 dragAnchorX{0.0f};

    bool draggingPan{false};
    keel::f32 panGrabMouseX{0.0f};
    keel::f32 panGrabMinX{0.0f};
    keel::f32 panGrabMaxX{0.0f};

    bool draggingThreshold{false};
    keel::f32 thresholdGateDb{-62.0f};
    keel::f32 harmonicFundamentalX{1200.0f};
    keel::i32 selectedTrackId{-1};

    // CPU fallback raster buffer (only allocated when GPU shaders are disabled)
    keel::Array<keel::u32> cpuFallbackPixels{};
    keel::u32 cpuFallbackW{0};
    keel::u32 cpuFallbackH{0};
    keel::u64 cpuFallbackKey{0};

    // Two-Layer Retained Base Cache (Pass A: Rulers, Grids, Colorbar, Transparent GPU Cutout)
    keel::u64 baseLayerKey{0};
    keel::u32 baseLayerW{0};
    keel::u32 baseLayerH{0};
    keel::Array<keel::u32> baseLayerPixels{};
    bool baseLayerValid{false};

    void TrimInactiveCaches() noexcept {
        cpuFallbackPixels = keel::Array<keel::u32>{};
        cpuFallbackW = 0;
        cpuFallbackH = 0;
        cpuFallbackKey = 0;
        baseLayerPixels = keel::Array<keel::u32>{};
        baseLayerValid = false;
        baseLayerKey = 0;
    }
};

struct WaterfallViewPayload {
    WaterfallViewOptions opts{};
    WaterfallViewResponse resp{};
    WaterfallViewState* state{nullptr};
};

struct WaterfallGeometry {
    Rect outerRect{};
    Rect traceRect{};
    Rect wfRect{};
    Rect xRulerRect{};
    Rect colorbarRect{};
    keel::f32 leftMargin{52.0f};
    keel::f32 rightMargin{52.0f};
    keel::f32 topHeaderH{0.0f};
    keel::f32 bottomRulerH{24.0f};
    keel::f32 splitGap{6.0f};

    keel::f32 domainMinX{0.0f};
    keel::f32 domainMaxX{24000.0f};
    keel::f32 viewMinX{0.0f};
    keel::f32 viewMaxX{24000.0f};
    keel::f32 dbFloor{-110.0f};
    keel::f32 dbCeil{0.0f};
    keel::f32 historyDurationSec{30.0f};
    bool invertTimeAxis{false};
    bool hasTrace{true};
    bool hasWaterfall{true};

    [[nodiscard]] keel::f32 XToScreen(keel::f32 xVal) const noexcept {
        const keel::f32 span = keel::max(1e-5f, viewMaxX - viewMinX);
        const keel::f32 u = (xVal - viewMinX) / span;
        const Rect& ref = hasWaterfall ? wfRect : traceRect;
        return ref.x + u * ref.w;
    }

    [[nodiscard]] keel::f32 ScreenToX(keel::f32 px) const noexcept {
        const Rect& ref = hasWaterfall ? wfRect : traceRect;
        const keel::f32 u = keel::clamp((px - ref.x) / keel::max(1.0f, ref.w), 0.0f, 1.0f);
        return viewMinX + u * (viewMaxX - viewMinX);
    }

    [[nodiscard]] keel::f32 DbToTraceY(keel::f32 db) const noexcept {
        const keel::f32 span = keel::max(1.0f, dbCeil - dbFloor);
        const keel::f32 norm = keel::clamp((db - dbFloor) / span, 0.0f, 1.0f);
        return traceRect.Bottom() - norm * traceRect.h;
    }

    [[nodiscard]] keel::f32 TraceYToDb(keel::f32 py) const noexcept {
        const keel::f32 norm = keel::clamp((traceRect.Bottom() - py) / keel::max(1.0f, traceRect.h), 0.0f, 1.0f);
        return dbFloor + norm * (dbCeil - dbFloor);
    }

    [[nodiscard]] keel::f32 TimeAgoToWaterfallY(keel::f32 timeAgoSec) const noexcept {
        const keel::f32 norm = keel::clamp(timeAgoSec / keel::max(0.1f, historyDurationSec), 0.0f, 1.0f);
        const keel::f32 v = invertTimeAxis ? (1.0f - norm) : norm;
        return wfRect.y + v * wfRect.h;
    }

    [[nodiscard]] keel::f32 WaterfallYToTimeAgo(keel::f32 py) const noexcept {
        const keel::f32 v = keel::clamp((py - wfRect.y) / keel::max(1.0f, wfRect.h), 0.0f, 1.0f);
        const keel::f32 norm = invertTimeAxis ? (1.0f - v) : v;
        return norm * historyDurationSec;
    }
};

[[nodiscard]] WaterfallGeometry ComputeWaterfallGeometry(
    const Rect& bounds,
    const WaterfallViewOptions& opts,
    keel::f32 viewMinX,
    keel::f32 viewMaxX
) noexcept {
    WaterfallGeometry g{};
    g.outerRect = bounds;
    g.leftMargin = 54.0f;
    g.rightMargin = opts.showColorbar ? 52.0f : 12.0f;
    g.bottomRulerH = 24.0f;
    g.topHeaderH = 8.0f;
    g.splitGap = 6.0f;

    g.domainMinX = opts.minX;
    g.domainMaxX = keel::max(opts.minX + 1e-3f, opts.maxX);
    g.viewMinX = viewMinX;
    g.viewMaxX = keel::max(viewMinX + 1e-3f, viewMaxX);
    g.dbFloor = opts.dbFloor;
    g.dbCeil = keel::max(opts.dbFloor + 5.0f, opts.dbCeil);
    g.historyDurationSec = keel::max(1.0f, opts.historyDurationSec);
    g.invertTimeAxis = opts.invertTimeAxis;

    const keel::f32 plotX = bounds.x + g.leftMargin;
    const keel::f32 plotW = keel::max(40.0f, bounds.w - g.leftMargin - g.rightMargin);
    const keel::f32 availY = bounds.y + g.topHeaderH;
    const keel::f32 availH = keel::max(40.0f, bounds.h - g.topHeaderH - g.bottomRulerH);

    if (opts.viewLayout == WaterfallLayout::SplitTraceAndWaterfall) {
        g.hasTrace = true;
        g.hasWaterfall = true;
        const keel::f32 ratio = keel::clamp(opts.topTraceHeightRatio, 0.18f, 0.55f);
        const keel::f32 traceH = keel::max(54.0f, keel::floor((availH - g.splitGap) * ratio));
        const keel::f32 wfH = keel::max(60.0f, availH - traceH - g.splitGap);
        g.traceRect = Rect(plotX, availY, plotW, traceH);
        g.wfRect = Rect(plotX, availY + traceH + g.splitGap, plotW, wfH);
    } else if (opts.viewLayout == WaterfallLayout::WaterfallOnly) {
        g.hasTrace = false;
        g.hasWaterfall = true;
        g.traceRect = Rect(plotX, availY, plotW, 0.0f);
        g.wfRect = Rect(plotX, availY, plotW, availH);
    } else {
        g.hasTrace = true;
        g.hasWaterfall = false;
        g.traceRect = Rect(plotX, availY, plotW, availH);
        g.wfRect = Rect(plotX, availY + availH, plotW, 0.0f);
    }

    const keel::f32 bottomY = g.hasWaterfall ? g.wfRect.Bottom() : g.traceRect.Bottom();
    g.xRulerRect = Rect(plotX, bottomY, plotW, g.bottomRulerH);
    g.colorbarRect = opts.showColorbar
        ? Rect(plotX + plotW + 4.0f, availY, keel::max(0.0f, g.rightMargin - 8.0f), availH)
        : Rect(plotX + plotW, availY, 0.0f, 0.0f);
    return g;
}

[[nodiscard]] keel::FixedString<32> FormatXValue(keel::f32 xVal, WaterfallMode mode, keel::f32 visibleSpan) noexcept {
    if (mode == WaterfallMode::DoaBtr) {
        if (visibleSpan < 25.0f) {
            return keel::format_fixed<32>("%+06.1f°", xVal);
        }
        return keel::format_fixed<32>("%+04.0f°", xVal);
    }

    const keel::f32 absX = keel::abs(xVal);
    if (absX >= 1000000.0f) {
        return keel::format_fixed<32>("%.3f MHz", xVal / 1000000.0f);
    }
    if (absX >= 1000.0f || visibleSpan >= 2000.0f) {
        if (visibleSpan < 1500.0f) {
            return keel::format_fixed<32>("%.2f kHz", xVal / 1000.0f);
        }
        return keel::format_fixed<32>("%.1f kHz", xVal / 1000.0f);
    }
    return keel::format_fixed<32>("%.0f Hz", xVal);
}

[[nodiscard]] keel::f32 SampleRingBufferDb(
    const WaterfallViewState& state,
    const WaterfallViewOptions& opts,
    keel::f32 xVal,
    keel::f32 timeAgoSec
) noexcept {
    if (state.ringBufferDb.empty() || state.numBins == 0 || state.historyRows == 0 || state.validRows == 0) {
        return opts.dbFloor;
    }
    const keel::f32 domainSpan = keel::max(1e-5f, opts.maxX - opts.minX);
    const keel::f32 uNorm = keel::clamp((xVal - opts.minX) / domainSpan, 0.0f, 1.0f);
    const keel::u32 binIdx = keel::min(
        state.numBins - 1,
        keel::round_u32(uNorm * keel::as<keel::f32>(state.numBins - 1))
    );

    const keel::f32 ageNorm = keel::clamp(timeAgoSec / keel::max(0.1f, opts.historyDurationSec), 0.0f, 1.0f);
    const keel::u32 ageRows = keel::min(
        state.validRows - 1,
        keel::round_u32(ageNorm * keel::as<keel::f32>(state.historyRows - 1))
    );
    const keel::u32 rowIdx = (state.headRow + state.historyRows - (ageRows % state.historyRows)) % state.historyRows;

    return state.ringBufferDb[keel::as<keel::usize>(rowIdx) * state.numBins + binIdx] + opts.gainDb;
}

[[nodiscard]] keel::f32 SampleTraceVectorDb(
    keel::ArrayRef<keel::f32> vec,
    const WaterfallViewOptions& opts,
    keel::f32 xVal
) noexcept {
    if (vec.empty()) return opts.dbFloor;
    const keel::f32 domainSpan = keel::max(1e-5f, opts.maxX - opts.minX);
    const keel::f32 uNorm = keel::clamp((xVal - opts.minX) / domainSpan, 0.0f, 1.0f);
    const keel::f32 fIdx = uNorm * keel::as<keel::f32>(vec.size() - 1);
    const keel::usize i0 = keel::as<keel::usize>(keel::floor(fIdx));
    const keel::usize i1 = keel::min(vec.size() - 1, i0 + 1);
    const keel::f32 frac = fIdx - keel::as<keel::f32>(i0);
    return (vec[i0] + frac * (vec[i1] - vec[i0])) + opts.gainDb;
}

void DrawWaterfallViewNode(const Node* node, ICanvas& canvas, const UiTheme& theme) {
    const auto* payload = node->As<WaterfallViewPayload>();
    if (!payload || !payload->state) return;

    const WaterfallViewOptions& opts = payload->opts;
    const WaterfallViewResponse& resp = payload->resp;
    WaterfallViewState& state = *payload->state;

    const Rect r = node->computedRect;
    if (r.w <= 20.0f || r.h <= 20.0f) return;

    const WaterfallGeometry geom = ComputeWaterfallGeometry(r, opts, resp.visibleMinX, resp.visibleMaxX);
    const keel::f32 radius = opts.border.radius.has_value()
        ? *opts.border.radius
        : theme.GetRadius(node->uiSize);

    const Color bgOuter = opts.background.Resolve(node->hovered, node->pressed, node->disabled, Color{7, 10, 19, 255});
    const Color bgPanel{5, 8, 16, 255};
    const Color borderCol = opts.border.color.Resolve(node->hovered, node->pressed, node->disabled, Color{30, 41, 59, 255});
    const keel::f32 borderW = (opts.border.width > 0.0f) ? opts.border.width : 1.2f;
    const Color gridLineCol{56, 189, 248, 34};
    const Color gridMajorCol{56, 189, 248, 72};
    const Color labelTextCol{148, 163, 184, 225};
    const Color accentTextCol{56, 189, 248, 245};

    const auto packFloat = [](keel::f32 v) noexcept -> keel::u64 {
        return keel::as<keel::u64>(keel::bit_cast<keel::u32>(v));
    };

    keel::u64 staticKey = 14695981039346656037ULL;
    auto mixKey = [&](keel::u64 v) noexcept {
        staticKey ^= v;
        staticKey *= 1099511628211ULL;
    };
    mixKey(packFloat(r.x));
    mixKey(packFloat(r.y));
    mixKey(packFloat(r.w));
    mixKey(packFloat(r.h));
    mixKey(packFloat(canvas.GetScale()));
    mixKey(keel::as<keel::u64>(opts.mode));
    mixKey(keel::as<keel::u64>(opts.viewLayout));
    mixKey(keel::as<keel::u64>(opts.colormap));
    mixKey(packFloat(geom.viewMinX));
    mixKey(packFloat(geom.viewMaxX));
    mixKey(packFloat(geom.dbFloor));
    mixKey(packFloat(geom.dbCeil));
    mixKey(packFloat(geom.historyDurationSec));
    mixKey(packFloat(opts.topTraceHeightRatio));
    mixKey(opts.invertTimeAxis ? 1ULL : 0ULL);
    mixKey(opts.showColorbar ? 1ULL : 0ULL);
    mixKey(opts.showGrid ? 1ULL : 0ULL);
    mixKey(keel::as<keel::u64>(theme.mode));
    mixKey(bgOuter.ToUint32());
    mixKey(borderCol.ToUint32());
    mixKey(packFloat(borderW));
    mixKey(packFloat(radius));

    const keel::f32 domainSpan = keel::max(1e-5f, geom.domainMaxX - geom.domainMinX);
    const keel::f32 xMinNorm = keel::clamp((geom.viewMinX - geom.domainMinX) / domainSpan, 0.0f, 1.0f);
    const keel::f32 xMaxNorm = keel::clamp((geom.viewMaxX - geom.domainMinX) / domainSpan, 0.0f, 1.0f);

    static constexpr keel::StringRef kWaterfallFragGlsl = R"(
#ifdef GL_ES
precision highp float;
#endif
varying vec2 v_uv;

uniform vec2 u_GridDims;    // x=texWidth (bins), y=texHeight (historyRows)
uniform vec3 u_BgColor;
uniform vec4 u_Params[8];
// u_Params[0]: x=dbFloor, y=dbCeil, z=xMinNorm, w=xMaxNorm
// u_Params[1]: x=gainDb, y=thresholdGateDb, z=gateSuppression, w=gamma
// u_Params[2]: x=headRow, y=validRows, z=invertTimeAxis, w=unused
uniform sampler2D u_DataTex;
uniform sampler2D u_LutTex;

float decodeRG16(vec4 texel) {
    return (texel.r * 65280.0 + texel.g * 255.0) / 65535.0;
}

float sampleRingDbTexel(float xIdx, float rowIdx) {
    vec2 uv = vec2((xIdx + 0.5) / u_GridDims.x, (rowIdx + 0.5) / u_GridDims.y);
    return -140.0 + decodeRG16(texture2D(u_DataTex, uv)) * 160.0;
}

void main() {
    vec4 dbAndView = u_Params[0];
    vec4 dspParams = u_Params[1];
    vec4 ringParams = u_Params[2];

    float timeUv = (ringParams.z > 0.5) ? (1.0 - v_uv.y) : v_uv.y;
    float ageFloat = clamp(timeUv * max(1.0, u_GridDims.y - 1.0), 0.0, max(0.0, u_GridDims.y - 1.0));
    if (ageFloat > (ringParams.y - 0.5)) {
        gl_FragColor = vec4(u_BgColor, 1.0);
        return;
    }

    float uNorm = mix(dbAndView.z, dbAndView.w, v_uv.x);
    float xFloat = clamp(uNorm * u_GridDims.x - 0.5, 0.0, max(0.0, u_GridDims.x - 1.0));

    float x0 = floor(xFloat);
    float x1 = min(u_GridDims.x - 1.0, x0 + 1.0);
    float fx = xFloat - x0;

    float age0 = floor(ageFloat);
    float age1 = min(max(0.0, ringParams.y - 1.0), age0 + 1.0);
    float fy = ageFloat - age0;

    float row0 = mod(ringParams.x - age0 + u_GridDims.y * 4.0, u_GridDims.y);
    float row1 = mod(ringParams.x - age1 + u_GridDims.y * 4.0, u_GridDims.y);

    float v00 = sampleRingDbTexel(x0, row0);
    float v10 = sampleRingDbTexel(x1, row0);
    float v01 = sampleRingDbTexel(x0, row1);
    float v11 = sampleRingDbTexel(x1, row1);

    float v0 = v00 + fx * (v10 - v00);
    float v1 = v01 + fx * (v11 - v01);
    float dbVal = (v0 + fy * (v1 - v0)) + dspParams.x;

    float dbFloor = dbAndView.x;
    float dbCeil = dbAndView.y;
    float norm = clamp((dbVal - dbFloor) / max(1.0, dbCeil - dbFloor), 0.0, 1.0);

    float threshDb = dspParams.y;
    float gateSupp = clamp(dspParams.z, 0.0, 1.0);
    if (gateSupp > 0.001 && dbVal < threshDb) {
        float soft = clamp((dbVal - (threshDb - 10.0)) / 10.0, 0.0, 1.0);
        norm *= mix(1.0, soft * 0.25, gateSupp);
    }

    float gamma = max(0.25, dspParams.w);
    norm = pow(clamp(norm, 0.0, 1.0), gamma);

    vec4 mapped = texture2D(u_LutTex, vec2((norm * 255.0 + 0.5) / 256.0, 0.5));
    gl_FragColor = vec4(mapped.rgb, 1.0);
}
)";

    auto buildGpuDesc = [&]() noexcept -> ICanvas::GpuShaderPassDesc {
        ICanvas::GpuShaderPassDesc desc{};
        desc.widgetId = keel::as<keel::u64>(node->id.value);
        desc.shader = {
            .shaderId = "guikit.waterfall.v1",
            .glslEs2 = kWaterfallFragGlsl
        };
        desc.plotRect = geom.wfRect;
        desc.clipRect = geom.wfRect;
        desc.bgColor = bgPanel;
        desc.dataField = {state.ringBufferDb, state.historyRows, state.numBins};
        desc.fullGeneration = state.fullGeneration;
        desc.sliceGeneration = state.sliceGeneration;
        desc.headRow = state.headRow;
        desc.dataNormOffset = -140.0f;
        desc.dataNormScale = 160.0f;
        desc.colormapId = keel::as<keel::u8>(opts.colormap);
        desc.lutAbgr256 = GetColormapLutAbgr(opts.colormap);

        desc.params[0] = {geom.dbFloor, geom.dbCeil, xMinNorm, xMaxNorm};
        desc.params[1] = {opts.gainDb, state.thresholdGateDb, opts.gateSuppression, opts.gamma};
        desc.params[2] = {
            keel::as<keel::f32>(state.headRow % keel::max(1u, state.historyRows)),
            keel::as<keel::f32>(keel::min(state.validRows, state.historyRows)),
            opts.invertTimeAxis ? 1.0f : 0.0f,
            0.0f
        };
        return desc;
    };

    bool restoredBase = false;
    if (canvas.SupportsGpuShaders() && state.baseLayerValid && state.baseLayerKey == staticKey) {
        if (canvas.RestoreRegion(r, {state.baseLayerPixels.as_ref(), state.baseLayerW, state.baseLayerH})) {
            if (geom.hasWaterfall && !state.ringBufferDb.empty()) {
                canvas.PushClip(geom.wfRect);
                const auto gpuDesc = buildGpuDesc();
                (void)canvas.DrawGpuShaderPass(gpuDesc, false);
                canvas.PopClip();
            }
            restoredBase = true;
        }
    }

    if (!restoredBase) {
        state.baseLayerValid = false;
        // =================================================================
        // PASS A: Static Instrument Base Layer (Rulers, Grids, Colorbar, Cutout)
        // =================================================================
        canvas.DrawRoundRect(r, radius, bgOuter, borderCol, borderW);

        // 1. Upper Trace Panel Static Background & Graticule
        if (geom.hasTrace && geom.traceRect.h > 10.0f) {
            canvas.DrawRect(geom.traceRect, bgPanel);
            canvas.DrawRoundRect(geom.traceRect, 0.0f, Color::Transparent(), borderCol, 1.0f);

            if (opts.showGrid) {
                constexpr keel::i32 kDbRows = 4;
                for (keel::i32 i = 0; i <= kDbRows; ++i) {
                    const keel::f32 frac = keel::as<keel::f32>(i) / keel::as<keel::f32>(kDbRows);
                    const keel::f32 y = geom.traceRect.y + frac * geom.traceRect.h;
                    const keel::f32 dbVal = geom.dbCeil - frac * (geom.dbCeil - geom.dbFloor);
                    if (i > 0 && i < kDbRows) {
                        canvas.DrawLine({geom.traceRect.x, y}, {geom.traceRect.Right(), y}, gridLineCol, 1.0f);
                    }
                    char dbBuf[24];
                    const keel::StringRef dbStr = keel::format_buf(dbBuf, "%+.0fdB", dbVal);
                    canvas.DrawText(dbStr, {r.x + 6.0f, y - 5.5f}, 9.5f, labelTextCol);
                }

                constexpr keel::i32 kXCols = 8;
                for (keel::i32 c = 1; c < kXCols; ++c) {
                    const keel::f32 u = keel::as<keel::f32>(c) / keel::as<keel::f32>(kXCols);
                    const keel::f32 x = geom.traceRect.x + u * geom.traceRect.w;
                    const bool isCenter = (c == kXCols / 2);
                    canvas.DrawLine({x, geom.traceRect.y}, {x, geom.traceRect.Bottom()},
                                    isCenter ? gridMajorCol : gridLineCol, isCenter ? 1.2f : 1.0f);
                }
            }
        }

        // 2. Lower Waterfall Panel GPU Underlay Cutout & Graticule
        if (geom.hasWaterfall && geom.wfRect.h > 10.0f) {
            bool gpuHandled = false;
            if (canvas.SupportsGpuShaders() && !state.ringBufferDb.empty()) {
                canvas.PushClip(geom.wfRect);
                const auto gpuDesc = buildGpuDesc();
                gpuHandled = canvas.DrawGpuShaderPass(gpuDesc, true);
                canvas.PopClip();
            }

            if (!gpuHandled) {
                canvas.DrawRect(geom.wfRect, bgPanel);
                // CPU Software Fallback when GPU switch is disabled
                if (!state.ringBufferDb.empty() && state.numBins > 0 && state.historyRows > 0) {
                    const auto& lutAbgr = GetColormapLutAbgr(opts.colormap);
                    const keel::u32 fbW = keel::clamp(keel::as<keel::u32>(geom.wfRect.w * 0.5f), 64u, 320u);
                    const keel::u32 fbH = keel::clamp(keel::as<keel::u32>(geom.wfRect.h * 0.5f), 48u, 200u);
                    keel::usize pixelCount = 0;
                    if (keel::checked_mul(keel::as<keel::usize>(fbW), keel::as<keel::usize>(fbH), pixelCount)) {
                        state.cpuFallbackPixels.resize(pixelCount, 0u);
                        const keel::f32 dbSpan = keel::max(1.0f, geom.dbCeil - geom.dbFloor);
                        const keel::f32 gamma = keel::max(0.25f, opts.gamma);
                        const keel::u32 bgPanelAbgr = PackAbgr(bgPanel);
                        for (keel::u32 py = 0; py < fbH; ++py) {
                            const keel::f32 v = keel::as<keel::f32>(py) / keel::as<keel::f32>(keel::max(1u, fbH - 1));
                            const keel::f32 timeUv = opts.invertTimeAxis ? (1.0f - v) : v;
                            const keel::u32 ageRow = keel::round_u32(timeUv * keel::as<keel::f32>(state.historyRows - 1));
                            if (ageRow >= state.validRows) {
                                keel::fill(
                                    keel::MutArrayRef<keel::u32>(state.cpuFallbackPixels.data() + keel::as<keel::usize>(py) * fbW, fbW),
                                    bgPanelAbgr
                                );
                                continue;
                            }
                            const keel::u32 ringRow = (state.headRow + state.historyRows - (ageRow % state.historyRows)) % state.historyRows;
                            const keel::f32* rowPtr = state.ringBufferDb.data() + keel::as<keel::usize>(ringRow) * state.numBins;
                            keel::u32* dstRow = state.cpuFallbackPixels.data() + keel::as<keel::usize>(py) * fbW;
                            for (keel::u32 px = 0; px < fbW; ++px) {
                                const keel::f32 u = xMinNorm + (keel::as<keel::f32>(px) / keel::as<keel::f32>(keel::max(1u, fbW - 1))) * (xMaxNorm - xMinNorm);
                                const keel::u32 bin = keel::min(
                                    state.numBins - 1,
                                    keel::round_u32(u * keel::as<keel::f32>(state.numBins - 1))
                                );
                                const keel::f32 dbVal = rowPtr[bin] + opts.gainDb;
                                keel::f32 norm = keel::clamp((dbVal - geom.dbFloor) / dbSpan, 0.0f, 1.0f);
                                if (opts.gateSuppression > 0.001f && dbVal < state.thresholdGateDb) {
                                    const keel::f32 soft = keel::clamp((dbVal - (state.thresholdGateDb - 10.0f)) / 10.0f, 0.0f, 1.0f);
                                    norm *= (1.0f - opts.gateSuppression) + opts.gateSuppression * (soft * 0.25f);
                                }
                                norm = keel::pow(norm, gamma);
                                const keel::usize lutIdx = keel::as<keel::usize>(keel::clamp(norm * 255.0f, 0.0f, 255.0f));
                                dstRow[px] = lutAbgr[lutIdx];
                            }
                        }
                        canvas.DrawPixels({state.cpuFallbackPixels, fbW, fbH}, geom.wfRect);
                    }
                }
            }

            canvas.DrawRoundRect(geom.wfRect, 0.0f, Color::Transparent(), borderCol, 1.0f);

            // Time-axis Y labels & waterfall graticule
            constexpr keel::i32 kTimeTicks = 6;
            for (keel::i32 t = 0; t <= kTimeTicks; ++t) {
                const keel::f32 frac = keel::as<keel::f32>(t) / keel::as<keel::f32>(kTimeTicks);
                const keel::f32 timeAgo = frac * geom.historyDurationSec;
                const keel::f32 y = geom.TimeAgoToWaterfallY(timeAgo);
                if (opts.showGrid && t > 0 && t < kTimeTicks) {
                    canvas.DrawLine({geom.wfRect.x, y}, {geom.wfRect.Right(), y}, gridLineCol, 1.0f);
                }
                char tBuf[24];
                const keel::StringRef tStr = (t == 0)
                    ? keel::StringRef("0.0s")
                    : keel::format_buf(tBuf, "-%.0fs", timeAgo);
                canvas.DrawText(tStr, {r.x + 8.0f, y - 5.5f}, 9.5f, (t == 0) ? accentTextCol : labelTextCol);
            }

            if (opts.showGrid) {
                constexpr keel::i32 kXCols = 8;
                for (keel::i32 c = 1; c < kXCols; ++c) {
                    const keel::f32 u = keel::as<keel::f32>(c) / keel::as<keel::f32>(kXCols);
                    const keel::f32 x = geom.wfRect.x + u * geom.wfRect.w;
                    const bool isCenter = (c == kXCols / 2);
                    canvas.DrawLine({x, geom.wfRect.y}, {x, geom.wfRect.Bottom()},
                                    isCenter ? gridMajorCol : gridLineCol, isCenter ? 1.2f : 1.0f);
                }
            }
        }

        // 3. Bottom X-Axis Ruler (Frequency Hz/kHz/MHz or Azimuth Bearing deg)
        {
            const keel::f32 visSpan = geom.viewMaxX - geom.viewMinX;
            constexpr keel::i32 kXCols = 8;
            for (keel::i32 c = 0; c <= kXCols; ++c) {
                const keel::f32 u = keel::as<keel::f32>(c) / keel::as<keel::f32>(kXCols);
                const keel::f32 x = geom.xRulerRect.x + u * geom.xRulerRect.w;
                const keel::f32 xVal = geom.viewMinX + u * visSpan;
                canvas.DrawLine({x, geom.xRulerRect.y}, {x, geom.xRulerRect.y + 4.5f}, labelTextCol, 1.1f);

                const auto xBuf = FormatXValue(xVal, opts.mode, visSpan);
                const keel::f32 textOff = (c == 0) ? 0.0f : ((c == kXCols) ? -42.0f : -20.0f);
                canvas.DrawText(xBuf, {x + textOff, geom.xRulerRect.y + 6.0f}, 10.0f,
                                (c == kXCols / 2) ? accentTextCol : labelTextCol);
            }
        }

        // 4. Right-Side dB Intensity Colorbar
        if (opts.showColorbar && geom.colorbarRect.w >= 18.0f) {
            canvas.PushClip(geom.colorbarRect);
            canvas.DrawRect(geom.colorbarRect, Color(9, 13, 24, 255));
            canvas.DrawLine({geom.colorbarRect.x, geom.colorbarRect.y},
                            {geom.colorbarRect.x, geom.colorbarRect.Bottom()}, borderCol, 1.0f);

            const Rect barStrip(geom.colorbarRect.x + 7.0f, geom.colorbarRect.y + 20.0f,
                                11.0f, keel::max(24.0f, geom.colorbarRect.h - 32.0f));
            DrawColormapBar(canvas, barStrip, opts.colormap, true, false, 2.0f, Color(148, 163, 184, 120));

            canvas.DrawText("dB", {geom.colorbarRect.x + 7.0f, geom.colorbarRect.y + 4.0f}, 9.5f, accentTextCol);
            for (keel::i32 i = 0; i <= 4; ++i) {
                const keel::f32 frac = keel::as<keel::f32>(i) / 4.0f;
                const keel::f32 y = barStrip.y + frac * barStrip.h;
                const keel::f32 dbVal = geom.dbCeil - frac * (geom.dbCeil - geom.dbFloor);
                char dbBuf[16];
                const keel::StringRef dbStr = keel::format_buf(dbBuf, "%.0f", dbVal);
                canvas.DrawText(dbStr, {barStrip.Right() + 3.0f, y - 5.0f}, 9.0f, labelTextCol);
            }
            canvas.PopClip();
        }

        if (canvas.SupportsGpuShaders()) {
            if (canvas.CaptureRegion(r, state.baseLayerPixels, state.baseLayerW, state.baseLayerH)) {
                state.baseLayerKey = staticKey;
                state.baseLayerValid = true;
            }
        }
    }

    // =====================================================================
    // PASS B: Dynamic Tactical & Spectral Overlay (Traces, Tracks, Cursors, HUD)
    // =====================================================================

    // Dynamic Threshold Marker on Colorbar
    if (opts.showColorbar && geom.colorbarRect.w >= 18.0f) {
        const Rect barStrip(geom.colorbarRect.x + 7.0f, geom.colorbarRect.y + 20.0f,
                            11.0f, keel::max(24.0f, geom.colorbarRect.h - 32.0f));
        const keel::f32 gateNorm = keel::clamp((state.thresholdGateDb - geom.dbFloor) / keel::max(1.0f, geom.dbCeil - geom.dbFloor), 0.0f, 1.0f);
        const keel::f32 gateY = barStrip.Bottom() - gateNorm * barStrip.h;
        canvas.DrawLine({barStrip.x - 3.0f, gateY}, {barStrip.Right() + 3.0f, gateY}, Color(244, 63, 94, 240), 2.0f);
    }

    // 1. Upper Instantaneous Trace Panel (Average, Peak Hold, Live Slice, Threshold Gate, Auto Peaks)
    if (geom.hasTrace && geom.traceRect.h > 10.0f && !state.liveSliceDb.empty()) {
        canvas.PushClip(geom.traceRect);

        // Selected Band / Sector Highlight in Trace Panel
        if (resp.hasSelectionBand) {
            const keel::f32 sx0 = keel::clamp(geom.XToScreen(resp.selectionStartX), geom.traceRect.x, geom.traceRect.Right());
            const keel::f32 sx1 = keel::clamp(geom.XToScreen(resp.selectionEndX), geom.traceRect.x, geom.traceRect.Right());
            const keel::f32 left = keel::min(sx0, sx1);
            const keel::f32 width = keel::abs(sx1 - sx0);
            if (width > 1.0f) {
                canvas.DrawRect(Rect(left, geom.traceRect.y, width, geom.traceRect.h), Color(56, 189, 248, 36));
                canvas.DrawLine({left, geom.traceRect.y}, {left, geom.traceRect.Bottom()}, Color(56, 189, 248, 210), 1.3f);
                canvas.DrawLine({left + width, geom.traceRect.y}, {left + width, geom.traceRect.Bottom()}, Color(56, 189, 248, 210), 1.3f);
            }
        }

        // Harmonic Comb Lines in Trace Panel
        if (opts.showHarmonicComb && state.harmonicFundamentalX > 0.0f) {
            for (keel::i32 h = 1; h <= 6; ++h) {
                const keel::f32 hx = state.harmonicFundamentalX * keel::as<keel::f32>(h);
                if (hx >= geom.viewMinX && hx <= geom.viewMaxX) {
                    const keel::f32 px = geom.XToScreen(hx);
                    const Color hc = (h == 1) ? Color(250, 204, 21, 210) : Color(250, 204, 21, 120);
                    canvas.DrawLine({px, geom.traceRect.y}, {px, geom.traceRect.Bottom()}, hc, (h == 1) ? 1.5f : 1.0f);
                    char hBuf[12];
                    const keel::StringRef hStr = keel::format_buf(hBuf, "H%d", h);
                    canvas.DrawText(hStr, {px + 3.0f, geom.traceRect.y + 4.0f}, 9.0f, hc);
                }
            }
        }

        const keel::i32 numSteps = keel::clamp(keel::as<keel::i32>(geom.traceRect.w / 2.5f), 64, 360);
        auto drawTraceCurve = [&](keel::ArrayRef<keel::f32> vec, Color strokeCol, keel::f32 strokeW, bool fillBars) {
            if (vec.empty()) return;
            Vec2 prevPt{};
            for (keel::i32 s = 0; s <= numSteps; ++s) {
                const keel::f32 u = keel::as<keel::f32>(s) / keel::as<keel::f32>(numSteps);
                const keel::f32 xVal = geom.viewMinX + u * (geom.viewMaxX - geom.viewMinX);
                const keel::f32 dbVal = SampleTraceVectorDb(vec, opts, xVal);
                const Vec2 pt{geom.traceRect.x + u * geom.traceRect.w, geom.DbToTraceY(dbVal)};
                if (fillBars && (s % 2 == 0)) {
                    const Color barCol(strokeCol.r, strokeCol.g, strokeCol.b, 28);
                    canvas.DrawLine({pt.x, geom.traceRect.Bottom()}, pt, barCol, 1.5f);
                }
                if (s > 0) {
                    canvas.DrawLine(prevPt, pt, strokeCol, strokeW);
                }
                prevPt = pt;
            }
        };

        if (opts.showAverageTrace) {
            drawTraceCurve(state.averageDb.as_ref(), Color(251, 191, 36, 185), 1.3f, false);
        }
        if (opts.showPeakHold) {
            drawTraceCurve(state.peakHoldDb.as_ref(), Color(192, 132, 252, 200), 1.2f, false);
        }
        if (opts.showLiveTrace) {
            const Color liveCol = (opts.mode == WaterfallMode::DoaBtr)
                ? Color(52, 211, 153, 245)
                : Color(56, 189, 248, 245);
            drawTraceCurve(state.liveSliceDb.as_ref(), liveCol, 1.7f, true);
        }

        // Interactive Detection Threshold Gate Line
        if (opts.showThresholdGate) {
            const keel::f32 gateY = geom.DbToTraceY(state.thresholdGateDb);
            const Color gateCol = state.draggingThreshold ? Color(250, 204, 21, 245) : Color(244, 63, 94, 205);
            canvas.DrawLine({geom.traceRect.x, gateY}, {geom.traceRect.Right(), gateY}, gateCol, 1.4f);

            char gateTag[32];
            const keel::StringRef gateStr = keel::format_buf(gateTag, "GATE %+.0f dB", state.thresholdGateDb);
            const Rect pill(geom.traceRect.Right() - 78.0f, keel::clamp(gateY - 9.0f, geom.traceRect.y + 2.0f, geom.traceRect.Bottom() - 18.0f), 74.0f, 16.0f);
            canvas.DrawRoundRect(pill, 4.0f, Color(15, 23, 42, 225), gateCol, 1.0f);
            canvas.DrawText(gateStr, {pill.x + 6.0f, pill.y + 2.5f}, 9.0f, gateCol);
        }

        // Automatic Peak / DOA Lobe Detection & Callout Reticles
        if (opts.showPeakMarkers && state.liveSliceDb.size() >= 8) {
            struct PeakCandidate {
                keel::f32 xVal;
                keel::f32 dbVal;
            };
            keel::SmallArray<PeakCandidate, 32> peaks;
            const keel::usize n = state.liveSliceDb.size();
            const keel::usize minSepBins = keel::max<keel::usize>(4, n / 28);
            for (keel::usize i = 2; i + 2 < n; ++i) {
                const keel::f32 db = state.liveSliceDb[i] + opts.gainDb;
                if (db < state.thresholdGateDb) continue;
                if (db >= state.liveSliceDb[i - 1] + opts.gainDb &&
                    db >= state.liveSliceDb[i + 1] + opts.gainDb &&
                    db > state.liveSliceDb[i - 2] + opts.gainDb &&
                    db > state.liveSliceDb[i + 2] + opts.gainDb) {
                    const keel::f32 xVal = geom.domainMinX + (keel::as<keel::f32>(i) / keel::as<keel::f32>(n - 1)) * (geom.domainMaxX - geom.domainMinX);
                    if (xVal < geom.viewMinX || xVal > geom.viewMaxX) continue;
                    bool merged = false;
                    for (auto& existing : peaks) {
                        const keel::f32 existBin = ((existing.xVal - geom.domainMinX) / domainSpan) * keel::as<keel::f32>(n - 1);
                        if (keel::abs(keel::as<keel::f32>(i) - existBin) < keel::as<keel::f32>(minSepBins)) {
                            if (db > existing.dbVal) {
                                existing.xVal = xVal;
                                existing.dbVal = db;
                            }
                            merged = true;
                            break;
                        }
                    }
                    if (!merged) {
                        peaks.push_back({xVal, db});
                    }
                }
            }
            keel::sort(peaks.as_mut_ref(), [](const PeakCandidate& a, const PeakCandidate& b) noexcept {
                return a.dbVal > b.dbVal;
            });
            const keel::usize maxPeaks = keel::min<keel::usize>(4, peaks.size());
            for (keel::usize k = 0; k < maxPeaks; ++k) {
                const Vec2 pkPt{geom.XToScreen(peaks[k].xVal), geom.DbToTraceY(peaks[k].dbVal)};
                const Color pkCol = (k == 0) ? Color(250, 204, 21, 255) : Color(56, 189, 248, 235);
                canvas.DrawTriangle({pkPt.x, pkPt.y - 5.0f}, {pkPt.x - 4.0f, pkPt.y}, {pkPt.x + 4.0f, pkPt.y}, pkCol);
                canvas.DrawTriangle({pkPt.x, pkPt.y + 5.0f}, {pkPt.x - 4.0f, pkPt.y}, {pkPt.x + 4.0f, pkPt.y}, pkCol);

                const auto xStr = FormatXValue(peaks[k].xVal, opts.mode, geom.viewMaxX - geom.viewMinX);
                char pkLabel[64];
                const keel::StringRef pkStr = keel::format_buf(pkLabel, "%s (%.0fdB)", xStr, peaks[k].dbVal);
                const keel::f32 lx = keel::clamp(pkPt.x - 24.0f, geom.traceRect.x + 4.0f, geom.traceRect.Right() - 96.0f);
                const keel::f32 ly = keel::max(geom.traceRect.y + 3.0f, pkPt.y - 17.0f);
                canvas.DrawText(pkStr, {lx, ly}, 9.2f, pkCol);
            }
        }

        // Synchronized Cursor Hairline in Trace Panel
        if (resp.cursorInside) {
            const keel::f32 cx = geom.XToScreen(resp.cursorX);
            canvas.DrawLine({cx, geom.traceRect.y}, {cx, geom.traceRect.Bottom()}, Color(255, 255, 255, 165), 1.0f);
            const keel::f32 cy = geom.DbToTraceY(resp.cursorLiveDb);
            canvas.DrawRoundRect(Rect(cx - 3.5f, cy - 3.5f, 7.0f, 7.0f), 3.5f, Color(56, 189, 248, 255), Color(255, 255, 255, 240), 1.2f);
        }

        canvas.PopClip();
    }

    // 2. Lower Waterfall Panel Overlays (Selection Band, Harmonic Comb, DOA Target Tracks, Crosshair, HUD)
    if (geom.hasWaterfall && geom.wfRect.h > 10.0f) {
        canvas.PushClip(geom.wfRect);

        // Selected Band / Sector Highlight in Waterfall
        if (resp.hasSelectionBand) {
            const keel::f32 sx0 = keel::clamp(geom.XToScreen(resp.selectionStartX), geom.wfRect.x, geom.wfRect.Right());
            const keel::f32 sx1 = keel::clamp(geom.XToScreen(resp.selectionEndX), geom.wfRect.x, geom.wfRect.Right());
            const keel::f32 left = keel::min(sx0, sx1);
            const keel::f32 width = keel::abs(sx1 - sx0);
            if (width > 1.0f) {
                canvas.DrawRect(Rect(left, geom.wfRect.y, width, geom.wfRect.h), Color(56, 189, 248, 28));
                canvas.DrawLine({left, geom.wfRect.y}, {left, geom.wfRect.Bottom()}, Color(56, 189, 248, 210), 1.3f);
                canvas.DrawLine({left + width, geom.wfRect.y}, {left + width, geom.wfRect.Bottom()}, Color(56, 189, 248, 210), 1.3f);
            }
        }

        // Harmonic Comb Lines in Waterfall
        if (opts.showHarmonicComb && state.harmonicFundamentalX > 0.0f) {
            for (keel::i32 h = 1; h <= 6; ++h) {
                const keel::f32 hx = state.harmonicFundamentalX * keel::as<keel::f32>(h);
                if (hx >= geom.viewMinX && hx <= geom.viewMaxX) {
                    const keel::f32 px = geom.XToScreen(hx);
                    const Color hc = (h == 1) ? Color(250, 204, 21, 175) : Color(250, 204, 21, 95);
                    canvas.DrawLine({px, geom.wfRect.y}, {px, geom.wfRect.Bottom()}, hc, 1.0f);
                }
            }
        }

        // DOA / Emitter Target Track Trajectories & Gate Corridors
        if (opts.showTracks && !opts.tracks.empty()) {
            for (const auto& trk : opts.tracks) {
                const bool isSel = (trk.id == resp.selectedTrackId);
                const bool isHov = (trk.id == resp.hoveredTrackId);

                Color trkCol{52, 211, 153, 235};
                if (trk.status == WaterfallTrackStatus::Alarm) {
                    trkCol = Color(244, 63, 94, 245);
                } else if (trk.status == WaterfallTrackStatus::Coasting) {
                    trkCol = Color(251, 191, 36, 220);
                }
                if (isSel) {
                    trkCol = Color(250, 204, 21, 255);
                }

                // Draw historical trajectory polyline & gate corridor
                if (trk.history.size() >= 2) {
                    const keel::f32 gateHalfPx = (trk.gateWidth * 0.5f / keel::max(1e-5f, geom.viewMaxX - geom.viewMinX)) * geom.wfRect.w;
                    for (keel::usize i = 0; i + 1 < trk.history.size(); ++i) {
                        const auto& p0 = trk.history[i];
                        const auto& p1 = trk.history[i + 1];
                        if (p0.timeAgoSec > geom.historyDurationSec && p1.timeAgoSec > geom.historyDurationSec) continue;

                        const Vec2 s0{geom.XToScreen(p0.xValue), geom.TimeAgoToWaterfallY(p0.timeAgoSec)};
                        const Vec2 s1{geom.XToScreen(p1.xValue), geom.TimeAgoToWaterfallY(p1.timeAgoSec)};
                        canvas.DrawLine(s0, s1, trkCol, isSel ? 2.4f : 1.8f);

                        if (opts.showTrackGates && gateHalfPx >= 3.0f && (i % 2 == 0)) {
                            const Color gateCol(trkCol.r, trkCol.g, trkCol.b, isSel ? 115 : 68);
                            canvas.DrawLine({s0.x - gateHalfPx, s0.y}, {s1.x - gateHalfPx, s1.y}, gateCol, 1.0f);
                            canvas.DrawLine({s0.x + gateHalfPx, s0.y}, {s1.x + gateHalfPx, s1.y}, gateCol, 1.0f);
                        }
                    }
                }

                // Draw current track head reticle at t = 0s
                if (trk.currentX >= geom.viewMinX && trk.currentX <= geom.viewMaxX) {
                    const Vec2 headPt{geom.XToScreen(trk.currentX), geom.TimeAgoToWaterfallY(0.0f) + (opts.invertTimeAxis ? -8.0f : 8.0f)};
                    const keel::f32 sz = (isSel || isHov) ? 6.5f : 5.2f;
                    canvas.DrawTriangle({headPt.x, headPt.y - sz}, {headPt.x - sz, headPt.y}, {headPt.x + sz, headPt.y}, trkCol);
                    canvas.DrawTriangle({headPt.x, headPt.y + sz}, {headPt.x - sz, headPt.y}, {headPt.x + sz, headPt.y}, trkCol);

                    // Drift rate prediction vector (5 seconds projection)
                    const keel::f32 projX = trk.currentX - trk.driftRatePerSec * 4.0f;
                    const Vec2 vecEnd{geom.XToScreen(projX), headPt.y + (opts.invertTimeAxis ? -22.0f : 22.0f)};
                    canvas.DrawLine(headPt, vecEnd, Color(255, 255, 255, 210), 1.4f);

                    // Track Callsign & Bearing/Drift Telemetry Badge
                    const auto xStr = FormatXValue(trk.currentX, opts.mode, geom.viewMaxX - geom.viewMinX);
                    char tagBuf[96];
                    const keel::StringRef tagStr = (opts.mode == WaterfallMode::DoaBtr)
                        ? keel::format_buf(tagBuf, "%s %s (%+.1f°/s)", trk.callsign, xStr, trk.driftRatePerSec)
                        : keel::format_buf(tagBuf, "%s %s (%.0fdB)", trk.callsign, xStr, trk.levelDb);
                    const keel::f32 tagW = 148.0f;
                    const keel::f32 tagX = keel::clamp(headPt.x + 9.0f, geom.wfRect.x + 4.0f, geom.wfRect.Right() - tagW - 4.0f);
                    const keel::f32 tagY = keel::clamp(headPt.y - 7.0f, geom.wfRect.y + 4.0f, geom.wfRect.Bottom() - 20.0f);
                    canvas.DrawRoundRect(Rect(tagX, tagY, tagW, 17.0f), 4.0f, Color(9, 14, 26, 220), trkCol, isSel ? 1.4f : 1.0f);
                    canvas.DrawText(tagStr, {tagX + 5.0f, tagY + 2.5f}, 9.2f, trkCol);
                }
            }
        }

        // Synchronized Crosshair & Live Readout Tooltip in Waterfall
        if (resp.cursorInside) {
            const keel::f32 cx = geom.XToScreen(resp.cursorX);
            canvas.DrawLine({cx, geom.wfRect.y}, {cx, geom.wfRect.Bottom()}, Color(255, 255, 255, 165), 1.0f);

            if (resp.cursorInWaterfall) {
                const keel::f32 cy = geom.TimeAgoToWaterfallY(resp.cursorTimeAgoSec);
                canvas.DrawLine({geom.wfRect.x, cy}, {geom.wfRect.Right(), cy}, Color(255, 255, 255, 130), 1.0f);
                canvas.DrawRoundRect(Rect(cx - 4.0f, cy - 4.0f, 8.0f, 8.0f), 4.0f, Color::Transparent(), Color(250, 204, 21, 245), 1.4f);

                const auto xStr = FormatXValue(resp.cursorX, opts.mode, geom.viewMaxX - geom.viewMinX);
                char tipBuf[96];
                const keel::StringRef tipStr = keel::format_buf(
                    tipBuf, "%s  |  -%.1fs  |  WF: %+.1f dB  (Live: %+.1f dB)",
                    xStr, resp.cursorTimeAgoSec, resp.cursorWaterfallDb, resp.cursorLiveDb
                );
                const keel::f32 boxW = 248.0f;
                const keel::f32 boxH = 20.0f;
                const keel::f32 bx = (cx + 12.0f + boxW <= geom.wfRect.Right()) ? (cx + 12.0f) : (cx - boxW - 12.0f);
                const keel::f32 by = keel::clamp(cy - 24.0f, geom.wfRect.y + 6.0f, geom.wfRect.Bottom() - boxH - 6.0f);
                canvas.DrawRoundRect(Rect(bx, by, boxW, boxH), 4.0f, Color(9, 13, 24, 232), Color(56, 189, 248, 180), 1.0f);
                canvas.DrawText(tipStr, {bx + 7.0f, by + 4.0f}, 9.5f, Color(241, 245, 249, 250));
            }
        }

        // Corner Operator Telemetry HUD Cards
        if (opts.showHudOverlay) {
            const Rect tlCard(geom.wfRect.x + 10.0f, geom.wfRect.y + 10.0f, 226.0f, 56.0f);
            canvas.DrawRoundRect(tlCard, 6.0f, Color(9, 14, 26, 215), Color(56, 189, 248, 85), 1.0f);

            const keel::StringRef modeTitle = (opts.mode == WaterfallMode::DoaBtr)
                ? keel::StringRef("DOA / BTR AZIMUTH WATERFALL")
                : keel::StringRef("RTSA SPECTRAL WATERFALL");
            canvas.DrawText(modeTitle, {tlCard.x + 9.0f, tlCard.y + 5.0f}, 10.0f, accentTextCol);

            char info1[64];
            const keel::StringRef infoStr1 = keel::format_buf(
                info1, "RING: %ux%u  HIST: %.0fs  ZOOM: %.1fx",
                opts.numBins, opts.historyRows, geom.historyDurationSec, resp.zoomFactor
            );
            canvas.DrawText(infoStr1, {tlCard.x + 9.0f, tlCard.y + 22.0f}, 9.5f, Color(226, 232, 240, 230));

            const auto peakXStr = FormatXValue(resp.dominantPeakX, opts.mode, geom.viewMaxX - geom.viewMinX);
            char info2[64];
            const keel::StringRef infoStr2 = keel::format_buf(info2, "PEAK: %s (%+.1f dB)", peakXStr, resp.dominantPeakDb);
            canvas.DrawText(infoStr2, {tlCard.x + 9.0f, tlCard.y + 38.0f}, 9.5f, Color(250, 204, 21, 235));

            if (resp.hasSelectionBand) {
                const Rect trCard(geom.wfRect.Right() - 228.0f, geom.wfRect.y + 10.0f, 218.0f, 42.0f);
                canvas.DrawRoundRect(trCard, 6.0f, Color(9, 14, 26, 222), Color(56, 189, 248, 150), 1.0f);
                const auto cStr = FormatXValue(resp.selectionCenterX, opts.mode, geom.viewMaxX - geom.viewMinX);
                const auto sStr = FormatXValue(resp.selectionSpanX, opts.mode, geom.viewMaxX - geom.viewMinX);
                char bLine1[80], bLine2[64];
                const keel::StringRef bStr1 = keel::format_buf(
                    bLine1, "%s: %s  SPAN: %s",
                    (opts.mode == WaterfallMode::DoaBtr) ? "SECTOR CTR" : "BAND Fc", cStr, sStr
                );
                const keel::StringRef bStr2 = keel::format_buf(bLine2, "INTEGRATED POWER: %+.1f dB", resp.selectionBandPowerDb);
                canvas.DrawText(bStr1, {trCard.x + 8.0f, trCard.y + 6.0f}, 9.5f, accentTextCol);
                canvas.DrawText(bStr2, {trCard.x + 8.0f, trCard.y + 22.0f}, 9.5f, Color(52, 211, 153, 245));
            }
        }

        canvas.PopClip();
    }
}

} // namespace

Vec2 WaterfallView::Measure(
    keel::ArrayRef<keel::f32> /*latestSliceDb*/,
    const WaterfallViewOptions& opts,
    UiSize /*size*/,
    ICanvas* /*canvas*/,
    const UiTheme& /*theme*/,
    keel::f32 availableWidth
) {
    keel::f32 defaultW = 520.0f;
    if (opts.width.IsFixed()) {
        defaultW = opts.width.Dp();
    } else if (opts.width.ResolveMode(SizeMode::Stretch) == SizeMode::Stretch && availableWidth > 0.0f) {
        defaultW = availableWidth;
    }
    const keel::f32 defaultH = opts.height.IsFixed()
        ? opts.height.Dp()
        : 420.0f;
    return {defaultW, defaultH};
}

WaterfallViewResponse WaterfallView::Run(
    Context& ctx,
    keel::ArrayRef<keel::f32> latestSliceDb,
    const WaterfallViewOptions& opts,
    keel::SourceLocation loc
) {

    const Id id = ctx.MakeChildId(loc);
    const UiSize size = opts.size;
    const bool effDisabled = opts.disabled || ctx.IsDisabled();

    Node* node = ctx.CreateNode(keel::from_underlying<NodeType>(keel::as<keel::u16>(NodeType::User) + 3u), id);
    auto* payload = ctx.AllocPayload<WaterfallViewPayload>();
    payload->opts = opts;
    payload->opts.tooltip = ctx.AllocText(opts.tooltip);

    auto& state = ctx.GetOrCreatePersistentState<WaterfallViewState>(id);
    payload->state = &state;

    const keel::u32 numBins = keel::max(16u, opts.numBins);
    const keel::u32 historyRows = keel::max(16u, opts.historyRows);
    const keel::f64 nowSec = ctx.GetCurrentTime();
    const keel::f32 dtSec = (state.lastUpdateTimeSec > 0.0)
        ? keel::as<keel::f32>(keel::clamp(nowSec - state.lastUpdateTimeSec, 0.0, 0.25))
        : 0.016f;
    state.lastUpdateTimeSec = nowSec;

    // 1. Initialize or resize Circular Ring Buffer
    const bool dimsChanged = (!state.initialized || state.numBins != numBins || state.historyRows != historyRows);
    const bool domainChanged = (keel::abs(state.lastDomainMinX - opts.minX) > 1e-3f ||
                                keel::abs(state.lastDomainMaxX - opts.maxX) > 1e-3f);
    if (dimsChanged || opts.requestClearHistory) {
        state.numBins = numBins;
        state.historyRows = historyRows;
        state.ringBufferDb.assign(keel::as<keel::usize>(historyRows) * numBins, opts.dbFloor);
        state.liveSliceDb.assign(numBins, opts.dbFloor);
        state.peakHoldDb.assign(numBins, opts.dbFloor);
        state.averageDb.assign(numBins, opts.dbFloor);
        state.headRow = historyRows - 1;
        state.validRows = 0;

        if (opts.initialHistoryMatrix.size() >= keel::as<keel::usize>(historyRows) * numBins) {
            // Bulk preload: initialHistoryMatrix row 0 = newest (t = 0s), row (historyRows - 1) = oldest
            for (keel::u32 age = 0; age < historyRows; ++age) {
                const keel::u32 ringRow = (state.headRow + historyRows - age) % historyRows;
                const keel::f32* srcRow = opts.initialHistoryMatrix.data() + keel::as<keel::usize>(age) * numBins;
                keel::f32* dstRow = state.ringBufferDb.data() + keel::as<keel::usize>(ringRow) * numBins;
                keel::mem_copy(dstRow, srcRow, keel::as<keel::usize>(numBins) * sizeof(keel::f32));
            }
            state.validRows = historyRows;
            keel::mem_copy(state.liveSliceDb.data(), opts.initialHistoryMatrix.data(), keel::as<keel::usize>(numBins) * sizeof(keel::f32));
            keel::mem_copy(state.peakHoldDb.data(), state.liveSliceDb.data(), keel::as<keel::usize>(numBins) * sizeof(keel::f32));
            keel::mem_copy(state.averageDb.data(), state.liveSliceDb.data(), keel::as<keel::usize>(numBins) * sizeof(keel::f32));

            const keel::u32 scanRows = keel::min(historyRows, 48u);
            for (keel::u32 age = 1; age < scanRows; ++age) {
                const keel::f32* srcRow = opts.initialHistoryMatrix.data() + keel::as<keel::usize>(age) * numBins;
                for (keel::u32 b = 0; b < numBins; ++b) {
                    state.peakHoldDb[b] = keel::max(state.peakHoldDb[b], srcRow[b]);
                    state.averageDb[b] = state.averageDb[b] * 0.9f + srcRow[b] * 0.1f;
                }
            }
        } else if (latestSliceDb.size() >= numBins) {
            keel::mem_copy(state.ringBufferDb.data() + keel::as<keel::usize>(state.headRow) * numBins,
                           latestSliceDb.data(), keel::as<keel::usize>(numBins) * sizeof(keel::f32));
            keel::mem_copy(state.liveSliceDb.data(), latestSliceDb.data(), keel::as<keel::usize>(numBins) * sizeof(keel::f32));
            keel::mem_copy(state.peakHoldDb.data(), latestSliceDb.data(), keel::as<keel::usize>(numBins) * sizeof(keel::f32));
            keel::mem_copy(state.averageDb.data(), latestSliceDb.data(), keel::as<keel::usize>(numBins) * sizeof(keel::f32));
            state.validRows = 1;
        }

        state.lastSliceSequence = opts.sliceSequence;
        state.fullGeneration++;
        state.sliceGeneration++;
        state.initialized = true;
    } else if (opts.sliceSequence != state.lastSliceSequence && latestSliceDb.size() >= numBins) {
        // Streaming O(1) Ring-Buffer Advance: write only the single new row at headRow!
        state.headRow = (state.headRow + 1) % historyRows;
        state.validRows = keel::min(historyRows, state.validRows + 1);
        keel::f32* dstRow = state.ringBufferDb.data() + keel::as<keel::usize>(state.headRow) * numBins;
        keel::mem_copy(dstRow, latestSliceDb.data(), keel::as<keel::usize>(numBins) * sizeof(keel::f32));
        keel::mem_copy(state.liveSliceDb.data(), latestSliceDb.data(), keel::as<keel::usize>(numBins) * sizeof(keel::f32));

        const keel::f32 decayStep = keel::max(0.0f, opts.peakDecayDbPerSec) * dtSec;
        const keel::f32 alpha = keel::clamp(opts.averageAlpha, 0.01f, 1.0f);
        for (keel::u32 b = 0; b < numBins; ++b) {
            const keel::f32 v = latestSliceDb[b];
            state.peakHoldDb[b] = keel::max(state.peakHoldDb[b] - decayStep, v);
            state.averageDb[b] += alpha * (v - state.averageDb[b]);
        }

        state.lastSliceSequence = opts.sliceSequence;
        state.sliceGeneration++;
    }

    if (opts.requestResetPeakHold && !state.liveSliceDb.empty()) {
        state.peakHoldDb.assign(state.liveSliceDb.as_ref());
    }

    if (domainChanged || opts.requestResetZoom || (state.viewMaxX <= state.viewMinX)) {
        state.viewMinX = opts.minX;
        state.viewMaxX = opts.maxX;
        state.lastDomainMinX = opts.minX;
        state.lastDomainMaxX = opts.maxX;
        if (opts.requestResetZoom) {
            state.hasSelectionBand = false;
        }
    }

    // Two-way bindings sync in
    if (opts.viewMinX && opts.viewMaxX && *opts.viewMaxX > *opts.viewMinX) {
        state.viewMinX = keel::clamp(*opts.viewMinX, opts.minX, opts.maxX);
        state.viewMaxX = keel::clamp(*opts.viewMaxX, state.viewMinX + 1e-3f, opts.maxX);
    }
    if (!state.draggingThreshold) {
        state.thresholdGateDb = opts.thresholdGateDbPtr ? *opts.thresholdGateDbPtr : opts.thresholdGateDb;
    }
    if (opts.harmonicFundamentalX && *opts.harmonicFundamentalX > 0.0f) {
        state.harmonicFundamentalX = *opts.harmonicFundamentalX;
    }
    if (opts.selectedTrackId) {
        state.selectedTrackId = *opts.selectedTrackId;
    }

    Response baseResp = ctx.Interact(id);
    WaterfallViewResponse resp{};
    static_cast<Response&>(resp) = baseResp;
    resp.mode = opts.mode;

    Rect prevBounds = baseResp.rect;
    if (prevBounds.w <= 1.0f || prevBounds.h <= 1.0f) {
        prevBounds = ctx.GetParentPrevFrameBounds();
    }
    const Rect ctrlBounds = prevBounds;
    const WaterfallGeometry geom = ComputeWaterfallGeometry(ctrlBounds, payload->opts, state.viewMinX, state.viewMaxX);

    const Vec2 mousePos = ctx.GetMousePos();
    const bool ctrlHovered = !effDisabled && ctx.IsRectHovered(id, ctrlBounds);
    const bool inTrace = ctrlHovered && geom.hasTrace && geom.traceRect.Contains(mousePos);
    const bool inWf = ctrlHovered && geom.hasWaterfall && geom.wfRect.Contains(mousePos);
    const bool inPlot = inTrace || inWf;

    if (inPlot) {
        // 1. Mouse Wheel X-Axis Zoom centered on cursorX
        const Vec2 wheel = ctx.GetMouseWheel();
        if (keel::abs(wheel.y) > 0.01f) {
            const keel::f32 cursorX = geom.ScreenToX(mousePos.x);
            const keel::f32 domainSpan = keel::max(1e-3f, opts.maxX - opts.minX);
            const keel::f32 curSpan = keel::max(1e-3f, state.viewMaxX - state.viewMinX);
            const keel::f32 zoomFactor = (wheel.y > 0.0f) ? 0.80f : 1.25f;
            const keel::f32 newSpan = keel::clamp(curSpan * zoomFactor, domainSpan / 32.0f, domainSpan);
            const keel::f32 rel = (cursorX - state.viewMinX) / curSpan;
            keel::f32 newMin = cursorX - rel * newSpan;
            keel::f32 newMax = newMin + newSpan;
            if (newMin < opts.minX) {
                newMin = opts.minX;
                newMax = newMin + newSpan;
            }
            if (newMax > opts.maxX) {
                newMax = opts.maxX;
                newMin = keel::max(opts.minX, newMax - newSpan);
            }
            state.viewMinX = newMin;
            state.viewMaxX = newMax;
            resp.changed = true;
            ctx.ConsumeMouseWheel();
            ctx.RequestRepaint(2);
        }

        // 2. Hit-test DOA / Emitter Tracks
        for (const auto& trk : opts.tracks) {
            const keel::f32 headX = geom.XToScreen(trk.currentX);
            const keel::f32 headY = geom.TimeAgoToWaterfallY(0.0f) + (opts.invertTimeAxis ? -8.0f : 8.0f);
            if (keel::abs(mousePos.x - headX) <= 14.0f && keel::abs(mousePos.y - headY) <= 14.0f) {
                resp.hoveredTrackId = trk.id;
                break;
            }
            for (const auto& pt : trk.history) {
                if (pt.timeAgoSec > geom.historyDurationSec) continue;
                const keel::f32 px = geom.XToScreen(pt.xValue);
                const keel::f32 py = geom.TimeAgoToWaterfallY(pt.timeAgoSec);
                if (keel::abs(mousePos.x - px) <= 10.0f && keel::abs(mousePos.y - py) <= 10.0f) {
                    resp.hoveredTrackId = trk.id;
                    break;
                }
            }
            if (resp.hoveredTrackId != -1) break;
        }

        // 3. Mouse Click / Drag Interactions
        if (ctx.IsMouseClicked()) {
            const KeyModifiers mods = ctx.GetKeyModifiers();
            const keel::f32 gateY = geom.DbToTraceY(state.thresholdGateDb);
            if (inTrace && opts.showThresholdGate && keel::abs(mousePos.y - gateY) <= 7.0f) {
                state.draggingThreshold = true;
            } else if (mods.shift) {
                state.draggingPan = true;
                state.panGrabMouseX = mousePos.x;
                state.panGrabMinX = state.viewMinX;
                state.panGrabMaxX = state.viewMaxX;
            } else if (resp.hoveredTrackId != -1) {
                if (state.selectedTrackId != resp.hoveredTrackId) {
                    state.selectedTrackId = resp.hoveredTrackId;
                    resp.trackSelectionChanged = true;
                    resp.changed = true;
                }
            } else {
                state.draggingSelection = true;
                state.dragAnchorX = geom.ScreenToX(mousePos.x);
                state.selectionStartX = state.dragAnchorX;
                state.selectionEndX = state.dragAnchorX;
            }
        }
    }

    if (ctx.IsMouseDown()) {
        if (state.draggingThreshold && geom.hasTrace) {
            state.thresholdGateDb = keel::clamp(geom.TraceYToDb(mousePos.y), opts.dbFloor + 2.0f, opts.dbCeil - 2.0f);
            if (opts.thresholdGateDbPtr) {
                *opts.thresholdGateDbPtr = state.thresholdGateDb;
            }
            resp.changed = true;
            ctx.RequestRepaint(2);
        } else if (state.draggingPan) {
            const keel::f32 span = state.panGrabMaxX - state.panGrabMinX;
            const keel::f32 refW = keel::max(1.0f, geom.hasWaterfall ? geom.wfRect.w : geom.traceRect.w);
            const keel::f32 dxVal = ((mousePos.x - state.panGrabMouseX) / refW) * span;
            keel::f32 newMin = state.panGrabMinX - dxVal;
            keel::f32 newMax = state.panGrabMaxX - dxVal;
            if (newMin < opts.minX) {
                newMin = opts.minX;
                newMax = newMin + span;
            }
            if (newMax > opts.maxX) {
                newMax = opts.maxX;
                newMin = keel::max(opts.minX, newMax - span);
            }
            state.viewMinX = newMin;
            state.viewMaxX = newMax;
            resp.changed = true;
            ctx.RequestRepaint(2);
        } else if (state.draggingSelection) {
            const keel::f32 curX = geom.ScreenToX(mousePos.x);
            state.selectionStartX = keel::min(state.dragAnchorX, curX);
            state.selectionEndX = keel::max(state.dragAnchorX, curX);
            const keel::f32 minPxSpan = ((state.viewMaxX - state.viewMinX) / keel::max(1.0f, geom.wfRect.w)) * 4.0f;
            state.hasSelectionBand = (state.selectionEndX - state.selectionStartX) > minPxSpan;
            ctx.RequestRepaint(2);
        }
    } else {
        if (state.draggingSelection) {
            const keel::f32 minPxSpan = ((state.viewMaxX - state.viewMinX) / keel::max(1.0f, geom.wfRect.w)) * 4.0f;
            if ((state.selectionEndX - state.selectionStartX) <= minPxSpan) {
                state.hasSelectionBand = false;
                if (opts.showHarmonicComb && state.dragAnchorX > 0.0f) {
                    state.harmonicFundamentalX = state.dragAnchorX;
                    if (opts.harmonicFundamentalX) {
                        *opts.harmonicFundamentalX = state.harmonicFundamentalX;
                    }
                }
            }
        }
        state.draggingThreshold = false;
        state.draggingPan = false;
        state.draggingSelection = false;
    }

    // Write back two-way bindings
    if (opts.viewMinX) *opts.viewMinX = state.viewMinX;
    if (opts.viewMaxX) *opts.viewMaxX = state.viewMaxX;
    if (opts.selectionStartX) *opts.selectionStartX = state.selectionStartX;
    if (opts.selectionEndX) *opts.selectionEndX = state.selectionEndX;
    if (opts.selectedTrackId) *opts.selectedTrackId = state.selectedTrackId;

    // Populate Telemetry Response
    resp.visibleMinX = state.viewMinX;
    resp.visibleMaxX = state.viewMaxX;
    resp.zoomFactor = keel::max(1.0f, (opts.maxX - opts.minX) / keel::max(1e-3f, state.viewMaxX - state.viewMinX));
    resp.selectedTrackId = state.selectedTrackId;
    resp.populatedRows = state.validRows;

    if (inPlot) {
        resp.cursorInside = true;
        resp.cursorInTrace = inTrace;
        resp.cursorInWaterfall = inWf;
        resp.cursorX = geom.ScreenToX(mousePos.x);
        resp.cursorTimeAgoSec = inWf ? geom.WaterfallYToTimeAgo(mousePos.y) : 0.0f;
        resp.cursorLiveDb = SampleTraceVectorDb(state.liveSliceDb.as_ref(), opts, resp.cursorX);
        resp.cursorPeakDb = SampleTraceVectorDb(state.peakHoldDb.as_ref(), opts, resp.cursorX);
        resp.cursorAvgDb = SampleTraceVectorDb(state.averageDb.as_ref(), opts, resp.cursorX);
        resp.cursorWaterfallDb = inWf
            ? SampleRingBufferDb(state, opts, resp.cursorX, resp.cursorTimeAgoSec)
            : resp.cursorLiveDb;
    }

    if (state.hasSelectionBand) {
        resp.hasSelectionBand = true;
        resp.selectionStartX = state.selectionStartX;
        resp.selectionEndX = state.selectionEndX;
        resp.selectionCenterX = 0.5f * (state.selectionStartX + state.selectionEndX);
        resp.selectionSpanX = state.selectionEndX - state.selectionStartX;

        if (!state.liveSliceDb.empty()) {
            const keel::f32 domainSpan = keel::max(1e-5f, opts.maxX - opts.minX);
            const keel::usize b0 = keel::as<keel::usize>(keel::clamp(((resp.selectionStartX - opts.minX) / domainSpan) * keel::as<keel::f32>(numBins - 1), 0.0f, keel::as<keel::f32>(numBins - 1)));
            const keel::usize b1 = keel::as<keel::usize>(keel::clamp(((resp.selectionEndX - opts.minX) / domainSpan) * keel::as<keel::f32>(numBins - 1), 0.0f, keel::as<keel::f32>(numBins - 1)));
            keel::f32 maxBandDb = opts.dbFloor;
            keel::f64 linSum = 0.0;
            keel::usize count = 0;
            for (keel::usize b = keel::min(b0, b1); b <= keel::max(b0, b1); ++b) {
                const keel::f32 db = state.liveSliceDb[b] + opts.gainDb;
                maxBandDb = keel::max(maxBandDb, db);
                linSum += keel::pow(10.0, keel::as<keel::f64>(db) / 10.0);
                count++;
            }
            resp.selectionBandPowerDb = (count > 0)
                ? keel::as<keel::f32>(10.0 * keel::log10(keel::max(1e-14, linSum / keel::as<keel::f64>(count))))
                : maxBandDb;
        }
    }

    // Dominant Peak across visible range
    if (!state.liveSliceDb.empty()) {
        keel::f32 bestDb = -200.0f;
        keel::f32 bestX = state.viewMinX;
        const keel::f32 domainSpan = keel::max(1e-5f, opts.maxX - opts.minX);
        for (keel::u32 b = 0; b < numBins; ++b) {
            const keel::f32 xVal = opts.minX + (keel::as<keel::f32>(b) / keel::as<keel::f32>(numBins - 1)) * domainSpan;
            if (xVal < state.viewMinX || xVal > state.viewMaxX) continue;
            const keel::f32 db = state.liveSliceDb[b] + opts.gainDb;
            if (db > bestDb) {
                bestDb = db;
                bestX = xVal;
            }
        }
        resp.dominantPeakX = bestX;
        resp.dominantPeakDb = bestDb;
    }

    state.renderedTracks.clear();
    state.renderedTracks.reserve(opts.tracks.size());
    keel::u64 tracksFp = detail::HashArgs(opts.tracks.size());
    for (const auto& trk : opts.tracks) {
        state.renderedTracks.push_back(trk.clone());
        tracksFp = detail::HashArgs(
            tracksFp,
            trk.id,
            trk.callsign,
            keel::as<keel::u8>(trk.status),
            trk.currentX,
            trk.driftRatePerSec,
            trk.levelDb,
            trk.history.size()
        );
    }

    payload->opts.tracks = state.renderedTracks.as_ref();
    payload->resp = resp;

    // Compute visual hash for frame-to-frame damage tracking
    const auto packF = [](keel::f32 v) noexcept -> keel::u64 {
        return keel::as<keel::u64>(keel::bit_cast<keel::u32>(v));
    };
    keel::u64 h = 14695981039346656037ULL;
    auto mix = [&](keel::u64 v) noexcept {
        h ^= v;
        h *= 1099511628211ULL;
    };
    mix(state.fullGeneration);
    mix(state.sliceGeneration);
    mix(tracksFp);
    mix(keel::as<keel::u64>(opts.mode));
    mix(keel::as<keel::u64>(opts.viewLayout));
    mix(keel::as<keel::u64>(opts.colormap));
    mix(packF(resp.visibleMinX));
    mix(packF(resp.visibleMaxX));
    mix(packF(opts.dbFloor));
    mix(packF(opts.dbCeil));
    mix(packF(opts.gainDb));
    mix(packF(state.thresholdGateDb));
    mix(packF(opts.gateSuppression));
    mix(packF(opts.gamma));
    mix(resp.cursorInside ? 1ULL : 0ULL);
    if (resp.cursorInside) {
        mix(packF(resp.cursorX));
        mix(packF(resp.cursorTimeAgoSec));
    }
    mix(resp.hasSelectionBand ? 1ULL : 0ULL);
    if (resp.hasSelectionBand) {
        mix(packF(resp.selectionStartX));
        mix(packF(resp.selectionEndX));
    }
    mix(keel::as<keel::u64>(resp.selectedTrackId + 2));
    mix(keel::as<keel::u64>(resp.hoveredTrackId + 2));
    mix(detail::HashArgs(opts.background, opts.border));
    node->visualHash = h;

    node->text = "WaterfallView";
    node->SetPayload(payload);
    node->measureFn = [](const Node* n, keel::f32 availW, ICanvas* c, const UiTheme& t) noexcept -> Vec2 {
        const auto* p = n->As<WaterfallViewPayload>();
        if (!p) return Vec2{520.0f, 420.0f};
        WaterfallViewOptions effOpts = p->opts;
        if (n->widthMode == SizeMode::Stretch && !effOpts.width.IsFixed()) {
            effOpts.width = SizeMode::Stretch;
        }
        return WaterfallView::Measure({}, effOpts, n->uiSize, c, t, availW);
    };
    node->renderFn = &DrawWaterfallViewNode;

    node->uiSize = size;
    node->disabled = effDisabled;
    node->hovered = baseResp.hovered;
    node->pressed = baseResp.pressed;

    ControlSpec ctrlSpec{};
    ctrlSpec.width = opts.width;
    ctrlSpec.height = opts.height;
    ctrlSpec.defaultWidthMode = SizeMode::Stretch;
    ctrlSpec.defaultHeightMode = SizeMode::Stretch;
    ctrlSpec.minHeight = opts.height.IsFixed() ? opts.height.Dp() : 280.0f;
    Control::SetupNode(ctx, node, ctrlSpec);
    if (!opts.tooltip.empty() && baseResp.hovered) {
        ctx.SetTooltip(opts.tooltip, prevBounds);
    }

    return resp;
}

} // namespace guikit
