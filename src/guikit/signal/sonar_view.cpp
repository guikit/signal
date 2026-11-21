#include "guikit/signal/sonar_view.hpp"
#include "guikit/backend/i_canvas.hpp"
#include "guikit/context.hpp"
#include "guikit/core/control.hpp"
#include "guikit/core/node.hpp"
#include "guikit/graphics/colormap.hpp"
#include "keel/bits.h"
#include "keel/format.h"
#include "keel/math.h"
#include "keel/static_array.h"

namespace guikit {

namespace {

struct SonarViewState {
    keel::u32 numBeams{0};
    keel::u32 numRangeBins{0};
    keel::Array<keel::f32> persistentGrid{};
    keel::Array<keel::u32> abgrPixels{};
    keel::Array<SonarTarget> renderedTargets{};
    keel::u32 cachedImgW{0};
    keel::u32 cachedImgH{0};
    keel::u64 gridGeneration{1};
    keel::u64 lastInputHash{0};

    keel::f32 rangeZoom{1.0f};
    keel::f32 eblBearingDeg{35.0f};
    keel::f32 vrmRangeMeters{220.0f};
    bool eblVrmDragging{false};
    keel::i32 selectedTargetId{-1};
    keel::f32 prevSweepAngleDeg{0.0f};
    bool initialized{false};

    keel::u64 baseLayerKey{0};
    keel::u32 baseLayerW{0};
    keel::u32 baseLayerH{0};
    keel::Array<keel::u32> baseLayerPixels{};
    bool baseLayerValid{false};

    void TrimInactiveCaches() noexcept {
        abgrPixels = keel::Array<keel::u32>{};
        cachedImgW = 0;
        cachedImgH = 0;
        lastInputHash = 0;
        baseLayerPixels = keel::Array<keel::u32>{};
        baseLayerValid = false;
        baseLayerKey = 0;
    }
};

struct SonarViewPayload {
    SonarViewOptions opts{};
    SonarViewResponse resp{};
    SonarViewState* state{nullptr};
    keel::u32 imgW{0};
    keel::u32 imgH{0};
    const keel::u32* abgrData{nullptr};
};

/// Normalizes an angle in degrees to [0, 360).
[[nodiscard]] inline keel::f32 NormalizeBearingDeg(keel::f32 deg) noexcept {
    keel::f32 r = keel::fmod(deg, 360.0f);
    if (r < 0.0f) r += 360.0f;
    return r;
}

/// Computes signed shortest angular difference (a - b) in degrees within [-180, +180].
[[nodiscard]] inline keel::f32 SignedAngleDiffDeg(keel::f32 a, keel::f32 b) noexcept {
    return keel::fmod(a - b + 540.0f, 360.0f) - 180.0f;
}

/// Returns true if `bearingDeg` lies inside the clockwise sector from `startDeg` to `endDeg`.
[[nodiscard]] inline bool IsBearingInSector(keel::f32 bearingDeg, keel::f32 startDeg, keel::f32 endDeg) noexcept {
    const keel::f32 b = NormalizeBearingDeg(bearingDeg);
    const keel::f32 s = NormalizeBearingDeg(startDeg);
    const keel::f32 e = NormalizeBearingDeg(endDeg);
    if (keel::abs(endDeg - startDeg) >= 359.9f) return true;
    if (s <= e) {
        return b >= s && b <= e;
    }
    return b >= s || b <= e;
}

/// Geometry helper for mapping between polar (rangeMeters, bearingDeg) and screen (x, y).
struct SonarPlotGeometry {
    Rect plotRect{};
    SonarDisplayMode mode{SonarDisplayMode::PPI_360};
    keel::f32 visibleMaxRange{500.0f};
    keel::f32 minRange{0.0f};
    keel::f32 headingDeg{0.0f};
    keel::f32 sectorApertureDeg{120.0f};
    keel::f32 startBearingDeg{0.0f};
    keel::f32 endBearingDeg{360.0f};

    Vec2 origin{};          ///< Screen pixel origin of the transducer / radar center
    keel::f32 maxRadiusPx{1.0f};///< Pixel radius corresponding to `visibleMaxRange`

    static SonarPlotGeometry Compute(const Rect& rect, const SonarViewOptions& opts, keel::f32 rangeZoom) noexcept {
        SonarPlotGeometry g;
        g.plotRect = rect;
        g.mode = opts.displayMode;
        g.visibleMaxRange = keel::max(10.0f, opts.maxRangeMeters * keel::clamp(rangeZoom, 0.2f, 1.0f));
        g.minRange = keel::clamp(opts.minRangeMeters, 0.0f, g.visibleMaxRange * 0.5f);
        g.headingDeg = opts.headingDeg;
        g.sectorApertureDeg = keel::clamp(opts.sectorApertureDeg, 30.0f, 240.0f);
        g.startBearingDeg = opts.matrixStartBearingDeg;
        g.endBearingDeg = opts.matrixEndBearingDeg;

        if (g.mode == SonarDisplayMode::PPI_360) {
            const keel::f32 pad = 28.0f;
            g.origin = {rect.x + rect.w * 0.5f, rect.y + rect.h * 0.5f};
            g.maxRadiusPx = keel::max(24.0f, keel::min(rect.w, rect.h) * 0.5f - pad);
        } else if (g.mode == SonarDisplayMode::SectorWedge) {
            const keel::f32 topPad = 32.0f;
            const keel::f32 bottomPad = 26.0f;
            const keel::f32 sidePad = 32.0f;
            const keel::f32 halfAngleRad = keel::deg_to_rad(g.sectorApertureDeg * 0.5f);
            const keel::f32 sinHalf = keel::max(0.25f, keel::sin(keel::min(halfAngleRad, keel::kHalfPiF)));
            const keel::f32 cosHalf = keel::min(0.0f, keel::cos(halfAngleRad)); // negative if > 180 deg

            const keel::f32 availH = keel::max(40.0f, rect.h - topPad - bottomPad);
            const keel::f32 availW = keel::max(40.0f, rect.w - sidePad * 2.0f);
            const keel::f32 rByH = availH / (1.0f - cosHalf);
            const keel::f32 rByW = (availW * 0.5f) / sinHalf;
            g.maxRadiusPx = keel::max(24.0f, keel::min(rByH, rByW));

            const keel::f32 totalFanH = g.maxRadiusPx * (1.0f - cosHalf);
            const keel::f32 topY = rect.y + (rect.h - totalFanH) * 0.5f + 4.0f;
            g.origin = {rect.x + rect.w * 0.5f, topY + g.maxRadiusPx};
        } else {
            g.origin = {rect.x + 48.0f, rect.Bottom() - 28.0f};
            g.maxRadiusPx = keel::max(24.0f, rect.h - 48.0f);
        }
        return g;
    }

    [[nodiscard]] Rect GetBScanInnerRect() const noexcept {
        return Rect(
            plotRect.x + 50.0f,
            plotRect.y + 18.0f,
            keel::max(20.0f, plotRect.w - 66.0f),
            keel::max(20.0f, plotRect.h - 46.0f)
        );
    }

    [[nodiscard]] Vec2 PolarToScreen(keel::f32 rangeMeters, keel::f32 bearingDeg) const noexcept {
        const keel::f32 rNorm = keel::clamp(rangeMeters / keel::max(1.0f, visibleMaxRange), 0.0f, 1.5f);
        if (mode == SonarDisplayMode::PPI_360) {
            const keel::f32 rad = keel::deg_to_rad(bearingDeg);
            const keel::f32 rPx = rNorm * maxRadiusPx;
            return {origin.x + keel::sin(rad) * rPx, origin.y - keel::cos(rad) * rPx};
        } else if (mode == SonarDisplayMode::SectorWedge) {
            const keel::f32 relDeg = SignedAngleDiffDeg(bearingDeg, headingDeg);
            const keel::f32 rad = keel::deg_to_rad(relDeg);
            const keel::f32 rPx = rNorm * maxRadiusPx;
            return {origin.x + keel::sin(rad) * rPx, origin.y - keel::cos(rad) * rPx};
        } else {
            const Rect inner = GetBScanInnerRect();
            const keel::f32 halfAp = sectorApertureDeg * 0.5f;
            const keel::f32 relDeg = SignedAngleDiffDeg(bearingDeg, headingDeg);
            const keel::f32 u = keel::clamp((relDeg + halfAp) / keel::max(1.0f, sectorApertureDeg), 0.0f, 1.0f);
            const keel::f32 x = inner.x + u * inner.w;
            const keel::f32 y = inner.Bottom() - keel::clamp(rNorm, 0.0f, 1.0f) * inner.h;
            return {x, y};
        }
    }

    [[nodiscard]] bool ScreenToPolar(Vec2 pt, keel::f32& outRangeMeters, keel::f32& outBearingDeg) const noexcept {
        if (!plotRect.Contains(pt)) return false;
        if (mode == SonarDisplayMode::PPI_360) {
            const keel::f32 dx = pt.x - origin.x;
            const keel::f32 dy = origin.y - pt.y;
            const keel::f32 distPx = keel::sqrt(dx * dx + dy * dy);
            if (distPx > maxRadiusPx) return false;
            outRangeMeters = (distPx / maxRadiusPx) * visibleMaxRange;
            outBearingDeg = NormalizeBearingDeg(keel::rad_to_deg(keel::atan2(dx, dy)));
            return outRangeMeters >= minRange;
        } else if (mode == SonarDisplayMode::SectorWedge) {
            const keel::f32 dx = pt.x - origin.x;
            const keel::f32 dy = origin.y - pt.y;
            const keel::f32 distPx = keel::sqrt(dx * dx + dy * dy);
            if (distPx > maxRadiusPx) return false;
            const keel::f32 relDeg = keel::rad_to_deg(keel::atan2(dx, dy));
            if (keel::abs(relDeg) > sectorApertureDeg * 0.5f) return false;
            outRangeMeters = (distPx / maxRadiusPx) * visibleMaxRange;
            outBearingDeg = NormalizeBearingDeg(headingDeg + relDeg);
            return outRangeMeters >= minRange;
        } else {
            const Rect inner = GetBScanInnerRect();
            if (!inner.Contains(pt)) return false;
            const keel::f32 u = (pt.x - inner.x) / keel::max(1.0f, inner.w);
            const keel::f32 v = (inner.Bottom() - pt.y) / keel::max(1.0f, inner.h);
            outRangeMeters = v * visibleMaxRange;
            const keel::f32 relDeg = (u - 0.5f) * sectorApertureDeg;
            outBearingDeg = NormalizeBearingDeg(headingDeg + relDeg);
            return outRangeMeters >= minRange;
        }
    }
};

/// Samples `persistentGrid` with bilinear interpolation in `(beamIdx, rangeBin)` and applies Gain, TVG, and Threshold.
[[nodiscard]] keel::f32 SampleProcessedEcho(
    const SonarViewState& state,
    const SonarViewOptions& opts,
    keel::f32 rangeMeters,
    keel::f32 bearingDeg
) noexcept {
    if (state.persistentGrid.empty() || state.numBeams == 0 || state.numRangeBins == 0) {
        return 0.0f;
    }
    if (rangeMeters < opts.minRangeMeters || rangeMeters > opts.maxRangeMeters) {
        return 0.0f;
    }

    const keel::f32 rNorm = keel::clamp(rangeMeters / keel::max(1.0f, opts.maxRangeMeters), 0.0f, 1.0f);
    const keel::f32 spanDeg = opts.matrixEndBearingDeg - opts.matrixStartBearingDeg;
    keel::f32 beamNorm = 0.0f;
    if (keel::abs(spanDeg) >= 359.5f) {
        beamNorm = NormalizeBearingDeg(bearingDeg - opts.matrixStartBearingDeg) / 360.0f;
    } else {
        const keel::f32 rel = SignedAngleDiffDeg(bearingDeg, opts.headingDeg);
        const keel::f32 halfSpan = keel::max(1.0f, keel::abs(spanDeg) * 0.5f);
        if (keel::abs(rel) > halfSpan) return 0.0f;
        beamNorm = (rel + halfSpan) / (2.0f * halfSpan);
    }

    const keel::f32 bFloat = keel::clamp(beamNorm, 0.0f, 1.0f) * keel::as<keel::f32>(state.numBeams - 1);
    const keel::f32 rFloat = rNorm * keel::as<keel::f32>(state.numRangeBins - 1);

    const keel::u32 b0 = keel::as<keel::u32>(bFloat);
    const keel::u32 b1 = keel::min(state.numBeams - 1, b0 + 1);
    const keel::f32 fb = bFloat - keel::as<keel::f32>(b0);

    const keel::u32 r0 = keel::as<keel::u32>(rFloat);
    const keel::u32 r1 = keel::min(state.numRangeBins - 1, r0 + 1);
    const keel::f32 fr = rFloat - keel::as<keel::f32>(r0);

    const keel::f32 v00 = state.persistentGrid[b0 * state.numRangeBins + r0];
    const keel::f32 v01 = state.persistentGrid[b0 * state.numRangeBins + r1];
    const keel::f32 v10 = state.persistentGrid[b1 * state.numRangeBins + r0];
    const keel::f32 v11 = state.persistentGrid[b1 * state.numRangeBins + r1];

    const keel::f32 v0 = v00 + fr * (v01 - v00);
    const keel::f32 v1 = v10 + fr * (v11 - v10);
    keel::f32 raw = v0 + fb * (v1 - v0);

    // Apply Time-Varied Gain (TVG): compensates spherical spreading + absorption with distance
    const keel::f32 tvgBoost = 1.0f + opts.tvgAmount * (1.35f * rNorm + 0.55f * keel::sqrt(rNorm));
    // Apply Receiver Gain (dB -> linear amplitude multiplier)
    const keel::f32 gainLinear = keel::pow(10.0f, opts.gainDb / 28.0f);
    keel::f32 val = raw * tvgBoost * gainLinear;

    // Clutter rejection threshold
    const keel::f32 thresh = keel::clamp(opts.clutterThreshold, 0.0f, 0.85f);
    if (val <= thresh) {
        return 0.0f;
    }
    val = (val - thresh) / keel::max(0.05f, 1.0f - thresh);
    return keel::clamp(val, 0.0f, 1.0f);
}

void DrawCircleOutline(
    ICanvas& canvas,
    Vec2 center,
    keel::f32 radiusPx,
    Color fill,
    Color stroke = Color::Transparent(),
    keel::f32 strokeWidth = 0.0f
) {
    if (radiusPx <= 0.5f) return;
    canvas.DrawRoundRect(
        Rect(center.x - radiusPx, center.y - radiusPx, radiusPx * 2.0f, radiusPx * 2.0f),
        radiusPx,
        fill,
        stroke,
        strokeWidth
    );
}

void DrawClosedPolygon(
    ICanvas& canvas,
    keel::ArrayRef<Vec2> pts,
    Color fill,
    Color stroke,
    keel::f32 strokeWidth
) {
    if (pts.size() < 3) return;
    if (fill.a > 0) {
        for (keel::usize i = 1; i + 1 < pts.size(); ++i) {
            canvas.DrawTriangle(pts[0], pts[i], pts[i + 1], fill);
        }
    }
    if (stroke.a > 0 && strokeWidth > 0.0f) {
        for (keel::usize i = 0; i < pts.size(); ++i) {
            canvas.DrawLine(pts[i], pts[(i + 1) % pts.size()], stroke, strokeWidth);
        }
    }
}

void DrawArcPolyline(
    ICanvas& canvas,
    Vec2 center,
    keel::f32 radiusPx,
    keel::f32 startScreenDeg,
    keel::f32 endScreenDeg,
    Color color,
    keel::f32 thickness,
    keel::i32 /*segments*/ = 48
) {
    if (radiusPx <= 1.0f || color.a == 0u || thickness <= 0.0f) return;
    // Sonar screen deg has 0 deg at North (0, -1), which is -90 deg in standard +X screen coordinates
    canvas.DrawArc(
        center,
        radiusPx,
        keel::deg_to_rad(startScreenDeg - 90.0f),
        keel::deg_to_rad(endScreenDeg - 90.0f),
        color,
        thickness,
        /*roundCaps=*/false
    );
}

void DrawTargetSymbol(
    ICanvas& canvas,
    Vec2 pos,
    const SonarTarget& target,
    bool isSelected,
    bool isHovered,
    bool inGuardZone,
    const SonarPlotGeometry& geom
) {
    Color symColor{56, 189, 248, 240};
    switch (target.classification) {
        case SonarTargetClass::Diver:
            symColor = inGuardZone ? Color(244, 63, 94, 255) : Color(251, 146, 60, 255);
            break;
        case SonarTargetClass::UUV:
            symColor = Color(192, 132, 252, 255);
            break;
        case SonarTargetClass::SurfaceCraft:
            symColor = Color(52, 211, 153, 255);
            break;
        case SonarTargetClass::Biologic:
            symColor = Color(56, 189, 248, 220);
            break;
        case SonarTargetClass::StaticAnomaly:
            symColor = Color(250, 204, 21, 235);
            break;
    }

    // 1. Snail trail (historical positions)
    if (target.trail.size() >= 2) {
        for (keel::usize i = 0; i < target.trail.size(); ++i) {
            const keel::f32 alphaFrac = keel::as<keel::f32>(i + 1) / keel::as<keel::f32>(target.trail.size() + 1);
            const Vec2 tp = geom.PolarToScreen(target.trail[i].rangeMeters, target.trail[i].bearingDeg);
            if (geom.plotRect.Contains(tp)) {
                Color tc = symColor;
                tc.a = keel::as<keel::u8>(alphaFrac * 150.0f);
                DrawCircleOutline(canvas, tp, 2.2f, tc);
            }
        }
    }

    // 2. Velocity vector (Course Over Ground / Speed leader line)
    if (target.speedMetersPerSec > 0.05f && geom.mode != SonarDisplayMode::BScan) {
        const keel::f32 leaderLenPx = keel::clamp(target.speedMetersPerSec * 14.0f, 12.0f, 38.0f);
        keel::f32 courseScreenRad = keel::deg_to_rad(target.courseDeg);
        if (geom.mode == SonarDisplayMode::SectorWedge) {
            courseScreenRad = keel::deg_to_rad(SignedAngleDiffDeg(target.courseDeg, geom.headingDeg));
        }
        const Vec2 tip{
            pos.x + keel::sin(courseScreenRad) * leaderLenPx,
            pos.y - keel::cos(courseScreenRad) * leaderLenPx
        };
        canvas.DrawLine(pos, tip, symColor, 1.6f);
    }

    // 3. Tactical symbol geometry
    const keel::f32 r = isSelected ? 7.5f : (isHovered ? 6.8f : 5.8f);
    if (target.classification == SonarTargetClass::Diver) {
        const keel::StaticArray<Vec2, 4> diamond = {{
            {pos.x, pos.y - r},
            {pos.x + r, pos.y},
            {pos.x, pos.y + r},
            {pos.x - r, pos.y}
        }};
        Color fill = symColor;
        fill.a = inGuardZone ? 95 : 55;
        DrawClosedPolygon(canvas, diamond.as_ref(), fill, symColor, 1.8f);
        DrawCircleOutline(canvas, pos, 2.0f, symColor);
    } else if (target.classification == SonarTargetClass::UUV) {
        keel::StaticArray<Vec2, 6> hex{};
        for (keel::usize i = 0; i < 6; ++i) {
            const keel::f32 a = keel::deg_to_rad(keel::as<keel::f32>(i) * 60.0f - 30.0f);
            hex[i] = {pos.x + keel::cos(a) * r, pos.y + keel::sin(a) * r};
        }
        Color fill = symColor;
        fill.a = 55;
        DrawClosedPolygon(canvas, hex.as_ref(), fill, symColor, 1.7f);
        DrawCircleOutline(canvas, pos, 2.0f, symColor);
    } else if (target.classification == SonarTargetClass::StaticAnomaly) {
        Color fill = symColor;
        fill.a = 55;
        canvas.DrawTriangle(
            {pos.x, pos.y - r},
            {pos.x + r * 0.9f, pos.y + r * 0.75f},
            {pos.x - r * 0.9f, pos.y + r * 0.75f},
            fill,
            symColor,
            1.6f
        );
    } else {
        Color fill = symColor;
        fill.a = 50;
        DrawCircleOutline(canvas, pos, r * 0.85f, fill, symColor, 1.6f);
        DrawCircleOutline(canvas, pos, 1.8f, symColor);
    }

    // 4. Selection / lock-on corner brackets
    if (isSelected || isHovered) {
        const keel::f32 b = r + 4.5f;
        const keel::f32 leg = 4.5f;
        const Color lockCol = isSelected ? Color(250, 204, 21, 255) : Color(226, 232, 240, 210);
        canvas.DrawLine({pos.x - b, pos.y - b}, {pos.x - b + leg, pos.y - b}, lockCol, 1.6f);
        canvas.DrawLine({pos.x - b, pos.y - b}, {pos.x - b, pos.y - b + leg}, lockCol, 1.6f);
        canvas.DrawLine({pos.x + b, pos.y - b}, {pos.x + b - leg, pos.y - b}, lockCol, 1.6f);
        canvas.DrawLine({pos.x + b, pos.y - b}, {pos.x + b, pos.y - b + leg}, lockCol, 1.6f);
        canvas.DrawLine({pos.x - b, pos.y + b}, {pos.x - b + leg, pos.y + b}, lockCol, 1.6f);
        canvas.DrawLine({pos.x - b, pos.y + b}, {pos.x - b, pos.y + b - leg}, lockCol, 1.6f);
        canvas.DrawLine({pos.x + b, pos.y + b}, {pos.x + b - leg, pos.y + b}, lockCol, 1.6f);
        canvas.DrawLine({pos.x + b, pos.y + b}, {pos.x + b, pos.y + b - leg}, lockCol, 1.6f);
    }
}

void RenderSonarViewNode(const Node* node, ICanvas& canvas, const UiTheme& theme) noexcept {
    const auto* payload = node->As<SonarViewPayload>();
    if (!payload) return;

    const SonarViewOptions& opts = payload->opts;
    const SonarViewResponse& resp = payload->resp;

    const Rect r = node->computedRect;
    if (r.w <= 8.0f || r.h <= 8.0f) return;

    const keel::f32 radius = opts.border.radius.has_value()
        ? *opts.border.radius
        : theme.GetRadius(node->uiSize);
    const Color bgConsole = opts.background.Resolve(node->hovered, node->pressed, node->disabled, Color{6, 10, 18, 255});
    const Color borderCol = opts.border.color.Resolve(node->hovered, node->pressed, node->disabled, theme.palette.borderSubtle);
    const keel::f32 borderW = (opts.border.width > 0.0f) ? opts.border.width : 1.0f;
    const Color gridLineCol{56, 189, 248, 48};
    const Color gridMajorCol{56, 189, 248, 95};
    const Color labelTextCol{148, 163, 184, 225};
    const Color accentTextCol{56, 189, 248, 235};

    const keel::f32 colorbarW = opts.showColorbar ? 52.0f : 0.0f;
    const Rect plotRect(
        r.x + 2.0f,
        r.y + 2.0f,
        keel::max(16.0f, r.w - colorbarW - 4.0f),
        keel::max(16.0f, r.h - 4.0f)
    );
    const Rect colorbarRect(
        plotRect.Right() + 1.0f,
        r.y + 2.0f,
        keel::max(0.0f, colorbarW - 3.0f),
        keel::max(16.0f, r.h - 4.0f)
    );

    const SonarPlotGeometry geom = SonarPlotGeometry::Compute(plotRect, opts, resp.rangeZoom);

    static constexpr keel::StringRef kSonarFragGlsl = R"(
#ifdef GL_ES
precision highp float;
#endif
varying vec2 v_uv;

uniform vec4 u_PlotRect;
uniform vec2 u_GridDims;  // x=numRangeBins, y=numBeams
uniform vec3 u_BgColor;
uniform vec4 u_Params[8];
// u_Params[0]: x=origin.x, y=origin.y, z=maxRadiusPx, w=unused
// u_Params[1]: x=minRange, y=visibleMaxRange, z=matrixMaxRange, w=displayMode
// u_Params[2]: x=matrixStartBrg, y=matrixEndBrg, z=headingDeg, w=sectorApertureDeg
// u_Params[3]: x=bscanRect.x, y=bscanRect.y, z=bscanRect.w, w=bscanRect.h
// u_Params[4]: x=gainLinear, y=tvgAmount, z=clutterThreshold, w=unused
uniform sampler2D u_DataTex;
uniform sampler2D u_LutTex;

float normalizeBearingDeg(float deg) {
    float r = mod(deg, 360.0);
    return (r < 0.0) ? (r + 360.0) : r;
}

float signedAngleDiffDeg(float a, float b) {
    float d = mod(a - b + 540.0, 360.0) - 180.0;
    return d;
}

float decodeRG16(vec4 texel) {
    return (texel.r * 65280.0 + texel.g * 255.0) / 65535.0;
}

float samplePolarTexel(float rIdx, float bIdx) {
    vec2 uv = vec2((rIdx + 0.5) / u_GridDims.x, (bIdx + 0.5) / u_GridDims.y);
    return decodeRG16(texture2D(u_DataTex, uv));
}

void main() {
    vec2 pt = v_uv * u_PlotRect.zw;
    vec2 origin = u_Params[0].xy;
    float maxRadiusPx = u_Params[0].z;

    float minRange = u_Params[1].x;
    float visibleMaxRange = u_Params[1].y;
    float matrixMaxRange = u_Params[1].z;
    int mode = int(u_Params[1].w + 0.5);

    float matrixStartBrg = u_Params[2].x;
    float matrixEndBrg = u_Params[2].y;
    float headingDeg = u_Params[2].z;
    float sectorApertureDeg = u_Params[2].w;

    vec4 bscanRect = u_Params[3];
    vec4 dspParams = u_Params[4];

    float rangeMeters = 0.0;
    float bearingDeg = 0.0;
    bool validGeom = false;

    if (mode == 0) {
        float dx = pt.x - origin.x;
        float dy = origin.y - pt.y;
        float distPx = sqrt(dx * dx + dy * dy);
        if (distPx <= maxRadiusPx) {
            rangeMeters = (distPx / max(1.0, maxRadiusPx)) * visibleMaxRange;
            float rad = (abs(dx) < 1e-5 && abs(dy) < 1e-5) ? 0.0 : atan(dx, dy);
            bearingDeg = normalizeBearingDeg(rad * 57.2957795);
            validGeom = (rangeMeters >= minRange);
        }
    } else if (mode == 1) {
        float dx = pt.x - origin.x;
        float dy = origin.y - pt.y;
        float distPx = sqrt(dx * dx + dy * dy);
        if (distPx <= maxRadiusPx) {
            float rad = (abs(dx) < 1e-5 && abs(dy) < 1e-5) ? 0.0 : atan(dx, dy);
            float relDeg = rad * 57.2957795;
            if (abs(relDeg) <= sectorApertureDeg * 0.5) {
                rangeMeters = (distPx / max(1.0, maxRadiusPx)) * visibleMaxRange;
                bearingDeg = normalizeBearingDeg(headingDeg + relDeg);
                validGeom = (rangeMeters >= minRange);
            }
        }
    } else {
        if (pt.x >= bscanRect.x && pt.x <= (bscanRect.x + bscanRect.z) &&
            pt.y >= bscanRect.y && pt.y <= (bscanRect.y + bscanRect.w)) {
            float u = (pt.x - bscanRect.x) / max(1.0, bscanRect.z);
            float v = ((bscanRect.y + bscanRect.w) - pt.y) / max(1.0, bscanRect.w);
            rangeMeters = v * visibleMaxRange;
            float relDeg = (u - 0.5) * sectorApertureDeg;
            bearingDeg = normalizeBearingDeg(headingDeg + relDeg);
            validGeom = (rangeMeters >= minRange);
        }
    }

    if (!validGeom) {
        gl_FragColor = vec4(u_BgColor, 1.0);
        return;
    }

    float echo = 0.0;
    if (rangeMeters >= minRange && rangeMeters <= matrixMaxRange) {
        float rNorm = clamp(rangeMeters / max(1.0, matrixMaxRange), 0.0, 1.0);
        float spanDeg = matrixEndBrg - matrixStartBrg;
        float beamNorm = 0.0;
        bool inCoverage = true;
        if (abs(spanDeg) >= 359.5) {
            beamNorm = normalizeBearingDeg(bearingDeg - matrixStartBrg) / 360.0;
        } else {
            float rel = signedAngleDiffDeg(bearingDeg, headingDeg);
            float halfSpan = max(1.0, abs(spanDeg) * 0.5);
            if (abs(rel) > halfSpan) {
                inCoverage = false;
            } else {
                beamNorm = (rel + halfSpan) / (2.0 * halfSpan);
            }
        }

        if (inCoverage) {
            float bFloat = clamp(beamNorm, 0.0, 1.0) * max(0.0, u_GridDims.y - 1.0);
            float rFloat = rNorm * max(0.0, u_GridDims.x - 1.0);

            float b0 = floor(bFloat);
            float b1 = min(u_GridDims.y - 1.0, b0 + 1.0);
            float fb = bFloat - b0;

            float r0 = floor(rFloat);
            float r1 = min(u_GridDims.x - 1.0, r0 + 1.0);
            float fr = rFloat - r0;

            float v00 = samplePolarTexel(r0, b0);
            float v01 = samplePolarTexel(r1, b0);
            float v10 = samplePolarTexel(r0, b1);
            float v11 = samplePolarTexel(r1, b1);

            float v0 = v00 + fr * (v01 - v00);
            float v1 = v10 + fr * (v11 - v10);
            float raw = v0 + fb * (v1 - v0);

            float gainLinear = dspParams.x;
            float tvgAmount = dspParams.y;
            float thresh = clamp(dspParams.z, 0.0, 0.85);

            float tvgBoost = 1.0 + tvgAmount * (1.35 * rNorm + 0.55 * sqrt(rNorm));
            float val = raw * tvgBoost * gainLinear;
            if (val > thresh) {
                echo = clamp((val - thresh) / max(0.05, 1.0 - thresh), 0.0, 1.0);
            }
        }
    }

    vec4 mapped = texture2D(u_LutTex, vec2((echo * 255.0 + 0.5) / 256.0, 0.5));
    gl_FragColor = vec4(mapped.rgb, 1.0);
}
)";

    auto makeSonarGpuDesc = [&](SonarViewState& state) {
        const keel::f32 s = canvas.GetScale();
        const Rect bscan = geom.GetBScanInnerRect();
        const keel::f32 gainLinear = keel::pow(10.0f, opts.gainDb / 28.0f);

        ICanvas::GpuShaderPassDesc gpuDesc{};
        gpuDesc.widgetId = node->id.value;
        gpuDesc.shader = {
            .shaderId = "guikit.sonar.v1",
            .glslEs2 = kSonarFragGlsl
        };
        gpuDesc.plotRect = plotRect;
        gpuDesc.bgColor = bgConsole;
        gpuDesc.dataField = {state.persistentGrid, state.numBeams, state.numRangeBins};
        gpuDesc.fullGeneration = state.gridGeneration;
        gpuDesc.dataNormOffset = 0.0f;
        gpuDesc.dataNormScale = 1.0f;
        gpuDesc.colormapId = keel::as<keel::u8>(opts.colormap);
        gpuDesc.lutAbgr256 = GetColormapLutAbgr(opts.colormap);

        // Pass origin and B-Scan sub-rect relative to plotRect so the shader is invariant under PushTransform
        gpuDesc.params[0] = {
            (geom.origin.x - plotRect.x) * s,
            (geom.origin.y - plotRect.y) * s,
            geom.maxRadiusPx * s,
            0.0f
        };
        gpuDesc.params[1] = {geom.minRange, geom.visibleMaxRange, opts.maxRangeMeters, keel::as<keel::f32>(keel::to_underlying(geom.mode))};
        gpuDesc.params[2] = {opts.matrixStartBearingDeg, opts.matrixEndBearingDeg, geom.headingDeg, geom.sectorApertureDeg};
        gpuDesc.params[3] = {
            (bscan.x - plotRect.x) * s,
            (bscan.y - plotRect.y) * s,
            bscan.w * s,
            bscan.h * s
        };
        gpuDesc.params[4] = {gainLinear, opts.tvgAmount, opts.clutterThreshold, 0.0f};
        return gpuDesc;
    };

    const keel::u64 staticKey = detail::HashArgs(
        keel::round(r.x * 2.0f),
        keel::round(r.y * 2.0f),
        keel::round(r.w * 2.0f),
        keel::round(r.h * 2.0f),
        canvas.GetScale(),
        canvas.SupportsGpuShaders(),
        keel::as<keel::u8>(opts.displayMode),
        keel::as<keel::u8>(opts.colormap),
        opts.maxRangeMeters,
        opts.minRangeMeters,
        opts.headingDeg,
        opts.sectorApertureDeg,
        opts.rangeRingCount,
        opts.showRangeRings,
        opts.showBearingDial,
        opts.showColorbar,
        opts.clutterThreshold,
        opts.guardZone.enabled,
        opts.guardZone.minRangeMeters,
        opts.guardZone.maxRangeMeters,
        opts.guardZone.startBearingDeg,
        opts.guardZone.endBearingDeg,
        resp.guardZoneAlarm,
        resp.rangeZoom,
        bgConsole.ToUint32(),
        borderCol.ToUint32(),
        borderW,
        radius
    );

    bool restoredFromCache = false;
    if (payload->state && canvas.SupportsGpuShaders()) {
        SonarViewState& state = *payload->state;
        if (state.baseLayerValid && state.baseLayerKey == staticKey) {
            if (canvas.RestoreRegion(r, {state.baseLayerPixels.as_ref(), state.baseLayerW, state.baseLayerH})) {
                restoredFromCache = true;
                canvas.PushClip(plotRect);
                const auto gpuDesc = makeSonarGpuDesc(state);
                canvas.DrawGpuShaderPass(gpuDesc, false);
                canvas.PopClip();
            }
        }
    }

    if (!restoredFromCache) {
        if (payload->state) {
            payload->state->baseLayerValid = false;
        }
        // =====================================================================
        // PASS A: Static Base Layer (Console frame, GPU hole, Guard Zone, Range Rings, Dial, Colorbar)
        // =====================================================================
        canvas.DrawRoundRect(r, radius, bgConsole, borderCol, borderW);
        canvas.PushClip(plotRect);

        if (payload->state) {
            SonarViewState& state = *payload->state;
            const auto gpuDesc = makeSonarGpuDesc(state);
            if (!canvas.DrawGpuShaderPass(gpuDesc, true)) {
                const auto& lutAbgr = GetColormapLutAbgr(opts.colormap);
                const keel::f32 resScale = keel::min(
                    0.80f,
                    keel::min(
                        560.0f / keel::max(1.0f, plotRect.w),
                        460.0f / keel::max(1.0f, plotRect.h)
                    )
                );
                const keel::u32 imgW = keel::max(64u, keel::round_u32(plotRect.w * resScale));
                const keel::u32 imgH = keel::max(64u, keel::round_u32(plotRect.h * resScale));
                keel::usize pixelCount = 0;
                if (keel::checked_mul(keel::as<keel::usize>(imgW), keel::as<keel::usize>(imgH), pixelCount)) {
                    state.cachedImgW = imgW;
                    state.cachedImgH = imgH;
                    state.abgrPixels.resize(pixelCount, 0u);

                    const keel::u32 bgAbgr = PackAbgr(bgConsole);
                    const keel::f32 stepX = plotRect.w / keel::as<keel::f32>(imgW);
                    const keel::f32 stepY = plotRect.h / keel::as<keel::f32>(imgH);

                    for (keel::u32 py = 0; py < imgH; ++py) {
                        keel::u32* rowPtr = state.abgrPixels.data() + keel::as<keel::usize>(py) * imgW;
                        const keel::f32 sy = plotRect.y + (keel::as<keel::f32>(py) + 0.5f) * stepY;
                        for (keel::u32 px = 0; px < imgW; ++px) {
                            const keel::f32 sx = plotRect.x + (keel::as<keel::f32>(px) + 0.5f) * stepX;
                            keel::f32 rMeters = 0.0f;
                            keel::f32 brgDeg = 0.0f;
                            if (geom.ScreenToPolar({sx, sy}, rMeters, brgDeg)) {
                                const keel::f32 echo = SampleProcessedEcho(state, opts, rMeters, brgDeg);
                                const keel::usize idx = keel::as<keel::usize>(keel::clamp(echo * 255.0f, 0.0f, 255.0f));
                                rowPtr[px] = lutAbgr[idx];
                            } else {
                                rowPtr[px] = bgAbgr;
                            }
                        }
                    }

                    canvas.DrawPixels({state.abgrPixels, imgW, imgH}, plotRect);
                }
            }
        }

        // 2. Guard Zone tactical sector overlay
        if (opts.guardZone.enabled) {
            const keel::f32 gzMinR = keel::clamp(opts.guardZone.minRangeMeters, 0.0f, geom.visibleMaxRange);
            const keel::f32 gzMaxR = keel::clamp(opts.guardZone.maxRangeMeters, gzMinR, geom.visibleMaxRange);
            if (gzMaxR > gzMinR + 1.0f) {
                const Color gzStroke = resp.guardZoneAlarm
                    ? Color(244, 63, 94, 225)
                    : Color(34, 211, 238, 145);
                const Color gzFill = resp.guardZoneAlarm
                    ? Color(244, 63, 94, 38)
                    : Color(34, 211, 238, 20);

                if (geom.mode == SonarDisplayMode::PPI_360 || geom.mode == SonarDisplayMode::SectorWedge) {
                    keel::f32 span = keel::fmod(opts.guardZone.endBearingDeg - opts.guardZone.startBearingDeg + 360.0f, 360.0f);
                    if (span <= 0.01f) span = 360.0f;

                    constexpr keel::i32 kArcSteps = 28;
                    for (keel::i32 i = 0; i < kArcSteps; ++i) {
                        const keel::f32 b0 = opts.guardZone.startBearingDeg + (keel::as<keel::f32>(i) / kArcSteps) * span;
                        const keel::f32 b1 = opts.guardZone.startBearingDeg + (keel::as<keel::f32>(i + 1) / kArcSteps) * span;
                        const Vec2 outer0 = geom.PolarToScreen(gzMaxR, b0);
                        const Vec2 outer1 = geom.PolarToScreen(gzMaxR, b1);
                        const Vec2 inner0 = geom.PolarToScreen(gzMinR, b0);
                        const Vec2 inner1 = geom.PolarToScreen(gzMinR, b1);
                        canvas.DrawTriangle(inner0, outer0, outer1, gzFill);
                        canvas.DrawTriangle(inner0, outer1, inner1, gzFill);
                        canvas.DrawLine(outer0, outer1, gzStroke, resp.guardZoneAlarm ? 1.8f : 1.2f);
                        canvas.DrawLine(inner0, inner1, gzStroke, resp.guardZoneAlarm ? 1.8f : 1.2f);
                    }
                    canvas.DrawLine(
                        geom.PolarToScreen(gzMinR, opts.guardZone.startBearingDeg),
                        geom.PolarToScreen(gzMaxR, opts.guardZone.startBearingDeg),
                        gzStroke,
                        resp.guardZoneAlarm ? 1.8f : 1.2f
                    );
                    canvas.DrawLine(
                        geom.PolarToScreen(gzMinR, opts.guardZone.endBearingDeg),
                        geom.PolarToScreen(gzMaxR, opts.guardZone.endBearingDeg),
                        gzStroke,
                        resp.guardZoneAlarm ? 1.8f : 1.2f
                    );
                } else {
                    const Vec2 p0 = geom.PolarToScreen(gzMaxR, opts.guardZone.startBearingDeg);
                    const Vec2 p1 = geom.PolarToScreen(gzMinR, opts.guardZone.endBearingDeg);
                    const Rect gzRect(
                        keel::min(p0.x, p1.x),
                        keel::min(p0.y, p1.y),
                        keel::abs(p1.x - p0.x),
                        keel::abs(p1.y - p0.y)
                    );
                    canvas.DrawRoundRect(gzRect, 2.0f, gzFill, gzStroke, 1.4f);
                }
            }
        }

        // 3. Range Rings & Azimuth Spokes / Bearing Dial
        const keel::u32 ringCount = keel::clamp(opts.rangeRingCount, 2u, 10u);
        if (geom.mode == SonarDisplayMode::PPI_360) {
            if (opts.showRangeRings) {
                for (keel::u32 i = 1; i <= ringCount; ++i) {
                    const keel::f32 frac = keel::as<keel::f32>(i) / keel::as<keel::f32>(ringCount);
                    const keel::f32 rPx = frac * geom.maxRadiusPx;
                    const Color c = (i == ringCount) ? gridMajorCol : gridLineCol;
                    DrawCircleOutline(canvas, geom.origin, rPx, Color::Transparent(), c, (i == ringCount) ? 1.5f : 1.0f);

                    char rBuf[32];
                    const keel::StringRef rStr = keel::format_buf(rBuf, "%.0fm", frac * geom.visibleMaxRange);
                    canvas.DrawText(rStr, {geom.origin.x + 5.0f, geom.origin.y - rPx + 2.0f}, 10.0f, labelTextCol);
                }
            }

            if (opts.showBearingDial) {
                for (keel::i32 deg = 0; deg < 360; deg += 30) {
                    const keel::f32 rad = keel::deg_to_rad(keel::as<keel::f32>(deg));
                    const Vec2 inner{geom.origin.x + keel::sin(rad) * (geom.maxRadiusPx * 0.12f),
                                     geom.origin.y - keel::cos(rad) * (geom.maxRadiusPx * 0.12f)};
                    const Vec2 outer{geom.origin.x + keel::sin(rad) * geom.maxRadiusPx,
                                     geom.origin.y - keel::cos(rad) * geom.maxRadiusPx};
                    const bool cardinal = (deg % 90 == 0);
                    canvas.DrawLine(inner, outer, cardinal ? gridMajorCol : gridLineCol, cardinal ? 1.2f : 0.9f);
                }

                for (keel::i32 deg = 0; deg < 360; deg += 5) {
                    const keel::f32 rad = keel::deg_to_rad(keel::as<keel::f32>(deg));
                    const bool major = (deg % 30 == 0);
                    const bool med = (deg % 10 == 0);
                    const keel::f32 tickLen = major ? 7.0f : (med ? 4.5f : 2.5f);
                    const Vec2 p0{geom.origin.x + keel::sin(rad) * geom.maxRadiusPx,
                                  geom.origin.y - keel::cos(rad) * geom.maxRadiusPx};
                    const Vec2 p1{geom.origin.x + keel::sin(rad) * (geom.maxRadiusPx + tickLen),
                                  geom.origin.y - keel::cos(rad) * (geom.maxRadiusPx + tickLen)};
                    canvas.DrawLine(p0, p1, major ? accentTextCol : labelTextCol, major ? 1.4f : 1.0f);

                    if (major) {
                        char bBuf[16];
                        keel::StringRef bStr;
                        if (deg == 0) bStr = "000 N";
                        else if (deg == 90) bStr = "090 E";
                        else if (deg == 180) bStr = "180 S";
                        else if (deg == 270) bStr = "270 W";
                        else bStr = keel::format_buf(bBuf, "%03d", deg);

                        const keel::f32 labelR = geom.maxRadiusPx + 15.0f;
                        const Vec2 lp{geom.origin.x + keel::sin(rad) * labelR - 14.0f,
                                      geom.origin.y - keel::cos(rad) * labelR - 6.0f};
                        canvas.DrawText(bStr, lp, 10.5f, (deg % 90 == 0) ? accentTextCol : labelTextCol);
                    }
                }
            }
        } else if (geom.mode == SonarDisplayMode::SectorWedge) {
            const keel::f32 halfAp = geom.sectorApertureDeg * 0.5f;
            if (opts.showRangeRings) {
                for (keel::u32 i = 1; i <= ringCount; ++i) {
                    const keel::f32 frac = keel::as<keel::f32>(i) / keel::as<keel::f32>(ringCount);
                    const keel::f32 rPx = frac * geom.maxRadiusPx;
                    const bool outer = (i == ringCount);
                    DrawArcPolyline(
                        canvas,
                        geom.origin,
                        rPx,
                        -halfAp,
                        +halfAp,
                        outer ? gridMajorCol : gridLineCol,
                        outer ? 1.6f : 1.0f,
                        48
                    );

                    const keel::f32 rightRad = keel::deg_to_rad(halfAp);
                    const Vec2 rp{
                        geom.origin.x + keel::sin(rightRad) * rPx + 6.0f,
                        geom.origin.y - keel::cos(rightRad) * rPx - 6.0f
                    };
                    char rBuf[32];
                    const keel::StringRef rStr = keel::format_buf(rBuf, "%.0fm", frac * geom.visibleMaxRange);
                    canvas.DrawText(rStr, rp, 10.0f, labelTextCol);
                }
            }

            const Vec2 leftEdge = geom.PolarToScreen(geom.visibleMaxRange, geom.headingDeg - halfAp);
            const Vec2 rightEdge = geom.PolarToScreen(geom.visibleMaxRange, geom.headingDeg + halfAp);
            canvas.DrawLine(geom.origin, leftEdge, gridMajorCol, 1.5f);
            canvas.DrawLine(geom.origin, rightEdge, gridMajorCol, 1.5f);

            if (opts.showBearingDial) {
                const keel::i32 stepDeg = (geom.sectorApertureDeg <= 90.0f) ? 15 : 30;
                for (keel::f32 rel = -halfAp; rel <= halfAp + 0.1f; rel += keel::as<keel::f32>(stepDeg)) {
                    const keel::f32 brg = NormalizeBearingDeg(geom.headingDeg + rel);
                    const Vec2 pOuter = geom.PolarToScreen(geom.visibleMaxRange, brg);
                    const bool centerRay = (keel::abs(rel) < 0.5f);
                    canvas.DrawLine(geom.origin, pOuter, centerRay ? gridMajorCol : gridLineCol, centerRay ? 1.3f : 0.9f);

                    const keel::f32 rad = keel::deg_to_rad(rel);
                    const Vec2 lp{
                        geom.origin.x + keel::sin(rad) * (geom.maxRadiusPx + 14.0f) - 12.0f,
                        geom.origin.y - keel::cos(rad) * (geom.maxRadiusPx + 14.0f) - 7.0f
                    };
                    char bBuf[16];
                    const keel::StringRef bStr = keel::format_buf(bBuf, "%03.0f°", brg);
                    canvas.DrawText(bStr, lp, 10.0f, centerRay ? accentTextCol : labelTextCol);
                }
            }
        } else {
            const Rect inner = geom.GetBScanInnerRect();
            canvas.DrawRoundRect(inner, 0.0f, Color::Transparent(), gridMajorCol, 1.2f);
            if (opts.showRangeRings) {
                for (keel::u32 i = 1; i <= ringCount; ++i) {
                    const keel::f32 frac = keel::as<keel::f32>(i) / keel::as<keel::f32>(ringCount);
                    const keel::f32 y = inner.Bottom() - frac * inner.h;
                    canvas.DrawLine({inner.x, y}, {inner.Right(), y}, gridLineCol, 1.0f);
                    char rBuf[32];
                    const keel::StringRef rStr = keel::format_buf(rBuf, "%.0fm", frac * geom.visibleMaxRange);
                    canvas.DrawText(rStr, {plotRect.x + 6.0f, y - 6.0f}, 10.0f, labelTextCol);
                }
            }
            if (opts.showBearingDial) {
                const keel::f32 halfAp = geom.sectorApertureDeg * 0.5f;
                for (keel::i32 i = 0; i <= 6; ++i) {
                    const keel::f32 u = keel::as<keel::f32>(i) / 6.0f;
                    const keel::f32 x = inner.x + u * inner.w;
                    canvas.DrawLine({x, inner.y}, {x, inner.Bottom()}, gridLineCol, 1.0f);
                    const keel::f32 brg = NormalizeBearingDeg(geom.headingDeg + (u - 0.5f) * 2.0f * halfAp);
                    char bBuf[16];
                    const keel::StringRef bStr = keel::format_buf(bBuf, "%03.0f°", brg);
                    canvas.DrawText(bStr, {x - 12.0f, inner.Bottom() + 4.0f}, 10.0f, labelTextCol);
                }
            }
        }

        canvas.PopClip();

        // Right-hand Acoustic Backscatter dB Colorbar
        if (opts.showColorbar && colorbarRect.w >= 20.0f) {
            canvas.PushClip(colorbarRect);
            canvas.DrawRect(colorbarRect, Color(9, 13, 24, 255));
            canvas.DrawLine({colorbarRect.x, colorbarRect.y}, {colorbarRect.x, colorbarRect.Bottom()}, borderCol, 1.0f);

            const Rect barStrip(colorbarRect.x + 8.0f, colorbarRect.y + 24.0f, 12.0f, keel::max(24.0f, colorbarRect.h - 48.0f));
            DrawColormapBar(canvas, barStrip, opts.colormap, true, false, 2.0f, Color(148, 163, 184, 120));

            const keel::f32 threshY = barStrip.Bottom() - keel::clamp(opts.clutterThreshold, 0.0f, 0.85f) * barStrip.h;
            canvas.DrawLine({barStrip.x - 3.0f, threshY}, {barStrip.Right() + 3.0f, threshY}, Color(244, 63, 94, 230), 1.8f);

            canvas.DrawText("dB", {colorbarRect.x + 8.0f, colorbarRect.y + 6.0f}, 10.0f, accentTextCol);
            for (keel::i32 i = 0; i <= 4; ++i) {
                const keel::f32 frac = keel::as<keel::f32>(i) / 4.0f;
                const keel::f32 y = barStrip.y + frac * barStrip.h;
                const keel::i32 dbVal = -i * 15;
                char dbBuf[16];
                const keel::StringRef dbStr = keel::format_buf(dbBuf, "%d", dbVal);
                canvas.DrawText(dbStr, {barStrip.Right() + 4.0f, y - 5.0f}, 9.5f, labelTextCol);
            }
            canvas.PopClip();
        }

        if (payload->state && canvas.SupportsGpuShaders()) {
            SonarViewState& state = *payload->state;
            if (canvas.CaptureRegion(r, state.baseLayerPixels, state.baseLayerW, state.baseLayerH)) {
                state.baseLayerKey = staticKey;
                state.baseLayerValid = true;
            }
        }
    }

    // =====================================================================
    // PASS B: Dynamic Tactical Overlay (Sweep Line, EBL/VRM, Contacts, Crosshair, HUD)
    // =====================================================================
    canvas.PushClip(plotRect);

    // 4. Active Rotating Sweep Line
    if (opts.scanMode == SonarScanMode::RotatingSweep) {
        const Vec2 sweepTip = geom.PolarToScreen(geom.visibleMaxRange, opts.sweepAngleDeg);
        const Color sweepCol = (opts.colormap == Colormap::Green)
            ? Color(74, 222, 128, 235)
            : Color(56, 189, 248, 230);
        if (geom.mode != SonarDisplayMode::BScan) {
            canvas.DrawLine(geom.origin, sweepTip, sweepCol, 2.0f);
        } else {
            const Rect inner = geom.GetBScanInnerRect();
            canvas.DrawLine({sweepTip.x, inner.y}, {sweepTip.x, inner.Bottom()}, sweepCol, 2.0f);
        }
    }

    // 5. Electronic Bearing Line (EBL) & Variable Range Marker (VRM)
    if (opts.showEblVrm) {
        const Color eblCol{250, 204, 21, 185};
        const Color vrmCol{56, 189, 248, 185};
        const keel::f32 vrmR = keel::clamp(resp.vrmRangeMeters, 0.0f, geom.visibleMaxRange);

        if (geom.mode == SonarDisplayMode::PPI_360) {
            const Vec2 eblEnd = geom.PolarToScreen(geom.visibleMaxRange, resp.eblBearingDeg);
            canvas.DrawLine(geom.origin, eblEnd, eblCol, 1.3f);
            const keel::f32 vrmPx = (vrmR / geom.visibleMaxRange) * geom.maxRadiusPx;
            DrawCircleOutline(canvas, geom.origin, vrmPx, Color::Transparent(), vrmCol, 1.3f);
        } else if (geom.mode == SonarDisplayMode::SectorWedge) {
            const keel::f32 halfAp = geom.sectorApertureDeg * 0.5f;
            if (keel::abs(SignedAngleDiffDeg(resp.eblBearingDeg, geom.headingDeg)) <= halfAp) {
                const Vec2 eblEnd = geom.PolarToScreen(geom.visibleMaxRange, resp.eblBearingDeg);
                canvas.DrawLine(geom.origin, eblEnd, eblCol, 1.3f);
            }
            const keel::f32 vrmPx = (vrmR / geom.visibleMaxRange) * geom.maxRadiusPx;
            DrawArcPolyline(canvas, geom.origin, vrmPx, -halfAp, +halfAp, vrmCol, 1.3f, 40);
        } else {
            const Rect inner = geom.GetBScanInnerRect();
            const Vec2 isect = geom.PolarToScreen(vrmR, resp.eblBearingDeg);
            canvas.DrawLine({isect.x, inner.y}, {isect.x, inner.Bottom()}, eblCol, 1.3f);
            canvas.DrawLine({inner.x, isect.y}, {inner.Right(), isect.y}, vrmCol, 1.3f);
        }

        const Vec2 markerPt = geom.PolarToScreen(vrmR, resp.eblBearingDeg);
        if (plotRect.Contains(markerPt)) {
            DrawCircleOutline(canvas, markerPt, 4.0f, Color(250, 204, 21, 230));
        }
    }

    // 6. Tracked Contacts / Diver Targets
    if (opts.showTargets) {
        for (const auto& t : opts.targets) {
            if (t.rangeMeters > geom.visibleMaxRange) continue;
            if (geom.mode != SonarDisplayMode::PPI_360) {
                if (keel::abs(SignedAngleDiffDeg(t.bearingDeg, geom.headingDeg)) > geom.sectorApertureDeg * 0.5f) {
                    continue;
                }
            }
            const Vec2 pos = geom.PolarToScreen(t.rangeMeters, t.bearingDeg);
            if (!plotRect.Contains(pos)) continue;

            const bool isSel = (t.id == resp.selectedTargetId);
            const bool isHov = (t.id == resp.hoveredTargetId);
            const bool inGz = opts.guardZone.enabled &&
                              t.rangeMeters >= opts.guardZone.minRangeMeters &&
                              t.rangeMeters <= opts.guardZone.maxRangeMeters &&
                              IsBearingInSector(t.bearingDeg, opts.guardZone.startBearingDeg, opts.guardZone.endBearingDeg);

            DrawTargetSymbol(canvas, pos, t, isSel, isHov, inGz, geom);

            if (opts.showTargetLabels) {
                char tagBuf[96];
                const keel::StringRef tagStr = keel::format_buf(
                    tagBuf, "%s  %.0fm %03.0f°",
                    t.callsign, t.rangeMeters, NormalizeBearingDeg(t.bearingDeg)
                );
                const Color tagTextCol = isSel ? Color(250, 204, 21, 255) : Color(226, 232, 240, 235);
                canvas.DrawText(tagStr, {pos.x + 10.0f, pos.y - 6.0f}, 10.5f, tagTextCol);
            }
        }
    }

    // 7. Live Cursor Crosshair
    if (resp.cursorInsidePlot) {
        const Vec2 cp = geom.PolarToScreen(resp.cursorRangeMeters, resp.cursorBearingDeg);
        DrawCircleOutline(canvas, cp, 4.5f, Color::Transparent(), Color(255, 255, 255, 190), 1.2f);
    }

    // 8. Corner Tactical HUD Overlays
    if (opts.showHudOverlay) {
        const Rect tlCard(plotRect.x + 10.0f, plotRect.y + 10.0f, 204.0f, 58.0f);
        canvas.DrawRoundRect(tlCard, 6.0f, Color(11, 17, 30, 215), Color(56, 189, 248, 80), 1.0f);

        const keel::StringRef modeTitle = (opts.displayMode == SonarDisplayMode::PPI_360)
            ? keel::StringRef("PPI 360° OMNI SONAR")
            : (opts.displayMode == SonarDisplayMode::SectorWedge
                ? keel::StringRef("DDS MULTIBEAM SECTOR")
                : keel::StringRef("B-SCAN AZIMUTH MATRIX"));
        canvas.DrawText(modeTitle, {tlCard.x + 10.0f, tlCard.y + 6.0f}, 10.5f, accentTextCol);

        char rngLine[64];
        const keel::StringRef rngStr = keel::format_buf(
            rngLine, "SCALE: %.0f m   BINS: %ux%u",
            geom.visibleMaxRange, opts.numBeams, opts.numRangeBins
        );
        canvas.DrawText(rngStr, {tlCard.x + 10.0f, tlCard.y + 23.0f}, 10.0f, Color(226, 232, 240, 230));

        char dspLine[64];
        const keel::StringRef dspStr = keel::format_buf(
            dspLine, "GAIN: %+.1f dB   TVG: %.0f%%",
            opts.gainDb, opts.tvgAmount * 100.0f
        );
        canvas.DrawText(dspStr, {tlCard.x + 10.0f, tlCard.y + 39.0f}, 10.0f, labelTextCol);

        const Rect trCard(plotRect.Right() - 228.0f, plotRect.y + 10.0f, 218.0f, 76.0f);
        const Color trBorder = resp.guardZoneAlarm ? Color(244, 63, 94, 215) : Color(56, 189, 248, 80);
        canvas.DrawRoundRect(trCard, 6.0f, Color(11, 17, 30, 220), trBorder, 1.2f);

        if (resp.guardZoneAlarm) {
            char alarmBuf[64];
            const keel::StringRef alarmStr = keel::format_buf(alarmBuf, "ALARM: PERIMETER BREACH (%u)", resp.guardZoneIntruderCount);
            canvas.DrawText(alarmStr, {trCard.x + 10.0f, trCard.y + 6.0f}, 10.5f, Color(244, 63, 94, 255));
        } else {
            canvas.DrawText("GUARD ZONE: SECURE", {trCard.x + 10.0f, trCard.y + 6.0f}, 10.5f, Color(52, 211, 153, 240));
        }

        const SonarTarget* selTarget = nullptr;
        for (const auto& t : opts.targets) {
            if (t.id == resp.selectedTargetId) {
                selTarget = &t;
                break;
            }
        }
        if (selTarget) {
            char tLine1[96];
            const keel::StringRef tStr1 = keel::format_buf(
                tLine1, "LOCK: %s  (SNR +%.1fdB)",
                selTarget->callsign, selTarget->snrDb
            );
            canvas.DrawText(tStr1, {trCard.x + 10.0f, trCard.y + 24.0f}, 10.0f, Color(250, 204, 21, 255));

            char tLine2[64];
            const keel::StringRef tStr2 = keel::format_buf(
                tLine2, "RNG %.1fm | BRG %05.1f°",
                selTarget->rangeMeters, NormalizeBearingDeg(selTarget->bearingDeg)
            );
            canvas.DrawText(tStr2, {trCard.x + 10.0f, trCard.y + 40.0f}, 10.0f, Color(226, 232, 240, 235));

            char tLine3[64];
            const keel::StringRef tStr3 = keel::format_buf(
                tLine3, "SPD %.2fm/s | COG %03.0f° | Z -%.1fm",
                selTarget->speedMetersPerSec, selTarget->courseDeg, selTarget->depthMeters
            );
            canvas.DrawText(tStr3, {trCard.x + 10.0f, trCard.y + 56.0f}, 9.5f, labelTextCol);
        } else {
            canvas.DrawText("NO TARGET LOCKED", {trCard.x + 10.0f, trCard.y + 28.0f}, 10.0f, labelTextCol);
            canvas.DrawText("Click contact reticle to track", {trCard.x + 10.0f, trCard.y + 46.0f}, 9.5f, labelTextCol);
        }

        const Rect blCard(plotRect.x + 10.0f, plotRect.Bottom() - 44.0f, 244.0f, 36.0f);
        canvas.DrawRoundRect(blCard, 6.0f, Color(11, 17, 30, 215), Color(56, 189, 248, 70), 1.0f);
        char eblLine[80];
        const keel::StringRef eblStr = keel::format_buf(
            eblLine, "EBL: %05.1f°   VRM: %5.1f m",
            NormalizeBearingDeg(resp.eblBearingDeg), resp.vrmRangeMeters
        );
        canvas.DrawText(eblStr, {blCard.x + 10.0f, blCard.y + 4.0f}, 10.0f, Color(250, 204, 21, 240));

        char curLine[80];
        const keel::StringRef curStr = resp.cursorInsidePlot
            ? keel::format_buf(curLine, "CUR: %5.1f m @ %05.1f°  (%+.1f dB)",
                               resp.cursorRangeMeters, NormalizeBearingDeg(resp.cursorBearingDeg), resp.cursorEchoDb)
            : keel::StringRef("CUR: ---.- m @ ---.-°  (Shift+Drag EBL)");
        canvas.DrawText(curStr, {blCard.x + 10.0f, blCard.y + 19.0f}, 9.5f, labelTextCol);
    }

    canvas.PopClip();
}

} // namespace

Vec2 SonarView::Measure(
    keel::ArrayRef<keel::f32> /*polarMatrix*/,
    const SonarViewOptions& opts,
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

SonarViewResponse SonarView::Run(
    Context& ctx,
    keel::ArrayRef<keel::f32> polarMatrix,
    const SonarViewOptions& opts,
    keel::SourceLocation loc
) {

    const Id id = ctx.MakeChildId(loc);
    const bool isDisabled = opts.disabled || ctx.IsDisabled();
    const UiSize effectiveSize = opts.size;
    ICanvas* canvas = ctx.GetCanvas();

    Response baseRes = ctx.Interact(id);
    SonarViewResponse resp{};
    static_cast<Response&>(resp) = baseRes;

    Rect prevBounds = baseRes.rect;
    if (prevBounds.w <= 1.0f || prevBounds.h <= 1.0f) {
        prevBounds = ctx.GetParentPrevFrameBounds();
    }
    const Rect activeRect = prevBounds;

    auto& state = ctx.GetOrCreatePersistentState<SonarViewState>(id);

    const keel::u32 numBeams = keel::max(8u, opts.numBeams);
    const keel::u32 numBins = keel::max(8u, opts.numRangeBins);
    const keel::usize totalCells = keel::as<keel::usize>(numBeams) * numBins;

    if (!state.initialized || state.numBeams != numBeams || state.numRangeBins != numBins || opts.requestClearPersistence) {
        state.numBeams = numBeams;
        state.numRangeBins = numBins;
        state.persistentGrid.assign(totalCells, 0.0f);
        state.prevSweepAngleDeg = opts.sweepAngleDeg;
        state.gridGeneration++;
        if (!state.initialized) {
            state.rangeZoom = opts.viewRangeZoom ? keel::clamp(*opts.viewRangeZoom, 0.2f, 1.0f) : 1.0f;
            state.eblBearingDeg = opts.eblBearingDeg ? *opts.eblBearingDeg : 35.0f;
            state.vrmRangeMeters = opts.vrmRangeMeters ? *opts.vrmRangeMeters : (opts.maxRangeMeters * 0.48f);
            state.selectedTargetId = opts.selectedTargetId ? *opts.selectedTargetId : -1;
            state.initialized = true;
        }
    }

    if (opts.requestResetView) {
        state.rangeZoom = 1.0f;
        if (opts.viewRangeZoom) *opts.viewRangeZoom = 1.0f;
    }
    if (opts.viewRangeZoom) {
        state.rangeZoom = keel::clamp(*opts.viewRangeZoom, 0.2f, 1.0f);
    }
    if (opts.eblBearingDeg) {
        state.eblBearingDeg = *opts.eblBearingDeg;
    }
    if (opts.vrmRangeMeters) {
        state.vrmRangeMeters = *opts.vrmRangeMeters;
    }
    if (opts.selectedTargetId) {
        state.selectedTargetId = *opts.selectedTargetId;
    }

    // 1. Update Polar Persistence Buffer from `polarMatrix`
    const keel::f32 persist = keel::clamp(opts.persistence, 0.0f, 0.97f);
    if (polarMatrix.size() >= totalCells) {
        keel::u64 inHash = detail::HashArgs(opts.sweepAngleDeg, keel::as<keel::u8>(opts.scanMode), persist);
        const keel::usize step = keel::max<keel::usize>(1, totalCells / 64);
        for (keel::usize i = 0; i < totalCells; i += step) {
            inHash = detail::HashArgs(inHash, polarMatrix[i]);
        }
        if (inHash != state.lastInputHash || state.gridGeneration <= 1) {
            state.lastInputHash = inHash;
            state.gridGeneration++;
            if (opts.scanMode == SonarScanMode::MultibeamContinuous) {
                for (keel::usize i = 0; i < totalCells; ++i) {
                    const keel::f32 incoming = keel::clamp(polarMatrix[i], 0.0f, 1.0f);
                    const keel::f32 decayed = state.persistentGrid[i] * persist;
                    state.persistentGrid[i] = keel::max(incoming, decayed * 0.92f + incoming * 0.08f);
                }
            } else {
                const keel::f32 frameDecay = 0.965f + 0.032f * persist;
                const keel::f32 spanDeg = opts.matrixEndBearingDeg - opts.matrixStartBearingDeg;
                const keel::f32 halfSweep = keel::max(3.0f, opts.sweepBeamWidthDeg * 0.5f);
                for (keel::u32 b = 0; b < numBeams; ++b) {
                    const keel::f32 beamNorm = keel::as<keel::f32>(b) / keel::as<keel::f32>(keel::max(1u, numBeams - 1));
                    keel::f32 beamBrg = 0.0f;
                    if (keel::abs(spanDeg) >= 359.5f) {
                        beamBrg = NormalizeBearingDeg(opts.matrixStartBearingDeg + beamNorm * 360.0f);
                    } else {
                        beamBrg = NormalizeBearingDeg(opts.headingDeg + (beamNorm - 0.5f) * spanDeg);
                    }
                    const keel::f32 diff = keel::abs(SignedAngleDiffDeg(beamBrg, opts.sweepAngleDeg));
                    const bool inBeam = (diff <= halfSweep);
                    keel::f32* row = state.persistentGrid.data() + keel::as<keel::usize>(b) * numBins;
                    const keel::f32* srcRow = polarMatrix.data() + keel::as<keel::usize>(b) * numBins;
                    for (keel::u32 r = 0; r < numBins; ++r) {
                        keel::f32 v = row[r] * frameDecay;
                        if (inBeam) {
                            const keel::f32 beamWeight = 1.0f - (diff / (halfSweep + 0.1f)) * 0.35f;
                            v = keel::max(v, keel::clamp(srcRow[r] * beamWeight, 0.0f, 1.0f));
                        }
                        row[r] = v;
                    }
                }
            }
        }
    }
    state.prevSweepAngleDeg = opts.sweepAngleDeg;

    // 2. Interactive Input Handling
    const keel::f32 colorbarW = opts.showColorbar ? 52.0f : 0.0f;
    const Rect plotRect(
        activeRect.x + 2.0f,
        activeRect.y + 2.0f,
        keel::max(16.0f, activeRect.w - colorbarW - 4.0f),
        keel::max(16.0f, activeRect.h - 4.0f)
    );
    SonarPlotGeometry geom = SonarPlotGeometry::Compute(plotRect, opts, state.rangeZoom);

    const Vec2 mousePos = ctx.GetMousePos();
    const bool hovered = !isDisabled && ctx.IsRectHovered(id, activeRect);
    resp.hovered = hovered;

    if (hovered && plotRect.Contains(mousePos)) {
        const Vec2 wheel = ctx.GetMouseWheel();
        if (keel::abs(wheel.y) > 0.01f) {
            const keel::f32 factor = (wheel.y > 0.0f) ? 0.88f : 1.14f;
            state.rangeZoom = keel::clamp(state.rangeZoom * factor, 0.2f, 1.0f);
            if (opts.viewRangeZoom) *opts.viewRangeZoom = state.rangeZoom;
            geom = SonarPlotGeometry::Compute(plotRect, opts, state.rangeZoom);
            ctx.ConsumeMouseWheel();
            ctx.RequestRepaint(2);
            resp.changed = true;
        }

        keel::f32 curR = 0.0f;
        keel::f32 curBrg = 0.0f;
        if (geom.ScreenToPolar(mousePos, curR, curBrg)) {
            resp.cursorInsidePlot = true;
            resp.cursorRangeMeters = curR;
            resp.cursorBearingDeg = curBrg;
            const keel::f32 echoNorm = SampleProcessedEcho(state, opts, curR, curBrg);
            resp.cursorEchoDb = -60.0f + echoNorm * 60.0f;
        }

        keel::f32 bestDistSq = 18.0f * 18.0f;
        for (const auto& t : opts.targets) {
            if (t.rangeMeters > geom.visibleMaxRange) continue;
            const Vec2 tp = geom.PolarToScreen(t.rangeMeters, t.bearingDeg);
            const keel::f32 dx = mousePos.x - tp.x;
            const keel::f32 dy = mousePos.y - tp.y;
            const keel::f32 d2 = dx * dx + dy * dy;
            if (d2 <= bestDistSq) {
                bestDistSq = d2;
                resp.hoveredTargetId = t.id;
            }
        }

        const Vec2 eblMarkerPt = geom.PolarToScreen(state.vrmRangeMeters, state.eblBearingDeg);
        const keel::f32 eblDistSq = (mousePos.x - eblMarkerPt.x) * (mousePos.x - eblMarkerPt.x) +
                                (mousePos.y - eblMarkerPt.y) * (mousePos.y - eblMarkerPt.y);

        if (ctx.IsMouseClicked()) {
            if (resp.hoveredTargetId != -1) {
                state.selectedTargetId = resp.hoveredTargetId;
                if (opts.selectedTargetId) *opts.selectedTargetId = state.selectedTargetId;
                resp.targetSelectionChanged = true;
                resp.clicked = true;
                resp.changed = true;
            } else if (opts.showEblVrm && (ctx.GetKeyModifiers().shift || eblDistSq <= 16.0f * 16.0f)) {
                state.eblVrmDragging = true;
                ctx.SetActiveId(id);
            }
        }
    }

    if (state.eblVrmDragging) {
        if (ctx.IsMouseDown()) {
            keel::f32 curR = 0.0f;
            keel::f32 curBrg = 0.0f;
            if (geom.ScreenToPolar(mousePos, curR, curBrg)) {
                state.vrmRangeMeters = keel::clamp(curR, 5.0f, geom.visibleMaxRange);
                state.eblBearingDeg = curBrg;
                if (opts.vrmRangeMeters) *opts.vrmRangeMeters = state.vrmRangeMeters;
                if (opts.eblBearingDeg) *opts.eblBearingDeg = state.eblBearingDeg;
                resp.changed = true;
                ctx.RequestRepaint(2);
            }
        } else {
            state.eblVrmDragging = false;
            if (ctx.GetActiveId() == id) ctx.SetActiveId(Id::Null());
        }
    }

    // 3. Evaluate Guard Zone Intrusion Alarm
    if (opts.guardZone.enabled) {
        for (const auto& t : opts.targets) {
            if (t.classification == SonarTargetClass::Diver || t.classification == SonarTargetClass::UUV) {
                if (t.rangeMeters >= opts.guardZone.minRangeMeters &&
                    t.rangeMeters <= opts.guardZone.maxRangeMeters &&
                    IsBearingInSector(t.bearingDeg, opts.guardZone.startBearingDeg, opts.guardZone.endBearingDeg)) {
                    resp.guardZoneAlarm = true;
                    resp.guardZoneIntruderCount++;
                }
            }
        }
    }

    resp.displayMode = opts.displayMode;
    resp.visibleMaxRangeMeters = geom.visibleMaxRange;
    resp.rangeZoom = state.rangeZoom;
    resp.eblVrmActive = opts.showEblVrm;
    resp.eblBearingDeg = state.eblBearingDeg;
    resp.vrmRangeMeters = state.vrmRangeMeters;
    resp.selectedTargetId = state.selectedTargetId;

    if (hovered && !opts.tooltip.empty()) {
        ctx.SetTooltip(opts.tooltip, baseRes.rect);
    }

    state.renderedTargets.clear();
    state.renderedTargets.reserve(opts.targets.size());
    keel::u64 targetsFp = detail::HashArgs(opts.targets.size(), resp.hoveredTargetId);
    for (const auto& t : opts.targets) {
        state.renderedTargets.push_back(t.clone());
        targetsFp = detail::HashArgs(
            targetsFp,
            t.id,
            t.callsign,
            keel::as<keel::u8>(t.classification),
            t.rangeMeters,
            t.bearingDeg,
            t.speedMetersPerSec,
            t.courseDeg,
            t.trail.size()
        );
    }

    Node* node = ctx.CreateNode(keel::from_underlying<NodeType>(keel::as<keel::u16>(NodeType::User) + 2u), id);
    auto* payload = ctx.AllocPayload<SonarViewPayload>();
    payload->opts = opts;
    payload->opts.targets = state.renderedTargets.as_ref();
    payload->opts.tooltip = ctx.AllocText(opts.tooltip);
    payload->resp = resp;
    payload->state = &state;

    node->text = "SonarView";
    node->SetPayload(payload);

    keel::u64 gridFp = 0;
    if (!state.persistentGrid.empty()) {
        const keel::usize stride = keel::max<keel::usize>(1, state.persistentGrid.size() / 64);
        for (keel::usize i = 0; i < state.persistentGrid.size(); i += stride) {
            gridFp = detail::HashArgs(gridFp, state.persistentGrid[i]);
        }
    }

    node->visualHash = detail::HashArgs(
        gridFp,
        targetsFp,
        keel::as<keel::u8>(opts.displayMode),
        keel::as<keel::u8>(opts.scanMode),
        keel::as<keel::u8>(opts.colormap),
        opts.gainDb,
        opts.tvgAmount,
        opts.clutterThreshold,
        opts.persistence,
        opts.sweepAngleDeg,
        opts.sectorApertureDeg,
        state.rangeZoom,
        state.eblBearingDeg,
        state.vrmRangeMeters,
        state.selectedTargetId,
        resp.cursorInsidePlot ? resp.cursorRangeMeters : -1.0f,
        resp.cursorInsidePlot ? resp.cursorBearingDeg : -1.0f,
        resp.guardZoneAlarm,
        resp.guardZoneIntruderCount,
        plotRect.w,
        plotRect.h,
        effectiveSize,
        isDisabled,
        canvas ? canvas->SupportsGpuShaders() : false,
        opts.background,
        opts.border
    );

    node->measureFn = [](const Node* n, keel::f32 availableWidth, ICanvas* c, const UiTheme& th) noexcept -> Vec2 {
        const auto* p = n->As<SonarViewPayload>();
        if (!p) return {0.0f, 0.0f};
        SonarViewOptions effOpts = p->opts;
        if (n->widthMode == SizeMode::Stretch && !effOpts.width.IsFixed()) {
            effOpts.width = SizeMode::Stretch;
        }
        return SonarView::Measure({}, effOpts, n->uiSize, c, th, availableWidth);
    };

    node->renderFn = &RenderSonarViewNode;

    node->uiSize = effectiveSize;
    node->disabled = isDisabled;
    node->hovered = hovered;
    node->pressed = baseRes.pressed;

    ControlSpec ctrlSpec{};
    ctrlSpec.width = opts.width;
    ctrlSpec.height = opts.height;
    ctrlSpec.defaultWidthMode = SizeMode::Stretch;
    ctrlSpec.defaultHeightMode = SizeMode::Stretch;
    ctrlSpec.minHeight = opts.height.IsFixed() ? opts.height.Dp() : 260.0f;
    Control::SetupNode(ctx, node, ctrlSpec);

    return resp;
}

} // namespace guikit
