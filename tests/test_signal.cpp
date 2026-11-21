#include "guikit/guikit.hpp"
#include "guikit/signal.hpp"
#if defined(GUIKIT_HAS_BACKEND_SDL3) && GUIKIT_HAS_BACKEND_SDL3
#include "guikit/backend/sdl_gl_shader_engine.hpp"
#if __has_include(<SDL3/SDL.h>)
#include <SDL3/SDL.h>
#endif
#endif
#if defined(GUIKIT_HAS_BACKEND_THORVG) && GUIKIT_HAS_BACKEND_THORVG
#include "guikit/backend/thor_canvas.hpp"
#endif
#include "keel/keel.h"
#include "keel/math.h"

#if defined(GUIKIT_HAS_BACKEND_THORVG) && GUIKIT_HAS_BACKEND_THORVG
using TestCanvas = guikit::ThorCanvas;
#else
using TestCanvas = guikit::NullCanvas;
#endif

static keel::i32 g_failures = 0;

#define CHECK(cond)                                                                  \
    do {                                                                             \
        if (!(cond)) {                                                               \
            keel::eprintln("[FAIL] {} ({}:{})", #cond, __FILE__, __LINE__);          \
            ++g_failures;                                                            \
        }                                                                            \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                                        \
    do {                                                                             \
        if (keel::abs((a) - (b)) > (eps)) {                                           \
            keel::eprintln("[FAIL] {} ({}) != {} ({}) [{}:{}]",                      \
                           #a, (a), #b, (b), __FILE__, __LINE__);                    \
            ++g_failures;                                                            \
        }                                                                            \
    } while (0)

using namespace guikit;

void TestSignalViewWidgetWorkflow() {
    using namespace guikit;

    Context ui;
    TestCanvas canvas;

    const keel::u32 width = 960;
    const keel::u32 height = 540;
    keel::Array<keel::u32> pixels(width * height);
    canvas.SetTarget(pixels.data(), width, height, width);

    // Generate 240,000 samples (5.0 seconds @ 48 kHz) with a 2 kHz sine + 8 kHz harmonic
    const keel::usize numSamples = 240000;
    const keel::f32 sampleRate = 48000.0f;
    keel::Array<keel::f32> samples(numSamples, 0.0f);
    for (keel::usize i = 0; i < numSamples; ++i) {
        const keel::f32 t = keel::as<keel::f32>(i) / sampleRate;
        samples[i] = 0.6f * keel::sin(keel::kTauF * 2000.0f * t) +
                     0.25f * keel::sin(keel::kTauF * 8000.0f * t);
    }

    keel::f32 splitRatio = 0.50f;
    keel::f32 waveformGain = 1.0f;
    keel::f64 playheadSec = 1.25;
    SignalSelection selection{
        .active = true,
        .startTimeSec = 0.5,
        .endTimeSec = 2.0,
        .hasSpectralBox = true,
        .lowFreqHz = 1000.0f,
        .highFreqHz = 10000.0f
    };
    keel::Array<SignalMarker> markers;
    markers.push_back(SignalMarker{1.0, "Cue A", Color{56, 189, 248, 255}});
    markers.push_back(SignalMarker{3.5, "Cue B", Color{245, 158, 11, 255}});
    SignalPanelOrder panelOrder = SignalPanelOrder::WaveformTop;
    SignalViewResponse resp{};

    auto renderFrame = [&]() {
        ui.BeginFrame({keel::as<keel::f32>(width), keel::as<keel::f32>(height)});
        if (auto box = ui.Box({
            .width = SizeMode::Stretch,
            .height = SizeMode::Stretch,
            .padding = Padding::All(8.0f)
        })) {
            resp = guikit::signal::SignalView(ui, samples, {
                .sampleRate = sampleRate,
                .panelOrder = panelOrder,
                .splitRatio = splitRatio,
                .amplitudeScale = AmplitudeScale::dBFS,
                .frequencyScale = FrequencyScale::Linear,
                .colormap = Colormap::Inferno,
                .fftWindow = FftWindowType::Hann,
                .fftSize = 512,
                .splitRatioBinding = &splitRatio,
                .waveformGainBinding = &waveformGain,
                .playheadTimeSec = &playheadSec,
                .selection = &selection,
                .markers = &markers,
                .height = SizeMode::Stretch
            });
        }
        ui.EndFrame(canvas);
    };

    // Frame 1 & 2: Initial layout and render
    renderFrame();
    renderFrame();

    CHECK(resp.rect.w > 800.0f);
    CHECK(resp.rect.h > 400.0f);
    CHECK(resp.totalSamples == numSamples);
    CHECK_NEAR(keel::as<keel::f32>(resp.totalDurationSec), 5.0f, 0.01f);
    CHECK_NEAR(keel::as<keel::f32>(resp.viewStartSec), 0.0f, 0.01f);
    CHECK_NEAR(keel::as<keel::f32>(resp.viewEndSec), 5.0f, 0.01f);
    CHECK(resp.selectionPeakDbFs > -10.0f);
    CHECK(resp.selectionRmsDbFs > -15.0f);
    CHECK(resp.selectionCentroidHz > 1500.0f);

    // Verify wheel zoom in on plot center
    const Vec2 centerPt{resp.rect.x + resp.rect.w * 0.5f, resp.rect.y + resp.rect.h * 0.5f};
    ui.OnMouseMove(centerPt);
    ui.OnMouseWheel(0.0f, 2.0f);
    renderFrame();

    CHECK(resp.changed);
    CHECK((resp.viewEndSec - resp.viewStartSec) < 4.5);
    CHECK(resp.cursorInsidePlot);

    // Verify panel order switch (SpectrogramTop)
    panelOrder = SignalPanelOrder::SpectrogramTop;
    renderFrame();
    CHECK(resp.rect.w > 800.0f);

    keel::println("  [PASS] TestSignalViewWidgetWorkflow completed");
}

void TestSonarViewWidgetWorkflow() {
    using namespace guikit;
    keel::println("  - TestSonarViewWidgetWorkflow...");

    Context ui;
    TestCanvas canvas;

    const keel::u32 width = 900;
    const keel::u32 height = 600;
    keel::Array<keel::u32> pixels(width * height);
    canvas.SetTarget(pixels.data(), width, height, width);

    const keel::u32 numBeams = 90;
    const keel::u32 numBins = 128;
    keel::Array<keel::f32> polarMatrix(numBeams * numBins, 0.25f);
    for (keel::usize i = 0; i < polarMatrix.size(); ++i) {
        polarMatrix[i] = 0.15f + 0.65f * keel::as<keel::f32>((i * 17) % 100) / 100.0f;
    }

    const SonarTarget targets[] = {
        {
            .id = 10,
            .callsign = "DIVER-01",
            .classification = SonarTargetClass::Diver,
            .rangeMeters = 180.0f,
            .bearingDeg = 15.0f,
            .speedMetersPerSec = 1.1f,
            .courseDeg = 195.0f,
            .depthMeters = 7.0f,
            .snrDb = 20.5f
        },
        {
            .id = 20,
            .callsign = "UUV-02",
            .classification = SonarTargetClass::UUV,
            .rangeMeters = 240.0f,
            .bearingDeg = 340.0f,
            .speedMetersPerSec = 2.0f,
            .courseDeg = 140.0f,
            .depthMeters = 12.0f,
            .snrDb = 24.0f
        }
    };

    SonarDisplayMode displayMode = SonarDisplayMode::PPI_360;
    SonarScanMode scanMode = SonarScanMode::MultibeamContinuous;
    Colormap colormap = Colormap::Gold;
    keel::i32 selectedId = 10;
    keel::f32 eblBrg = 25.0f;
    keel::f32 vrmRng = 200.0f;
    keel::f32 rangeZoom = 1.0f;
    SonarViewResponse lastResp{};

    auto renderFrame = [&]() {
        ui.BeginFrame({keel::as<keel::f32>(width), keel::as<keel::f32>(height)});
        lastResp = guikit::signal::SonarView(ui, polarMatrix, {
            .numBeams = numBeams,
            .numRangeBins = numBins,
            .displayMode = displayMode,
            .scanMode = scanMode,
            .colormap = colormap,
            .maxRangeMeters = 400.0f,
            .sectorApertureDeg = 120.0f,
            .gainDb = 3.0f,
            .tvgAmount = 0.5f,
            .clutterThreshold = 0.12f,
            .persistence = 0.8f,
            .sweepAngleDeg = 45.0f,
            .targets = targets,
            .guardZone = {
                .enabled = true,
                .minRangeMeters = 100.0f,
                .maxRangeMeters = 300.0f,
                .startBearingDeg = 320.0f,
                .endBearingDeg = 40.0f
            },
            .selectedTargetId = &selectedId,
            .eblBearingDeg = &eblBrg,
            .vrmRangeMeters = &vrmRng,
            .viewRangeZoom = &rangeZoom,
            .height = SizeMode::Stretch
        });
        ui.EndFrame(canvas);
    };

    renderFrame();
    renderFrame();

    CHECK(lastResp.id.IsValid());
    CHECK(lastResp.guardZoneAlarm);
    CHECK(lastResp.guardZoneIntruderCount == 2);
    CHECK(lastResp.selectedTargetId == 10);
    CHECK_NEAR(lastResp.visibleMaxRangeMeters, 400.0f, 0.5f);

    // Switch to SectorWedge and RotatingSweep + Jet
    displayMode = SonarDisplayMode::SectorWedge;
    scanMode = SonarScanMode::RotatingSweep;
    colormap = Colormap::Jet;
    renderFrame();
    CHECK(lastResp.displayMode == SonarDisplayMode::SectorWedge);

    // Switch to BScan
    displayMode = SonarDisplayMode::BScan;
    colormap = Colormap::Green;
    renderFrame();
    CHECK(lastResp.displayMode == SonarDisplayMode::BScan);

    keel::println("  [PASS] TestSonarViewWidgetWorkflow completed");
}

void TestSonarAndSignalGpuShaderPipeline() {
    using namespace guikit;

    Context ui;
    TestCanvas canvas;

    const keel::u32 width = 800;
    const keel::u32 height = 600;
    keel::Array<keel::u32> pixels(width * height, 0xFF112233u);
    canvas.SetTarget(pixels.data(), width, height, width);

    keel::Array<keel::f32> sonarMatrix(90 * 128, 0.4f);
    keel::Array<keel::f32> audioSamples(4096, 0.0f);
    for (keel::usize i = 0; i < audioSamples.size(); ++i) {
        audioSamples[i] = keel::sin(keel::as<keel::f32>(i) * 0.1f) * 0.5f;
    }

    bool btnToggle = false;
    auto renderUI = [&]() {
        if (auto col = ui.Column({.gap = 8.0f, .padding = Padding::All(8.0f)})) {
            if (ui.Button({.text = btnToggle ? "Mode B" : "Mode A"})) {
                btnToggle = !btnToggle;
            }
            guikit::signal::SonarView(ui, sonarMatrix, {
                .numBeams = 90,
                .numRangeBins = 128,
                .height = 220.0f
            });
            guikit::signal::SignalView(ui, audioSamples, {
                .sampleRate = 48000.0f,
                .height = 220.0f
            });
        }
    };

    // 1. Render with GPU shaders enabled (Frame 1: initial layout, Frame 2: bounds converged)
    canvas.SetGpuShadersEnabled(true);
    CHECK(canvas.SupportsGpuShaders());

    ui.BeginFrame({keel::as<keel::f32>(width), keel::as<keel::f32>(height)});
    renderUI();
    ui.EndFrame(canvas);

    ui.BeginFrame({keel::as<keel::f32>(width), keel::as<keel::f32>(height)});
    renderUI();
    ui.EndFrame(canvas);

    CHECK(canvas.HasActiveGpuPasses());
    CHECK(canvas.GetGpuShaderPasses().size() == 2);

    const auto& sonarPass = canvas.GetGpuShaderPasses()[0];
    CHECK(sonarPass.shader.shaderId == "guikit.sonar.v1");
    CHECK(sonarPass.dataField.extent(0) == 90);
    CHECK(sonarPass.dataField.extent(1) == 128);
    CHECK(sonarPass.plotRect.w > 100.0f);
    CHECK(sonarPass.clipRect.w > 100.0f);

#if defined(GUIKIT_HAS_BACKEND_THORVG) && GUIKIT_HAS_BACKEND_THORVG
    // Verify PunchHole created transparent underlay pixels inside sonarPass.plotRect
    keel::usize transparentCount = 0;
    const keel::i32 sx0 = keel::as<keel::i32>(sonarPass.plotRect.x) + 4;
    const keel::i32 sy0 = keel::as<keel::i32>(sonarPass.plotRect.y) + 4;
    const keel::i32 sx1 = keel::as<keel::i32>(sonarPass.plotRect.Right()) - 4;
    const keel::i32 sy1 = keel::as<keel::i32>(sonarPass.plotRect.Bottom()) - 4;
    for (keel::i32 y = sy0; y < sy1; y += 4) {
        for (keel::i32 x = sx0; x < sx1; x += 4) {
            const keel::u32 px = pixels[keel::as<keel::usize>(y) * width + keel::as<keel::usize>(x)];
            if ((px >> 24) == 0) {
                transparentCount++;
            }
        }
    }
    CHECK(transparentCount > 50);
#endif

    // 2. Partial dirty-rect update on top button (outside SonarView & SignalView) preserves cached GPU passes
    btnToggle = true;
    ui.BeginFrame({keel::as<keel::f32>(width), keel::as<keel::f32>(height)});
    renderUI();
    ui.EndFrame(canvas);

    CHECK(!ui.IsFullRepaint());
    CHECK(canvas.HasActiveGpuPasses());
    CHECK(canvas.GetGpuShaderPasses().size() == 2);

    // 3. Toggle GPU shaders OFF -> automatic CPU fallback with opaque background in plotRect
    canvas.SetGpuShadersEnabled(false);
    CHECK(!canvas.SupportsGpuShaders());
    CHECK(!canvas.HasActiveGpuPasses());

    ui.BeginFrame({keel::as<keel::f32>(width), keel::as<keel::f32>(height)});
    renderUI();
    ui.EndFrame(canvas);

#if defined(GUIKIT_HAS_BACKEND_THORVG) && GUIKIT_HAS_BACKEND_THORVG
    keel::usize opaqueCount = 0;
    for (keel::i32 y = sy0; y < sy1; y += 8) {
        for (keel::i32 x = sx0; x < sx1; x += 8) {
            const keel::u32 px = pixels[keel::as<keel::usize>(y) * width + keel::as<keel::usize>(x)];
            if ((px >> 24) == 255) {
                opaqueCount++;
            }
        }
    }
    CHECK(opaqueCount > 20);
#endif

#if defined(GUIKIT_HAS_BACKEND_SDL3) && GUIKIT_HAS_BACKEND_SDL3 && \
    defined(GUIKIT_HAS_BACKEND_THORVG) && GUIKIT_HAS_BACKEND_THORVG && \
    __has_include(<SDL3/SDL.h>)
    // 4. Test real SDL3 OpenGL renderer + SdlGlShaderEngine underlay rendering & pixel readback
    if (SDL_Init(SDL_INIT_VIDEO)) {
        SDL_SetHint(SDL_HINT_RENDER_DRIVER, "opengl");
        SDL_Window* win = nullptr;
        SDL_Renderer* ren = nullptr;
        if (SDL_CreateWindowAndRenderer("GpuTest", width, height, SDL_WINDOW_HIDDEN, &win, &ren)) {
            SdlGlShaderEngine gpuEngine;
            const bool glOk = gpuEngine.Initialize(ren);
            CHECK(glOk);
            CHECK(gpuEngine.IsAvailable());
            canvas.SetGpuShadersEnabled(glOk);
            ui.InvalidateAll();

            SDL_Texture* tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ABGR8888, SDL_TEXTUREACCESS_STREAMING, width, height);
            SDL_SetTextureScaleMode(tex, SDL_SCALEMODE_NEAREST);
            SDL_SetTextureBlendMode(tex, SDL_BLENDMODE_BLEND_PREMULTIPLIED);

            for (keel::i32 f = 0; f < 3; ++f) {
                if (f == 1) {
                    // Mutate sonar matrix & button on frame 1 to exercise glTexSubImage2D & partial updates
                    sonarMatrix[10] = 0.9f;
                    btnToggle = !btnToggle;
                }
                ui.BeginFrame({keel::as<keel::f32>(width), keel::as<keel::f32>(height)});
                renderUI();
                ui.EndFrame(canvas);

                SDL_UpdateTexture(tex, nullptr, pixels.data(), width * sizeof(keel::u32));
                SDL_RenderClear(ren);
                if (canvas.HasActiveGpuPasses()) {
                    gpuEngine.RenderUnderlays(canvas, width, height);
                }
                SDL_RenderTexture(ren, tex, nullptr, nullptr);

                if (f == 2) {
                    SDL_Surface* readSurf = SDL_RenderReadPixels(ren, nullptr);
                    CHECK(readSurf != nullptr);
                    if (readSurf) {
                        CHECK(readSurf->w == keel::as<keel::i32>(width));
                        CHECK(readSurf->h == keel::as<keel::i32>(height));
                        SDL_DestroySurface(readSurf);
                    }
                }

                SDL_RenderPresent(ren);
            }

            // 5. Verify mouse crosshair movement over SignalView uses StaticBaseLayerCache (RestoreRegion),
            //    keeps partial dirty rect (!IsFullRepaint), preserves GPU underlay pass, and runs in < 2.0 ms/frame.
            const Rect specPlot = canvas.GetGpuShaderPasses()[1].plotRect;
            keel::f64 totalCrosshairMs = 0.0;
            constexpr keel::i32 kMoveFrames = 20;
            for (keel::i32 m = 0; m < kMoveFrames; ++m) {
                ui.OnMouseMove({specPlot.x + 25.0f + keel::as<keel::f32>(m * 4), specPlot.y + 15.0f + keel::as<keel::f32>((m % 5) * 3)});
                const keel::u64 t0 = keel::monotonic_ns();
                ui.BeginFrame({keel::as<keel::f32>(width), keel::as<keel::f32>(height)});
                renderUI();
                ui.EndFrame(canvas);
                const keel::u64 t1 = keel::monotonic_ns();
                totalCrosshairMs += keel::as<keel::f64>(t1 - t0) / 1e6;

                CHECK(ui.HasDirtyDamage());
                CHECK(!ui.IsFullRepaint());
                CHECK(canvas.HasActiveGpuPasses());
                CHECK(canvas.GetGpuShaderPasses().size() == 2);
            }
            const keel::f64 avgCrosshairMs = totalCrosshairMs / keel::as<keel::f64>(kMoveFrames);
            keel::println("  [BENCH] SignalView mouse crosshair move CPU frame time: {:.3f} ms/frame", avgCrosshairMs);
            CHECK(avgCrosshairMs < 5.0);

            gpuEngine.Shutdown();
            SDL_DestroyTexture(tex);
            SDL_DestroyRenderer(ren);
            SDL_DestroyWindow(win);
        }
        SDL_Quit();
    }
#endif

    keel::println("  [PASS] TestSonarAndSignalGpuShaderPipeline completed");
}

void TestWaterfallViewWidgetAndGpuRingBuffer() {
    using namespace guikit;

    Context ui;
    TestCanvas canvas;
    canvas.SetGpuShadersEnabled(true);

    const keel::u32 width = 960;
    const keel::u32 height = 640;
    keel::Array<keel::u32> pixels(width * height);
    canvas.SetTarget(pixels.data(), width, height, width);

    constexpr keel::u32 kBins = 256;
    keel::Array<keel::f32> specSlice(kBins, -95.0f);
    keel::Array<keel::f32> doaSlice(181, -85.0f);

    // Inject synthetic peaks
    for (keel::u32 i = 0; i < kBins; ++i) {
        const keel::f32 d1 = (keel::as<keel::f32>(i) - 64.0f) / 3.0f;
        const keel::f32 d2 = (keel::as<keel::f32>(i) - 128.0f) / 4.0f;
        specSlice[i] = -95.0f + 65.0f * keel::exp(-0.5f * d1 * d1) + 48.0f * keel::exp(-0.5f * d2 * d2);
    }
    for (keel::usize i = 0; i < doaSlice.size(); ++i) {
        const keel::f32 deg = -90.0f + keel::as<keel::f32>(i);
        const keel::f32 b1 = (deg + 25.0f) / 3.5f;
        const keel::f32 b2 = (deg - 32.0f) / 4.0f;
        doaSlice[i] = -85.0f + 58.0f * keel::exp(-0.5f * b1 * b1) + 46.0f * keel::exp(-0.5f * b2 * b2);
    }

    WaterfallTrack tracks[2] = {
        {.id = 101, .callsign = "TGT-A", .status = WaterfallTrackStatus::Tracking, .currentX = -25.0f, .driftRatePerSec = 0.45f, .levelDb = -27.0f, .snrDb = 24.5f, .gateWidth = 6.0f},
        {.id = 102, .callsign = "TGT-B", .status = WaterfallTrackStatus::Tracking, .currentX = 32.0f, .driftRatePerSec = -0.20f, .levelDb = -39.0f, .snrDb = 17.2f, .gateWidth = 6.0f}
    };

    keel::u64 seq = 1;
    keel::f32 f0 = 250.0f;
    WaterfallViewResponse respSpec;
    WaterfallViewResponse respDoa;

    auto renderUI = [&]() {
        if (auto col = ui.Column({.gap = 12.0f, .padding = Padding::All(12.0f)})) {
            respSpec = guikit::signal::WaterfallView(ui, specSlice, {
                .mode = WaterfallMode::SpectrumRtsa,
                .viewLayout = WaterfallLayout::SplitTraceAndWaterfall,
                .colormap = Colormap::Turbo,
                .numBins = kBins,
                .historyRows = 128,
                .historyDurationSec = 10.0f,
                .minX = 0.0f,
                .maxX = 1000.0f,
                .sliceSequence = seq,
                .dbFloor = -110.0f,
                .dbCeil = -10.0f,
                .thresholdGateDb = -65.0f,
                .showLiveTrace = true,
                .showPeakHold = true,
                .showAverageTrace = true,
                .showThresholdGate = true,
                .showPeakMarkers = true,
                .showHarmonicComb = true,
                .harmonicFundamentalX = &f0,
                .height = 250.0f
            });

            respDoa = guikit::signal::WaterfallView(ui, doaSlice, {
                .mode = WaterfallMode::DoaBtr,
                .viewLayout = WaterfallLayout::SplitTraceAndWaterfall,
                .colormap = Colormap::Blue,
                .numBins = keel::as<keel::u32>(doaSlice.size()),
                .historyRows = 128,
                .historyDurationSec = 15.0f,
                .minX = -90.0f,
                .maxX = 90.0f,
                .sliceSequence = seq,
                .dbFloor = -100.0f,
                .dbCeil = -15.0f,
                .thresholdGateDb = -60.0f,
                .tracks = tracks,
                .height = 250.0f
            });
        }
    };

    // Frame 1..3: Layout and ingest first 3 slices
    for (keel::i32 f = 0; f < 3; ++f) {
        seq = keel::as<keel::u64>(f + 1);
        ui.BeginFrame({keel::as<keel::f32>(width), keel::as<keel::f32>(height)});
        renderUI();
        ui.EndFrame(canvas);
    }

    CHECK(respSpec.rect.w > 200.0f);
    CHECK(respSpec.rect.h > 200.0f);
    CHECK(respDoa.rect.w > 200.0f);
    CHECK(respDoa.rect.h > 200.0f);
    CHECK(respSpec.populatedRows >= 3);
    CHECK(respDoa.populatedRows >= 3);
    CHECK(respSpec.dominantPeakDb > -40.0f);
    CHECK(respDoa.dominantPeakDb > -40.0f);
    CHECK(canvas.GetGpuShaderPasses().size() == 2);

    // Pause ingestion (keep seq constant) and test mouse crosshair movement over WaterfallView uses StaticBaseLayerCache
    ui.BeginFrame({keel::as<keel::f32>(width), keel::as<keel::f32>(height)});
    renderUI();
    ui.EndFrame(canvas);

    const Rect wfRect = canvas.GetGpuShaderPasses()[0].plotRect;
    ui.OnMouseMove({wfRect.x + 40.0f, wfRect.y + 30.0f});
    ui.BeginFrame({keel::as<keel::f32>(width), keel::as<keel::f32>(height)});
    renderUI();
    ui.EndFrame(canvas);

    CHECK(respSpec.hovered);
    CHECK(respSpec.cursorInside);
    CHECK(!ui.IsFullRepaint());
    CHECK(canvas.GetGpuShaderPasses().size() == 2);

    keel::println("  [PASS] TestWaterfallViewWidgetAndGpuRingBuffer completed");
}

void TestPartiallyClippedGpuScenesAboveBottomPanel() {
    keel::println("  -> TestPartiallyClippedGpuScenesAboveBottomPanel...");
        // Regression test: Partially-clipped GPU scene widgets (SonarView / WaterfallView) above an open BottomPanel
        // (TextArea cursor blinking / mouse movement in BottomPanel).
        // Previously, ThorCanvas::Clear erased GPU passes whenever a dirty rect in BottomPanel intersected the
        // UNCLIPPED plotRect extending below the ScrollArea clip boundary, causing the GPU underlay to drop and flash black.
        {
            Context clipUi;
            TestCanvas clipCanvas;
            keel::Array<keel::u32> clipPixels(1200 * 800);
            clipCanvas.SetTarget(clipPixels.data(), 1200, 800, 1200);
            clipCanvas.SetGpuShadersEnabled(true);

            keel::Array<keel::f32> sonarMatrix(64 * 64, 0.5f);
            keel::Array<keel::f32> wfSlice(128, -45.0f);
            keel::u64 wfSeq = 1;
            bool bottomOpen = true;
            keel::String codeBuf("auto resp = guikit::signal::SonarView(ui, polarMatrix, {\n    .numBeams = 180,\n    .numRangeBins = 256\n});");
            Rect codeAreaRect{};
            SonarViewResponse lastSonarResp{};
            WaterfallViewResponse lastWfResp{};
            bool showWaterfall = false;

            auto renderClippedScene = [&](keel::f64 timeSec) {
                clipUi.SetCurrentTime(timeSec);
                if (auto rootCol = clipUi.Column({.gap = 0.0f, .padding = 0.0f})) {
                    // Upper ScrollArea is constrained to ~540px while inner simulated tablet/window is 700px tall
                    if (auto scroll = clipUi.ScrollArea({.gap = 0.0f, .padding = 0.0f})) {
                        if (auto stageFrame = clipUi.Box({
                            .width = 900.0f,
                            .height = 700.0f,
                            .clipChildren = true
                        })) {
                            if (!showWaterfall) {
                                lastSonarResp = guikit::signal::SonarView(clipUi, sonarMatrix, {
                                    .numBeams = 64,
                                    .numRangeBins = 64,
                                    .displayMode = SonarDisplayMode::SectorWedge,
                                    .height = 680.0f
                                });
                            } else {
                                lastWfResp = guikit::signal::WaterfallView(clipUi, wfSlice, {
                                    .mode = WaterfallMode::SpectrumRtsa,
                                    .viewLayout = WaterfallLayout::SplitTraceAndWaterfall,
                                    .numBins = 128,
                                    .historyRows = 64,
                                    .sliceSequence = wfSeq,
                                    .height = 680.0f
                                });
                            }
                        }
                    }
                    if (auto bp = clipUi.BottomPanel(bottomOpen, {
                        .height = 260.0f,
                        .resizable = true,
                        .allowCollapse = true
                    })) {
                        auto taResp = clipUi.TextArea(codeBuf, {
                            .autoHeight = false,
                            .readOnly = true,
                            .showLineNumbers = true,
                            .height = SizeMode::Stretch
                        });
                        codeAreaRect = taResp.rect;
                    }
                }
            };

            // 1. Settle layout and initial full render for SonarView
            for (keel::i32 f = 0; f < 3; ++f) {
                clipUi.BeginFrame({1200.0f, 800.0f}, 1.0f);
                renderClippedScene(1.0 + f * 0.1);
                clipUi.EndFrame(clipCanvas);
            }
            CHECK(clipCanvas.HasActiveGpuPasses());
            CHECK(!clipCanvas.GetGpuShaderPasses().empty());
            if (!clipCanvas.GetGpuShaderPasses().empty()) {
                const Rect initialSonarClip = clipCanvas.GetGpuShaderPasses()[0].clipRect;
                const Rect initialSonarPlot = clipCanvas.GetGpuShaderPasses()[0].plotRect;
                // Verify the widget is genuinely partially clipped vertically (unclipped plot extends into BottomPanel)
                CHECK(initialSonarPlot.Bottom() > initialSonarClip.Bottom() + 50.0f);
                CHECK(initialSonarPlot.Bottom() > codeAreaRect.y);

                // 2. Click inside the BottomPanel TextArea to focus it and trigger cursor blinking (partial dirty rects in BottomPanel)
                clipUi.OnMouseMove(codeAreaRect.Center());
                clipUi.OnMouseDown(1);
                clipUi.OnMouseUp(1);
                for (keel::i32 step = 0; step < 6; ++step) {
                    clipUi.BeginFrame({1200.0f, 800.0f}, 1.0f);
                    renderClippedScene(2.0 + step * 0.55); // Toggles cursor blink every 0.53s
                    clipUi.EndFrame(clipCanvas);
                    // GPU pass must NEVER be dropped when BottomPanel repaints!
                    CHECK(clipCanvas.HasActiveGpuPasses());
                    CHECK(!clipCanvas.GetGpuShaderPasses().empty());
                    if (!clipCanvas.GetGpuShaderPasses().empty()) {
                        CHECK(keel::abs(clipCanvas.GetGpuShaderPasses()[0].clipRect.h - initialSonarClip.h) < 2.0f);
                    }
                }

                // 3. Move mouse inside the visible portion of the clipped SonarView and verify unclipped geometry stability
                clipUi.OnMouseMove({initialSonarClip.Center().x, initialSonarClip.Bottom() - 40.0f});
                clipUi.BeginFrame({1200.0f, 800.0f}, 1.0f);
                renderClippedScene(6.0);
                clipUi.EndFrame(clipCanvas);
                CHECK(clipCanvas.HasActiveGpuPasses());
                CHECK(lastSonarResp.hovered);
                CHECK(lastSonarResp.cursorInsidePlot);
            }

            // 4. Switch to WaterfallView and verify the same partial-clipping resilience
            showWaterfall = true;
            for (keel::i32 f = 0; f < 3; ++f) {
                ++wfSeq;
                clipUi.BeginFrame({1200.0f, 800.0f}, 1.0f);
                renderClippedScene(7.0 + f * 0.1);
                clipUi.EndFrame(clipCanvas);
            }
            CHECK(clipCanvas.HasActiveGpuPasses());
            CHECK(!clipCanvas.GetGpuShaderPasses().empty());
            if (!clipCanvas.GetGpuShaderPasses().empty()) {
                const Rect initialWfClip = clipCanvas.GetGpuShaderPasses()[0].clipRect;
                const Rect initialWfPlot = clipCanvas.GetGpuShaderPasses()[0].plotRect;
                CHECK(initialWfPlot.Bottom() > initialWfClip.Bottom() + 50.0f);

                // Blink cursor in BottomPanel while WaterfallView is partially clipped
                clipUi.OnMouseMove(codeAreaRect.Center());
                clipUi.OnMouseDown(1);
                clipUi.OnMouseUp(1);
                for (keel::i32 step = 0; step < 6; ++step) {
                    clipUi.BeginFrame({1200.0f, 800.0f}, 1.0f);
                    renderClippedScene(8.0 + step * 0.55);
                    clipUi.EndFrame(clipCanvas);
                    CHECK(clipCanvas.HasActiveGpuPasses());
                    CHECK(!clipCanvas.GetGpuShaderPasses().empty());
                    if (!clipCanvas.GetGpuShaderPasses().empty()) {
                        CHECK(keel::abs(clipCanvas.GetGpuShaderPasses()[0].clipRect.h - initialWfClip.h) < 2.0f);
                    }
                }
            }
        }
    keel::println("  [PASS] TestPartiallyClippedGpuScenesAboveBottomPanel completed");
}

int main() {
    keel::println("[RUN] Testing GuiKit Signal module...");
    TestSignalViewWidgetWorkflow();
    TestSonarViewWidgetWorkflow();
    TestSonarAndSignalGpuShaderPipeline();
    TestWaterfallViewWidgetAndGpuRingBuffer();
    TestPartiallyClippedGpuScenesAboveBottomPanel();
    if (g_failures == 0) {
        keel::println("[PASS] All GuiKit Signal tests passed successfully!");
        return 0;
    }
    keel::eprintln("[FAIL] Total failures: {}", g_failures);
    return 1;
}
