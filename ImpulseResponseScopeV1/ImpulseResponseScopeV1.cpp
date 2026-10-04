#include "mp_sdk_audio.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

using namespace gmpi;

namespace
{
    constexpr int kDisplayBins = 1536;
    constexpr float kMaxWindowSeconds = 8.0f;
    constexpr float kMaxPreTriggerFraction = 0.25f;

    inline float clamp01(float v)
    {
        if (!std::isfinite(v)) return 0.0f;
        return (std::max)(0.0f, (std::min)(1.0f, v));
    }

    // SynthEdit Structure View displays 10x the SDK internal audio-pin voltage.
    inline float volts01ToNormalized(float internalVoltage)
    {
        return clamp01(internalVoltage * 10.0f);
    }

    inline float lerp(float a, float b, float t)
    {
        return a + (b - a) * t;
    }

    inline float expMap(float lo, float hi, float t)
    {
        t = clamp01(t);
        return lo * std::pow(hi / lo, t);
    }

    struct ScopeHeader
    {
        std::uint32_t magic = 0x49525331u; // "IRS1"
        std::uint32_t version = 1;
        std::uint32_t binCount = kDisplayBins;
        std::uint32_t reserved = 0;
        float sampleRate = 44100.0f;
        float windowSeconds = 1.0f;
        float preTriggerFraction = 0.05f;
        float peak = 0.0f;
        float triggerTimeSeconds = 0.0f;
        float captureSamples = 0.0f;
    };
}

class ImpulseResponseScopeV1 final : public MpBase2
{
public:
    ImpulseResponseScopeV1()
    {
        // Visible Structure View pins. Order MUST match XML <Audio>.
        initializePin(inSignal);          // 0
        initializePin(inTimeWindow);      // 1
        initializePin(inPreTrigger);      // 2
        initializePin(inThreshold);       // 3
        initializePin(inVerticalGain);    // 4
        initializePin(inFreeze);          // 5
        initializePin(inForceTrigger);    // 6
        initializePin(inBgA);             // 7
        initializePin(inBgR);             // 8
        initializePin(inBgG);             // 9
        initializePin(inBgB);             // 10
        initializePin(inWaveA);           // 11
        initializePin(inWaveR);           // 12
        initializePin(inWaveG);           // 13
        initializePin(inWaveB);           // 14
        initializePin(inGridA);           // 15
        initializePin(inWidthPx);         // 16
        initializePin(inHeightPx);        // 17

        // Private DSP -> GUI bridge.
        initializePin(outCapture);        // 18
        initializePin(outTimeWindow);     // 19
        initializePin(outPreTrigger);     // 20
        initializePin(outThreshold);      // 21
        initializePin(outVerticalGain);   // 22
        initializePin(outFreeze);         // 23
        initializePin(outBgA);            // 24
        initializePin(outBgR);            // 25
        initializePin(outBgG);            // 26
        initializePin(outBgB);            // 27
        initializePin(outWaveA);          // 28
        initializePin(outWaveR);          // 29
        initializePin(outWaveG);          // 30
        initializePin(outWaveB);          // 31
        initializePin(outGridA);          // 32
        initializePin(outWidthPx);        // 33
        initializePin(outHeightPx);       // 34
    }

    void onSetPins() override
    {
        pushGuiControls(-1);
        setSleep(false);
        setSubProcess(&ImpulseResponseScopeV1::subProcess);
    }

    void subProcess(int sampleFrames)
    {
        if (sampleFrames <= 0)
            return;

        ensureBuffers();
        updateSettings(sampleFrames - 1);
        pushGuiControls(sampleFrames - 1);

        float* input = getBuffer(inSignal);
        if (!input)
            return;

        const bool frozen = freezeNorm_ >= 0.5f;
        const bool forceNow = forceNorm_ >= 0.5f && previousForceNorm_ < 0.5f;
        previousForceNorm_ = forceNorm_;

        if (frozen)
        {
            capturing_ = false;
            armed_ = true;
            capture_.clear();
        }

        for (int i = 0; i < sampleFrames; ++i)
        {
            const float x = std::isfinite(input[i]) ? input[i] : 0.0f;
            const float ax = std::fabs(x);

            if (!frozen)
            {
                bool trigger = false;
                if (!capturing_ && armed_)
                {
                    // The absolute-value crossing is ideal for clicks/impulses because
                    // either polarity can trigger. It also avoids missing a one-sample spike.
                    trigger = (previousAbs_ < triggerThreshold_ && ax >= triggerThreshold_);
                    if (forceNow && i == 0)
                        trigger = true;
                }

                if (trigger)
                    beginCapture(x);
                else if (capturing_)
                    appendCapture(x);

                // Rearm only after signal falls clearly below threshold. This prevents
                // one impulse/reverb tail from creating multiple captures.
                if (!capturing_ && !armed_ && ax < triggerThreshold_ * 0.5f)
                    armed_ = true;
            }

            previousAbs_ = ax;
            pushPreRing(x);
        }
    }

private:
    void ensureBuffers()
    {
        if (!preRing_.empty())
            return;

        const float sr = (std::max)(1000.0f, getSampleRate());
        const int maxWindowSamples = static_cast<int>(std::ceil(sr * kMaxWindowSeconds));
        capture_.reserve(static_cast<size_t>(maxWindowSamples + 8));
        preRing_.assign(static_cast<size_t>((std::max)(64, static_cast<int>(std::ceil(sr * kMaxWindowSeconds * kMaxPreTriggerFraction))) + 8), 0.0f);
    }

    void updateSettings(int samplePos)
    {
        const auto readNorm = [samplePos](const AudioInPin& p)
        {
            return volts01ToNormalized(p.getValue(samplePos));
        };

        timeNorm_ = readNorm(inTimeWindow);
        preNorm_ = readNorm(inPreTrigger);
        thresholdNorm_ = readNorm(inThreshold);
        gainNorm_ = readNorm(inVerticalGain);
        freezeNorm_ = readNorm(inFreeze);
        forceNorm_ = readNorm(inForceTrigger);

        // 1 ms .. 8 sec, logarithmic. Lets you zoom deeply into the click while
        // still having enough horizontal range for a long reverb tail.
        windowSeconds_ = expMap(0.001f, kMaxWindowSeconds, timeNorm_);
        preTriggerFraction_ = kMaxPreTriggerFraction * preNorm_;

        // -90 dBFS .. 0 dBFS threshold.
        const float thresholdDb = lerp(-90.0f, 0.0f, thresholdNorm_);
        triggerThreshold_ = std::pow(10.0f, thresholdDb / 20.0f);

        // 0.25x .. 64x display gain.
        verticalGain_ = expMap(0.25f, 64.0f, gainNorm_);
    }

    void beginCapture(float triggerSample)
    {
        const float sr = (std::max)(1000.0f, getSampleRate());
        targetSamples_ = (std::max)(32, static_cast<int>(std::round(windowSeconds_ * sr)));
        preSamplesForCapture_ = (std::min)(targetSamples_ - 1,
            static_cast<int>(std::round(targetSamples_ * preTriggerFraction_)));

        capture_.clear();
        capture_.reserve(static_cast<size_t>(targetSamples_));

        copyPreTrigger(preSamplesForCapture_);
        capture_.push_back(triggerSample);
        triggerIndexInCapture_ = static_cast<int>(capture_.size()) - 1;

        capturing_ = true;
        armed_ = false;

        if (static_cast<int>(capture_.size()) >= targetSamples_)
            finishCapture();
    }

    void appendCapture(float x)
    {
        if (!capturing_)
            return;

        capture_.push_back(x);
        if (static_cast<int>(capture_.size()) >= targetSamples_)
            finishCapture();
    }

    void finishCapture()
    {
        capturing_ = false;
        if (capture_.empty())
            return;

        ScopeHeader header;
        header.sampleRate = (std::max)(1000.0f, getSampleRate());
        header.windowSeconds = static_cast<float>(capture_.size()) / header.sampleRate;
        header.preTriggerFraction = capture_.empty() ? 0.0f : static_cast<float>(triggerIndexInCapture_) / static_cast<float>(capture_.size());
        header.triggerTimeSeconds = static_cast<float>(triggerIndexInCapture_) / header.sampleRate;
        header.captureSamples = static_cast<float>(capture_.size());

        // Blob layout: header, then kDisplayBins mins, then kDisplayBins maxes,
        // then kDisplayBins RMS values. Min/max keeps even a one-sample click visible
        // when several thousand source samples collapse into one screen pixel/bin.
        const size_t floatCount = static_cast<size_t>(kDisplayBins) * 3u;
        blob_.resize(sizeof(ScopeHeader) + floatCount * sizeof(float));
        std::memcpy(blob_.data(), &header, sizeof(ScopeHeader));
        float* mins = reinterpret_cast<float*>(blob_.data() + sizeof(ScopeHeader));
        float* maxs = mins + kDisplayBins;
        float* rms = maxs + kDisplayBins;

        float peak = 0.0f;
        const int n = static_cast<int>(capture_.size());
        for (int b = 0; b < kDisplayBins; ++b)
        {
            int a = static_cast<int>((static_cast<std::int64_t>(b) * n) / kDisplayBins);
            int z = static_cast<int>((static_cast<std::int64_t>(b + 1) * n) / kDisplayBins);
            if (z <= a) z = (std::min)(n, a + 1);
            a = (std::min)(a, n - 1);

            float mn = capture_[a];
            float mx = capture_[a];
            double e = 0.0;
            int count = 0;
            for (int s = a; s < z; ++s)
            {
                const float v = capture_[s];
                mn = (std::min)(mn, v);
                mx = (std::max)(mx, v);
                peak = (std::max)(peak, std::fabs(v));
                e += static_cast<double>(v) * static_cast<double>(v);
                ++count;
            }
            mins[b] = mn;
            maxs[b] = mx;
            rms[b] = count > 0 ? static_cast<float>(std::sqrt(e / static_cast<double>(count))) : 0.0f;
        }

        // Update peak in already-built header.
        reinterpret_cast<ScopeHeader*>(blob_.data())->peak = peak;

        outCapture.setValueRaw(static_cast<int>(blob_.size()), blob_.data());
        outCapture.sendPinUpdate();
    }

    void pushPreRing(float x)
    {
        if (preRing_.empty())
            return;
        preRing_[preWrite_] = x;
        preWrite_ = (preWrite_ + 1) % preRing_.size();
        preValid_ = (std::min)(preRing_.size(), preValid_ + 1);
    }

    void copyPreTrigger(int wanted)
    {
        if (wanted <= 0 || preRing_.empty() || preValid_ == 0)
            return;

        const size_t count = (std::min)(static_cast<size_t>(wanted), preValid_);
        const size_t start = (preWrite_ + preRing_.size() - count) % preRing_.size();
        for (size_t i = 0; i < count; ++i)
            capture_.push_back(preRing_[(start + i) % preRing_.size()]);
    }

    void pushGuiControls(int samplePos)
    {
        const auto readNorm = [samplePos](const AudioInPin& p)
        {
            return volts01ToNormalized(p.getValue(samplePos));
        };

        auto pushFloat = [](FloatOutPin& out, float value, float& cache)
        {
            constexpr float eps = 0.0005f;
            if (!std::isfinite(cache) || std::fabs(value - cache) > eps)
            {
                cache = value;
                out = value;
            }
        };

        pushFloat(outTimeWindow,   readNorm(inTimeWindow),   guiCache_[0]);
        pushFloat(outPreTrigger,   readNorm(inPreTrigger),   guiCache_[1]);
        pushFloat(outThreshold,    readNorm(inThreshold),    guiCache_[2]);
        pushFloat(outVerticalGain, readNorm(inVerticalGain), guiCache_[3]);
        pushFloat(outFreeze,       readNorm(inFreeze),       guiCache_[4]);
        pushFloat(outBgA,          readNorm(inBgA),          guiCache_[5]);
        pushFloat(outBgR,          readNorm(inBgR),          guiCache_[6]);
        pushFloat(outBgG,          readNorm(inBgG),          guiCache_[7]);
        pushFloat(outBgB,          readNorm(inBgB),          guiCache_[8]);
        pushFloat(outWaveA,        readNorm(inWaveA),        guiCache_[9]);
        pushFloat(outWaveR,        readNorm(inWaveR),        guiCache_[10]);
        pushFloat(outWaveG,        readNorm(inWaveG),        guiCache_[11]);
        pushFloat(outWaveB,        readNorm(inWaveB),        guiCache_[12]);
        pushFloat(outGridA,        readNorm(inGridA),        guiCache_[13]);

        int w = (std::max)(240, (std::min)(3000, inWidthPx.getValue()));
        int h = (std::max)(120, (std::min)(1600, inHeightPx.getValue()));
        if (w != lastWidth_) { lastWidth_ = w; outWidthPx = w; }
        if (h != lastHeight_) { lastHeight_ = h; outHeightPx = h; }
    }

    // Visible inputs.
    AudioInPin inSignal;
    AudioInPin inTimeWindow;
    AudioInPin inPreTrigger;
    AudioInPin inThreshold;
    AudioInPin inVerticalGain;
    AudioInPin inFreeze;
    AudioInPin inForceTrigger;
    AudioInPin inBgA, inBgR, inBgG, inBgB;
    AudioInPin inWaveA, inWaveR, inWaveG, inWaveB;
    AudioInPin inGridA;
    IntInPin inWidthPx, inHeightPx;

    // Private bridge outputs.
    BlobOutPin outCapture;
    FloatOutPin outTimeWindow, outPreTrigger, outThreshold, outVerticalGain, outFreeze;
    FloatOutPin outBgA, outBgR, outBgG, outBgB;
    FloatOutPin outWaveA, outWaveR, outWaveG, outWaveB;
    FloatOutPin outGridA;
    IntOutPin outWidthPx, outHeightPx;

    std::vector<float> preRing_;
    std::vector<float> capture_;
    std::vector<std::uint8_t> blob_;
    size_t preWrite_ = 0;
    size_t preValid_ = 0;

    bool capturing_ = false;
    bool armed_ = true;
    int targetSamples_ = 0;
    int preSamplesForCapture_ = 0;
    int triggerIndexInCapture_ = 0;
    float previousAbs_ = 0.0f;
    float previousForceNorm_ = 0.0f;

    float timeNorm_ = 0.0f;
    float preNorm_ = 0.2f;
    float thresholdNorm_ = 0.45f;
    float gainNorm_ = 0.25f;
    float freezeNorm_ = 0.0f;
    float forceNorm_ = 0.0f;
    float windowSeconds_ = 1.0f;
    float preTriggerFraction_ = 0.05f;
    float triggerThreshold_ = 0.01f;
    float verticalGain_ = 1.0f;

    float guiCache_[14] = { NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN };
    int lastWidth_ = -1;
    int lastHeight_ = -1;
};

namespace
{
    auto registration = Register<ImpulseResponseScopeV1>::withId(L"Pandocrator Impulse Response Scope v1");
}
