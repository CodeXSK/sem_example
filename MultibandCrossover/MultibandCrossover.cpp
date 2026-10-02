#include "mp_sdk_audio.h"

#include <algorithm>
#include <array>
#include <cmath>

using namespace gmpi;

namespace
{
    constexpr float kInternalToDisplayedUnits = 10.0f;
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kMinCrossoverHz = 20.0f;
    constexpr float kMaxCrossoverHz = 20000.0f;
    constexpr float kMinFrequencyRatio = 1.05f;

    inline float clampf(float x, float lo, float hi)
    {
        return (std::max)(lo, (std::min)(hi, x));
    }

    inline float finiteOrZero(float x)
    {
        return std::isfinite(x) ? x : 0.0f;
    }

    inline float zapDenormal(float x)
    {
        return std::abs(x) < 1.0e-30f ? 0.0f : x;
    }

    struct BiquadCoefficients
    {
        float b0 = 1.0f;
        float b1 = 0.0f;
        float b2 = 0.0f;
        float a1 = 0.0f;
        float a2 = 0.0f;
    };

    struct BiquadState
    {
        float z1 = 0.0f;
        float z2 = 0.0f;

        void reset()
        {
            z1 = 0.0f;
            z2 = 0.0f;
        }

        float process(float x, const BiquadCoefficients& c)
        {
            // Transposed Direct Form II: numerically well behaved and cheap.
            const float y = c.b0 * x + z1;
            z1 = zapDenormal(c.b1 * x - c.a1 * y + z2);
            z2 = zapDenormal(c.b2 * x - c.a2 * y);
            return zapDenormal(y);
        }
    };

    BiquadCoefficients makeLowpass(float frequencyHz, float sampleRate, float q)
    {
        const float nyquistSafe = 0.45f * sampleRate;
        const float f = clampf(frequencyHz, kMinCrossoverHz,
            (std::min)(kMaxCrossoverHz, nyquistSafe));

        const float w0 = 2.0f * kPi * f / sampleRate;
        const float cw = std::cos(w0);
        const float sw = std::sin(w0);
        const float alpha = sw / (2.0f * q);

        const float a0 = 1.0f + alpha;
        const float invA0 = 1.0f / a0;

        BiquadCoefficients c;
        c.b0 = 0.5f * (1.0f - cw) * invA0;
        c.b1 = (1.0f - cw) * invA0;
        c.b2 = c.b0;
        c.a1 = (-2.0f * cw) * invA0;
        c.a2 = (1.0f - alpha) * invA0;
        return c;
    }

    BiquadCoefficients makeHighpass(float frequencyHz, float sampleRate, float q)
    {
        const float nyquistSafe = 0.45f * sampleRate;
        const float f = clampf(frequencyHz, kMinCrossoverHz,
            (std::min)(kMaxCrossoverHz, nyquistSafe));

        const float w0 = 2.0f * kPi * f / sampleRate;
        const float cw = std::cos(w0);
        const float sw = std::sin(w0);
        const float alpha = sw / (2.0f * q);

        const float a0 = 1.0f + alpha;
        const float invA0 = 1.0f / a0;

        BiquadCoefficients c;
        c.b0 = 0.5f * (1.0f + cw) * invA0;
        c.b1 = -(1.0f + cw) * invA0;
        c.b2 = c.b0;
        c.a1 = (-2.0f * cw) * invA0;
        c.a2 = (1.0f - alpha) * invA0;
        return c;
    }

    // A Linkwitz-Riley 24 dB/oct branch is two cascaded 2nd-order Butterworth
    // sections (Q = 1/sqrt(2)).
    //
    // A Linkwitz-Riley 48 dB/oct branch is two cascaded 4th-order Butterworth
    // filters. A 4th-order Butterworth is decomposed into Q = 0.5411961 and
    // Q = 1.306563 biquads, therefore each Q occurs twice in LR8.
    constexpr std::array<float, 4> kQ24 = {
        0.70710678118f, 0.70710678118f, 0.70710678118f, 0.70710678118f
    };

    constexpr std::array<float, 4> kQ48 = {
        0.54119610015f, 1.30656296488f,
        0.54119610015f, 1.30656296488f
    };

    class LinkwitzRileySplit
    {
    public:
        void reset()
        {
            for (auto& state : lowL_) state.reset();
            for (auto& state : lowR_) state.reset();
            for (auto& state : highL_) state.reset();
            for (auto& state : highR_) state.reset();
        }

        void set(float frequencyHz, float sampleRate, bool slope48)
        {
            const int newSections = slope48 ? 4 : 2;
            const auto& qs = slope48 ? kQ48 : kQ24;

            frequencyHz_ = frequencyHz;
            sections_ = newSections;

            for (int i = 0; i < sections_; ++i)
            {
                lowCoeff_[i] = makeLowpass(frequencyHz, sampleRate, qs[i]);
                highCoeff_[i] = makeHighpass(frequencyHz, sampleRate, qs[i]);
            }
        }

        void process(float inL, float inR,
            float& lowL, float& lowR, float& highL, float& highR)
        {
            lowL = inL;
            lowR = inR;
            highL = inL;
            highR = inR;

            for (int i = 0; i < sections_; ++i)
            {
                lowL = lowL_[i].process(lowL, lowCoeff_[i]);
                lowR = lowR_[i].process(lowR, lowCoeff_[i]);
                highL = highL_[i].process(highL, highCoeff_[i]);
                highR = highR_[i].process(highR, highCoeff_[i]);
            }
        }

    private:
        std::array<BiquadCoefficients, 4> lowCoeff_{};
        std::array<BiquadCoefficients, 4> highCoeff_{};
        std::array<BiquadState, 4> lowL_{};
        std::array<BiquadState, 4> lowR_{};
        std::array<BiquadState, 4> highL_{};
        std::array<BiquadState, 4> highR_{};

        float frequencyHz_ = 1000.0f;
        int sections_ = 2;
    };
}

// -----------------------------------------------------------------------------
// Pandocrator Multiband Crossover v1
// -----------------------------------------------------------------------------
// Stereo 2/3/4-band Linkwitz-Riley crossover for SynthEdit 1.5.
//
// Topology:
//   Stage 1: Input      -> Band 1 + High remainder
//   Stage 2: High rem.  -> Band 2 + High remainder
//   Stage 3: High rem.  -> Band 3 + Band 4
//
// Modes:
//   2 bands -> Band1 = Stage1 Low,  Band2 = Stage1 High
//   3 bands -> Band1 = Stage1 Low,  Band2 = Stage2 Low, Band3 = Stage2 High
//   4 bands -> Band1 = Stage1 Low,  Band2 = Stage2 Low,
//              Band3 = Stage3 Low,  Band4 = Stage3 High
//
// Slope:
//   24 dB/oct = LR4
//   48 dB/oct = LR8
//
// This is an IIR crossover: it has no explicit sample delay / lookahead and
// therefore reports zero latency. As every causal IIR crossover does, it has
// frequency-dependent phase rotation / group delay around the crossover points.
class MultibandCrossover final : public MpBase2
{
public:
    MultibandCrossover()
    {
        // Pin order MUST match MultibandCrossover.xml.
        initializePin(pinInputL);
        initializePin(pinInputR);
        initializePin(pinBands);
        initializePin(pinSlopeDbOct);
        initializePin(pinCrossover1Hz);
        initializePin(pinCrossover2Hz);
        initializePin(pinCrossover3Hz);

        initializePin(pinBand1L);
        initializePin(pinBand1R);
        initializePin(pinBand2L);
        initializePin(pinBand2R);
        initializePin(pinBand3L);
        initializePin(pinBand3R);
        initializePin(pinBand4L);
        initializePin(pinBand4R);
    }

    void subProcess(int sampleFrames)
    {
        auto inputL = getBuffer(pinInputL);
        auto inputR = getBuffer(pinInputR);

        auto bandsPin = getBuffer(pinBands);
        auto slopePin = getBuffer(pinSlopeDbOct);
        auto crossover1Pin = getBuffer(pinCrossover1Hz);
        auto crossover2Pin = getBuffer(pinCrossover2Hz);
        auto crossover3Pin = getBuffer(pinCrossover3Hz);

        auto band1L = getBuffer(pinBand1L);
        auto band1R = getBuffer(pinBand1R);
        auto band2L = getBuffer(pinBand2L);
        auto band2R = getBuffer(pinBand2R);
        auto band3L = getBuffer(pinBand3L);
        auto band3R = getBuffer(pinBand3R);
        auto band4L = getBuffer(pinBand4L);
        auto band4R = getBuffer(pinBand4R);

        updateSampleRateIfNeeded();

        const int bandCount = (std::max)(2, (std::min)(4,
            static_cast<int>(std::lround(
                finiteOrZero(*bandsPin) * kInternalToDisplayedUnits))));

        const float requestedSlope =
            finiteOrZero(*slopePin) * kInternalToDisplayedUnits;
        const bool slope48 = requestedSlope >= 36.0f;

        const float maxHz = (std::min)(kMaxCrossoverHz, 0.45f * currentSampleRate_);

        float f1 = clampf(
            finiteOrZero(*crossover1Pin) * kInternalToDisplayedUnits,
            kMinCrossoverHz, maxHz);

        float f2 = clampf(
            finiteOrZero(*crossover2Pin) * kInternalToDisplayedUnits,
            kMinCrossoverHz, maxHz);

        float f3 = clampf(
            finiteOrZero(*crossover3Pin) * kInternalToDisplayedUnits,
            kMinCrossoverHz, maxHz);

        // Keep crossover points strictly ordered. This prevents accidental
        // overlapping/reversed bands when automating controls.
        f1 = (std::min)(f1, maxHz / (kMinFrequencyRatio * kMinFrequencyRatio));
        f2 = clampf(f2, f1 * kMinFrequencyRatio, maxHz / kMinFrequencyRatio);
        f3 = clampf(f3, f2 * kMinFrequencyRatio, maxHz);

        updateCrossovers(f1, f2, f3, slope48);

        for (int s = 0; s < sampleFrames; ++s)
        {
            const float inL = finiteOrZero(*inputL++);
            const float inR = finiteOrZero(*inputR++);

            float low1L, low1R, high1L, high1R;
            float low2L, low2R, high2L, high2R;
            float low3L, low3R, high3L, high3R;

            split1_.process(inL, inR, low1L, low1R, high1L, high1R);
            split2_.process(high1L, high1R, low2L, low2R, high2L, high2R);
            split3_.process(high2L, high2R, low3L, low3R, high3L, high3R);

            *band1L++ = low1L;
            *band1R++ = low1R;

            if (bandCount == 2)
            {
                *band2L++ = high1L;
                *band2R++ = high1R;
                *band3L++ = 0.0f;
                *band3R++ = 0.0f;
                *band4L++ = 0.0f;
                *band4R++ = 0.0f;
            }
            else if (bandCount == 3)
            {
                *band2L++ = low2L;
                *band2R++ = low2R;
                *band3L++ = high2L;
                *band3R++ = high2R;
                *band4L++ = 0.0f;
                *band4R++ = 0.0f;
            }
            else
            {
                *band2L++ = low2L;
                *band2R++ = low2R;
                *band3L++ = low3L;
                *band3R++ = low3R;
                *band4L++ = high3L;
                *band4R++ = high3R;
            }
        }
    }

    void onSetPins() override
    {
        pinBand1L.setStreaming(true);
        pinBand1R.setStreaming(true);
        pinBand2L.setStreaming(true);
        pinBand2R.setStreaming(true);
        pinBand3L.setStreaming(true);
        pinBand3R.setStreaming(true);
        pinBand4L.setStreaming(true);
        pinBand4R.setStreaming(true);

        setSleep(false);
        setSubProcess(&MultibandCrossover::subProcess);
    }

private:
    void updateSampleRateIfNeeded()
    {
        const float newRate = (std::max)(1000.0f, finiteOrZero(getSampleRate()));
        if (sampleRateInitialized_ && std::abs(newRate - currentSampleRate_) < 0.5f)
            return;

        sampleRateInitialized_ = true;
        currentSampleRate_ = newRate;
        coefficientsValid_ = false;

        split1_.reset();
        split2_.reset();
        split3_.reset();
    }

    void updateCrossovers(float f1, float f2, float f3, bool slope48)
    {
        constexpr float kFrequencyEpsilonHz = 0.01f;

        const bool changed = !coefficientsValid_
            || std::abs(f1 - lastF1_) > kFrequencyEpsilonHz
            || std::abs(f2 - lastF2_) > kFrequencyEpsilonHz
            || std::abs(f3 - lastF3_) > kFrequencyEpsilonHz
            || slope48 != lastSlope48_;

        if (!changed)
            return;

        // Reset only when changing filter order. Frequency moves retain state,
        // which avoids hard discontinuities during normal knob movement.
        if (coefficientsValid_ && slope48 != lastSlope48_)
        {
            split1_.reset();
            split2_.reset();
            split3_.reset();
        }

        split1_.set(f1, currentSampleRate_, slope48);
        split2_.set(f2, currentSampleRate_, slope48);
        split3_.set(f3, currentSampleRate_, slope48);

        lastF1_ = f1;
        lastF2_ = f2;
        lastF3_ = f3;
        lastSlope48_ = slope48;
        coefficientsValid_ = true;
    }

    AudioInPin pinInputL;
    AudioInPin pinInputR;

    AudioInPin pinBands;
    AudioInPin pinSlopeDbOct;
    AudioInPin pinCrossover1Hz;
    AudioInPin pinCrossover2Hz;
    AudioInPin pinCrossover3Hz;

    AudioOutPin pinBand1L;
    AudioOutPin pinBand1R;
    AudioOutPin pinBand2L;
    AudioOutPin pinBand2R;
    AudioOutPin pinBand3L;
    AudioOutPin pinBand3R;
    AudioOutPin pinBand4L;
    AudioOutPin pinBand4R;

    LinkwitzRileySplit split1_;
    LinkwitzRileySplit split2_;
    LinkwitzRileySplit split3_;

    float currentSampleRate_ = 44100.0f;
    bool sampleRateInitialized_ = false;
    bool coefficientsValid_ = false;

    float lastF1_ = -1.0f;
    float lastF2_ = -1.0f;
    float lastF3_ = -1.0f;
    bool lastSlope48_ = false;
};

namespace
{
    auto registration =
        Register<MultibandCrossover>::withId(L"Pandocrator Multiband Crossover v1");
}
