#include "FilterBase.h"

#include <algorithm>
#include <cmath>

using namespace gmpi;

namespace
{
    constexpr double kPi = 3.1415926535897932384626433832795;
    constexpr double kTiny = 1.0e-8;

    inline double clamp01(double v)
    {
        return (std::max)(0.0, (std::min)(1.0, v));
    }

    struct Biquad
    {
        double b0 = 1.0;
        double b1 = 0.0;
        double b2 = 0.0;
        double a1 = 0.0;
        double a2 = 0.0;
        double z1 = 0.0;
        double z2 = 0.0;

        void reset()
        {
            z1 = 0.0;
            z2 = 0.0;
        }

        bool stateIsFinite() const
        {
            return std::isfinite(z1) && std::isfinite(z2);
        }

        void setLowpass(double sampleRate, double cutoffHz, double q)
        {
            const double safeFs = (std::max)(sampleRate, 1000.0);
            const double safeFc = (std::max)(1.0, (std::min)(cutoffHz, safeFs * 0.499));
            const double omega = 2.0 * kPi * safeFc / safeFs;
            const double c = std::cos(omega);
            const double s = std::sin(omega);
            const double alpha = s / (2.0 * q);

            const double a0 = 1.0 + alpha;
            b0 = ((1.0 - c) * 0.5) / a0;
            b1 = (1.0 - c) / a0;
            b2 = b0;
            a1 = (-2.0 * c) / a0;
            a2 = (1.0 - alpha) / a0;
        }

        inline double process(double x)
        {
            const double y = b0 * x + z1;
            z1 = b1 * x - a1 * y + z2;
            z2 = b2 * x - a2 * y;
            return y;
        }
    };

    // Stable generalized soft clipper for one side of the waveform.
    // a     : positive magnitude of the sample.
    // shape : 0..1. 0 = perfectly linear, 1 = almost-hard but smooth.
    // limit : USER limit for this side. There is no built-in +/-1 ceiling.
    inline double shapeMagnitude(double a, double shape, double limit)
    {
        if (shape <= 1.0e-8)
            return a;

        const double s = clamp01(shape);
        const double L = (std::max)(std::abs(limit), kTiny);
        const double u = a / L;

        // Low values give a round S-curve. At Shape=1 the exponent is high enough
        // to look/behave almost like a hard clip, while remaining continuous/smooth.
        const double hardness = 1.5 + 28.0 * s * s;

        // Generalized saturator:
        //   f(u) = u / (1 + u^p)^(1/p)
        // The alternate form for u > 1 avoids overflow for extreme input levels.
        double saturatedNormalized = 0.0;
        if (u <= 1.0)
        {
            saturatedNormalized = u / std::pow(1.0 + std::pow(u, hardness), 1.0 / hardness);
        }
        else
        {
            const double inv = 1.0 / u;
            saturatedNormalized = 1.0 / std::pow(1.0 + std::pow(inv, hardness), 1.0 / hardness);
        }

        const double saturated = L * saturatedNormalized;

        // Shape itself is the morph amount:
        // 0 = y=x exactly, 0.5 = clear S-curve, 1 = near-hard soft clip.
        return a + s * (saturated - a);
    }

    inline double softCurve(double x, double shape, double minLimit, double maxLimit)
    {
        if (!std::isfinite(x))
            return 0.0;

        if (!std::isfinite(shape))
            shape = 0.0;
        if (!std::isfinite(minLimit))
            minLimit = -0.5; // -5 V in SynthEdit.
        if (!std::isfinite(maxLimit))
            maxLimit = 0.5;  // +5 V in SynthEdit.

        const double s = clamp01(shape);
        if (s <= 1.0e-8)
            return x;

        if (x >= 0.0)
            return shapeMagnitude(x, s, maxLimit);

        // Separate negative threshold allows asymmetric saturation.
        return -shapeMagnitude(-x, s, minLimit);
    }

    // Oversampling is an AUDIO pin so it remains maximally compatible with
    // SynthEdit 1.5 / FilterBase. SynthEdit audio-pin controls use volts:
    // 4.0 shown on a slider arrives at the DSP as 0.4, so convert back x10 here.
    inline int quantizeOversampling(float internalVoltage)
    {
        if (!std::isfinite(internalVoltage))
            return 4;

        const float displayedValue = internalVoltage * 10.0f;

        // Nearest regions for 1x, 2x, 4x, 8x.
        if (displayedValue < 1.5f) return 1;
        if (displayedValue < 3.0f) return 2;
        if (displayedValue < 6.0f) return 4;
        return 8;
    }
}

class SoftCurveSaturator final : public FilterBase
{
    AudioInPin  pinSignalIn;
    AudioInPin  pinShape;
    AudioInPin  pinMinLimit;
    AudioInPin  pinMaxLimit;
    AudioInPin  pinOversampling;
    AudioOutPin pinSignalOut;

    double previousInput_ = 0.0;
    bool havePreviousInput_ = false;

    int oversamplingFactor_ = 0; // force configuration on first process call.
    bool filtersNeedPrime_ = false;

    // 8th-order Butterworth anti-alias filter after the nonlinear stage.
    Biquad post_[4];

public:
    SoftCurveSaturator()
    {
        initializePin(pinSignalIn);
        initializePin(pinShape);
        initializePin(pinMinLimit);
        initializePin(pinMaxLimit);
        initializePin(pinOversampling);
        initializePin(pinSignalOut);
    }

    void resetFilters()
    {
        for (auto& f : post_)
            f.reset();
    }

    void configureOversampling(int factor)
    {
        factor = (std::max)(1, (std::min)(8, factor));
        if (factor == oversamplingFactor_)
            return;

        oversamplingFactor_ = factor;
        resetFilters();
        havePreviousInput_ = false;
        filtersNeedPrime_ = factor > 1;

        if (factor == 1)
            return;

        const double hostFs = (std::max)(1000.0, static_cast<double>(getSampleRate()));
        const double osFs = hostFs * static_cast<double>(factor);

        // 90% of the original Nyquist. Harmonics above this are strongly removed
        // before returning to the host sample rate.
        const double cutoff = hostFs * 0.45;

        // 8th-order Butterworth Q values (four second-order sections).
        post_[0].setLowpass(osFs, cutoff, 0.5097955791041592);
        post_[1].setLowpass(osFs, cutoff, 0.6013448869350453);
        post_[2].setLowpass(osFs, cutoff, 0.8999762231364156);
        post_[3].setLowpass(osFs, cutoff, 2.5629154477415060);
    }

    inline double runPostFilter(double x)
    {
        x = post_[0].process(x);
        x = post_[1].process(x);
        x = post_[2].process(x);
        x = post_[3].process(x);
        return x;
    }

    void subProcess(int sampleFrames)
    {
        doStabilityCheck(); // FilterBase recommendation: keep this first.

        auto signalIn = getBuffer(pinSignalIn);
        auto shapeIn = getBuffer(pinShape);
        auto minIn = getBuffer(pinMinLimit);
        auto maxIn = getBuffer(pinMaxLimit);
        auto oversamplingIn = getBuffer(pinOversampling);
        auto signalOut = getBuffer(pinSignalOut);

        // Configuration control: read once for this process slice.
        configureOversampling(quantizeOversampling(*oversamplingIn));

        for (int s = sampleFrames; s > 0; --s)
        {
            double x = static_cast<double>(*signalIn++);
            const double shape = static_cast<double>(*shapeIn++);
            const double minLimit = static_cast<double>(*minIn++);
            const double maxLimit = static_cast<double>(*maxIn++);

            if (!std::isfinite(x))
            {
                x = 0.0;
                previousInput_ = 0.0;
                havePreviousInput_ = false;
                resetFilters();
                filtersNeedPrime_ = oversamplingFactor_ > 1;
            }

            // Absolutely transparent at 1x + Shape=0 and no hidden output clamp.
            if (oversamplingFactor_ == 1)
            {
                const double y = softCurve(x, shape, minLimit, maxLimit);
                *signalOut++ = std::isfinite(y) ? static_cast<float>(y) : 0.0f;
                previousInput_ = x;
                havePreviousInput_ = true;
                continue;
            }

            if (!havePreviousInput_)
            {
                previousInput_ = x;
                havePreviousInput_ = true;
            }

            // Prime the IIR at the current shaped DC value to prevent a large startup click.
            if (filtersNeedPrime_)
            {
                resetFilters();
                const double dc = softCurve(x, shape, minLimit, maxLimit);
                for (int i = 0; i < 128; ++i)
                    (void)runPostFilter(dc);
                filtersNeedPrime_ = false;
            }

            const double step = (x - previousInput_) / static_cast<double>(oversamplingFactor_);
            double y = 0.0;

            // Linear interpolation supplies the intermediate samples. The nonlinear
            // waveshaper runs at Fs*N, then the steep LPF removes out-of-band harmonics.
            for (int k = 1; k <= oversamplingFactor_; ++k)
            {
                const double osInput = previousInput_ + step * static_cast<double>(k);
                const double shaped = softCurve(osInput, shape, minLimit, maxLimit);
                y = runPostFilter(shaped);
            }

            previousInput_ = x;

            // NO clamp to +/-1. The only saturation bounds are the user's Min/Max.
            *signalOut++ = std::isfinite(y) ? static_cast<float>(y) : 0.0f;
        }
    }

    void onSetPins(void) override
    {
        pinSignalOut.setStreaming(true);
        setSubProcess(&SoftCurveSaturator::subProcess);
        initSettling(); // must be last for FilterBase.
    }

    bool isFilterSettling() override
    {
        return !pinSignalIn.isStreaming()
            && !pinShape.isStreaming()
            && !pinMinLimit.isStreaming()
            && !pinMaxLimit.isStreaming()
            && !pinOversampling.isStreaming();
    }

    AudioOutPin& getOutputPin() override
    {
        return pinSignalOut;
    }

    void StabilityCheck() override
    {
        bool ok = std::isfinite(previousInput_);
        for (const auto& f : post_)
            ok = ok && f.stateIsFinite();

        if (!ok)
        {
            previousInput_ = 0.0;
            havePreviousInput_ = false;
            resetFilters();
            filtersNeedPrime_ = oversamplingFactor_ > 1;
        }
    }
};

namespace
{
    auto r = Register<SoftCurveSaturator>::withId(L"Pandocrator Soft Curve Saturator");
}
