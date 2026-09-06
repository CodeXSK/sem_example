#include "FilterBase.h"

#include <algorithm>
#include <cmath>

using namespace gmpi;

namespace
{
    constexpr double kPi = 3.1415926535897932384626433832795;
    constexpr double kTiny = 1.0e-9;

    inline double clamp01(double v)
    {
        return (std::max)(0.0, (std::min)(1.0, v));
    }

    // One Direct-Form-II-transposed biquad.
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

    // A C1-continuous soft-knee clip curve.
    // Shape 0.0 = exactly linear.
    // Shape 1.0 = almost hard clip, but with 1.5% residual slope above the knee,
    // so there is NEVER a hidden hard clamp at +/-1 or at the user limits.
    inline double shapeMagnitude(double a, double shape, double limit)
    {
        if (shape <= 1.0e-7)
            return a;

        const double s = clamp01(shape);
        const double L = (std::max)(std::abs(limit), 1.0e-6);

        // Very broad knee at low Shape, very narrow knee at high Shape.
        const double oneMinus = 1.0 - s;
        const double kneeHalfWidth = L * (0.08 + 1.75 * oneMinus * oneMinus);
        const double x0 = (std::max)(0.0, L - kneeHalfWidth);
        const double x1 = L + kneeHalfWidth;

        // 100% slope near Shape=0, falling to 1.5% at Shape=1.
        // This is what keeps the last mode "almost" hard rather than mathematically flat.
        const double residualSlope = 0.015 + 0.985 * oneMinus * oneMinus;

        if (a <= x0)
            return a;

        const auto postKnee = [L, residualSlope](double x)
        {
            return L + residualSlope * (x - L);
        };

        if (a >= x1)
            return postKnee(a);

        // Cubic Hermite interpolation between:
        //   (x0, x0), slope = 1
        // and
        //   (x1, postKnee(x1)), slope = residualSlope.
        // This avoids a derivative discontinuity at the knee boundaries.
        const double width = (std::max)(x1 - x0, kTiny);
        const double t = (a - x0) / width;
        const double t2 = t * t;
        const double t3 = t2 * t;

        const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
        const double h10 = t3 - 2.0 * t2 + t;
        const double h01 = -2.0 * t3 + 3.0 * t2;
        const double h11 = t3 - t2;

        const double y0 = x0;
        const double y1 = postKnee(x1);

        return h00 * y0
             + h10 * width * 1.0
             + h01 * y1
             + h11 * width * residualSlope;
    }

    inline double softCurve(double x, double shape, double minLimit, double maxLimit)
    {
        if (!std::isfinite(x))
            return 0.0;

        if (!std::isfinite(shape))
            shape = 0.0;
        if (!std::isfinite(minLimit))
            minLimit = -1.0;
        if (!std::isfinite(maxLimit))
            maxLimit = 1.0;

        const double s = clamp01(shape);
        if (s <= 1.0e-7)
            return x; // mathematically transparent at Shape = 0.

        if (x >= 0.0)
            return shapeMagnitude(x, s, maxLimit);

        // Min Limit is expected to be negative, but abs() makes the pin forgiving.
        return -shapeMagnitude(-x, s, minLimit);
    }

    inline int quantizeOversampling(float value)
    {
        if (!std::isfinite(value))
            return 4;

        // The pin is intentionally a normal numeric pin for maximum SE 1.5 compatibility.
        // Feed 1, 2, 4 or 8. Intermediate values are quantized to the nearest region.
        if (value < 1.5f) return 1;
        if (value < 3.0f) return 2;
        if (value < 6.0f) return 4;
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

    // Interpolation state.
    double previousInput_ = 0.0;
    bool havePreviousInput_ = false;

    int oversamplingFactor_ = 1;
    bool filtersNeedPrime_ = false;

    // 4th-order Butterworth before the nonlinearity (2 biquads).
    Biquad pre_[2];

    // 8th-order Butterworth after the nonlinearity (4 biquads).
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
        for (auto& f : pre_)
            f.reset();
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

        // Anti-imaging / anti-alias cutoff. 0.45*hostFs = 90% of host Nyquist.
        // The steep post filter removes most of the new harmonics above the host band
        // before the signal is decimated back to the project sample rate.
        const double cutoff = hostFs * 0.45;

        // 4th-order Butterworth section Qs.
        pre_[0].setLowpass(osFs, cutoff, 0.5411961001461970);
        pre_[1].setLowpass(osFs, cutoff, 1.3065629648763766);

        // 8th-order Butterworth section Qs.
        post_[0].setLowpass(osFs, cutoff, 0.5097955791041592);
        post_[1].setLowpass(osFs, cutoff, 0.6013448869350453);
        post_[2].setLowpass(osFs, cutoff, 0.8999762231364156);
        post_[3].setLowpass(osFs, cutoff, 2.5629154477415060);
    }

    inline double runPreFilter(double x)
    {
        x = pre_[0].process(x);
        x = pre_[1].process(x);
        return x;
    }

    inline double runPostFilter(double x)
    {
        x = post_[0].process(x);
        x = post_[1].process(x);
        x = post_[2].process(x);
        x = post_[3].process(x);
        return x;
    }

    void primeFilters(double shapedDc)
    {
        // Prime filters with a constant value so enabling/changing oversampling does not
        // begin from zero-state and create a large artificial startup transient.
        if (oversamplingFactor_ <= 1)
            return;

        const int iterations = 96;
        for (int i = 0; i < iterations; ++i)
        {
            double v = runPreFilter(shapedDc);
            v = runPostFilter(v);
            (void)v;
        }
        filtersNeedPrime_ = false;
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

        // Oversampling is treated as a configuration control, not an audio-rate modulation.
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
                resetFilters();
                havePreviousInput_ = false;
                filtersNeedPrime_ = oversamplingFactor_ > 1;
            }

            if (oversamplingFactor_ == 1)
            {
                *signalOut++ = static_cast<float>(softCurve(x, shape, minLimit, maxLimit));
                previousInput_ = x;
                havePreviousInput_ = true;
                continue;
            }

            if (!havePreviousInput_)
            {
                previousInput_ = x;
                havePreviousInput_ = true;
            }

            if (filtersNeedPrime_)
            {
                // Pre-filter sees DC x, then the nonlinear section, then the post filter.
                resetFilters();
                for (int i = 0; i < 96; ++i)
                {
                    double v = x;
                    v = runPreFilter(v);
                    v = softCurve(v, shape, minLimit, maxLimit);
                    v = runPostFilter(v);
                }
                filtersNeedPrime_ = false;
            }

            double y = 0.0;
            const double step = (x - previousInput_) / static_cast<double>(oversamplingFactor_);

            // Linear interpolation generates the intermediate samples. The pre-filter
            // further suppresses interpolation images before the nonlinear waveshaper.
            for (int k = 1; k <= oversamplingFactor_; ++k)
            {
                double os = previousInput_ + step * static_cast<double>(k);
                os = runPreFilter(os);
                os = softCurve(os, shape, minLimit, maxLimit);
                y = runPostFilter(os);
            }

            previousInput_ = x;

            // Deliberately NO clamp here. Output may exceed +/-1 and may exceed the
            // Min/Max knee references because the near-hard mode retains residual slope.
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

        for (const auto& f : pre_)
            ok = ok && f.stateIsFinite();
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
