#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "mp_sdk_audio.h"

#include <algorithm>
#include <cmath>

using namespace gmpi;

// -----------------------------------------------------------------------------
// Phase Rotator Zero Latency - PRODUCTION SAFE v2
// SynthEdit SDK3 / sem_example compatible.
//
// IMPORTANT DSP CHANGE:
// This version implements the SAME first-order all-pass transfer function as
// the previous version, but in a normalized lossless-lattice form:
//
//   y[n]     = a*x[n] + b*s[n-1]
//   s[n]     = b*x[n] - a*s[n-1]
//   b        = sqrt(1 - a*a)
//
// For a fixed coefficient this is mathematically equivalent to:
//
//   H(z) = (a + z^-1) / (1 + a*z^-1)
//
// The normalized lattice is substantially safer when the Phase control moves:
// the 2x2 scattering matrix is orthogonal for every |a| <= 1, so coefficient
// changes cannot create unbounded recursive energy by themselves.
//
// Additional hardening:
//   * NaN/Inf input and control sanitizing.
//   * Denormal/subnormal suppression.
//   * Fixed 5 ms Phase slew independent of host block size.
//   * Invalid-state recovery.
//   * No allocation, locking, I/O, or exceptions in the audio path.
//
// Phase 0..1 = approximately 0..90 degrees phase-lag magnitude at 1 kHz.
// This remains a causal, frequency-dependent all-pass phase response.
// There is no fixed sample-delay buffer and no reported latency.
// -----------------------------------------------------------------------------
class PhaseRotatorZeroLatency final : public MpBase2
{
    AudioInPin  pinAudioIn;   // XML pin 0
    AudioOutPin pinAudioOut;  // XML pin 1
    FloatInPin  pinPhase;     // XML pin 2

public:
    PhaseRotatorZeroLatency()
    {
        // Pin initialization order MUST match the XML IDs.
        initializePin(pinAudioIn);
        initializePin(pinAudioOut);
        initializePin(pinPhase);
    }

    void onGraphStart() override
    {
        sampleRate_ = getSampleRate();

        if (!std::isfinite(sampleRate_)
            || sampleRate_ < kMinimumValidSampleRate
            || sampleRate_ > kMaximumValidSampleRate)
        {
            sampleRate_ = 44100.0f;
        }

        const double omega =
            2.0 * kPi * static_cast<double>(kReferenceFrequencyHz)
            / static_cast<double>(sampleRate_);

        tanHalfOmega_ = std::tan(0.5 * omega);

        if (!std::isfinite(tanHalfOmega_)
            || std::abs(tanHalfOmega_) < 1.0e-15)
        {
            // Known-safe 44.1 kHz fallback.
            sampleRate_ = 44100.0f;

            const double safeOmega =
                2.0 * kPi * static_cast<double>(kReferenceFrequencyHz)
                / static_cast<double>(sampleRate_);

            tanHalfOmega_ = std::tan(0.5 * safeOmega);
        }

        phaseSlewPerSample_ =
            1.0f
            / (std::max)(
                1.0f,
                sampleRate_ * (kPhaseSlewMilliseconds * 0.001f));

        currentPhase_ =
            safePhase(static_cast<float>(pinPhase));

        latticeState_ = 0.0f;

        // Explicitly report no fixed latency.
        host.SetLatency(0);

        pinAudioOut.setStreaming(true);

        // An all-pass has recursive state/tail. Keep processing active so the
        // host cannot truncate state evolution unexpectedly.
        setSleep(false);
    }

    void onSetPins() override
    {
        pinAudioOut.setStreaming(true);
        setSleep(false);

        setSubProcess(
            &PhaseRotatorZeroLatency::subProcess);
    }

    void subProcess(int sampleFrames)
    {
        if (sampleFrames <= 0)
            return;

        auto* input  = getBuffer(pinAudioIn);
        auto* output = getBuffer(pinAudioOut);

        const float targetPhase =
            safePhase(static_cast<float>(pinPhase));

        // Common stationary-control fast path. One coefficient calculation for
        // the complete SynthEdit process span.
        if (std::abs(targetPhase - currentPhase_)
            <= kStaticPhaseEpsilon)
        {
            currentPhase_ = targetPhase;

            if (currentPhase_ <= kBypassPhaseThreshold)
            {
                // Exact dry output at zero Phase. Reset the hidden lattice state
                // so moving away from zero never resurrects stale energy.
                latticeState_ = 0.0f;

                for (int i = 0; i < sampleFrames; ++i)
                    output[i] = sanitizeAudio(input[i]);

                return;
            }

            const float a = coefficientForPhase(currentPhase_);
            const float b = complementaryCoefficient(a);

            for (int i = 0; i < sampleFrames; ++i)
            {
                output[i] = processLatticeSample(
                    sanitizeAudio(input[i]),
                    a,
                    b);
            }

            return;
        }

        // Moving-control path. The target can change at sample-accurate event
        // boundaries, but the internal Phase never jumps faster than the fixed
        // slew rate. This avoids block-size-dependent zipper/transient behaviour.
        for (int i = 0; i < sampleFrames; ++i)
        {
            currentPhase_ = approach(
                currentPhase_,
                targetPhase,
                phaseSlewPerSample_);

            const float x = sanitizeAudio(input[i]);

            if (currentPhase_ <= kBypassPhaseThreshold)
            {
                latticeState_ = 0.0f;
                output[i] = x;
                continue;
            }

            const float a = coefficientForPhase(currentPhase_);
            const float b = complementaryCoefficient(a);

            output[i] = processLatticeSample(x, a, b);
        }
    }

private:
    static constexpr double kPi =
        3.141592653589793238462643383279502884;

    static constexpr double kHalfPi =
        1.570796326794896619231321691639751442;

    static constexpr float kReferenceFrequencyHz =
        1000.0f;

    static constexpr float kPhaseSlewMilliseconds =
        5.0f;

    static constexpr float kMinimumValidSampleRate =
        8000.0f;

    // Very generous upper bound. Values above this are treated as host faults.
    static constexpr float kMaximumValidSampleRate =
        768000.0f;

    static constexpr float kBypassPhaseThreshold =
        1.0e-7f;

    static constexpr float kStaticPhaseEpsilon =
        1.0e-8f;

    static constexpr float kDenormalThreshold =
        1.0e-30f;

    // +120 dBFS. No legitimate audio signal should approach this. Clamping only
    // absurd finite values protects the recursive state from floating overflow.
    static constexpr float kMaximumSafeAudioMagnitude =
        1.0e6f;

    float sampleRate_ = 44100.0f;
    double tanHalfOmega_ = 0.0;
    float phaseSlewPerSample_ = 1.0f;
    float currentPhase_ = 0.0f;

    // Normalized lattice memory state.
    float latticeState_ = 0.0f;

    static float safePhase(float value)
    {
        if (!std::isfinite(value))
            return 0.0f;

        return (std::max)(
            0.0f,
            (std::min)(1.0f, value));
    }

    static float sanitizeAudio(float value)
    {
        if (!std::isfinite(value))
            return 0.0f;

        value = (std::max)(
            -kMaximumSafeAudioMagnitude,
            (std::min)(kMaximumSafeAudioMagnitude, value));

        if (std::abs(value) < kDenormalThreshold)
            return 0.0f;

        return value;
    }

    static float zapDenormal(float value)
    {
        if (!std::isfinite(value))
            return 0.0f;

        if (std::abs(value) < kDenormalThreshold)
            return 0.0f;

        return value;
    }

    static float approach(
        float current,
        float target,
        float maximumStep)
    {
        if (!std::isfinite(current))
            current = 0.0f;

        target = safePhase(target);

        if (!std::isfinite(maximumStep)
            || maximumStep <= 0.0f)
        {
            return target;
        }

        if (current < target)
            return (std::min)(current + maximumStep, target);

        if (current > target)
            return (std::max)(current - maximumStep, target);

        return target;
    }

    float coefficientForPhase(float normalizedPhase) const
    {
        normalizedPhase = safePhase(normalizedPhase);

        if (normalizedPhase <= kBypassPhaseThreshold)
            return 1.0f;

        const double desiredPhase =
            static_cast<double>(normalizedPhase) * kHalfPi;

        const double tanDesiredHalf =
            std::tan(0.5 * desiredPhase);

        if (!std::isfinite(tanDesiredHalf)
            || !std::isfinite(tanHalfOmega_)
            || std::abs(tanHalfOmega_) < 1.0e-15)
        {
            return 0.0f;
        }

        const double ratio =
            tanDesiredHalf / tanHalfOmega_;

        if (!std::isfinite(ratio))
            return -kMaximumCoefficientMagnitude;

        const double denominator =
            1.0 + ratio;

        if (!std::isfinite(denominator)
            || std::abs(denominator) < 1.0e-15)
        {
            return -kMaximumCoefficientMagnitude;
        }

        double a =
            (1.0 - ratio) / denominator;

        if (!std::isfinite(a))
            a = 0.0;

        a = (std::max)(
            -static_cast<double>(kMaximumCoefficientMagnitude),
            (std::min)(
                static_cast<double>(kMaximumCoefficientMagnitude),
                a));

        return static_cast<float>(a);
    }

    static float complementaryCoefficient(float a)
    {
        a = (std::max)(
            -kMaximumCoefficientMagnitude,
            (std::min)(kMaximumCoefficientMagnitude, a));

        const double aa =
            static_cast<double>(a) * static_cast<double>(a);

        const double remaining =
            (std::max)(0.0, 1.0 - aa);

        const double b = std::sqrt(remaining);

        if (!std::isfinite(b))
            return 0.0f;

        return static_cast<float>(b);
    }

    float processLatticeSample(
        float x,
        float a,
        float b)
    {
        if (!std::isfinite(latticeState_))
            latticeState_ = 0.0f;

        // Use double intermediates. This is cheap for a first-order section and
        // gives large numerical headroom even if an upstream module outputs an
        // absurd but still finite signal.
        const double xd = static_cast<double>(x);
        const double sd = static_cast<double>(latticeState_);
        const double ad = static_cast<double>(a);
        const double bd = static_cast<double>(b);

        const double y =
            ad * xd + bd * sd;

        const double newState =
            bd * xd - ad * sd;

        if (!std::isfinite(y)
            || !std::isfinite(newState))
        {
            latticeState_ = 0.0f;
            return 0.0f;
        }

        float outputSample = sanitizeAudio(
            static_cast<float>((std::max)(
                -static_cast<double>(kMaximumSafeAudioMagnitude),
                (std::min)(
                    static_cast<double>(kMaximumSafeAudioMagnitude),
                    y))));

        latticeState_ = sanitizeAudio(
            static_cast<float>((std::max)(
                -static_cast<double>(kMaximumSafeAudioMagnitude),
                (std::min)(
                    static_cast<double>(kMaximumSafeAudioMagnitude),
                    newState))));

        outputSample = zapDenormal(outputSample);
        latticeState_ = zapDenormal(latticeState_);

        return outputSample;
    }

    static constexpr float kMaximumCoefficientMagnitude =
        0.999999f;
};

namespace
{
    // This ID MUST exactly match the XML Plugin id.
    auto registration =
        Register<PhaseRotatorZeroLatency>::withId(
            L"Pandocrator Phase Rotator Zero Latency SEM");
}
