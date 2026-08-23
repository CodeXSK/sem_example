#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "mp_sdk_audio.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

using namespace gmpi;

// -----------------------------------------------------------------------------
// Phase Rotator FIR - PRODUCTION SAFE v2
// SynthEdit SDK3 / sem_example compatible.
//
// Quality/robustness changes from the earlier 129-tap version:
//   * 1025-tap Hamming Hilbert FIR at sample rates <= 100 kHz.
//   * 2049-tap Hamming Hilbert FIR above 100 kHz (up to 192 kHz is the
//     intended high-rate quality target).
//   * Exploits exact Type-III FIR antisymmetry, so only half of the non-zero
//     tap pairs are multiplied. This gives much higher accuracy without a
//     naive 8x/16x CPU increase.
//   * Fixed 5 ms Phase slew, independent of host block size.
//   * NaN/Inf and denormal protection.
//   * No allocation, locks, I/O, or exceptions in the audio path.
//   * Exact fast paths at 0 and 180 degrees skip the Hilbert convolution.
//
// Phase mapping is intentionally unchanged:
//   0.00 ->   0 degrees
//   0.25 ->  45 degrees
//   0.50 ->  90 degrees
//   0.75 -> 135 degrees
//   1.00 -> 180 degrees
//
// FIR latency:
//   <= 100 kHz sample rate :  512 samples (1025 taps)
//   >  100 kHz sample rate : 1024 samples (2049 taps)
//
// The direct branch is delayed by the same group delay as the Hilbert branch.
// -----------------------------------------------------------------------------
class PhaseRotatorFIR final : public MpBase2
{
    static constexpr int kStandardTapCount = 1025;
    static constexpr int kHighRateTapCount = 2049;
    static constexpr int kMaximumTapCount = kHighRateTapCount;
    static constexpr int kMaximumGroupDelay =
        (kMaximumTapCount - 1) / 2;
    static constexpr int kMaximumPairCount =
        kMaximumGroupDelay / 2;

    AudioInPin  pinAudioIn;   // XML pin 0
    AudioOutPin pinAudioOut;  // XML pin 1
    FloatInPin  pinPhase;     // XML pin 2

public:
    PhaseRotatorFIR()
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

        configureForSampleRate();
        buildHilbertTransformer();
        resetState();

        currentPhase_ =
            safePhase(static_cast<float>(pinPhase));

        phaseSlewPerSample_ =
            1.0f
            / (std::max)(
                1.0f,
                sampleRate_ * (kPhaseSlewMilliseconds * 0.001f));

        // Report the exact group delay of the active linear-phase FIR.
        host.SetLatency(groupDelaySamples_);

        pinAudioOut.setStreaming(true);

        // The FIR needs to drain its history after the input stops. Keeping the
        // module awake is conservative and avoids truncating that finite tail.
        setSleep(false);
    }

    void onSetPins() override
    {
        pinAudioOut.setStreaming(true);
        setSleep(false);

        setSubProcess(
            &PhaseRotatorFIR::subProcess);
    }

    void subProcess(int sampleFrames)
    {
        if (sampleFrames <= 0)
            return;

        auto* input  = getBuffer(pinAudioIn);
        auto* output = getBuffer(pinAudioOut);

        const float targetPhase =
            safePhase(static_cast<float>(pinPhase));

        // Stationary-control fast path: calculate sin/cos once per process span.
        if (std::abs(targetPhase - currentPhase_)
            <= kStaticPhaseEpsilon)
        {
            currentPhase_ = targetPhase;

            const float theta =
                currentPhase_ * static_cast<float>(kPi);

            const float directCoefficient =
                std::cos(theta);

            const float quadratureCoefficient =
                std::sin(theta);

            const bool needsQuadrature =
                std::abs(quadratureCoefficient)
                > kQuadratureSkipThreshold;

            for (int i = 0; i < sampleFrames; ++i)
            {
                output[i] = processOneSample(
                    sanitizeAudio(input[i]),
                    directCoefficient,
                    quadratureCoefficient,
                    needsQuadrature);
            }

            return;
        }

        // Moving-control path with a fixed, sample-rate-independent slew time.
        for (int i = 0; i < sampleFrames; ++i)
        {
            currentPhase_ = approach(
                currentPhase_,
                targetPhase,
                phaseSlewPerSample_);

            const float theta =
                currentPhase_ * static_cast<float>(kPi);

            const float directCoefficient =
                std::cos(theta);

            const float quadratureCoefficient =
                std::sin(theta);

            output[i] = processOneSample(
                sanitizeAudio(input[i]),
                directCoefficient,
                quadratureCoefficient,
                std::abs(quadratureCoefficient)
                    > kQuadratureSkipThreshold);
        }
    }

private:
    static constexpr double kPi =
        3.141592653589793238462643383279502884;

    static constexpr float kPhaseSlewMilliseconds =
        5.0f;

    static constexpr float kHighRateThresholdHz =
        100000.0f;

    static constexpr float kMinimumValidSampleRate =
        8000.0f;

    static constexpr float kMaximumValidSampleRate =
        768000.0f;

    static constexpr float kStaticPhaseEpsilon =
        1.0e-8f;

    static constexpr float kQuadratureSkipThreshold =
        1.0e-7f;

    static constexpr float kDenormalThreshold =
        1.0e-30f;

    // +120 dBFS: enormous safety margin over any legitimate audio level.
    static constexpr float kMaximumSafeAudioMagnitude =
        1.0e6f;

    float sampleRate_ = 44100.0f;
    float phaseSlewPerSample_ = 1.0f;
    float currentPhase_ = 0.0f;

    int activeTapCount_ = kStandardTapCount;
    int groupDelaySamples_ = (kStandardTapCount - 1) / 2;
    int pairCount_ = groupDelaySamples_ / 2;

    // Only positive-side coefficients are stored. The negative-side tap is the
    // exact negative of each positive-side tap for a Type-III Hilbert FIR.
    std::array<float, kMaximumPairCount>
        pairCoefficients_ {};

    std::array<float, kMaximumTapCount>
        delayLine_ {};

    int writeIndex_ = 0;

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

    void configureForSampleRate()
    {
        if (sampleRate_ > kHighRateThresholdHz)
            activeTapCount_ = kHighRateTapCount;
        else
            activeTapCount_ = kStandardTapCount;

        groupDelaySamples_ =
            (activeTapCount_ - 1) / 2;

        pairCount_ =
            groupDelaySamples_ / 2;
    }

    void resetState()
    {
        delayLine_.fill(0.0f);
        writeIndex_ = 0;
    }

    int wrapDelayIndex(int delaySamples) const
    {
        int index =
            writeIndex_ - delaySamples;

        // delaySamples is always within [0, activeTapCount_-1], therefore one
        // addition is sufficient to wrap a negative index into range.
        if (index < 0)
            index += activeTapCount_;

        return index;
    }

    float calculateQuadrature() const
    {
        double accumulator = 0.0;

        for (int pair = 0; pair < pairCount_; ++pair)
        {
            const int k =
                2 * pair + 1;

            const int olderDelay =
                groupDelaySamples_ + k;

            const int newerDelay =
                groupDelaySamples_ - k;

            int olderIndex =
                writeIndex_ - olderDelay;

            if (olderIndex < 0)
                olderIndex += activeTapCount_;

            int newerIndex =
                writeIndex_ - newerDelay;

            if (newerIndex < 0)
                newerIndex += activeTapCount_;

            const double difference =
                static_cast<double>(delayLine_[static_cast<std::size_t>(olderIndex)])
                - static_cast<double>(delayLine_[static_cast<std::size_t>(newerIndex)]);

            accumulator +=
                static_cast<double>(pairCoefficients_[static_cast<std::size_t>(pair)])
                * difference;
        }

        if (!std::isfinite(accumulator))
            return 0.0f;

        accumulator = (std::max)(
            -static_cast<double>(kMaximumSafeAudioMagnitude),
            (std::min)(
                static_cast<double>(kMaximumSafeAudioMagnitude),
                accumulator));

        return zapDenormal(
            static_cast<float>(accumulator));
    }

    float processOneSample(
        float x,
        float directCoefficient,
        float quadratureCoefficient,
        bool needsQuadrature)
    {
        // writeIndex_ is always wrapped to [0, activeTapCount_-1].
        delayLine_[static_cast<std::size_t>(writeIndex_)] = x;

        const int directIndex =
            wrapDelayIndex(groupDelaySamples_);

        const float delayedInput =
            delayLine_[static_cast<std::size_t>(directIndex)];

        float quadrature = 0.0f;

        if (needsQuadrature)
            quadrature = calculateQuadrature();

        const double y =
            static_cast<double>(delayedInput)
                * static_cast<double>(directCoefficient)
            - static_cast<double>(quadrature)
                * static_cast<double>(quadratureCoefficient);

        float outputSample = 0.0f;

        if (std::isfinite(y))
        {
            const double clamped = (std::max)(
                -static_cast<double>(kMaximumSafeAudioMagnitude),
                (std::min)(
                    static_cast<double>(kMaximumSafeAudioMagnitude),
                    y));

            outputSample =
                zapDenormal(static_cast<float>(clamped));
        }
        else
        {
            // An invalid intermediate implies corrupt upstream data or state.
            // Reset the FIR history so one bad sample cannot contaminate the
            // following filter-length worth of output.
            resetState();
            outputSample = 0.0f;
            return outputSample;
        }

        ++writeIndex_;

        if (writeIndex_ >= activeTapCount_)
            writeIndex_ = 0;

        return outputSample;
    }

    void buildHilbertTransformer()
    {
        pairCoefficients_.fill(0.0f);

        // Hamming-windowed ideal Hilbert impulse response. Hamming is chosen
        // deliberately here: compared with the previous Blackman window it
        // gives a substantially narrower low-frequency transition while still
        // keeping very small passband ripple.
        for (int pair = 0; pair < pairCount_; ++pair)
        {
            const int k =
                2 * pair + 1;

            const int n =
                groupDelaySamples_ + k;

            const double angle =
                2.0
                * kPi
                * static_cast<double>(n)
                / static_cast<double>(activeTapCount_ - 1);

            const double hamming =
                0.54 - 0.46 * std::cos(angle);

            const double coefficient =
                2.0
                / (kPi * static_cast<double>(k))
                * hamming;

            if (std::isfinite(coefficient))
                pairCoefficients_[static_cast<std::size_t>(pair)] =
                    static_cast<float>(coefficient);
            else
                pairCoefficients_[static_cast<std::size_t>(pair)] = 0.0f;
        }
    }
};

namespace
{
    // This ID MUST exactly match the XML Plugin id.
    auto registration =
        Register<PhaseRotatorFIR>::withId(
            L"Pandocrator Phase Rotator FIR SEM");
}
