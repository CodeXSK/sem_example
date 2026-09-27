#include "mp_sdk_audio.h"

#include <algorithm>
#include <cmath>

using namespace gmpi;

namespace
{
    // SynthEdit's normal control/audio pin presentation is x10 relative to the
    // raw float arriving at the module. This matches the supplied working SEM.
    constexpr float kInternalToDisplayedUnits = 10.0f;

    constexpr float kMinAttackMs = 0.5f;
    constexpr float kMaxAttackMs = 80.0f;
    constexpr float kMinHoldMs = 0.0f;
    constexpr float kMaxHoldMs = 120.0f;
    constexpr float kMinReleaseMs = 5.0f;
    constexpr float kMaxReleaseMs = 600.0f;

    constexpr float kDetectorFloor = 1.0e-8f;
    constexpr float kSilenceFloorDb = -78.0f;

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

    inline float coefficientFromMilliseconds(float milliseconds, float sampleRate)
    {
        const float safeMs = (std::max)(0.001f, milliseconds);
        const float samples = (std::max)(1.0f, safeMs * 0.001f * sampleRate);
        return std::exp(-1.0f / samples);
    }

    inline float followEnvelope(
        float inputLevel,
        float currentEnvelope,
        float attackCoefficient,
        float releaseCoefficient)
    {
        const float c = inputLevel > currentEnvelope
            ? attackCoefficient
            : releaseCoefficient;

        return zapDenormal(
            c * currentEnvelope + (1.0f - c) * inputLevel);
    }

    inline float smoothStep(float x)
    {
        x = clampf(x, 0.0f, 1.0f);
        return x * x * (3.0f - 2.0f * x);
    }
}

// -----------------------------------------------------------------------------
// Pandocrator Transient Separator v1
// -----------------------------------------------------------------------------
// Zero-lookahead transient/sustain splitter.
//
// The detector is stereo-linked:
//   - a fast envelope follows the immediate peak/onset
//   - a slower envelope estimates the underlying body/sustain
//   - their difference is evaluated in dB
//   - Hold keeps a detected transient open briefly
//   - Release closes the transient mask smoothly
//
// The split itself is complementary on every sample:
//
//   transient = input * mask
//   sustain   = input * (1 - mask)
//
// Therefore Transient + Sustain reconstructs the original input (apart from
// normal floating-point rounding), which makes the module safe to use as a
// true separator rather than two unrelated processors.
//
// Controls:
//   Attack ms  : speed of the slower/body envelope. Larger values classify
//                a longer initial portion of a hit as transient.
//   Hold ms    : minimum time the transient mask remains at its detected peak.
//   Release ms : how quickly the mask returns to sustain after the hit.
//   Sensitivity: 0..1. Higher values lower the dB threshold and detect more.
//   Separation : 0..1. Higher values make the mask harder/cleaner and reduce
//                grey overlap between transient and sustain.
class TransientSeparator final : public MpBase2
{
public:
    TransientSeparator()
    {
        // Pin order MUST match TransientSeparator.xml.
        initializePin(pinInputL);
        initializePin(pinInputR);
        initializePin(pinAttackMs);
        initializePin(pinHoldMs);
        initializePin(pinReleaseMs);
        initializePin(pinSensitivity);
        initializePin(pinSeparation);
        initializePin(pinTransientL);
        initializePin(pinTransientR);
        initializePin(pinSustainL);
        initializePin(pinSustainR);
    }

    void subProcess(int sampleFrames)
    {
        auto inputL = getBuffer(pinInputL);
        auto inputR = getBuffer(pinInputR);

        auto attackPin = getBuffer(pinAttackMs);
        auto holdPin = getBuffer(pinHoldMs);
        auto releasePin = getBuffer(pinReleaseMs);
        auto sensitivityPin = getBuffer(pinSensitivity);
        auto separationPin = getBuffer(pinSeparation);

        auto transientL = getBuffer(pinTransientL);
        auto transientR = getBuffer(pinTransientR);
        auto sustainL = getBuffer(pinSustainL);
        auto sustainR = getBuffer(pinSustainR);

        updateSampleRateIfNeeded();

        const float attackMs = clampf(
            finiteOrZero(*attackPin) * kInternalToDisplayedUnits,
            kMinAttackMs, kMaxAttackMs);

        const float holdMs = clampf(
            finiteOrZero(*holdPin) * kInternalToDisplayedUnits,
            kMinHoldMs, kMaxHoldMs);

        const float releaseMs = clampf(
            finiteOrZero(*releasePin) * kInternalToDisplayedUnits,
            kMinReleaseMs, kMaxReleaseMs);

        const float sensitivity = clampf(
            finiteOrZero(*sensitivityPin) * kInternalToDisplayedUnits,
            0.0f, 1.0f);

        const float separation = clampf(
            finiteOrZero(*separationPin) * kInternalToDisplayedUnits,
            0.0f, 1.0f);

        // Very fast onset follower. A tiny but non-zero time avoids zippery
        // sample-to-sample peak behaviour on bright material.
        const float fastAttackCoefficient =
            coefficientFromMilliseconds(0.03f, currentSampleRate_);

        // Attack control also shapes how long the fast envelope remains above
        // the body estimate, but is bounded to remain responsive.
        const float fastReleaseCoefficient =
            coefficientFromMilliseconds(
                clampf(attackMs * 0.75f, 6.0f, 30.0f),
                currentSampleRate_);

        const float slowAttackCoefficient =
            coefficientFromMilliseconds(attackMs, currentSampleRate_);

        const float slowReleaseCoefficient =
            coefficientFromMilliseconds(
                (std::max)(releaseMs, 40.0f),
                currentSampleRate_);

        // Open the actual separation mask almost instantly, then let the user
        // determine the close behaviour with Release.
        const float maskAttackCoefficient =
            coefficientFromMilliseconds(0.03f, currentSampleRate_);

        const float maskReleaseCoefficient =
            coefficientFromMilliseconds(releaseMs, currentSampleRate_);

        const int holdSamples = (std::max)(0,
            static_cast<int>(std::lround(
                holdMs * 0.001f * currentSampleRate_)));

        // Sensitivity 0 -> conservative ~6 dB onset requirement.
        // Sensitivity 1 -> very sensitive ~0.35 dB onset requirement.
        const float thresholdDb = 6.0f + (0.35f - 6.0f) * sensitivity;

        // Separation determines how quickly the detector reaches a full mask.
        // High Separation narrows the transition band and applies extra
        // contrast, giving cleaner solo Transient/Sustain outputs.
        const float transitionWidthDb = 12.0f + (3.0f - 12.0f) * separation;
        const float fullScaleDb = thresholdDb + transitionWidthDb;
        const float contrastExponent = 1.0f + 3.5f * separation;

        for (int s = 0; s < sampleFrames; ++s)
        {
            const float inL = finiteOrZero(*inputL++);
            const float inR = finiteOrZero(*inputR++);

            const float absL = std::abs(inL);
            const float absR = std::abs(inR);
            const float peakLevel = (std::max)(absL, absR);
            const float rmsLevel = std::sqrt(0.5f * (inL * inL + inR * inR));

            // Peak-weighted detector retains punch while RMS stabilises it.
            const float detectorInput = 0.72f * peakLevel + 0.28f * rmsLevel;

            fastEnvelope_ = followEnvelope(
                detectorInput,
                fastEnvelope_,
                fastAttackCoefficient,
                fastReleaseCoefficient);

            slowEnvelope_ = followEnvelope(
                detectorInput,
                slowEnvelope_,
                slowAttackCoefficient,
                slowReleaseCoefficient);

            const float fastDb = 20.0f * std::log10(
                (std::max)(fastEnvelope_, kDetectorFloor));
            const float slowDb = 20.0f * std::log10(
                (std::max)(slowEnvelope_, kDetectorFloor));

            const float differenceDb = (std::max)(0.0f, fastDb - slowDb);

            float rawMask = 0.0f;
            if (fastDb > kSilenceFloorDb && differenceDb > thresholdDb)
            {
                const float normalised = clampf(
                    (differenceDb - thresholdDb) /
                        (std::max)(0.001f, fullScaleDb - thresholdDb),
                    0.0f, 1.0f);

                rawMask = smoothStep(normalised);
                rawMask = std::pow(rawMask, contrastExponent);
            }

            // Peak-hold behaviour. A stronger new transient can immediately
            // replace the previous held value.
            if (rawMask >= heldMask_)
            {
                heldMask_ = rawMask;
                holdSamplesRemaining_ = holdSamples;
            }
            else if (holdSamplesRemaining_ > 0)
            {
                --holdSamplesRemaining_;
            }
            else
            {
                heldMask_ = rawMask;
            }

            const float maskCoefficient = heldMask_ > transientMask_
                ? maskAttackCoefficient
                : maskReleaseCoefficient;

            transientMask_ = zapDenormal(
                maskCoefficient * transientMask_ +
                (1.0f - maskCoefficient) * heldMask_);
            transientMask_ = clampf(transientMask_, 0.0f, 1.0f);

            const float sustainMask = 1.0f - transientMask_;

            *transientL++ = zapDenormal(inL * transientMask_);
            *transientR++ = zapDenormal(inR * transientMask_);
            *sustainL++ = zapDenormal(inL * sustainMask);
            *sustainR++ = zapDenormal(inR * sustainMask);
        }
    }

    void onSetPins() override
    {
        pinTransientL.setStreaming(true);
        pinTransientR.setStreaming(true);
        pinSustainL.setStreaming(true);
        pinSustainR.setStreaming(true);

        // Keep detector state coherent even when SynthEdit would otherwise put
        // the module to sleep between short bursts.
        setSleep(false);
        setSubProcess(&TransientSeparator::subProcess);
    }

private:
    void updateSampleRateIfNeeded()
    {
        const float newRate = (std::max)(1000.0f, finiteOrZero(getSampleRate()));
        if (sampleRateInitialized_ && std::abs(newRate - currentSampleRate_) < 0.5f)
            return;

        sampleRateInitialized_ = true;
        currentSampleRate_ = newRate;
        resetDetector();
    }

    void resetDetector()
    {
        fastEnvelope_ = 0.0f;
        slowEnvelope_ = 0.0f;
        transientMask_ = 0.0f;
        heldMask_ = 0.0f;
        holdSamplesRemaining_ = 0;
    }

    AudioInPin pinInputL;
    AudioInPin pinInputR;

    AudioInPin pinAttackMs;
    AudioInPin pinHoldMs;
    AudioInPin pinReleaseMs;
    AudioInPin pinSensitivity;
    AudioInPin pinSeparation;

    AudioOutPin pinTransientL;
    AudioOutPin pinTransientR;
    AudioOutPin pinSustainL;
    AudioOutPin pinSustainR;

    float fastEnvelope_ = 0.0f;
    float slowEnvelope_ = 0.0f;
    float transientMask_ = 0.0f;
    float heldMask_ = 0.0f;
    int holdSamplesRemaining_ = 0;

    float currentSampleRate_ = 44100.0f;
    bool sampleRateInitialized_ = false;
};

namespace
{
    auto registration =
        Register<TransientSeparator>::withId(L"Pandocrator Transient Separator v1");
}
