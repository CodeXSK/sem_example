#pragma once

// Adaptive DC Blocker v1: independently testable signal processing engine.
// Copyright (c) 2026 Pandocrator. All rights reserved.
// The SynthEdit SDK is not included in this source distribution.

#include <algorithm>
#include <cmath>
#include <cstdint>

class AdaptiveDCBlockerDSP final
{
public:
    void prepare(float sampleRate)
    {
        if (!std::isfinite(sampleRate))
            sampleRate = 48000.0f;
        sampleRate_ = std::clamp(sampleRate, 8000.0f, 768000.0f);
        reset();
        update(5.0f, -90.0f, 5.0f, 3.0f, 2.0f);
    }

    // Recompute once per host block. Parameters are in physical units.
    void update(float cutoffHz, float thresholdDb, float holdMs,
                float tailFadeMs, float wakeFadeMs, float slope = 2.0f)
    {
        cutoffHz  = finiteClamp(cutoffHz, 5.0f, 0.1f, 200.0f);
        thresholdDb = finiteClamp(thresholdDb, -90.0f, -140.0f, -20.0f);
        holdMs = finiteClamp(holdMs, 5.0f, 0.2f, 1000.0f);
        tailFadeMs = finiteClamp(tailFadeMs, 3.0f, 0.2f, 100.0f);
        wakeFadeMs = finiteClamp(wakeFadeMs, 2.0f, 0.2f, 100.0f);
        slope_ = finiteClamp(slope, 2.0f, 1.0f, 2.0f) >= 1.5f ? 2 : 1;

        if (cutoffHz != lastCutoffHz_)
        {
            r_ = std::exp(-2.0f * 3.14159265358979323846f * cutoffHz / sampleRate_);
            lastCutoffHz_ = cutoffHz;
        }
        if (thresholdDb != lastThresholdDb_)
        {
            threshold_ = std::pow(10.0f, thresholdDb * 0.05f);
            lastThresholdDb_ = thresholdDb;
        }
        holdSamples_ = samplesFromMs(holdMs);
        tailSamples_ = samplesFromMs(tailFadeMs);
        wakeSamples_ = samplesFromMs(wakeFadeMs);
    }

    struct Sample
    {
        float left;
        float right;
        float gain;
    };

    // externalDetect=true: evaluate the unprocessed/dry detector input.
    // externalDetect=false: detect activity from max(abs(L), abs(R)).
    Sample process(float inL, float inR, float detectIn, bool externalDetect)
    {
        inL = safeAudio(inL);
        inR = safeAudio(inR);
        detectIn = safeAudio(detectIn);

        const float level = externalDetect
            ? std::abs(detectIn)
            : (std::max)(std::abs(inL), std::abs(inR));
        const bool activity = level > threshold_;

        if (activity)
        {
            inactiveSamples_ = 0;
            if (phase_ == Phase::Silent || phase_ == Phase::FadingOut)
                startFade(Phase::FadingIn, wakeSamples_);
        }
        else if (phase_ != Phase::Silent)
        {
            if (inactiveSamples_ < holdSamples_)
                ++inactiveSamples_;
            if (inactiveSamples_ >= holdSamples_
                && phase_ != Phase::FadingOut)
                startFade(Phase::FadingOut, tailSamples_);
        }

        if (phase_ == Phase::Silent)
            return {0.0f, 0.0f, 0.0f};

        // First-order DC blocker, unity gain at 0 Hz except exact DC null.
        // History gets reset only once the final fade reaches EXACT zero.
        const float outL = inL - previousInL_ + r_ * previousOutL_;
        const float outR = inR - previousInR_ + r_ * previousOutR_;
        previousInL_ = inL;
        previousInR_ = inR;
        previousOutL_ = zeroDenormal(outL);
        previousOutR_ = zeroDenormal(outR);

        // Optional second pole offers more attenuation of near-DC subsonics.
        // Always update both poles to keep state valid if slope is changed.
        const float out2L = outL - previousStage2InL_ + r_ * previousStage2OutL_;
        const float out2R = outR - previousStage2InR_ + r_ * previousStage2OutR_;
        previousStage2InL_ = outL;
        previousStage2InR_ = outR;
        previousStage2OutL_ = zeroDenormal(out2L);
        previousStage2OutR_ = zeroDenormal(out2R);
        const float selectedL = slope_ == 2 ? out2L : outL;
        const float selectedR = slope_ == 2 ? out2R : outR;

        if (phase_ == Phase::FadingIn || phase_ == Phase::FadingOut)
        {
            // Cubic smoothstep: zero first derivative at both ends.
            // Unlike an IIR envelope it finishes at exactly zero in finite time.
            if (fadeProgress_ < fadeDuration_)
                ++fadeProgress_;
            const float t = static_cast<float>(fadeProgress_)
                / static_cast<float>(fadeDuration_);
            const float smooth = t * t * (3.0f - 2.0f * t);
            const float target = phase_ == Phase::FadingIn ? 1.0f : 0.0f;
            gain_ = fadeStartGain_ + (target - fadeStartGain_) * smooth;

            if (fadeProgress_ >= fadeDuration_)
            {
                if (phase_ == Phase::FadingOut)
                {
                    reset(); // EXACT silence: no filter, fade or detector residue.
                    return {0.0f, 0.0f, 0.0f};
                }
                phase_ = Phase::Open;
                gain_ = 1.0f;
            }
        }
        if (!std::isfinite(outL) || !std::isfinite(outR)
            || !std::isfinite(out2L) || !std::isfinite(out2R)
            || !std::isfinite(gain_))
        {
            reset();
            return {0.0f, 0.0f, 0.0f};
        }

        return {selectedL * gain_, selectedR * gain_, gain_};
    }

    void reset()
    {
        previousInL_ = previousInR_ = 0.0f;
        previousOutL_ = previousOutR_ = 0.0f;
        previousStage2InL_ = previousStage2InR_ = 0.0f;
        previousStage2OutL_ = previousStage2OutR_ = 0.0f;
        gain_ = 0.0f;
        fadeStartGain_ = 0.0f;
        fadeProgress_ = 0;
        fadeDuration_ = 1;
        inactiveSamples_ = 0;
        phase_ = Phase::Silent;
    }

private:
    enum class Phase { Silent, FadingIn, Open, FadingOut };

    static float finiteClamp(float v, float fallback, float lo, float hi)
    {
        return std::isfinite(v) ? std::clamp(v, lo, hi) : fallback;
    }

    static float safeAudio(float v)
    {
        return finiteClamp(v, 0.0f, -128.0f, 128.0f);
    }

    static float zeroDenormal(float v)
    {
        return std::abs(v) < 1.0e-25f ? 0.0f : v;
    }

    int samplesFromMs(float ms) const
    {
        return (std::max)(1, static_cast<int>(std::lround(sampleRate_ * 0.001f * ms)));
    }

    void startFade(Phase next, int frames)
    {
        fadeStartGain_ = gain_;
        fadeProgress_ = 0;
        fadeDuration_ = (std::max)(1, frames);
        phase_ = next;
    }

    float sampleRate_ = 48000.0f;
    float r_ = 0.999f;
    float threshold_ = 0.0000316228f;
    float previousInL_ = 0.0f;
    float previousInR_ = 0.0f;
    float previousOutL_ = 0.0f;
    float previousOutR_ = 0.0f;
    float previousStage2InL_ = 0.0f;
    float previousStage2InR_ = 0.0f;
    float previousStage2OutL_ = 0.0f;
    float previousStage2OutR_ = 0.0f;
    int slope_ = 2;
    float gain_ = 0.0f;
    float fadeStartGain_ = 0.0f;
    int inactiveSamples_ = 0;
    int holdSamples_ = 240;
    int tailSamples_ = 144;
    int wakeSamples_ = 96;
    int fadeProgress_ = 0;
    int fadeDuration_ = 1;
    Phase phase_ = Phase::Silent;
    float lastCutoffHz_ = -1.0f;
    float lastThresholdDb_ = 12345.0f;
};
