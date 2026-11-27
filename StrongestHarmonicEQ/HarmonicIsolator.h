#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

// Independent, narrow tracking band-pass for the extra audio output.
// It does NOT touch StrongestPeakEngine's original EQ coefficients, state,
// detector, tracking, or processing path.
//
// An audio-rate band-pass cannot mathematically extract one perfect partial
// from an arbitrary mix.  It isolates a frequency neighbourhood around the
// strongest tracked spectral peak.  Isolation Q sets the width.
//
// TPT SVF band-pass, normalised to approximately 0 dB at its centre frequency.
// This filter topology affects ONLY the new Harmonic Only output.
class HarmonicIsolator
{
public:
    void prepare(float sampleRate) noexcept
    {
        sampleRate_ = finite(sampleRate) && sampleRate >= 1000.0f
                      && sampleRate <= 768000.0f ? sampleRate : 44100.0f;

        lowState_ = 0.0f;
        bandState_ = 0.0f;
        currentHz_ = 0.0f;
        qSmoothed_ = qTarget_;
        activity_ = 0.0f;
        tick_ = 0;
        g_ = 0.0f;
        k_ = 1.0f / qSmoothed_;
        a1_ = 1.0f;
        a2_ = 0.0f;
        a3_ = 0.0f;

        // Designed for a smooth *isolation* output. The main EQ has no
        // additional smoothing or latency and remains absolutely unchanged.
        frequencyAlpha_ = 1.0f - std::exp(-1.0f / (sampleRate_ * 0.010f));
        qAlpha_ = 1.0f - std::exp(-1.0f / (sampleRate_ * 0.015f));
        gateAlpha_ = 1.0f - std::exp(-1.0f / (sampleRate_ * 0.005f));
    }

    void setQ(float q) noexcept
    {
        if (finite(q))
            qTarget_ = std::clamp(q, 0.5f, 40.0f);
    }

    // trackedHz comes from original strongest-peak detector; validDetection
    // false when its latest FFT frame is silent. Audio is always the dry input.
    float process(float input, float trackedHz, bool validDetection) noexcept
    {
        if (!finite(input) || std::abs(input) > 1.0e20f)
            input = 0.0f;

        const float maxHz = 0.45f * sampleRate_;
        const bool valid = validDetection && finite(trackedHz)
                           && trackedHz >= 10.0f && trackedHz <= maxHz;

        if (valid && currentHz_ <= 0.0f)
        {
            currentHz_ = trackedHz;
            tick_ = 0; // never reuse coefficients from a cleared filter
        }

        if (valid)
            currentHz_ += frequencyAlpha_ * (trackedHz - currentHz_);

        qSmoothed_ += qAlpha_ * (qTarget_ - qSmoothed_);

        const float activityTarget = valid ? 1.0f : 0.0f;
        activity_ += gateAlpha_ * (activityTarget - activity_);

        if (currentHz_ <= 0.0f || (!valid && activity_ == 0.0f))
            return 0.0f;

        // Update on a short 8-sample cadence. Internal target frequency and Q
        // are smoothed per-sample; this avoids expensive tan() every sample.
        if ((tick_++ & 7) == 0)
        {
            const float f = std::clamp(currentHz_, 10.0f, maxHz);
            const float q = std::clamp(qSmoothed_, 0.5f, 40.0f);
            g_ = std::tan(3.14159265358979323846f * f / sampleRate_);
            k_ = 1.0f / q;
            a1_ = 1.0f / (1.0f + g_ * (g_ + k_));
            a2_ = g_ * a1_;
            a3_ = g_ * a2_;

            if (!finite(a1_) || !finite(a2_) || !finite(a3_))
            {
                clear();
                return 0.0f;
            }
        }

        // TPT SVF, equivalent to a normalised 2-pole band-pass at steady state.
        // Coefficient changes are confined to this parallel signal path.
        const float v3 = input - lowState_;
        const float v1 = a1_ * bandState_ + a2_ * v3;
        const float v2 = lowState_ + a2_ * bandState_ + a3_ * v3;

        bandState_ = 2.0f * v1 - bandState_;
        lowState_  = 2.0f * v2 - lowState_;

        const float result = activity_ * k_ * v1;

        if (!finite(result) || !finite(bandState_) || !finite(lowState_)
            || std::abs(result) > 1.0e8f
            || std::abs(bandState_) > 1.0e9f
            || std::abs(lowState_) > 1.0e9f)
        {
            clear();
            return 0.0f;
        }

        if (activity_ < 1.0e-5f && !valid)
        {
            // Stop long narrow-band resonances after the detector goes silent.
            bandState_ = 0.0f;
            lowState_ = 0.0f;
            activity_ = 0.0f;
            // Keep currentHz_ for the original tracking behaviour, but skip
            // processing on later silent samples until detection returns.
        }

        if (std::abs(bandState_) < 1.0e-20f) bandState_ = 0.0f;
        if (std::abs(lowState_) < 1.0e-20f) lowState_ = 0.0f;

        return result;
    }

private:
    static bool finite(float x) noexcept
    {
        std::uint32_t bits = 0;
        std::memcpy(&bits, &x, sizeof(bits));
        return (bits & 0x7f800000U) != 0x7f800000U;
    }

    void clear() noexcept
    {
        bandState_ = 0.0f;
        lowState_  = 0.0f;
        activity_ = 0.0f;
        currentHz_ = 0.0f;
    }

    float sampleRate_ = 44100.0f;
    float qTarget_ = 12.0f;
    float qSmoothed_ = 12.0f;
    float currentHz_ = 0.0f;
    float activity_ = 0.0f;
    float frequencyAlpha_ = 0.0f;
    float qAlpha_ = 0.0f;
    float gateAlpha_ = 0.0f;
    float lowState_ = 0.0f;
    float bandState_ = 0.0f;
    float g_ = 0.0f;
    float k_ = 1.0f / 12.0f;
    float a1_ = 1.0f;
    float a2_ = 0.0f;
    float a3_ = 0.0f;
    unsigned tick_ = 0;
};
