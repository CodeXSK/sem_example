#include "../AdaptiveDCBlockerDSP.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

int main()
{
    constexpr int sr = 48000;
    constexpr float pi = 3.14159265358979323846f;
    AdaptiveDCBlockerDSP d;
    d.prepare(sr);
    d.update(5.f, -90.f, 5.f, 3.f, 2.f);

    // The blocker must not retain an offset after settling with AC + DC.
    double mean = 0.0;
    int count = 0;
    for (int i = 0; i < 5 * sr; ++i)
    {
        const float input = 0.4f + 0.5f * std::sin(2.0f * pi * 120.0f * i / sr);
        auto s = d.process(input, input * 0.5f, 0.0f, false);
        assert(std::isfinite(s.left) && std::isfinite(s.right));
        if (i >= 4 * sr) { mean += s.left; ++count; }
    }
    mean /= count;
    assert(std::abs(mean) < 0.00002);

    // Stop the signal abruptly; fixed hold + fade MUST give exact zero.
    for (int i = 0; i < 1000; ++i)
    {
        auto s = d.process(0, 0, 0, false);
        if (i >= 410)
            assert(s.left == 0.f && s.right == 0.f && s.gain == 0.f);
    }

    // Even a low-frequency waveform regularly crossing zero stays open.
    d.prepare(sr);
    d.update(1.f, -90.f, 5.f, 3.f, 2.f);
    float minGate = 1.0f;
    for (int i = 0; i < sr; ++i)
    {
        const float x = 0.6f * std::sin(2.0f * pi * 20.0f * i / sr);
        auto s = d.process(x, 0.0f, 0.0f, false);
        if (i >= sr / 2)
            minGate = (std::min)(minGate, s.gain);
    }
    assert(minGate > 0.999f);

    // Two poles at 5 Hz should suppress a 1 Hz sine by roughly 28 dB.
    d.prepare(sr);
    d.update(5.f, -90.f, 5.f, 3.f, 2.f, 2.f);
    double inSquare = 0., outSquare = 0.;
    for (int i = 0; i < 12 * sr; ++i)
    {
        const float x = 0.5f * std::sin(2.0f * pi * i / sr);
        const auto s = d.process(x, 0.f, 0.f, false);
        if (i >= 10 * sr)
        {
            inSquare += x * x;
            outSquare += s.left * s.left;
        }
    }
    const double oneHzGain = std::sqrt(outSquare / inSquare);
    assert(oneHzGain > 0.03 && oneHzGain < 0.05);
    std::cout << "1 Hz at 5 Hz 2-pole: " << 20 * std::log10(oneHzGain) << " dB\n";

    // External detector allows the dry input to control the gated distorted path.
    d.prepare(sr);
    d.update(5.f, -90.f, 1.f, 2.f, 1.f);
    for (int i = 0; i < 2000; ++i)
    {
        const float dry = 0.2f * std::sin(2.0f * pi * 500.0f * i / sr);
        const float wet = dry * (dry + 3.0f);
        auto s = d.process(wet, wet, dry, true);
        assert(std::isfinite(s.left));
    }
    bool exactlySilent = false;
    for (int i = 0; i < 300; ++i)
    {
        auto s = d.process(0.f, 0.f, 0.f, true);
        if (i > 150 && s.left == 0.f && s.gain == 0.f)
            exactlySilent = true;
    }
    assert(exactlySilent);

    // Recover from NaN input without poisoning DSP state.
    auto bad = d.process(std::numeric_limits<float>::quiet_NaN(), 0.f, 0.f, false);
    assert(std::isfinite(bad.left) && std::isfinite(bad.right));
    std::cout << "PASS: DC removal, exact zero tail, low-frequency detection, external trigger, NaN safety.\n";
}
