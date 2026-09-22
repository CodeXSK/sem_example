#include "mp_sdk_audio.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

using namespace gmpi;

namespace
{
    constexpr int kMaxChannels = 16;
    constexpr float kInternalToDisplayedUnits = 10.0f;

    inline float finiteOrZero(float x)
    {
        return std::isfinite(x) ? x : 0.0f;
    }

    inline int clampi(int x, int lo, int hi)
    {
        return (std::max)(lo, (std::min)(hi, x));
    }

    inline std::uint32_t xorshift32(std::uint32_t& state)
    {
        if (state == 0)
            state = 0x6d2b79f5u;

        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return state;
    }

    inline float randomBipolar(std::uint32_t& state)
    {
        const std::uint32_t v = xorshift32(state);
        const float unit = static_cast<float>(v & 0x00ffffffu) / 16777215.0f;
        return unit * 2.0f - 1.0f;
    }

    int snapMatrixSize(int requested)
    {
        constexpr std::array<int, 4> sizes{{2, 4, 8, 16}};
        int best = sizes[0];
        int bestDistance = std::abs(requested - best);

        for (int s : sizes)
        {
            const int d = std::abs(requested - s);
            if (d < bestDistance || (d == bestDistance && s > best))
            {
                best = s;
                bestDistance = d;
            }
        }
        return best;
    }
}

// Generic energy-preserving matrix mixer for SynthEdit.
//
// Matrix Size: 2, 4, 8 or 16 active channels.
// Matrix Type (displayed integer):
//   0 = Identity / bypass matrix
//   1 = Normalized Hadamard
//   2 = Householder reflection
//   3 = Seeded dense random orthogonal matrix
//
// Shuffle Channels and Shuffle Polarity pre/post multiply the selected matrix
// by deterministic permutation/sign matrices derived from Seed. These operations
// preserve orthogonality, so the complete transform remains energy-preserving.
//
// Inactive outputs (above Matrix Size) are written as zero.
class OrthogonalMatrixMixer final : public MpBase2
{
public:
    OrthogonalMatrixMixer()
    {
        // Pin order MUST match OrthogonalMatrixMixer.xml.
        initializePin(pinMatrixSize);
        initializePin(pinMatrixType);
        initializePin(pinSeed);
        initializePin(pinShuffleChannels);
        initializePin(pinShufflePolarity);

        for (auto& pin : pinInputs)
            initializePin(pin);

        for (auto& pin : pinOutputs)
            initializePin(pin);

        rebuildConfiguration(8, 1, 1u, false, false);
    }

    void subProcess(int sampleFrames)
    {
        auto sizeBuffer = getBuffer(pinMatrixSize);
        auto typeBuffer = getBuffer(pinMatrixType);
        auto seedBuffer = getBuffer(pinSeed);
        auto shuffleChannelsBuffer = getBuffer(pinShuffleChannels);
        auto shufflePolarityBuffer = getBuffer(pinShufflePolarity);

        std::array<float*, kMaxChannels> inputBuffers{};
        std::array<float*, kMaxChannels> outputBuffers{};

        for (int i = 0; i < kMaxChannels; ++i)
        {
            inputBuffers[static_cast<std::size_t>(i)] =
                getBuffer(pinInputs[static_cast<std::size_t>(i)]);
            outputBuffers[static_cast<std::size_t>(i)] =
                getBuffer(pinOutputs[static_cast<std::size_t>(i)]);
        }

        // SynthEdit audio/CV pins store 1/10 of the voltage displayed in the editor.
        // Sample these routing controls once per processing block.
        const int requestedSize = static_cast<int>(std::lround(
            finiteOrZero(*sizeBuffer) * kInternalToDisplayedUnits));
        const int matrixSize = snapMatrixSize(requestedSize);

        const int matrixType = clampi(static_cast<int>(std::lround(
            finiteOrZero(*typeBuffer) * kInternalToDisplayedUnits)), 0, 3);

        int seedValue = static_cast<int>(std::lround(
            std::abs(finiteOrZero(*seedBuffer) * kInternalToDisplayedUnits)));
        seedValue = clampi(seedValue, 1, 1000000);
        const std::uint32_t seed = static_cast<std::uint32_t>(seedValue);

        const bool shuffleChannels =
            finiteOrZero(*shuffleChannelsBuffer) * kInternalToDisplayedUnits >= 0.5f;
        const bool shufflePolarity =
            finiteOrZero(*shufflePolarityBuffer) * kInternalToDisplayedUnits >= 0.5f;

        if (matrixSize != activeSize_
            || matrixType != activeType_
            || seed != activeSeed_
            || shuffleChannels != activeShuffleChannels_
            || shufflePolarity != activeShufflePolarity_)
        {
            rebuildConfiguration(
                matrixSize,
                matrixType,
                seed,
                shuffleChannels,
                shufflePolarity);
        }

        std::array<float, kMaxChannels> raw{};
        std::array<float, kMaxChannels> pre{};
        std::array<float, kMaxChannels> transformed{};
        std::array<float, kMaxChannels> finalOut{};

        for (int s = sampleFrames; s > 0; --s)
        {
            raw.fill(0.0f);
            pre.fill(0.0f);
            transformed.fill(0.0f);
            finalOut.fill(0.0f);

            // Read every input every sample, even if currently inactive, so pin
            // buffers remain correctly advanced if Matrix Size changes later.
            for (int i = 0; i < kMaxChannels; ++i)
                raw[static_cast<std::size_t>(i)] =
                    finiteOrZero(*inputBuffers[static_cast<std::size_t>(i)]++);

            // Optional input-side signed permutation.
            for (int i = 0; i < activeSize_; ++i)
            {
                const int source = sourcePermutation_[static_cast<std::size_t>(i)];
                pre[static_cast<std::size_t>(i)] =
                    raw[static_cast<std::size_t>(source)]
                    * inputSigns_[static_cast<std::size_t>(i)];
            }

            switch (activeType_)
            {
            case 0: // Identity.
                for (int i = 0; i < activeSize_; ++i)
                    transformed[static_cast<std::size_t>(i)] = pre[static_cast<std::size_t>(i)];
                break;

            case 1: // Normalized Walsh-Hadamard transform.
                transformed = pre;
                for (int half = 1; half < activeSize_; half <<= 1)
                {
                    const int block = half << 1;
                    for (int base = 0; base < activeSize_; base += block)
                    {
                        for (int j = 0; j < half; ++j)
                        {
                            const int aIndex = base + j;
                            const int bIndex = aIndex + half;
                            const float a = transformed[static_cast<std::size_t>(aIndex)];
                            const float b = transformed[static_cast<std::size_t>(bIndex)];
                            transformed[static_cast<std::size_t>(aIndex)] = a + b;
                            transformed[static_cast<std::size_t>(bIndex)] = a - b;
                        }
                    }
                }
                {
                    const float normalization = 1.0f / std::sqrt(static_cast<float>(activeSize_));
                    for (int i = 0; i < activeSize_; ++i)
                        transformed[static_cast<std::size_t>(i)] *= normalization;
                }
                break;

            case 2: // Householder: H = I - (2/N) * 11^T.
                {
                    float sum = 0.0f;
                    for (int i = 0; i < activeSize_; ++i)
                        sum += pre[static_cast<std::size_t>(i)];

                    const float common = 2.0f * sum / static_cast<float>(activeSize_);
                    for (int i = 0; i < activeSize_; ++i)
                        transformed[static_cast<std::size_t>(i)] =
                            pre[static_cast<std::size_t>(i)] - common;
                }
                break;

            case 3: // Dense seeded random orthogonal matrix.
            default:
                for (int row = 0; row < activeSize_; ++row)
                {
                    float sum = 0.0f;
                    for (int col = 0; col < activeSize_; ++col)
                    {
                        sum += randomOrthogonal_[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)]
                            * pre[static_cast<std::size_t>(col)];
                    }
                    transformed[static_cast<std::size_t>(row)] = finiteOrZero(sum);
                }
                break;
            }

            // Optional output-side signed permutation.
            for (int i = 0; i < activeSize_; ++i)
            {
                const int destination = destinationPermutation_[static_cast<std::size_t>(i)];
                finalOut[static_cast<std::size_t>(destination)] =
                    finiteOrZero(transformed[static_cast<std::size_t>(i)]
                        * outputSigns_[static_cast<std::size_t>(i)]);
            }

            for (int i = 0; i < kMaxChannels; ++i)
                *outputBuffers[static_cast<std::size_t>(i)]++ =
                    finalOut[static_cast<std::size_t>(i)];
        }
    }

    void onSetPins() override
    {
        for (auto& pin : pinOutputs)
            pin.setStreaming(true);

        setSleep(false);
        setSubProcess(&OrthogonalMatrixMixer::subProcess);
    }

private:
    void rebuildConfiguration(
        int matrixSize,
        int matrixType,
        std::uint32_t seed,
        bool shuffleChannels,
        bool shufflePolarity)
    {
        activeSize_ = snapMatrixSize(matrixSize);
        activeType_ = clampi(matrixType, 0, 3);
        activeSeed_ = seed == 0 ? 1u : seed;
        activeShuffleChannels_ = shuffleChannels;
        activeShufflePolarity_ = shufflePolarity;

        for (int i = 0; i < kMaxChannels; ++i)
        {
            sourcePermutation_[static_cast<std::size_t>(i)] = i;
            destinationPermutation_[static_cast<std::size_t>(i)] = i;
            inputSigns_[static_cast<std::size_t>(i)] = 1.0f;
            outputSigns_[static_cast<std::size_t>(i)] = 1.0f;
        }

        // Use independent deterministic streams for routing and the dense random
        // orthogonal matrix, so toggling Shuffle does not silently change Q.
        std::uint32_t routingState = activeSeed_ ^ 0xa341316cu;
        std::uint32_t matrixState = activeSeed_ ^ 0xc8013ea4u;

        if (activeShuffleChannels_)
        {
            for (int i = activeSize_ - 1; i > 0; --i)
            {
                const int j = static_cast<int>(
                    xorshift32(routingState) % static_cast<std::uint32_t>(i + 1));
                std::swap(
                    sourcePermutation_[static_cast<std::size_t>(i)],
                    sourcePermutation_[static_cast<std::size_t>(j)]);
            }

            for (int i = activeSize_ - 1; i > 0; --i)
            {
                const int j = static_cast<int>(
                    xorshift32(routingState) % static_cast<std::uint32_t>(i + 1));
                std::swap(
                    destinationPermutation_[static_cast<std::size_t>(i)],
                    destinationPermutation_[static_cast<std::size_t>(j)]);
            }
        }

        if (activeShufflePolarity_)
        {
            for (int i = 0; i < activeSize_; ++i)
            {
                inputSigns_[static_cast<std::size_t>(i)] =
                    randomBipolar(routingState) >= 0.0f ? 1.0f : -1.0f;
                outputSigns_[static_cast<std::size_t>(i)] =
                    randomBipolar(routingState) >= 0.0f ? 1.0f : -1.0f;
            }
        }

        buildRandomOrthogonal(activeSize_, matrixState);
    }

    void buildRandomOrthogonal(int size, std::uint32_t state)
    {
        // Start from the identity matrix.
        for (int row = 0; row < kMaxChannels; ++row)
        {
            for (int col = 0; col < kMaxChannels; ++col)
            {
                randomOrthogonal_[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)] =
                    (row == col && row < size) ? 1.0f : 0.0f;
            }
        }

        // Product of random Householder reflections. Each reflection is exactly
        // orthogonal in theory; their product produces a dense, deterministic,
        // energy-preserving matrix without an external linear-algebra library.
        std::array<double, kMaxChannels> v{};

        for (int reflection = 0; reflection < size; ++reflection)
        {
            double normSquared = 0.0;
            for (int i = 0; i < size; ++i)
            {
                const double value = static_cast<double>(randomBipolar(state));
                v[static_cast<std::size_t>(i)] = value;
                normSquared += value * value;
            }

            if (normSquared < 1.0e-12)
            {
                v.fill(0.0);
                v[static_cast<std::size_t>(reflection % size)] = 1.0;
                normSquared = 1.0;
            }

            const double inverseNorm = 1.0 / std::sqrt(normSquared);
            for (int i = 0; i < size; ++i)
                v[static_cast<std::size_t>(i)] *= inverseNorm;

            // Left multiply Q by H = I - 2*v*v^T.
            for (int col = 0; col < size; ++col)
            {
                double dot = 0.0;
                for (int row = 0; row < size; ++row)
                {
                    dot += v[static_cast<std::size_t>(row)]
                        * static_cast<double>(randomOrthogonal_[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)]);
                }

                for (int row = 0; row < size; ++row)
                {
                    const double updated =
                        static_cast<double>(randomOrthogonal_[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)])
                        - 2.0 * v[static_cast<std::size_t>(row)] * dot;

                    randomOrthogonal_[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)] =
                        static_cast<float>(updated);
                }
            }
        }
    }

    // Controls.
    AudioInPin pinMatrixSize;
    AudioInPin pinMatrixType;
    AudioInPin pinSeed;
    AudioInPin pinShuffleChannels;
    AudioInPin pinShufflePolarity;

    // Fixed maximum I/O keeps the SynthEdit pin layout predictable. Matrix Size
    // chooses how many of the first channels are active.
    std::array<AudioInPin, kMaxChannels> pinInputs;
    std::array<AudioOutPin, kMaxChannels> pinOutputs;

    // Current deterministic transform configuration.
    int activeSize_ = 8;
    int activeType_ = 1;
    std::uint32_t activeSeed_ = 1u;
    bool activeShuffleChannels_ = false;
    bool activeShufflePolarity_ = false;

    std::array<int, kMaxChannels> sourcePermutation_{};
    std::array<int, kMaxChannels> destinationPermutation_{};
    std::array<float, kMaxChannels> inputSigns_{};
    std::array<float, kMaxChannels> outputSigns_{};
    std::array<std::array<float, kMaxChannels>, kMaxChannels> randomOrthogonal_{};
};

namespace
{
    auto registration =
        Register<OrthogonalMatrixMixer>::withId(L"Pandocrator Orthogonal Matrix Mixer v1");
}
