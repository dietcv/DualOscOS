
#pragma once
#include "SC_PlugIn.hpp"
#include "wavetables.h"
#include <array>
#include <cmath>

namespace Utils {

// ===== BASIC MATH UTILITIES =====

inline float randomFloat(RGen& rgen) {
    return rgen.frand();
}

inline float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

// Jatin's log2 polynomial approximation (3rd order)
inline float jatin_log2_approx(float x) {
    // Coefficients from Jatin's log2_approx<float, 3>
    constexpr float alpha = 0.1640425613334452f;
    constexpr float beta = -1.098865286222744f;
    constexpr float gamma = 3.148297929334117f;
    constexpr float zeta = -2.213475204444817f;
    
    // Estrin's method for polynomial evaluation (more stable)
    float x2 = x * x;
    float p01 = alpha * x + beta;
    float p23 = gamma * x + zeta;
    return p01 * x2 + p23;
}

// Jatin's pow2 polynomial approximation (3rd order) 
inline float jatin_pow2_approx(float x) {
    // Coefficients from Jatin's pow2_approx<float, 3>
    constexpr float alpha = 0.07944154167983575f;
    constexpr float beta = 0.2274112777602189f;
    constexpr float gamma = 0.6931471805599453f;
    constexpr float zeta = 1.0f;
    
    // Estrin's method for polynomial evaluation
    float x2 = x * x;
    float p01 = alpha * x + beta;
    float p23 = gamma * x + zeta;
    return p01 * x2 + p23;
}

// Jatin's fast log2 implementation
inline float fastLog2(float x) {
    if (x <= 0.0f) return -1000.0f; // Handle edge case
    
    // Bit manipulation exactly like Jatin's logarithm<Ratio<2,1>, 3>
    const auto vi = reinterpret_cast<int32_t&>(x);
    const auto ex = vi & 0x7f800000;
    const auto e = (ex >> 23) - 127;
    const auto vfi = (vi - ex) | 0x3f800000;
    const auto vf = reinterpret_cast<const float&>(vfi);
    
    // log2_base_r = 1.0f / gcem::log2(2.0f) = 1.0f for log2
    return static_cast<float>(e) + jatin_log2_approx(vf);
}

// Jatin's fast exp implementation  
inline float fastExp(float x) {
    // Euler's number ratio: e ≈ 2.718281828
    constexpr float log2_e = 1.4426950408889634f; // log2(e)
    
    // Convert exp(x) to pow(e, x) using Jatin's algorithm
    x = std::max(-126.0f, log2_e * x);
    
    const auto xi = static_cast<int32_t>(x);
    const auto l = x < 0.0f ? xi - 1 : xi;
    const auto f = x - static_cast<float>(l);
    const auto vi = (l + 127) << 23;
    
    return reinterpret_cast<const float&>(vi) * jatin_pow2_approx(f);
}

// Fast approximations for mipmap calculations
inline float fastFloor(float x) {
    int i = static_cast<int>(x);
    return static_cast<float>(i - (x < i));
}

inline float fastCeil(float x) {
    int i = static_cast<int>(x);
    return static_cast<float>(i + (x > i));
}

inline float fastWrap01(float x) {
    return x - fastFloor(x);
}

// ===== BUFFER MANAGEMENT =====

struct BufUnit {
    float m_fbufnum{std::numeric_limits<float>::quiet_NaN()};
    SndBuf* m_buf{nullptr};
    bool m_buf_failed{false};
   
    BufUnit() {
        m_fbufnum = std::numeric_limits<float>::quiet_NaN();
        m_buf = nullptr;
        m_buf_failed = false;
    }
   
    bool GetTable(World* world, Graph* parent, float fbufnum, int inNumSamples, const SndBuf*& buf, const float*& bufData, int& tableSize) {
        if (fbufnum < 0.f) {                                                                      
            fbufnum = 0.f;                                                                                  
        }    
        if (fbufnum != m_fbufnum) {
            uint32 bufnum = static_cast<uint32>(fbufnum);
            if (bufnum >= world->mNumSndBufs) {
                uint32 localBufNum = bufnum - world->mNumSndBufs;
                if (localBufNum <= static_cast<uint32>(parent->localBufNum)) {
                    m_buf = parent->mLocalSndBufs + localBufNum;
                } else {
                    bufnum = 0;
                    m_buf = world->mSndBufs + bufnum;
                }
            } else {
                m_buf = world->mSndBufs + bufnum;
            }
            m_fbufnum = fbufnum;
        }
        buf = m_buf;
        if (!buf) {
            return false;
        }
        bufData = buf->data;
       
        if (!bufData) {
            return false;
        }
        tableSize = buf->samples;
        return true;
    }
};

// ===== PHASE PROCESSING UTILITIES =====

struct RampToSlope {
    float m_lastPhase{0.0f};
   
    float process(float currentPhase) {
        // Calculate ramp slope
        float delta = currentPhase - m_lastPhase;

        // Wrap delta to recenter between -0.5 and 0.5, for corrected slope during wrap
        if (delta > 0.5f)
            delta -= 1.0f;
        else if (delta < -0.5f)
            delta += 1.0f;    
    
        // Update state for next sample
        m_lastPhase = currentPhase;

        return delta;
    }
   
    void reset() {
        m_lastPhase = 0.0f;
    }
};

// ===== ONE POLE FILTER UTILITIES =====

struct OnePoleFilter {
    static constexpr float PI = 3.14159265358979323846f;
    float m_state{0.0f};
   
    void reset() {
        m_state = 0.0f;
    }
   
    float processLowpass(float input, float slope) {
        // Clip slope to full Nyquist range, then take absolute value
        float safeSlope = std::min(0.5f, std::abs(slope));
        //float safeSlope = std::abs(sc_clip(slope, -0.5f, 0.5f));
       
        // Calculate coefficient: b = exp(-2π * slope)
        float coeff = fastExp(-2.0f * PI * safeSlope);
        //float coeff = std::exp(-2.0f * PI * safeSlope);
       
        // OnePole formula: y[n] = x[n] * (1-b) + y[n-1] * b
        m_state = input * (1.0f - coeff) + m_state * coeff;
       
        return m_state;
    }
   
    float processHighpass(float input, float slope) {
        float lowpassed = processLowpass(input, slope);
        return input - lowpassed;
    }
};

// ===== SINC INTERPOLATION UTILITIES =====

struct SincTable {
    static constexpr int SIZE = 8192;
    static constexpr int COUNT = 8;
    static constexpr int SPACING = SIZE / COUNT;
    
    std::array<float, SIZE> table;
    std::array<int, COUNT> sinc_points{0, 1024, 2048, 3072, 4096, 5120, 6144, 7168};
    std::array<int, COUNT> wave_points{-4, -3, -2, -1, 0, 1, 2, 3};

    SincTable() {
        // Load table and convert in constructor
        auto doubleTable = get_sinc_window8();
        for (int i = 0; i < SIZE; ++i) {
            table[i] = static_cast<float>(doubleTable[i]);
        }
    }
    
    // table access
    const float* data() const { return table.data(); }
};

// ===== HIGH-PERFORMANCE BUFFER ACCESS =====

// Fast wrapping within a cycle range (startPos to endPos-1) - (for power-of-2 sizes)
inline int wrapIndex(int index, int startPos, int mask) {
    return startPos + (index & mask);
}

// Fast no-interpolation peek with bitwise wrapping - (for power-of-2 sizes)
inline float peekNoInterp(const float* buffer, int index, int mask) {
    const int wrappedIndex = index & mask;
    return buffer[wrappedIndex];
}

// Fast linear interpolation peek with bitwise wrapping - (for power-of-2 sizes)
inline float peekLinearInterp(const float* buffer, float phase, int mask) {

    const int intPart = static_cast<int>(phase);
    const float fracPart = phase - static_cast<float>(intPart);
    
    const int idx1 = intPart & mask;
    const int idx2 = (intPart + 1) & mask;
    
    const float a = buffer[idx1];
    const float b = buffer[idx2];
    
    return lerp(a, b, fracPart);
}

// ===== HIGH-PERFORMANCE SINC INTERPOLATION =====

inline float sincInterpolate(float scaledPhase, const float* buffer, int bufSize, int startPos, int endPos, int sampleSpacing, const SincTable& sincTable) {

    // const pointer to sincTable data
    const float* const sincData = sincTable.data();

    const float sampleIndex = scaledPhase / static_cast<float>(sampleSpacing);
    const int intPart = static_cast<int>(sampleIndex);
    const float fracPart = sampleIndex - static_cast<float>(intPart);

    // Pre-calculate offsets
    const float sincOffset = fracPart * SincTable::SPACING;
    const int waveOffset = intPart * sampleSpacing;

    // Pre-calculate masks
    const int sincMask = SincTable::SIZE - 1;
    const int waveMask = (endPos - startPos) - 1;

    float result = 0.0f;
    
    for (int i = 0; i < SincTable::COUNT; ++i) {

        // === WAVEFORM BUFFER ACCESS (no interpolation) ===
        const int waveIndex = sincTable.wave_points[i] * sampleSpacing + waveOffset;
        const int waveIndexWrapped = wrapIndex(waveIndex, startPos, waveMask);
        const float waveSample = buffer[waveIndexWrapped]; // endPos ∈ [0, bufSize-1], no additional wrapping needed already guaranteed < buSize - 1
        
        // === SINC TABLE ACCESS (linear interpolation) ===
        const float sincPos = static_cast<float>(sincTable.sinc_points[i]) - sincOffset;
        const float sincSample = peekLinearInterp(sincData, sincPos, sincMask);
        
        result += waveSample * sincSample;
    }
    
    return result;
}

// ===== MIPMAP UTILITIES =====

inline float mipmapInterpolate(float phase, const float* buffer, int bufSize, int startPos, int endPos, float slope, const SincTable& sincTable) {
    // Calculate mipmap parameters
    const float rangeSize = static_cast<float>(endPos - startPos);
    const float scaledPhase = phase * rangeSize;
    const float samplesPerFrame = std::abs(slope) * rangeSize;
    const float octave = std::max(0.0f, fastLog2(samplesPerFrame));
    const int layer = static_cast<int>(fastCeil(octave));
    
    // Calculate spacings for adjacent mipmap levels  
    const int spacing1 = 1 << layer;     
    const int spacing2 = spacing1 << 1;
    
    // Check for sinc kernel bandwidth limit (1024)
    if (spacing1 >= SincTable::SPACING) {
        // no crossfade to next mipmap layer
        return sincInterpolate(scaledPhase, buffer, bufSize, startPos, endPos, SincTable::SPACING, sincTable);
    } else {
        // Crossfade between adjacent mipmap layers
        const float crossfade = octave - fastFloor(octave);
        const float sig1 = sincInterpolate(scaledPhase, buffer, bufSize, startPos, endPos, spacing1, sincTable);
        const float sig2 = sincInterpolate(scaledPhase, buffer, bufSize, startPos, endPos, spacing2, sincTable);
        return lerp(sig1, sig2, crossfade);
    }
}

// ===== MULTI-CYCLE WAVETABLE UTILITIES =====

inline float wavetableInterpolate(float phase, const float* buffer, int bufSize, int cycleSamples, int numCycles, float cyclePos, float slope, const SincTable& sincTable) {

    // GO book approach: wrap cyclePos to 0-1, then scale by numCycles
    //const float wrappedPos = sc_wrap(cyclePos, 0.0f, 1.0f);
    //const float scaledPos = wrappedPos * static_cast<float>(numCycles);
    
    // OscOS approach: clip cyclePos to 0-1, then scale by (numCycles - 1)
    //const float clippedPos = sc_clip(cyclePos, 0.0f, 1.0f);
    //const float scaledPos = clippedPos * static_cast<float>(numCycles - 1);

    // Combined clip and scale optimization
    const float scale = static_cast<float>(numCycles - 1);
    const float scaledPos = std::min(scale, std::max(0.0f, cyclePos * scale));
    
    // Calculate frac and int part
    const int intPart = static_cast<int>(scaledPos);
    const float fracPart = scaledPos - static_cast<float>(intPart);
    
    // intPart ∈ [0, numCycles-1], no wrapping needed already guaranteed < numCycles
    const int cycleIndex1 = intPart;
    const int startPos1 = cycleIndex1 * cycleSamples;
    const int endPos1 = startPos1 + cycleSamples;
    
    // Early exit for fracPart == 0 (no crossfade needed)
    if (fracPart == 0.0f) {
        return mipmapInterpolate(phase, buffer, bufSize, startPos1, endPos1, slope, sincTable);
    }
    
    // Calculate second cycle only when needed
    const int cycleIndex2 = (intPart + 1) % numCycles;
    const int startPos2 = cycleIndex2 * cycleSamples;
    const int endPos2 = startPos2 + cycleSamples;
    
    // Process each cycle
    float sig1 = mipmapInterpolate(phase, buffer, bufSize, startPos1, endPos1, slope, sincTable);
    float sig2 = mipmapInterpolate(phase, buffer, bufSize, startPos2, endPos2, slope, sincTable);
    
    // Crossfade between the two cycles
    return lerp(sig1, sig2, fracPart);
}

// ===== DUAL OSCILLATOR WITH CROSS-MODULATION =====

struct DualOscillatorState {
    // 1-sample delay feedback
    float m_prevOscA{0.0f};
    float m_prevOscB{0.0f};
    
    // One-pole filters
    OnePoleFilter m_pmFilterA;
    OnePoleFilter m_pmFilterB;
    
    void reset() {
        m_prevOscA = 0.0f;
        m_prevOscB = 0.0f;
        m_pmFilterA.reset();
        m_pmFilterB.reset();
    }
    
    struct DualOscOutput {
        float oscA;
        float oscB;
        float modPhaseA;
        float modPhaseB;
    };
    
    DualOscOutput process(
        float phaseA, float phaseB,
        float cyclePosA, float cyclePosB,
        float slopeA, float slopeB,
        float pmIndexA, float pmIndexB,
        float pmFilterRatioA, float pmFilterRatioB,
        const float* bufferA, int bufSizeA, int cycleSamplesA, int numCyclesA,
        const float* bufferB, int bufSizeB, int cycleSamplesB, int numCyclesB,
        const SincTable& sincTable
    ) {
        static constexpr float TWO_PI_INV = 1.0f / 6.28318530717958647692f;
        
        // Generate phase modulation signals using previous sample outputs
        float pmSignalA = (m_prevOscB * TWO_PI_INV) * pmIndexA;
        float pmSignalB = (m_prevOscA * TWO_PI_INV) * pmIndexB;
        
        // Filter the phase modulation signals
        float filteredPmA = m_pmFilterA.processLowpass(pmSignalA, slopeA * pmFilterRatioA);
        float filteredPmB = m_pmFilterB.processLowpass(pmSignalB, slopeB * pmFilterRatioB);
        
        // Apply phase modulation and wrap
        //float modulatedPhaseA = sc_wrap(phaseA + filteredPmA, 0.0f, 1.0f);
        //float modulatedPhaseB = sc_wrap(phaseB + filteredPmB, 0.0f, 1.0f);
        float modulatedPhaseA = fastWrap01(phaseA + filteredPmA);
        float modulatedPhaseB = fastWrap01(phaseB + filteredPmB);
        
        // Generate oscillator outputs
        float oscA = wavetableInterpolate(modulatedPhaseA, bufferA, bufSizeA, 
                                         cycleSamplesA, numCyclesA, cyclePosA, slopeA, sincTable);
        float oscB = wavetableInterpolate(modulatedPhaseB, bufferB, bufSizeB, 
                                         cycleSamplesB, numCyclesB, cyclePosB, slopeB, sincTable);
        
        // Store current outputs for next sample
        m_prevOscA = oscA;
        m_prevOscB = oscB;
        
        return {oscA, oscB};
    }
};

// ===== SUPERSAW OSCILLATOR =====

struct SuperSawOscillator {
    // Individual oscillator states for the 7 sawtooth waves
    struct SawState {
        float m_phase{0.0f};
        
        float next(float phaseInc) {
            // Update internal phase (standard 0-1 range)
            m_phase += phaseInc;
            
            // Wrap phase to [0, 1] range
            if (m_phase >= 1.0f)
                m_phase -= 1.0f;
            else if (m_phase < 0.0f)
                m_phase += 1.0f;
            
            // Generate sawtooth output: ramp from -1 to +1
            return (m_phase * 2.0f) - 1.0f;
        }

        void reset(bool isCenter, RGen& rgen) {
            if (isCenter) {
                m_phase = 0.0f;  // Center oscillator always starts at phase 0
            } else {
                // Side oscillators get random phase
                m_phase = randomFloat(rgen) * 2.0f - 1.0f;
            }
        }
    };
    
    // 7 individual sawtooth oscillators (1 center + 6 sides)
    SawState m_centerOsc;
    std::array<SawState, 6> m_sideOscs;
    
    // JP-8000 polynomial curves for detune and gain compensation
    float detuneCurve(float x) const {
        return (10028.7312891634f * static_cast<float>(std::pow(x, 11))) -
            (50818.8652045924f * static_cast<float>(std::pow(x, 10))) +
            (111363.4808729368f * static_cast<float>(std::pow(x, 9))) -
            (138150.6761080548f * static_cast<float>(std::pow(x, 8))) +
            (106649.6679158292f * static_cast<float>(std::pow(x, 7))) -
            (53046.9642751875f * static_cast<float>(std::pow(x, 6))) +
            (17019.9518580080f * static_cast<float>(std::pow(x, 5))) -
            (3425.0836591318f * static_cast<float>(std::pow(x, 4))) +
            (404.2703938388f * static_cast<float>(std::pow(x, 3))) -
            (24.1878824391f * static_cast<float>(std::pow(x, 2))) +
            (0.6717417634f * x) +
            0.0030115596f;
    }
    
    float centerGain(float x) const {
        return (-0.55366f * x) + 0.99785f;
    }
    
    float sideGain(float x) const {
        return (-0.73764f * x * x) + (1.2841f * x) + 0.044372f;
    }
    
    void reset(RGen& rgen) {
        m_centerOsc.reset(true, rgen);   // Center: fixed phase at 0
        for (auto& osc : m_sideOscs) {
            osc.reset(false, rgen);      // Sides: random phase offsets
        }
    }
    
    float process(float slope, float mix, float detune) {

        // Calculate detune ratios using JP-8000 curve
        const float detuneAmount = detuneCurve(detune);
        const std::array<float, 6> detuneRatios = {
            -0.11002313f, 
            -0.06288439f,
            -0.01952356f,
            0.01991221f,  
            0.06216538f,
            0.10745242f
        };
        
        // Generate center oscillator
        float centerOut = m_centerOsc.next(slope);
        
        // Generate side oscillators with detune
        float sideOut = 0.0f;
        for (int i = 0; i < 6; ++i) {
            float detuneSlope = slope * (1.0f + (detuneAmount * detuneRatios[i]));
            sideOut += m_sideOscs[i].next(detuneSlope);
        }
        
        // Apply JP-8000 gain compensation curves
        float centerGained = centerOut * centerGain(mix);
        float sideGained = sideOut * sideGain(mix);
        
        // Sum all oscillators
        return centerGained + sideGained;
    }
};

} // namespace Utils