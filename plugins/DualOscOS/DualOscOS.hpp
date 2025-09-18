#pragma once
#include "SC_PlugIn.hpp"
#include "Utils.hpp"
#include "VariableOversampling.hpp"

// ===== DUAL WAVETABLE OSCILLATOR =====

class DualOscOS : public SCUnit {
public:
    DualOscOS();
    ~DualOscOS();

private:
    void next_aa(int nSamples);
    
    // Helper function
    bool getBufferData(Utils::BufUnit& bufUnit, float bufNum, int nSamples,
                       const float*& bufData, int& tableSize, const char* oscName);

    // Core processing
    Utils::RampToSlope m_rampToSlopeA;
    Utils::RampToSlope m_rampToSlopeB;
    Utils::SincTable m_sincTable;
    Utils::DualOscillatorState m_dualOsc;
    Utils::BufUnit m_bufUnitA;
    Utils::BufUnit m_bufUnitB;
    VariableOversampling<4> m_oversamplingA;
    VariableOversampling<4> m_oversamplingB;

    // Constants
    const float m_sampleRate;

    enum InputParams {
        // Oscillator A
        BufNumA,
        PhaseA,
        NumCyclesA,
        CyclePosA,
        
        // Oscillator B  
        BufNumB,
        PhaseB,
        NumCyclesB,
        CyclePosB,
        
        // Cross-modulation parameters
        PMIndexA,        // How much B modulates A
        PMIndexB,        // How much A modulates B
        PMFilterRatioA,  // Filter ratio for A's PM
        PMFilterRatioB,  // Filter ratio for B's PM
        
        // Global parameters
        Oversample      // 0=1x, 1=2x, 2=4x, 3=8x, 4=16x
    };

    enum Outputs { 
        OutA,    // Oscillator A output
        OutB    // Oscillator B output
    };
};

// ===== SINGLE WAVETABLE OSCILLATOR =====

class SingleOscOS : public SCUnit {
public:
    SingleOscOS();
    ~SingleOscOS();

private:
    void next_aa(int nSamples);
    
    // Helper function
    bool getBufferData(Utils::BufUnit& bufUnit, float bufNum, int nSamples,
                       const float*& bufData, int& tableSize, const char* oscName);

    // Core processing
    Utils::RampToSlope m_rampToSlope;
    Utils::SincTable m_sincTable;
    Utils::BufUnit m_bufUnit;
    VariableOversampling<4> m_oversampling;

    // Constants
    const float m_sampleRate;

    enum InputParams {
        BufNum,
        Phase,
        NumCycles,
        CyclePos,
        Oversample      // 0=1x, 1=2x, 2=4x, 3=8x, 4=16x
    };

    enum Outputs { 
        Out
    };
};

// ===== SUPERSAW OSCILLATOR =====

class SuperSawOS : public SCUnit {
public:
    SuperSawOS();
    ~SuperSawOS();
private:
    void next_aa(int nSamples);
    
    // Core processing
    Utils::SuperSawOscillator m_superSaw;
    Utils::OnePoleFilter m_pitchTrackingHPF;
    VariableOversampling<4> m_oversampling;

    // Constants
    const float m_sampleRate;
    
    enum InputParams {
        Freq,           // Base frequency
        Mix,            // SuperSaw mix (0-1): 0=center only, 1=full spread
        Detune,         // Detune amount (0-1)
        Oversample      // Oversampling factor (0=1x, 1=2x, 2=4x, 3=8x, 4=16x)
    };
    
    enum Outputs {
        Out
    };
};