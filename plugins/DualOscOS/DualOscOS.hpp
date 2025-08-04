#pragma once
#include "SC_PlugIn.hpp"
#include "Utils.hpp"
#include "VariableOversampling.hpp"

namespace DualOscOS {

class DualOscOS : public SCUnit {
public:
    DualOscOS();
    ~DualOscOS();

private:
    void next_aa(int nSamples);
    
    // Helper function
    bool getBufferData(Utils::BufUnit& bufUnit, float bufNum, int nSamples,
                       const float*& bufData, int& tableSize, const char* oscName);

    // State variables
    Utils::RampToSlope m_rampToSlopeA;
    Utils::RampToSlope m_rampToSlopeB;
    Utils::SincTable m_sincTable;
    Utils::DualOscillatorState m_dualOsc;
    Utils::BufUnit m_bufUnitA;
    Utils::BufUnit m_bufUnitB;
    VariableOversampling<4> m_oversamplingA;
    VariableOversampling<4> m_oversamplingB;

    // Input/Output parameters
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
        Oversample,      // 0=1x, 1=2x, 2=4x, 3=8x, 4=16x
        
        NumInputParams
    };

    enum Outputs { 
        OutA,    // Oscillator A output
        OutB,    // Oscillator B output
        NumOutputParams 
    };
};

} // namespace DualOscOS