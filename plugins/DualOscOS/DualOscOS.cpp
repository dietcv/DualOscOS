#include "DualOscOS.hpp"
#include "SC_PlugIn.hpp"

static InterfaceTable *ft;

namespace DualOscOS {

DualOscOS::DualOscOS() {
    m_oversamplingA.reset(static_cast<float>(sampleRate()));
    m_oversamplingB.reset(static_cast<float>(sampleRate()));
    m_dualOsc.reset();
    mCalcFunc = make_calc_function<DualOscOS, &DualOscOS::next_aa>();
    next_aa(1);
}

DualOscOS::~DualOscOS() = default;

bool DualOscOS::getBufferData(Utils::BufUnit& bufUnit, float bufNum, int nSamples,
                              const float*& bufData, int& tableSize, const char* oscName) {
    const SndBuf* buf;
    const auto verify_buf = bufUnit.GetTable(mWorld, mParent, bufNum, nSamples, buf, bufData, tableSize);
    if (!verify_buf) {
        if (!bufUnit.m_buf_failed) {
            Print("DualOscOS: buffer %s not found\n", oscName);
            bufUnit.m_buf_failed = true;
        }
        return false;
    }
    bufUnit.m_buf_failed = false;
    return true;
}

void DualOscOS::next_aa(int nSamples) {
    // Input parameters
    const float* phaseA = in(PhaseA);
    const float* phaseB = in(PhaseB);
    const float* cyclePosA = in(CyclePosA);
    const float* cyclePosB = in(CyclePosB);
    const float* pmIndexA = in(PMIndexA);
    const float* pmIndexB = in(PMIndexB);
    const float* pmFilterRatioA = in(PMFilterRatioA);
    const float* pmFilterRatioB = in(PMFilterRatioB);
    
    const float numCyclesA = sc_max(in0(NumCyclesA), 1.0f);
    const float numCyclesB = sc_max(in0(NumCyclesB), 1.0f);
    const float bufNumA = in0(BufNumA);
    const float bufNumB = in0(BufNumB);
    const int oversampleIndex = sc_clip(static_cast<int>(in0(Oversample)), 0, 4);
    
    // Output buffers
    float* outbufA = out(OutA);
    float* outbufB = out(OutB);
    
    // Get buffer data
    const float* bufDataA;
    const float* bufDataB;
    int tableSizeA, tableSizeB;
    
    if (!getBufferData(m_bufUnitA, bufNumA, nSamples, bufDataA, tableSizeA, "OscA") ||
        !getBufferData(m_bufUnitB, bufNumB, nSamples, bufDataB, tableSizeB, "OscB")) {
        ClearUnitOutputs(this, nSamples);
        return;
    }
    
    // Pre-calculate constants
    const int cycleSamplesA = tableSizeA / static_cast<int>(numCyclesA);
    const int cycleSamplesB = tableSizeB / static_cast<int>(numCyclesB);
    const int numCyclesIntA = static_cast<int>(numCyclesA);
    const int numCyclesIntB = static_cast<int>(numCyclesB);
    
    // Set oversampling factor for both channels
    m_oversamplingA.setOversamplingIndex(oversampleIndex);
    m_oversamplingB.setOversamplingIndex(oversampleIndex);
    
    if (oversampleIndex == 0) {
        // No oversampling - direct processing
        for (int i = 0; i < nSamples; ++i) {
            const float slopeA = m_rampToSlopeA.process(phaseA[i]);
            const float slopeB = m_rampToSlopeB.process(phaseB[i]);
            
            auto result = m_dualOsc.process(
                phaseA[i], phaseB[i], cyclePosA[i], cyclePosB[i],
                slopeA, slopeB, pmIndexA[i], pmIndexB[i],
                pmFilterRatioA[i], pmFilterRatioB[i],
                bufDataA, tableSizeA, cycleSamplesA, numCyclesIntA,
                bufDataB, tableSizeB, cycleSamplesB, numCyclesIntB,
                m_sincTable
            );
            
            outbufA[i] = result.oscA;
            outbufB[i] = result.oscB;
        }
    } else {
        // Oversampling enabled
        const int osRatioA = m_oversamplingA.getOversamplingRatio();
        const int osRatioB = m_oversamplingB.getOversamplingRatio();
        const float invOsRatioA = 1.0f / static_cast<float>(osRatioA);
        const float invOsRatioB = 1.0f / static_cast<float>(osRatioB);
        
        for (int i = 0; i < nSamples; ++i) {
            const float slopeA = m_rampToSlopeA.process(phaseA[i]);
            const float slopeB = m_rampToSlopeB.process(phaseB[i]);
            
            // Calculate phase difference per oversampled sample
            const float phaseDiffA = slopeA * invOsRatioA;
            const float phaseDiffB = slopeB * invOsRatioB;
            
            // Prepare oversampling buffers
            m_oversamplingA.upsample(0.0f);
            m_oversamplingB.upsample(0.0f);
            float* osBufferA = m_oversamplingA.getOSBuffer();
            float* osBufferB = m_oversamplingB.getOSBuffer();
            
            float osPhaseA = phaseA[i];
            float osPhaseB = phaseB[i];
            
            for (int k = 0; k < osRatioA; k++) {
                // Increment phases first
                osPhaseA += phaseDiffA;
                osPhaseB += phaseDiffB;
                
                // Then process with wrapped phases
                auto result = m_dualOsc.process(
                    sc_wrap(osPhaseA, 0.0f, 1.0f), sc_wrap(osPhaseB, 0.0f, 1.0f),
                    cyclePosA[i], cyclePosB[i],
                    slopeA, slopeB, pmIndexA[i], pmIndexB[i],
                    pmFilterRatioA[i], pmFilterRatioB[i],
                    bufDataA, tableSizeA, cycleSamplesA, numCyclesIntA,
                    bufDataB, tableSizeB, cycleSamplesB, numCyclesIntB,
                    m_sincTable
                );
                
                osBufferA[k] = result.oscA;
                osBufferB[k] = result.oscB;
            }
            
            outbufA[i] = m_oversamplingA.downsample();
            outbufB[i] = m_oversamplingB.downsample();
        }
    }
}

} // namespace DualOscOS

PluginLoad(DualOscOSUGens)
{
    ft = inTable;
    registerUnit<DualOscOS::DualOscOS>(ft, "DualOscOS", false);
}