#pragma once
#include <stdint.h>

// Politique commune au firmware et aux verifications de frontieres.
enum class LineRecoveryStage : uint8_t { BRIDGE, ARC, SWEEP, REVERSE_SWEEP, FINAL_SWEEP, EXPIRED };
constexpr LineRecoveryStage lineRecoveryStage(unsigned long elapsedMs)
{
    return elapsedMs < 120UL ? LineRecoveryStage::BRIDGE :
        elapsedMs < 800UL ? LineRecoveryStage::ARC :
        elapsedMs < 1500UL ? LineRecoveryStage::SWEEP :
        elapsedMs < 2300UL ? LineRecoveryStage::REVERSE_SWEEP :
        elapsedMs < 3000UL ? LineRecoveryStage::FINAL_SWEEP : LineRecoveryStage::EXPIRED;
}
constexpr int lineRecoveryDirection(LineRecoveryStage stage, int preferred)
{
    return stage == LineRecoveryStage::REVERSE_SWEEP ? -preferred : preferred;
}
