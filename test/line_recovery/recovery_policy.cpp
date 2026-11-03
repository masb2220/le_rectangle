#include "LineFollower/line_recovery.h"
static_assert(lineRecoveryStage(0) == LineRecoveryStage::BRIDGE, "Initial dropout bridges sensors");
static_assert(lineRecoveryStage(119) == LineRecoveryStage::BRIDGE, "No premature pivot");
static_assert(lineRecoveryStage(120) == LineRecoveryStage::ARC, "Arc replaces old 100ms stop");
static_assert(lineRecoveryStage(799) == LineRecoveryStage::ARC, "Arc duration");
static_assert(lineRecoveryStage(800) == LineRecoveryStage::SWEEP, "First sweep");
static_assert(lineRecoveryStage(1500) == LineRecoveryStage::REVERSE_SWEEP, "Opposite sweep");
static_assert(lineRecoveryStage(2300) == LineRecoveryStage::FINAL_SWEEP, "Final bounded attempt");
static_assert(lineRecoveryStage(2999) != LineRecoveryStage::EXPIRED, "Recovery still available");
static_assert(lineRecoveryStage(3000) == LineRecoveryStage::EXPIRED, "Stop at timeout");
static_assert(lineRecoveryStage(5000) == LineRecoveryStage::EXPIRED, "No unbounded search");
static_assert(lineRecoveryDirection(LineRecoveryStage::ARC, -1) == -1, "Follow last left direction");
static_assert(lineRecoveryDirection(LineRecoveryStage::ARC, 1) == 1, "Follow last right direction");
static_assert(lineRecoveryDirection(LineRecoveryStage::REVERSE_SWEEP, -1) == 1, "Reverse left search");
static_assert(lineRecoveryDirection(LineRecoveryStage::REVERSE_SWEEP, 1) == -1, "Reverse right search");
