#pragma once

typedef enum CANVenom_ControlMode_e {
    CANVenom_CtrlMode_kDisabled          = 0,
    CANVenom_CtrlMode_kProportional      = 1,  // Cmd (Pct)
    CANVenom_CtrlMode_kCurrentControl    = 3,  // Cmd (mA)
    CANVenom_CtrlMode_kSpeedControl      = 4,  // Cmd (RPM), FF (Pct)
    CANVenom_CtrlMode_kPositionControl   = 5,  // Cmd (Posn), MaxV (RPM), MaxA, MaxJ
    CANVenom_CtrlMode_kMotionProfile     = 6,  // Cmd (Posn), Cmd (RPM)  
    CANVenom_CtrlMode_kFollowTheLeader   = 7,  // Leader ID  
    CANVenom_CtrlMode_kVoltageControl    = 8
} CANVenom_ControlMode;

typedef enum CANVenom_BrakeCoastMode_e {
    CANVenom_BCMode_kCoast   = 0,
    CANVenom_BCMode_kBrake   = 1
} CANVenom_BrakeCoastMode;

typedef enum CANVenom_FaultBits_e {
    CANVenom_FaultBit_kNone                = 0,
    CANVenom_FaultBit_kNoHeartbeat         = 1,
    CANVenom_FaultBit_kNoLeaderHeartbeat   = 2,
    CANVenom_FaultBit_kBadLeaderID         = 4,
    CANVenom_FaultBit_kHighTemperature     = 8,
    CANVenom_FaultBit_kHighCurrent         = 16,
    CANVenom_FaultBit_kBadMode             = 32,
    CANVenom_FaultBit_kDuplicateID         = 64,
    CANVenom_FaultBit_kForwardLimit        = 128,
    CANVenom_FaultBit_kReverseLimit        = 256,
    CANVenom_FaultBit_kReset               = 512
} CANVenom_FaultBits;

typedef enum CANVenom_MotionProfileState_e {
   CANVenom_MotnProfSt_kInit               = 0,
   CANVenom_MotnProfSt_kRunning            = 1,
   CANVenom_MotnProfSt_kErrBufferCleared   = 2,
   CANVenom_MotnProfSt_kErrBufferUnderflow = 3,
   CANVenom_MotnProfSt_kErrBufferInvalid   = 4,
   CANVenom_MotnProfSt_kDone               = 5
} CANVenom_MotionProfileState;
