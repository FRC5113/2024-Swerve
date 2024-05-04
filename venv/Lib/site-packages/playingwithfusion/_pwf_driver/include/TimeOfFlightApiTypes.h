#pragma once

typedef enum TimeOfFlight_RangingMode_e {
    TimeOfFlight_RangingMode_kShort   = 0,
    TimeOfFlight_RangingMode_kMedium  = 1,
    TimeOfFlight_RangingMode_kLong    = 2
} TimeOfFlight_RangingMode;

typedef enum TimeOfFlight_Status_e {
    TimeOfFlight_Status_Valid           = 0,
    TimeOfFlight_Status_SigmaHigh       = 1,
    TimeOfFlight_Status_ReturnSignalLow = 2,
    TimeOfFlight_Status_ReturnPhaseBad  = 4,
    TimeOfFlight_Status_HardwareFailure = 5,
    TimeOfFlight_Status_WrappedTarget   = 7,
    TimeOfFlight_Status_InternalError   = 8,
    TimeOfFlight_Status_Invalid         = 14
} TimeOfFlight_Status;
