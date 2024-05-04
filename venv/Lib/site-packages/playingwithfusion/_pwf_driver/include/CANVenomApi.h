#ifndef CANVENOMAPI_H
#define CANVENOMAPI_H

#include <stdint.h>
#include "CANVenomApiTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

struct CANVenom_s;
typedef struct CANVenom_s CANVenom_t;

CANVenom_t *CANVenom_Create(uint8_t motorID);
void     CANVenom_Destroy(CANVenom_t *pHandle);

void     CANVenom_IdentifyMotor(CANVenom_t *pHandle);
void     CANVenom_ResetPosition(CANVenom_t *pHandle);
void     CANVenom_SetPosition(CANVenom_t *pHandle, double newPosition);
void     CANVenom_StopMotor(CANVenom_t *pHandle);

uint32_t CANVenom_GetFirmwareVersion(CANVenom_t *pHandle);
uint32_t CANVenom_GetSerialNumber(CANVenom_t *pHandle);


/**
 * Sets the motor command
 *
 * Sets motor proportional command.   If CANVenom_Disable() is called, CANVenom_Enable()
 * must be called before CANVenom_Set() will command motion
 *
 * @param pHandle Pointer to CANVenom_t structure returned by CANVenom_Create()
 * @param command Proportional motor command from -1 to 1
 */
void     CANVenom_Set(CANVenom_t *pHandle, double command);
void     CANVenom_PIDWrite(CANVenom_t *pHandle, double output);
void     CANVenom_SetInverted(CANVenom_t *pHandle, uint8_t isInverted);
double   CANVenom_Get(CANVenom_t *pHandle);
uint8_t  CANVenom_GetInverted(CANVenom_t *pHandle);
void     CANVenom_Disable(CANVenom_t *pHandle);
void     CANVenom_Enable(CANVenom_t *pHandle);

uint8_t  CANVenom_GetFwdLimitSwitchActive(CANVenom_t *pHandle);
uint8_t  CANVenom_GetRevLimitSwitchActive(CANVenom_t *pHandle);
void     CANVenom_EnableLimitSwitches(CANVenom_t *pHandle, uint8_t fwdLimitSwitchEnabled, uint8_t revLimitSwitchEnabled);

/**
 * Get number of empty motion profile points avaliable. 
 *
 * Gets number of motion profile point buffer locations avaliable in motor controller.
 * The motor will ignore additional calls to CANVenom_AddMotionProfilePoint() once all
 * buffer locations are full.
 *
 * @param pHandle Pointer to CANVenom_t structure returned by CANVenom_Create()
 */
uint16_t CANVenom_GetNumAvaliableMotionProfilePoints(CANVenom_t *pHandle);


uint16_t CANVenom_GetCurrentMotionProfilePoint(CANVenom_t *pHandle);
double   CANVenom_GetMotionProfilePositionTarget(CANVenom_t *pHandle);
double   CANVenom_GetMotionProfileSpeedTarget(CANVenom_t *pHandle);
bool     CANVenom_GetMotionProfileIsValid(CANVenom_t *pHandle);
CANVenom_MotionProfileState CANVenom_GetMotionProfileState(CANVenom_t *pHandle);

/**
 * Erase all motion profile points. 
 *
 * Clear all motion profile points from the motor controller buffer.  This function should
 * Be called first, each time a new motion profile path is loaded into the motor controlelr.
 *
 * @param pHandle Pointer to CANVenom_t structure returned by CANVenom_Create()
 */
void     CANVenom_ClearMotionProfilePoints(CANVenom_t *pHandle);

/**
 * Add single motion profile point.
 *
 * Add a single point to the motion profile buffer.   To load a motion profile, the application
 * should call CANVenom_ClearMotionProfilePoints(), then call CANVenom_AddMotionProfilePoint()
 * for each point.  The application should then close the path using CANVenom_CompleteMotionProfilePath()
 * Once a path is loaded, or partially loaded, the application may initiate the motion profile
 * using CANVenom_ExecutePath() or CANVenom_SetCommand(CONTROLMODE_MOTIONPROFILE, 0)
 * 
 * The motor will will lineraly interpolate commanded speed and position between motion profile
 * points.   Acceleration and Jerk limits are not used when executing a motion profile
 *
 * @param pHandle Pointer to CANVenom_t structure returned by CANVenom_Create()
 * @param time Time since the start of the profile in miliseconds
 * @param speed Commanded speed in rotations per second
 * @param position Commanded motor angle/position in revolutions
 */
void     CANVenom_AddMotionProfilePoint(CANVenom_t *pHandle, double time, double speed, double position);

/**
 * Add final point to motion profile.
 *
 * Add the last point to a motion profile.  The motor will attempt to hold the commanded poistion
 * indefinitly once reaching the final point. 
 *
 * @param pHandle Pointer to CANVenom_t structure returned by CANVenom_Create()
 * @param time Time since the start of the profile in seconds
 * @param position Commanded motor angle/position in revolutions
 */
void     CANVenom_CompleteMotionProfilePath(CANVenom_t *pHandle, double time, double position);

/**
 * Execute stored motion profile
 *
 * Instruct the motor to begin following the sotred motion profile 
 *
 * @param pHandle Pointer to CANVenom_t structure returned by CANVenom_Create()
 */
void     CANVenom_ExecutePath(CANVenom_t *pHandle);


double   CANVenom_GetBusVoltage(CANVenom_t *pHandle);
double   CANVenom_GetOutputVoltage(CANVenom_t *pHandle);
double   CANVenom_GetDutyCycle(CANVenom_t *pHandle);
double   CANVenom_GetOutputCurrent(CANVenom_t *pHandle);
double   CANVenom_GetTemperature(CANVenom_t *pHandle);
double   CANVenom_GetAuxVoltage(CANVenom_t *pHandle);
double   CANVenom_GetSpeed(CANVenom_t *pHandle);
double   CANVenom_GetPosition(CANVenom_t *pHandle);
double   CANVenom_GetPIDTarget(CANVenom_t *pHandle);
double   CANVenom_GetKF(CANVenom_t *pHandle);
double   CANVenom_GetB(CANVenom_t *pHandle);
double   CANVenom_GetKP(CANVenom_t *pHandle);
double   CANVenom_GetKI(CANVenom_t *pHandle);
double   CANVenom_GetKD(CANVenom_t *pHandle);
double   CANVenom_GetMinPILimit(CANVenom_t *pHandle);
double   CANVenom_GetMaxPILimit(CANVenom_t *pHandle);
double   CANVenom_GetMaxSpeed(CANVenom_t *pHandle);
double   CANVenom_GetMaxAcceleration(CANVenom_t *pHandle);
double   CANVenom_GetMaxJerk(CANVenom_t *pHandle);
CANVenom_ControlMode    CANVenom_GetControlMode(CANVenom_t *pHandle);
CANVenom_BrakeCoastMode CANVenom_GetBrakeCoastMode(CANVenom_t *pHandle);
CANVenom_ControlMode    CANVenom_GetActiveControlMode(CANVenom_t *pHandle);

CANVenom_FaultBits CANVenom_GetActiveFaults(CANVenom_t *pHandle);
CANVenom_FaultBits CANVenom_GetLatchedFaults(CANVenom_t *pHandle);
void     CANVenom_ClearLatchedFaults(CANVenom_t *pHandle);

void     CANVenom_SetCommandAndMode(CANVenom_t *pHandle, CANVenom_ControlMode mode, double command);
void     CANVenom_SetCommand(CANVenom_t *pHandle, CANVenom_ControlMode mode, double command, double kF, double b);
void     CANVenom_SetPID(CANVenom_t *pHandle, double kP, double kI, double kD, double kF, double b);
void     CANVenom_SetKF(CANVenom_t *pHandle, double kF);
void     CANVenom_SetB(CANVenom_t *pHandle, double b);
void     CANVenom_SetKP(CANVenom_t *pHandle, double kP);
void     CANVenom_SetKI(CANVenom_t *pHandle, double kI);
void     CANVenom_SetKD(CANVenom_t *pHandle, double kD);
void     CANVenom_SetMinPILimit(CANVenom_t *pHandle, double limit);
void     CANVenom_SetMaxPILimit(CANVenom_t *pHandle, double limit);
void     CANVenom_SetMaxSpeed(CANVenom_t *pHandle, double limit);
void     CANVenom_SetMaxAcceleration(CANVenom_t *pHandle, double limit);
void     CANVenom_SetMaxJerk(CANVenom_t *pHandle, double limit);
void     CANVenom_SetControlMode(CANVenom_t *pHandle, CANVenom_ControlMode controlMode);
void     CANVenom_SetBrakeCoastMode(CANVenom_t *pHandle, CANVenom_BrakeCoastMode brakeCoastMode);


#ifdef __cplusplus
}
#endif

#endif // CANVENOMAPI_H