from __future__ import annotations
import typing
import wpilib._wpilib
import wpilib.interfaces._interfaces
import wpiutil._wpiutil
__all__ = ['CANVenom', 'TMD37003', 'TimeOfFlight']
class CANVenom(wpilib._wpilib.MotorSafety, wpilib.interfaces._interfaces.MotorController, wpiutil._wpiutil.Sendable):
    """
    CAN based Venom motor controller instance
    
    This class is used to control a single Venom motor connected to the roboRIO
    through the CAN bus.  The motor supports many controls modes, including:
    Proportional Dutycycle, Voltage compensated proportional, closed-loop current,
    closed-loop speed with true s-curve trajector planning, closed-loop position
    control and motion profile execution
    """
    class BrakeCoastMode:
        """
        Members:
        
          kCoast : The motor is open circuited when the porportional or volatage command
        is zero, causing the motor to coast or free-wheel
        
          kBrake : The motor is shorted when the porportional or vlotage command is zero.
        This acts to brake/slow the motor to a stop.
        """
        __members__: typing.ClassVar[dict[str, CANVenom.BrakeCoastMode]]  # value = {'kCoast': <BrakeCoastMode.kCoast: 0>, 'kBrake': <BrakeCoastMode.kBrake: 1>}
        kBrake: typing.ClassVar[CANVenom.BrakeCoastMode]  # value = <BrakeCoastMode.kBrake: 1>
        kCoast: typing.ClassVar[CANVenom.BrakeCoastMode]  # value = <BrakeCoastMode.kCoast: 0>
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    class ControlMode:
        """
        Members:
        
          kDisabled : Motor is disabled and coasting
        
          kProportional : Proportional (duty-cycle) control
        
          kCurrentControl : Closed-loop current (torque) control
        
          kSpeedControl : Closed-loop speed control.
        
          kPositionControl : Closed-loop position (servo) control.
        
          kMotionProfile : Execute a motion control path.
        
        Motion profiles are generally
        loaded in Disabled mode.  The transition to the MotionProfile
        state causes Venom to begin following the path.
        
          kFollowTheLeader : Follow the duty cycle commanded to another Venom motor.
        
        Generally
        used when more that one Venom is geared together in a drivetrain
        application.  In that case one Venom, the leader, executes a motion
        profile or is placed in another control mode.  The other Venom(s)
        are placed in FolloewTheLeader and command the same duty cycle as
        the leader so that only the leader is used to calculate closed-loop
        commands.  This avoid implementing PID controllers on multiple motors
        which may "fight".
        
          kVoltageControl : Open-loop voltage control mode, also refered to voltage compensated
        proportional mode
        """
        __members__: typing.ClassVar[dict[str, CANVenom.ControlMode]]  # value = {'kDisabled': <ControlMode.kDisabled: 0>, 'kProportional': <ControlMode.kProportional: 1>, 'kCurrentControl': <ControlMode.kCurrentControl: 3>, 'kSpeedControl': <ControlMode.kSpeedControl: 4>, 'kPositionControl': <ControlMode.kPositionControl: 5>, 'kMotionProfile': <ControlMode.kMotionProfile: 6>, 'kFollowTheLeader': <ControlMode.kFollowTheLeader: 7>, 'kVoltageControl': <ControlMode.kVoltageControl: 8>}
        kCurrentControl: typing.ClassVar[CANVenom.ControlMode]  # value = <ControlMode.kCurrentControl: 3>
        kDisabled: typing.ClassVar[CANVenom.ControlMode]  # value = <ControlMode.kDisabled: 0>
        kFollowTheLeader: typing.ClassVar[CANVenom.ControlMode]  # value = <ControlMode.kFollowTheLeader: 7>
        kMotionProfile: typing.ClassVar[CANVenom.ControlMode]  # value = <ControlMode.kMotionProfile: 6>
        kPositionControl: typing.ClassVar[CANVenom.ControlMode]  # value = <ControlMode.kPositionControl: 5>
        kProportional: typing.ClassVar[CANVenom.ControlMode]  # value = <ControlMode.kProportional: 1>
        kSpeedControl: typing.ClassVar[CANVenom.ControlMode]  # value = <ControlMode.kSpeedControl: 4>
        kVoltageControl: typing.ClassVar[CANVenom.ControlMode]  # value = <ControlMode.kVoltageControl: 8>
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    class FaultFlag:
        """
        Members:
        
          kNone : No Active Faults
        
          kNoHeartbeat : Missing heartbeat from the roboRIO.  Ensure device ID matches device ID
        used by CANVenom class.
        
          kNoLeaderHeartbeat : Lead motor heartbeat is missing while in FollowTheLeader mode.
        
          kBadLeaderID : The lead motor ID is same as the motor ID.   One Venom cannot follow itself.
        Ensure the leader and follower have different IDs
        
          kHighTemperature : Motor temperature is too high
        
          kHighCurrent : Average motor current is too high
        
          kBadMode : An invalid control mode was specified by the roboRIO.   This should not
        occur when using PlayingWithFusionDriver.  Contact PWF Technical support.
        
          kDuplicateID : Another Venom with the same device ID was detected on the CAN bus.  All
        Venom device IDs must be unique
        
          kForwardLimit : The forward limit switch is enabled and is active
        
          kReverseLimit : The reverse limit switch is enabled and is active
        
          kReset : The Venom motor reset, lost power, or browned out since the last time
        the {@link #ClearLatchedFaults} function was called
        """
        __members__: typing.ClassVar[dict[str, CANVenom.FaultFlag]]  # value = {'kNone': <FaultFlag.kNone: 0>, 'kNoHeartbeat': <FaultFlag.kNoHeartbeat: 1>, 'kNoLeaderHeartbeat': <FaultFlag.kNoLeaderHeartbeat: 2>, 'kBadLeaderID': <FaultFlag.kBadLeaderID: 4>, 'kHighTemperature': <FaultFlag.kHighTemperature: 8>, 'kHighCurrent': <FaultFlag.kHighCurrent: 16>, 'kBadMode': <FaultFlag.kBadMode: 32>, 'kDuplicateID': <FaultFlag.kDuplicateID: 64>, 'kForwardLimit': <FaultFlag.kForwardLimit: 128>, 'kReverseLimit': <FaultFlag.kReverseLimit: 256>, 'kReset': <FaultFlag.kReset: 512>}
        kBadLeaderID: typing.ClassVar[CANVenom.FaultFlag]  # value = <FaultFlag.kBadLeaderID: 4>
        kBadMode: typing.ClassVar[CANVenom.FaultFlag]  # value = <FaultFlag.kBadMode: 32>
        kDuplicateID: typing.ClassVar[CANVenom.FaultFlag]  # value = <FaultFlag.kDuplicateID: 64>
        kForwardLimit: typing.ClassVar[CANVenom.FaultFlag]  # value = <FaultFlag.kForwardLimit: 128>
        kHighCurrent: typing.ClassVar[CANVenom.FaultFlag]  # value = <FaultFlag.kHighCurrent: 16>
        kHighTemperature: typing.ClassVar[CANVenom.FaultFlag]  # value = <FaultFlag.kHighTemperature: 8>
        kNoHeartbeat: typing.ClassVar[CANVenom.FaultFlag]  # value = <FaultFlag.kNoHeartbeat: 1>
        kNoLeaderHeartbeat: typing.ClassVar[CANVenom.FaultFlag]  # value = <FaultFlag.kNoLeaderHeartbeat: 2>
        kNone: typing.ClassVar[CANVenom.FaultFlag]  # value = <FaultFlag.kNone: 0>
        kReset: typing.ClassVar[CANVenom.FaultFlag]  # value = <FaultFlag.kReset: 512>
        kReverseLimit: typing.ClassVar[CANVenom.FaultFlag]  # value = <FaultFlag.kReverseLimit: 256>
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    class MotionProfileState:
        """
        Members:
        
          kInit : Initial state after Venom powerup
        
          kRunning : Motion profile is currently executing, but has not reached the final
        point yet.  No errors are active
        
          kErrBufferCleared : The motion profile buffer was cleared while the profile was being
        executed.  The motor cannot continue and will coast until the motor
        is placed in another control mode (to reset the error condition)
        
          kErrBufferUnderflow : The motor ran out of points while executing a motion profile.  Either
        new points were not sent to the motor fast enough or the profile wasn't
        terminated using the {@link #CompleteMotionProfilePath} function.
        The motor cannot continue and will coast until the motor is placed
        in another control mode (to reset the error condition)
        
          kErrBufferInvalid : Attempted to begin executing a motion profile but there was no valid
        start point.  This can happen if the {@link #ClearMotionProfilePoints}
        function is not called before loading a motion profile, or if too many
        points are loaded and the motor cannot buffer the entire path.
        
        Venom can buffer about 300 points.  The exact buffer length can be
        be determined by calling {@link #GetNumAvaliableMotionProfilePoints}
        immediatly after power up.  If the motion profile contains more points
        than can be stored in the buffer, the profile must be reloaded each time
        before begining to follow the profile.
        
        For large paths with many points, first load a subset of the points,
        begin executing the path, then continue to load new points as the motor
        executes the path.   Call the {@link #GetNumAvaliableMotionProfilePoints}
        function periodically to ensure the Venom buffer has sufficient space
        before loading additional points.
        
          kDone : The motion profile was successfully executed.  The motor will now
        hold the current position (at zero speed) until the motor is placed
        in another control mode.
        """
        __members__: typing.ClassVar[dict[str, CANVenom.MotionProfileState]]  # value = {'kInit': <MotionProfileState.kInit: 0>, 'kRunning': <MotionProfileState.kRunning: 1>, 'kErrBufferCleared': <MotionProfileState.kErrBufferCleared: 2>, 'kErrBufferUnderflow': <MotionProfileState.kErrBufferUnderflow: 3>, 'kErrBufferInvalid': <MotionProfileState.kErrBufferInvalid: 4>, 'kDone': <MotionProfileState.kDone: 5>}
        kDone: typing.ClassVar[CANVenom.MotionProfileState]  # value = <MotionProfileState.kDone: 5>
        kErrBufferCleared: typing.ClassVar[CANVenom.MotionProfileState]  # value = <MotionProfileState.kErrBufferCleared: 2>
        kErrBufferInvalid: typing.ClassVar[CANVenom.MotionProfileState]  # value = <MotionProfileState.kErrBufferInvalid: 4>
        kErrBufferUnderflow: typing.ClassVar[CANVenom.MotionProfileState]  # value = <MotionProfileState.kErrBufferUnderflow: 3>
        kInit: typing.ClassVar[CANVenom.MotionProfileState]  # value = <MotionProfileState.kInit: 0>
        kRunning: typing.ClassVar[CANVenom.MotionProfileState]  # value = <MotionProfileState.kRunning: 1>
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    def PIDWrite(self, output: float) -> None:
        """
        Used by an instance of PIDController to command the motor
        duty-cycle
        
        Places the motor in Proportional control mode and sets the motor
        proportional command.   If disable() or stopMotor() is
        called, enable() must be called before set() will command motion
        again.
        
        :param speed: Proportional motor command from -1.0 to 1.0
        """
    def __init__(self, motorID: int) -> None:
        """
        Create an instance of the CAN Venom Motor Controller driver.
        
        This is designed to support the Playing With Fusion Venom motor
        controller
        
        :param motorId: The 6-bit identifier used to select a particular
                        motor on the CAN bus.  This identifier may be set
                        through the PWF Device configuration page on the
                        roboRIO.
        """
    def addMotionProfilePoint(self, time: float, speed: float, position: float) -> None:
        """
        Add single motion profile point.
        
        Add a single point to the motion profile buffer.   To load a motion profile, the application
        should call clearMotionProfilePoints(), then call addMotionProfilePoint()
        for each point.  The application should then close the path using completeMotionProfilePath()
        Once a path is loaded, or partially loaded, the application may initiate the motion profile
        using executePath() or setCommand(ControlMode.MotionProfile, 0)
        
        The motor will will lineraly interpolate commanded speed and position between motion profile
        points.   Acceleration and Jerk limits are not used when executing a motion profile
        
        :param time:     Time since the start of the profile in miliseconds
        :param speed:    Commanded speed in rotations per second
        :param position: Commanded motor angle/position in revolutions
        """
    def clearLatchedFaults(self) -> None:
        """
        Clear all latched faults
        """
    def clearMotionProfilePoints(self) -> None:
        """
        Erase all motion profile points.
        
        Clear all motion profile points from the motor controller buffer.  This function should
        Be called first, each time a new motion profile path is loaded into the motor controller.
        """
    def completeMotionProfilePath(self, time: float, position: float) -> None:
        """
        Add final point to motion profile.
        
        Add the last point to a motion profile.  The motor will attempt to hold the commanded poistion
        indefinitly once reaching the final point.
        
        :param time:     Time since the start of the profile in milliseconds
        :param position: Commanded motor angle/position in revolution
        """
    def disable(self) -> None:
        """
        Stop applying power to the motor immediately.
        
        If Brake mode is active and the current control mode is Proportional
        or VoltageControl the motor will brake to a stop.  Otherwise the
        motor will coast.
        
        The enable() function must be called after a call to stopMotor()
        before motion may be commanded again.
        """
    def enable(self) -> None:
        """
        Enable the motor again after a call to stopMotor() or disable().
        """
    def enableLimitSwitches(self, fwdLimitSwitchEnabled: bool, revLimitSwitchEnabled: bool) -> None:
        """
        Enable/disable the forward and reverse limit switches.
        
        :param fwdLimitSwitchEnabled: Prevent forward rotation if this argument
                                      is true and the forward limit switch is active
        :param revLimitSwitchEnabled: Prevent reverse rotation if this argument
                                      is true and the forward limit switch is active
        """
    def executePath(self) -> None:
        """
        Execute stored motion profile.
        
        Instruct the motor to begin following the sotred motion profile.  This
        function is equivlelent to calling setCommand(ControlMode.MotionProfile, 0)
        """
    def follow(self, leadVenom: CANVenom) -> None:
        """
        Place the motor into FollowTheLeader mode and follow the specified motor
        
        This method is equivelent to calling SetComand(FollowTheLeader, ID_of_lead_motor);
        
        :param leadVenom: Reference to the CANVenom instance which reperesents the lead motor.
        """
    def get(self) -> float:
        """
        Get the motor duty-cycle.
        
        :returns: The motor duty cycle as a ratio between -1.0 and 1.0.
        """
    def getActiveControlMode(self) -> CANVenom.ControlMode:
        """
        Get the active Venom control mode.
        
        :returns: Get the active Venom control mode reported by the motor.
        """
    def getActiveFaults(self) -> CANVenom.FaultFlag:
        """
        Return set of active motor faults which curently limit or disable motor
        output.   More than one Fault may be active at a time
        
        :returns: Bitmask of active faults
        """
    def getAuxVoltage(self) -> float:
        """
        Get the measured voltage at the auxilary analog input on the limit
        switch breakout board.
        
        :returns: Auxilary analog input voltage in Volts.
        """
    def getB(self) -> float:
        """
        Get the feed-forward command offset in SpeedControl, PositionControl,
        CurrentCurrent and MotionProfile control modes.
        
        :returns: Feed-forward offset as duty-cycle between -2.0 and 2.0.
        """
    def getBrakeCoastMode(self) -> CANVenom.BrakeCoastMode:
        """
        Get the brake/coast behavior when zero is commanded in Proportional
        and VoltageControl control modes.
        
        :returns: The Brake or Coast behavion in Proportional and VoltageControl modes
        """
    def getBusVoltage(self) -> float:
        """
        Get the bus (battery) voltage supplying the Venmom motor.
        
        :returns: The bus voltage in Volts
        """
    def getControlMode(self) -> CANVenom.ControlMode:
        """
        Get the commanded Venom control mode.
        
        :returns: The commanded control mode
        """
    def getCurrentMotionProfilePoint(self) -> int:
        """
        Get current motion profile point
        
        Gets the active motion profile point while a motion profile is active.  The first point
        sent to the controller acter a call to clearMotionProfilePoints() is point 0.  The next point
        is 1, then 2, and so on.
        
        :returns: The motion profile point which is currently targeted by the Venom controller
        """
    def getDescription(self) -> str:
        """
        Return a description of this motor controller.
        
        :returns: The Venom motor controller description
        """
    def getDutyCycle(self) -> float:
        """
        Get the motor h-bridge duty cycle.
        
        :returns: The motor duty cycle as a ration between -1.0 and 1.0
        """
    def getFirmwareVersion(self) -> int:
        """
        Return the Venom motor firmware version of the motor
        asscioated with this instance of the CANVenom class.
        
        :returns: The Venom motor firmware version (multiplied by 100)
        """
    def getFwdLimitSwitchActive(self) -> bool:
        """
        Determine the state of the forward motion limit switch.
        
        An internal pull-up resistor activates the limit switch is nothing
        is connected.  Connect the limit switch to GND to deactivate the limit.
        
        :returns: true if the limit switch voltage is high (which would prevent
                  forward rotation if the limit was enabled)
        """
    def getInverted(self) -> bool:
        """
        Return the motor direction inversion state.
        
        :returns: True if the motor direction is inverted
        """
    def getKD(self) -> float:
        """
        Get the close-loop PID derative gain in SpeedControl,
        PositionControl, CurrentCurrent and MotionProfile control modes.
        
        :returns: PID derivative gain as ratio between 0.0 and 4.0.
        """
    def getKF(self) -> float:
        """
        Get the feed-forward gain in SpeedControl, PositionControl, CurrentCurrent
        and MotionProfile control modes.
        
        :returns: Feed-forward gain as ratio between -8.0 and 8.0.
        """
    def getKI(self) -> float:
        """
        Get the close-loop PID integral gain in SpeedControl,
        PositionControl, CurrentCurrent and MotionProfile control modes.
        
        :returns: PID integral gain as ratio between 0.0 and 4.0.
        """
    def getKP(self) -> float:
        """
        Get the close-loop PID proportional gain in SpeedControl,
        PositionControl, CurrentCurrent and MotionProfile control modes.
        
        :returns: PID proportional gain as ratio between 0.0 and 4.0.
        """
    def getLatchedFaults(self) -> CANVenom.FaultFlag:
        """
        Return set of latched motor faults which are curently active or
        were previously active since the last time the
        {@link #ClearLatchedFaults} function was called.  This function can be
        helpful when diagnosing harness or brownout issues which cause Venom
        to reset.  The Reset flag will be set each time venom starts up.
        
        :returns: Bitmask of latched faults
        """
    def getMaxAcceleration(self) -> float:
        """
        Get the maximum acceleration in the SpeedControl and PositionControl
        control modes.
        
        This number is used as part of the s-curve path planning
        in SpeedControl mode and the trapezoid planning in Position Control
        mode.
        
        Trajectory planning is disabled if the maximum accelleration is zero.
        
        :returns: Maximum acceleration RPM per second
        """
    def getMaxJerk(self) -> float:
        """
        Get the maximum jerk (second derivitive of speed) in the SpeedControl
        control mode.
        
        This number is used as part of the s-curve path planning.
        
        The jerk limit is disabled if the maximum jerk is 0..
        
        :returns: Maximum jerk RPM per second squared
        """
    def getMaxPILimit(self) -> float:
        """
        Get the maximum duty cycle that may be commanded by the PID in
        SpeedControl, PositionControl, CurrentCurrent and MotionProfile
        control modes.
        
        :returns: Maximum PID output duty-cycle as a ratio between -1.0 and 1.0.
        """
    def getMaxSpeed(self) -> float:
        """
        Get the maximum speed (absolute value of velocity) that may be commanded
        in the SpeedControl and PositionControl control modes.
        
        :returns: Maximum speed command in RPM.
        """
    def getMinPILimit(self) -> float:
        """
        Get the minimum duty cycle that may be commanded by the PID in
        SpeedControl, PositionControl, CurrentCurrent and MotionProfile
        control modes.
        
        :returns: Minimum PID output duty-cycle as a ratio between -1.0 and 1.0.
        """
    def getMotionProfileIsValid(self) -> bool:
        """
        Determine if the motor is prepared to execute a motion profile
        
        Determins is a valit start point is present and that the motor is read
        to begin executing a motion profile.
        
        :returns: True if the motion profile stored on the motor contains a valid start point
        """
    def getMotionProfilePositionTarget(self) -> float:
        """
        Get the instantaneous motion profile position commanded
        
        Gets the motor position commanded by the current motion profile point
        while a motion profile is active
        
        :returns: The commanded motor position in revolutions.
        """
    def getMotionProfileSpeedTarget(self) -> float:
        """
        Get the instantaneous motion profile speed commanded
        
        Gets the motor speed commanded by the current motion profile point
        while a motion profile is active
        
        :returns: The commanded speed position in revolutions per second (not RPM).
        """
    def getMotionProfileState(self) -> CANVenom.MotionProfileState:
        """
        Get the Motion Profile state.
        
        Gets the state of the internal Venom Motion Profile state machine.
        This state can be used to determine if a motion profile is being executed,
        has completed sucessfully, or has stopped due to an error.
        
        :returns: Venom Motion Profile state.
        """
    def getNumAvaliableMotionProfilePoints(self) -> int:
        """
        Get number of empty motion profile points avaliable.
        
        Gets number of motion profile point buffer locations avaliable in motor controller.
        The motor will ignore additional calls to addMotionProfilePoint() once all
        buffer locations are full.
        
        :returns: The number of remaining empty motion profile points which may be loaded into the Venom controller
        """
    def getOutputCurrent(self) -> float:
        """
        Get the measured motor current consumption.
        
        Current is measured between the Venom power leads (the battery) to the
        motor brushes.   Current is positive regardles of motor direction.  Only
        current from the battery to the motor is measured.   Zero amps are returned
        if the motor is charging the battery.
        
        :returns: The measured current Amps.
        """
    def getOutputVoltage(self) -> float:
        """
        Get the calculated voltage across the motor burshes.
        
        :returns: The calculated motor voltage in Volts.
        """
    def getPIDTarget(self) -> float:
        """
        Internal PID Target (position, speed, current).
        
        The PID target is equal to the motor command specified by setCommand() in CurrentControl mode.
        In SpeedControl and PositionControl modes, the PID command is the output of the s-curve or trapezoidal
        slew rate limit calculation.  In MotionProfile mode the PID command is equal to the current
        motion profile position command.
        
        In all closed-loop modes, the PID target represents the motor speed/position/current that the
        Venom PID is activly trying to achieve
        
        :returns: PID target in RPM, rotations, or Amps (based on current control mode)
        """
    def getPosition(self) -> float:
        """
        Signed motor revolutions (position) since the last time it was cleared.
        
        :returns: The signed motor position in revolutions.
        """
    def getRevLimitSwitchActive(self) -> bool:
        """
        Determine the state of the reverse motion limit switch.
        
        And internal pull-up resistor activates the limit switch is nothing
        is connected.  Connect the limit switch to GND to deactivate the limit.
        
        :returns: true if the limit switch voltage is high (which would prevent
                  reverse rotation if the limit was enabled)
        """
    def getSerialNumber(self) -> int:
        """
        Return the serial number of the motor asscioated with this
        instance of the CANVenom class.
        
        :returns: The Venom motor serial number
        """
    def getSpeed(self) -> float:
        """
        Measured signed motor velocity in RPM.
        
        :returns: Motor velocity in RPM.
        """
    def getTemperature(self) -> float:
        """
        The measured Venom backplate temperature.
        
        :returns: Measured backplate temperature in degrees C.
        """
    def identifyMotor(self) -> None:
        """
        Flash LED to identify motor.
        
        Identify the physical motor asscioated with this instance of the
        Venom driver by causing its LED to flash red and green for several
        seconds.
        """
    def initSendable(self, builder: wpiutil._wpiutil.SendableBuilder) -> None:
        """
        Initialize vaiiables and parameters to be passed into the
        smart dashboard.
        """
    def resetPosition(self) -> None:
        """
        Reset the motor revolution counter (position) to 0.
        """
    def set(self, command: float) -> None:
        """
        Sets the motor duty-cycle command.
        
        Places the motor in Proportional control mode and sets the motor
        proportional command.   If disable() or stopMotor() is
        called, enable() must be called before set() will command motion
        again.
        
        :param speed: Proportional motor duty-cycle command from -1.0 to 1.0
        """
    def setB(self, b: float) -> None:
        """
        Set Feed-forward duty cycle offset in closed loop control modes.
        
        :param b: Feed-forward offset as duty-cycle between -2.0 and 2.0.
        """
    def setBrakeCoastMode(self, brakeCoastMode: CANVenom.BrakeCoastMode) -> None:
        """
        Set the brake/coast behavior when zero is commanded in Proportional
        and VoltageControl control modes.
        
        :param brakeCoastMode: The Brake or Coast behavior in Proportional and VoltageControl modes.
        """
    @typing.overload
    def setCommand(self, mode: CANVenom.ControlMode, command: float) -> None:
        """
        Set the motor command and control mode.
        
        Where control mode is one of:
        
        - Proportional - command specifies the raw motor duty-cycle as a ratio between -1.0 and 1.0
        - CurrentControl - command specifies a target motor current in Amps between -40.0 and 40.0.
        - Note that the commanded is signed to specify the motor direction, but the measured motor
        current provided by the getOutputCurrent() function is unsigned (the absolute value of current)
        - In this mode a PID active controls the motor duty cycle to achieve the commanded current.
        - VoltageControl - command specified the voltage to be applied to the motor bushes as a value
        between 0.0 and 14.0 Volts.  - This mode is also refered to as voltage compensated proportional
        mode.  - Is is useful because the motor speed at a given voltage voltage will be constant if the
        battery voltage changes, as long as the voltage command is less than the battery voltage.
        - SpeedControl - command specifies the motor speed value between -6000.0 and 6000.0 RPM.  - In this
        mode a PID activle controls the motor duty cycle to achieve the commanded speed
        - PositionControl - command specifies the motor position as a value between -4096 and 4096 motor
        revolutions.  - This mode is sometime refered to as servo control because the motor attempts to
        hold a commanded position, just like a servo.   - In this mode a PID actively controls motor
        duty cycle to achieve the commanded position
        - MotionProfile - command is unused.
        - The motor attempts to follow a previously entered motion profile.  - Once the last point in the
        motion profile is reached the motor will hold the last command position from the profile.
        - See the addMotionProfilePoint() function for more details
        - FollowTheLeader - command specified the device ID of the Venom motor to follow.
        - In this mode the Venom motor will command the same duty cycle as the lead motor.
        - Generally used when more that one Venom is geared together in a drivetrain
        application.  In that case one Venom, the leader, executes a motion
        profile or is placed in another control mode.  The other Venom(s)
        are placed in FolloewTheLeader and command the same duty cycle as
        the leader so that only the leader is used to calculate closed-loop
        commands.  This avoid implementing PID controllers on multiple motors which may "fight".
        
        :param mode:    Motor control mode (Proportional, CurrentControl, SpeedControl, etc.)
        :param command: Motor command (%, Amps, RPM, etc)
        """
    @typing.overload
    def setCommand(self, mode: CANVenom.ControlMode, command: float, kF: float, b: float) -> None:
        """
        Set the motor command and control mode.
        
        Where control mode is one of:
        
        - Proportional - command specifies the raw motor duty-cycle as a ratio between -1.0 and 1.0
        - CurrentControl - command specifies a target motor current in Amps between -40.0 and 40.0.
        - Note that the commanded is signed to specify the motor direction, but the measured motor
        current provided by the getOutputCurrent() function is unsigned (the absolute value of current)
        - In this mode a PID active controls the motor duty cycle to achieve the commanded current.
        - VoltageControl - command specified the voltage to be applied to the motor bushes as a value
        between 0.0 and 14.0 Volts.  - This mode is also refered to as voltage compensated proportional
        mode.  - Is is useful because the motor speed at a given voltage voltage will be constant if the
        battery voltage changes, as long as the voltage command is less than the battery voltage.
        - SpeedControl - command specifies the motor speed value between -6000.0 and 6000.0 RPM.  - In this
        mode a PID activle controls the motor duty cycle to achieve the commanded speed
        - PositionControl - command specifies the motor position as a value between -4096 and 4096 motor
        revolutions.  - This mode is sometime refered to as servo control because the motor attempts to
        hold a commanded position, just like a servo.   - In this mode a PID actively controls motor
        duty cycle to achieve the commanded position
        - MotionProfile - command is unused.
        - The motor attempts to follow a previously entered motion profile.  - Once the last point in the
        motion profile is reached the motor will hold the last command position from the profile.
        - See the {@link #AddMotionProfilePoint} function for more details
        - FollowTheLeader - command specified the device ID of the Venom motor to follow.
        - In this mode the Venom motor will command the same duty cycle as the lead motor.
        - Generally used when more that one Venom is geared together in a drivetrain
        application.  In that case one Venom, the leader, executes a motion
        profile or is placed in another control mode.  The other Venom(s)
        are placed in FolloewTheLeader and command the same duty cycle as
        the leader so that only the leader is used to calculate closed-loop
        commands.  This avoid implementing PID controllers on multiple motors which may "fight".
        
        When the kF and b terms are included in this function, they are guarenteed to be sent to the
        motor in the same CAN frame as the control mode and command.   This is useful when an open-loop
        correction or feed-forward term is calculated by the roboRIO.
        
        :param mode:    Motor control mode (Proportional, CurrentControl, SpeedControl, etc.)
        :param command: Motor command (%, Amps, RPM, etc)
        :param kF:      Feed-forward gain as ratio between -8.0 and 8.0
        :param b:       Feed-forward offset as duty-cycle between -2.0 and 2.0
        """
    def setControlMode(self, controlMode: CANVenom.ControlMode) -> None:
        """
        Set the Venom motor control mode.
        
        Set the control mode without modifying the motor command.
        
        The prefered method to change the motor control mode is the
        {@link #setCommand} function.  Using setCommand() guarentees the
        control mode and the motor command will be received by the Venom
        controller at the same time.
        
        :param controlMode: The commanded control mode.
        """
    def setInverted(self, isInverted: bool) -> None:
        """
        Specify which direction the motor rotates in response to a posive
        motor command.
        
        When inverted the motor will spin the opposite direction it rotates
        when isInverted is false.  The motor will always report a positive
        speed when commanded in the 'forward' direction.
        
        This function is commonly used for drivetrain applications so that the
        and right motors both drive the frobot forward when given a forward
        command, even though one side is spinnig clockwise and the other is
        spinning counter clockwise.
        
        :param isInverted: True if the motor direction should be reversed
        """
    def setKD(self, kD: float) -> None:
        """
        Set PID Derivative gain.
        
        :param kD: Derivative gain as a ratio between 0.0 and 4.0.
        """
    def setKF(self, kF: float) -> None:
        """
        Set Feed-forward gain in closed loop control modes.
        
        :param kF: Feed-forward gain as a ratio between -2.0 and 2.0
        """
    def setKI(self, kI: float) -> None:
        """
        Set PID Integral gain.
        
        :param kI: Integral gain as a ratio between 0.0 and 4.0.
        """
    def setKP(self, kP: float) -> None:
        """
        Set PID Proportional gain.
        
        :param kP: Proportional gain as a ratio between 0.0 and 4.0.
        """
    def setMaxAcceleration(self, limit: float) -> None:
        """
        Set the maximum acceleration in the SpeedControl and PositionControl
        control modes.
        
        This number is used as part of the s-curve path planning
        in SpeedControl mode and the trapezoid planning in Position Control
        mode.
        
        Trajectory planning is disabled if the maximum acceleration is zero
        
        :param limit: Maximum acceleration between 0 and 25,500 RPM per second
        """
    def setMaxJerk(self, limit: float) -> None:
        """
        Set the maximum jerk (second derivitive of speed) in the SpeedControl
        control mode.
        
        This number is used as part of the s-curve path planning.
        
        The jerk limit is disabled if the maximum jerk is 0
        
        :param limit: Maximum jerk between 0 and 159,375 RPM per second squared.
        """
    def setMaxPILimit(self, limit: float) -> None:
        """
        Set the maximum duty cycle that may be commanded by the PID in
        SpeedControl, PositionControl, CurrentCurrent and MotionProfile
        control modes.
        
        :param limit: Maximum PID output duty-cycle as a ratio between -1.0 and 1.0.
        """
    def setMaxSpeed(self, limit: float) -> None:
        """
        Set the maximum speed (absolute value of velocity) that may be commanded
        in the SpeedControl and PositionControl control modes.
        
        :param limit: Maximum speed command between 0 and 6000 RPM.
        """
    def setMinPILimit(self, limit: float) -> None:
        """
        Set the minimum duty cycle that may be commanded by the PID in
        SpeedControl, PositionControl, CurrentCurrent and MotionProfile
        control modes.
        
        :param limit: Minimum PID output duty-cycle as a ratio between -1.0 and 1.0.
        """
    def setPID(self, kP: float, kI: float, kD: float, kF: float, b: float) -> None:
        """
        Set the PID gains for closed-loop control modes.
        
        Sets the proportional, integral, and derivative gains as well as
        the feed-forward gain and offset.  In general, the motor duty-cycle
        is calculated using:
        
        error = (commandedValue - measuredValue)
        dutyCycle = (kP * error) + (kI * integral(error)) + (kD * derrivative(error)) + (kF * commandedValue) + b
        
        :param kP: Proportional gain as a ratio between 0.0 and 4.0
        :param kI: Integral gain as a ratio between 0.0 and 4.0
        :param kD: Derivative gain as a ratio between 0.0 and 4.0
        :param kF: Feed-forward gain as a ratio between -2.0 and 2.0
        :param b:  Feed-forward offset as duty-cycle between -2.0 and 2.0
        """
    def setPosition(self, newPosition: float) -> None:
        """
        Reset the motor revolution counter (position) to the specified position.
        
        :param newPosition: Value to assign motor position in revolutions
        """
    def stopMotor(self) -> None:
        """
        Stop applying power to the motor immediately.
        
        If Brake mode is active and the current control mode is Proportional
        or VoltageControl the motor will brake to a stop.  Otherwise the
        motor will coast.
        
        The enable() function must be called after a call to stopMotor()
        before motion may be commanded again.
        """
class TMD37003:
    def __init__(self, i2cPort: wpilib._wpilib.I2C.Port) -> None:
        """
        Create Instance of TMD3700 color sensor driver.
        
        :param i2cPort: Internal/MXP I2C port on the roboRIO
        """
    def configureColorSense(self, alsIntegrationTime: float, alsGain: int) -> None:
        """
        Configure TMD3700 Color (Ambient Light Sensing) parameters.
        
        :param alsIntegrationTime: Color sensing sample time in milliseconds.  Value may
                                   range from 2.8 to 721ms.   Longer sample times act to
                                   filtered the sampled color.
        :param alsGain:            Color sensor gain as a value between 1 and 64.
        """
    def configureProximitySense(self, proximitySampleTime: float, proximityPulseLength: float, numProximityPulses: int, proximityGain: int, proximityLedCurrent: int) -> None:
        """
        Configure TMD3700 Proximity sense parameters.
        
        :param proximitySampleTime:  Proximity sensing sample time in milliseconds.  Value
                                     may range from 0.088 to 22.528 ms.
        :param proximityPulseLength: Lengh of each IR LED pulse during proximity measurement
                                     in milliseconds.  Value must fall between 0.004 and 0.032 ms.
        :param numProximityPulses:   Number of proximity IR LED pulses which occur during each
                                     sample period
        :param proximityGain:        Proximity sensor gain as a value between 1 and 8.
        :param proximityLedCurrent:  Proximity IR LED current in milliamps.  Value must fall
                                     between 6 and 192 mA
        """
    def getAmbientLightLevel(self) -> float:
        """
        Get clear (Ambient) channel value.
        
        :returns: Normalized clear channel value as ratio between 0 and 1.
        """
    def getBlue(self) -> float:
        """
        Get blue channel value.
        
        :returns: Normalized blue channel value as ratio between 0 and 1..
        """
    def getColor(self) -> wpilib._wpilib.Color:
        """
        Get gamma corrected RGB values from sensor
        
        :returns: Value of RGB samples
        """
    def getGreen(self) -> float:
        """
        Get green channel value.
        
        :returns: Normalized green channel value as ratio between 0 and 1..
        """
    def getHue(self) -> float:
        """
        Get the measured color (hue).
        
        :returns: Measured hue in degrees
        """
    def getProximity(self) -> float:
        """
        Get proximity value.
        
        :returns: Normalized proximity value as ratio between 0 and 1.
        """
    def getRed(self) -> float:
        """
        Get red channel value.
        
        :returns: Normalized red channel value as ratio between 0 and 1..
        """
    def getSaturation(self) -> float:
        """
        Get measured color saturation.
        
        :returns: Measured saturation as ratio between 0 and 1
        """
    def setGain(self, r: float, g: float, b: float, c: float, gamma: float) -> None:
        """
        Specifiy gains and gamma value to convert raw RGB samples to normalized
        RGB values to aproximate sRGB space.
        
        The default gains are calibrated for the built in white LED.  If another
        lighting source is used this function may be required to specify the
        white point.
        
        Channels are calculated using:
        {Normilized value} = ({Raw value} * gain) ^ (1/gamma)
        
        :param r:     Red channel gain
        :param g:     Green channel gain
        :param b:     Blue channel gain
        :param c:     Clear (ambient) channel gain
        :param gamma: Gamma vaulke used to convert raw (linear) samples to something
                      that responds like a human eye
        """
class TimeOfFlight(wpiutil._wpiutil.Sendable):
    class RangingMode:
        """
        Members:
        
          kShort
        
          kMedium
        
          kLong
        """
        __members__: typing.ClassVar[dict[str, TimeOfFlight.RangingMode]]  # value = {'kShort': <RangingMode.kShort: 0>, 'kMedium': <RangingMode.kMedium: 1>, 'kLong': <RangingMode.kLong: 2>}
        kLong: typing.ClassVar[TimeOfFlight.RangingMode]  # value = <RangingMode.kLong: 2>
        kMedium: typing.ClassVar[TimeOfFlight.RangingMode]  # value = <RangingMode.kMedium: 1>
        kShort: typing.ClassVar[TimeOfFlight.RangingMode]  # value = <RangingMode.kShort: 0>
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    class Status:
        """
        Members:
        
          kValid : Measured distance is valid
        
          kSigmaHigh : Sigma estimator check is above internally defined threshold.   The repeatability
        or standard deviation of the measurement is bad due to a decreasing signal noise
        ratio. Increasing the timing budget can improve the standard deviation.
        
          kReturnSignalLow : Return signal value is below the internal defined threshold.  The return signal is
        too week to return a good answer. The target may be too far, not reflective enough,
        or too small. Increasing the timing buget might help, but there may simply be no
        target available.
        
          kReturnPhaseBad : Return signal phase is out of bounds.   This means that the sensor is ranging in a
        "nonappropriated" zone and the measured result may be inconsistent. This status is
        considered as a warning but, in general, it happens when a target is at the maximum
        distance possible from the sensor.
        
          kHardwareFailure : Hardware failure
        
          kWrappedTarget : Wrapped target, non-matching phases.   This situation may occur when the target is
        very reflective and the distance to the target/sensor is longer than the physical
        limited distance measurable by the sensor. For example, approximately 5m when the senor
        is in Long distance mode and approximately 1.3 m when the sensor is in Short distance
        mode.
        
          kInternalError : Internal algorithm underflow or overflow
        
          kInvalid : The measured distance is invalid
        """
        __members__: typing.ClassVar[dict[str, TimeOfFlight.Status]]  # value = {'kValid': <Status.kValid: 0>, 'kSigmaHigh': <Status.kSigmaHigh: 1>, 'kReturnSignalLow': <Status.kReturnSignalLow: 2>, 'kReturnPhaseBad': <Status.kReturnPhaseBad: 4>, 'kHardwareFailure': <Status.kHardwareFailure: 5>, 'kWrappedTarget': <Status.kWrappedTarget: 7>, 'kInternalError': <Status.kInternalError: 8>, 'kInvalid': <Status.kInvalid: 14>}
        kHardwareFailure: typing.ClassVar[TimeOfFlight.Status]  # value = <Status.kHardwareFailure: 5>
        kInternalError: typing.ClassVar[TimeOfFlight.Status]  # value = <Status.kInternalError: 8>
        kInvalid: typing.ClassVar[TimeOfFlight.Status]  # value = <Status.kInvalid: 14>
        kReturnPhaseBad: typing.ClassVar[TimeOfFlight.Status]  # value = <Status.kReturnPhaseBad: 4>
        kReturnSignalLow: typing.ClassVar[TimeOfFlight.Status]  # value = <Status.kReturnSignalLow: 2>
        kSigmaHigh: typing.ClassVar[TimeOfFlight.Status]  # value = <Status.kSigmaHigh: 1>
        kValid: typing.ClassVar[TimeOfFlight.Status]  # value = <Status.kValid: 0>
        kWrappedTarget: typing.ClassVar[TimeOfFlight.Status]  # value = <Status.kWrappedTarget: 7>
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    def __init__(self, sensorID: int) -> None:
        """
        Create an instance of the CAN Time Of Flight sensor.
        
        This is designed to support the Playing With Fusion (PWF) SEN-36005 time of
        flight sensor
        
        :param sensorID: The 6-bit identifier used to select a particular
                         sensor on the CAN bus.  This identifier may be set
                         through the PWF Device configuration page on the
                         roboRIO.
        """
    def getAmbientLightLevel(self) -> float:
        """
        Get ambient lighting level in mega counts per second.
        
        :returns: Ambient lighting level in mega counts per second.
        """
    def getFirmwareVersion(self) -> int:
        """
        Determine the sensor firmware version
        
        :returns: The sensor firmware version
        """
    def getRange(self) -> float:
        """
        Get the distance between the sensor and the target
        
        :returns: Distance to target in millimeters
        """
    def getRangeSigma(self) -> float:
        """
        Get the standard deviation of the distance measurment in millimeters
        
        :returns: Standard deviation of distance measurment in millimeters
        """
    def getSerialNumber(self) -> int:
        """
        Determine the sensor hardware serial number
        
        :returns: The sensor hardware serial number
        """
    def getStatus(self) -> TimeOfFlight.Status:
        """
        Get status of the last distance measurement.
        
        :returns: Status of last measurement.  See VL53L1X datasheet for more details
        """
    def identifySensor(self) -> None:
        """
        Flash the time of flight sensor LED red and green
        to idicate the sensor asscioated with this instance
        of software
        """
    def initSendable(self, builder: wpiutil._wpiutil.SendableBuilder) -> None:
        ...
    def isRangeValid(self) -> bool:
        """
        Determine if the last measurment was valid
        
        :returns: TRUE if the Time Of Flight Sensor sucessfully measured distance to an object
        """
    def setRangeOfInterest(self, topLeftX: int, topLeftY: int, bottomRightX: int, bottomRightY: int) -> None:
        """
        Specify the region of the imaging sensor used for range measurement.
        
        The region of interest refers to the pixels within the time of flight imaging sensor
        that are used to detect reflected laser light form the target.  The sensor contains
        a 16x16 pixel grid.   By default, all 256 pixels are used which results in a roughly
        27 degree field of view.
        
        The Field of view refers to a cone which grows from the time of flight sensor outward.
        By reducing the size of the region of interest (by using a smaller area of pixels) it is
        possible to reduce the field of view.  This can be helpfull when measuring objects far
        away.  The smaller the field of view, the smaller the area where the sensor can detect
        targets.
        
        Reducing the region of interest will reduce the sensitivity of the sensor.  It may be
        necessary to increase the reflectivity of the target or to increase the sample time to
        compensate.
        
        The range of interest rows and columns must be greater or equal to zero and less than
        or equal to fifteen.   The top left corner row/column must be smaller than the bottom
        right column/row.  The region of interest must be at least four coulmns wide and four
        rows tall.   For example, to specify a 4x4 region at the center of the image sensor use:
        
        TimeOfFlight_SetRangeOfInterest(pHandle, 8, 8, 12, 12);
        
        :param topLeftX:     Column of the top left corner of the region of interest.
        :param topLeftY:     Row of the top left corner of the region of interest.
        :param bottomRightX: Column of the bottom right corner of the region of interest.
        :param bottomRightY: Row of the bottom right corner of the region of interest.
        """
    def setRangingMode(self, mode: TimeOfFlight.RangingMode, sampleTime: float) -> None:
        """
        Configure the ranging mode as well as the sample rate of the time
        of flight sensor
        
        The ranging mode specifies the trade off between maximum measure distance
        verses reliablity in bright situations.  Short mode (default) works the best
        in bright lighting conditions, but can only measure 1.3 meters.   Long mode
        can measure up to 4 meters in the dark, but may only be able to measure
        shorter distances depending on the lighting conditions.  See the Vl53L1x
        datasheet for more information
        
        The sample time specifies the how frequently the time of flight sensor attempts
        to measure the distance to a target.  The sample time must be between 24 and
        1000 milliseconds.
        
        :param mode:       The desired sensor ranging mode (short, medium, long)
        :param sampleTime: The desired sample time in milliseconds
        """
