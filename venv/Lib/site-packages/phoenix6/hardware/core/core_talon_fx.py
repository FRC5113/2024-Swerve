"""
Copyright (C) Cross The Road Electronics.  All rights reserved.
License information can be found in CTRE_LICENSE.txt
For support and suggestions contact support@ctr-electronics.com or file
an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
"""

from phoenix6.hardware.parent_device import ParentDevice, SupportsSendRequest
from phoenix6.spns.spn_value import SpnValue
from phoenix6.status_code import StatusCode
from phoenix6.status_signal import *
from phoenix6.units import *
from phoenix6.sim.device_type import DeviceType
from phoenix6.controls.duty_cycle_out import DutyCycleOut
from phoenix6.controls.torque_current_foc import TorqueCurrentFOC
from phoenix6.controls.voltage_out import VoltageOut
from phoenix6.controls.position_duty_cycle import PositionDutyCycle
from phoenix6.controls.position_voltage import PositionVoltage
from phoenix6.controls.position_torque_current_foc import PositionTorqueCurrentFOC
from phoenix6.controls.velocity_duty_cycle import VelocityDutyCycle
from phoenix6.controls.velocity_voltage import VelocityVoltage
from phoenix6.controls.velocity_torque_current_foc import VelocityTorqueCurrentFOC
from phoenix6.controls.motion_magic_duty_cycle import MotionMagicDutyCycle
from phoenix6.controls.motion_magic_voltage import MotionMagicVoltage
from phoenix6.controls.motion_magic_torque_current_foc import MotionMagicTorqueCurrentFOC
from phoenix6.controls.differential_duty_cycle import DifferentialDutyCycle
from phoenix6.controls.differential_voltage import DifferentialVoltage
from phoenix6.controls.differential_position_duty_cycle import DifferentialPositionDutyCycle
from phoenix6.controls.differential_position_voltage import DifferentialPositionVoltage
from phoenix6.controls.differential_velocity_duty_cycle import DifferentialVelocityDutyCycle
from phoenix6.controls.differential_velocity_voltage import DifferentialVelocityVoltage
from phoenix6.controls.differential_motion_magic_duty_cycle import DifferentialMotionMagicDutyCycle
from phoenix6.controls.differential_motion_magic_voltage import DifferentialMotionMagicVoltage
from phoenix6.controls.follower import Follower
from phoenix6.controls.strict_follower import StrictFollower
from phoenix6.controls.differential_follower import DifferentialFollower
from phoenix6.controls.differential_strict_follower import DifferentialStrictFollower
from phoenix6.controls.neutral_out import NeutralOut
from phoenix6.controls.coast_out import CoastOut
from phoenix6.controls.static_brake import StaticBrake
from phoenix6.controls.music_tone import MusicTone
from phoenix6.controls.motion_magic_velocity_duty_cycle import MotionMagicVelocityDutyCycle
from phoenix6.controls.motion_magic_velocity_torque_current_foc import MotionMagicVelocityTorqueCurrentFOC
from phoenix6.controls.motion_magic_velocity_voltage import MotionMagicVelocityVoltage
from phoenix6.controls.motion_magic_expo_duty_cycle import MotionMagicExpoDutyCycle
from phoenix6.controls.motion_magic_expo_voltage import MotionMagicExpoVoltage
from phoenix6.controls.motion_magic_expo_torque_current_foc import MotionMagicExpoTorqueCurrentFOC
from phoenix6.controls.dynamic_motion_magic_duty_cycle import DynamicMotionMagicDutyCycle
from phoenix6.controls.dynamic_motion_magic_voltage import DynamicMotionMagicVoltage
from phoenix6.controls.dynamic_motion_magic_torque_current_foc import DynamicMotionMagicTorqueCurrentFOC
from phoenix6.configs.talon_fx_configs import TalonFXConfigurator
from phoenix6.signals.spn_enums import ForwardLimitValue, ReverseLimitValue, AppliedRotorPolarityValue, ControlModeValue, MotionMagicIsRunningValue, DeviceEnableValue, DifferentialControlModeValue, BridgeOutputValue
from phoenix6.sim.talon_fx_sim_state import TalonFXSimState

class CoreTalonFX(ParentDevice):
    """
    Constructs a new Talon FX motor controller object.

    :param device_id: ID of the device, as configured in Phoenix Tuner.
    :type device_id: int
    :param canbus: Name of the CAN bus this device is on. Possible CAN bus strings are:
                       rio - The native roboRIO CAN bus
                       CANivore Name or Serial Number
                       SocketCAN interface - non-FRC Linux only
                       * - Any CANivore seen by the program
                       Empty String - Default for the system ("rio" for roboRIO, "can0" for linux, "*" for Windows)
    :type canbus: str, optional
    """

    def __init__(self, device_id: int, canbus: str = ""):
        super().__init__(device_id, "talon fx", canbus)
        self.configurator = TalonFXConfigurator(self._device_identifier)

        Native.instance().c_ctre_phoenix6_platform_sim_create(DeviceType.PRO_TalonFXType.value, device_id)
        self.__sim_state = None


    @property
    def sim_state(self) -> TalonFXSimState:
        """
        Get the simulation state for this device.

        This function reuses an allocated simulation state
        object, so it is safe to call this function multiple
        times in a robot loop.

        :returns: Simulation state
        :rtype: TalonFXSimState
        """

        if self.__sim_state is None:
            self.__sim_state = TalonFXSimState(self)
        return self.__sim_state


    def get_version_major(self) -> StatusSignal[int]:
        """
        App Major Version number.
        
          Minimum Value: 0
          Maximum Value: 255
          Default Value: 0
          Units: 
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: VersionMajor Status Signal Object
        :rtype: StatusSignal[int]
        """
        return self._common_lookup(SpnValue.VERSION_MAJOR.value, 0, None, "version_major", False, int)
    
    def get_version_minor(self) -> StatusSignal[int]:
        """
        App Minor Version number.
        
          Minimum Value: 0
          Maximum Value: 255
          Default Value: 0
          Units: 
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: VersionMinor Status Signal Object
        :rtype: StatusSignal[int]
        """
        return self._common_lookup(SpnValue.VERSION_MINOR.value, 0, None, "version_minor", False, int)
    
    def get_version_bugfix(self) -> StatusSignal[int]:
        """
        App Bugfix Version number.
        
          Minimum Value: 0
          Maximum Value: 255
          Default Value: 0
          Units: 
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: VersionBugfix Status Signal Object
        :rtype: StatusSignal[int]
        """
        return self._common_lookup(SpnValue.VERSION_BUGFIX.value, 0, None, "version_bugfix", False, int)
    
    def get_version_build(self) -> StatusSignal[int]:
        """
        App Build Version number.
        
          Minimum Value: 0
          Maximum Value: 255
          Default Value: 0
          Units: 
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: VersionBuild Status Signal Object
        :rtype: StatusSignal[int]
        """
        return self._common_lookup(SpnValue.VERSION_BUILD.value, 0, None, "version_build", False, int)
    
    def get_version(self) -> StatusSignal[int]:
        """
        Full Version.  The format is a four byte value.
        
        Full Version of firmware in device. The format is a four byte value.
        
          Minimum Value: 0
          Maximum Value: 4294967295
          Default Value: 0
          Units: 
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: Version Status Signal Object
        :rtype: StatusSignal[int]
        """
        return self._common_lookup(SpnValue.VERSION_FULL.value, 0, None, "version", False, int)
    
    def get_fault_field(self) -> StatusSignal[int]:
        """
        Integer representing all faults
        
        This returns the fault flags reported by the device. These are device
        specific and are not used directly in typical applications. Use the
        signal specific GetFault_*() methods instead.  
        
          Minimum Value: 0
          Maximum Value: 16777215
          Default Value: 0
          Units: 
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: FaultField Status Signal Object
        :rtype: StatusSignal[int]
        """
        return self._common_lookup(SpnValue.ALL_FAULTS.value, 0, None, "fault_field", True, int)
    
    def get_sticky_fault_field(self) -> StatusSignal[int]:
        """
        Integer representing all sticky faults
        
        This returns the persistent "sticky" fault flags reported by the
        device. These are device specific and are not used directly in typical
        applications. Use the signal specific GetStickyFault_*() methods
        instead.  
        
          Minimum Value: 0
          Maximum Value: 16777215
          Default Value: 0
          Units: 
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: StickyFaultField Status Signal Object
        :rtype: StatusSignal[int]
        """
        return self._common_lookup(SpnValue.ALL_STICKY_FAULTS.value, 0, None, "sticky_fault_field", True, int)
    
    def get_motor_voltage(self) -> StatusSignal[volt]:
        """
        The applied (output) motor voltage.
        
          Minimum Value: -40.96
          Maximum Value: 40.95
          Default Value: 0
          Units: V
        
        Default Rates:
          CAN 2.0: 100.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: MotorVoltage Status Signal Object
        :rtype: StatusSignal[volt]
        """
        return self._common_lookup(SpnValue.PRO_MOTOR_OUTPUT_MOTOR_VOLTAGE.value, 0, None, "motor_voltage", True, volt)
    
    def get_forward_limit(self) -> StatusSignal[ForwardLimitValue]:
        """
        Forward Limit Pin.
        
        
        Default Rates:
          CAN 2.0: 100.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: ForwardLimit Status Signal Object
        :rtype: StatusSignal[ForwardLimitValue]
        """
        return self._common_lookup(SpnValue.FORWARD_LIMIT.value, 0, None, "forward_limit", True, ForwardLimitValue)
    
    def get_reverse_limit(self) -> StatusSignal[ReverseLimitValue]:
        """
        Reverse Limit Pin.
        
        
        Default Rates:
          CAN 2.0: 100.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: ReverseLimit Status Signal Object
        :rtype: StatusSignal[ReverseLimitValue]
        """
        return self._common_lookup(SpnValue.REVERSE_LIMIT.value, 0, None, "reverse_limit", True, ReverseLimitValue)
    
    def get_applied_rotor_polarity(self) -> StatusSignal[AppliedRotorPolarityValue]:
        """
        The applied rotor polarity.  This typically is determined by the
        Inverted config, but can be overridden if using Follower features.
        
        
        Default Rates:
          CAN 2.0: 100.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: AppliedRotorPolarity Status Signal Object
        :rtype: StatusSignal[AppliedRotorPolarityValue]
        """
        return self._common_lookup(SpnValue.PRO_MOTOR_OUTPUT_ROTOR_POLARITY.value, 0, None, "applied_rotor_polarity", True, AppliedRotorPolarityValue)
    
    def get_duty_cycle(self) -> StatusSignal[float]:
        """
        The applied motor duty cycle.
        
          Minimum Value: -2.0
          Maximum Value: 1.9990234375
          Default Value: 0
          Units: fractional
        
        Default Rates:
          CAN 2.0: 100.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: DutyCycle Status Signal Object
        :rtype: StatusSignal[float]
        """
        return self._common_lookup(SpnValue.PRO_MOTOR_OUTPUT_DUTY_CYCLE.value, 0, None, "duty_cycle", True, float)
    
    def get_torque_current(self) -> StatusSignal[ampere]:
        """
        Current corresponding to the torque output by the motor. Similar to
        StatorCurrent. Users will likely prefer this current to calculate the
        applied torque to the rotor.
        
        Stator current where positive current means torque is applied in the
        forward direction as determined by the Inverted setting
        
          Minimum Value: -327.68
          Maximum Value: 327.67
          Default Value: 0
          Units: A
        
        Default Rates:
          CAN 2.0: 100.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: TorqueCurrent Status Signal Object
        :rtype: StatusSignal[ampere]
        """
        return self._common_lookup(SpnValue.PRO_MOTOR_OUTPUT_TORQUE_CURRENT.value, 0, None, "torque_current", True, ampere)
    
    def get_stator_current(self) -> StatusSignal[ampere]:
        """
        Current corresponding to the stator windings. Similar to
        TorqueCurrent. Users will likely prefer TorqueCurrent over
        StatorCurrent.
        
        Stator current where Positive current indicates motoring regardless of
        direction. Negative current indicates regenerative braking regardless
        of direction.
        
          Minimum Value: -327.68
          Maximum Value: 327.66
          Default Value: 0
          Units: A
        
        Default Rates:
          CAN 2.0: 4.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: StatorCurrent Status Signal Object
        :rtype: StatusSignal[ampere]
        """
        return self._common_lookup(SpnValue.PRO_SUPPLY_AND_TEMP_STATOR_CURRENT.value, 0, None, "stator_current", True, ampere)
    
    def get_supply_current(self) -> StatusSignal[ampere]:
        """
        Measured supply side current
        
          Minimum Value: -327.68
          Maximum Value: 327.66
          Default Value: 0
          Units: A
        
        Default Rates:
          CAN 2.0: 4.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: SupplyCurrent Status Signal Object
        :rtype: StatusSignal[ampere]
        """
        return self._common_lookup(SpnValue.PRO_SUPPLY_AND_TEMP_SUPPLY_CURRENT.value, 0, None, "supply_current", True, ampere)
    
    def get_supply_voltage(self) -> StatusSignal[volt]:
        """
        Measured supply voltage to the TalonFX.
        
          Minimum Value: 4
          Maximum Value: 29.575
          Default Value: 4
          Units: V
        
        Default Rates:
          CAN 2.0: 4.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: SupplyVoltage Status Signal Object
        :rtype: StatusSignal[volt]
        """
        return self._common_lookup(SpnValue.PRO_SUPPLY_AND_TEMP_SUPPLY_VOLTAGE.value, 0, None, "supply_voltage", True, volt)
    
    def get_device_temp(self) -> StatusSignal[celsius]:
        """
        Temperature of device
        
        This is the temperature that the device measures itself to be at.
        Similar to Processor Temperature.
        
          Minimum Value: 0.0
          Maximum Value: 255.0
          Default Value: 0
          Units: ℃
        
        Default Rates:
          CAN 2.0: 4.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: DeviceTemp Status Signal Object
        :rtype: StatusSignal[celsius]
        """
        return self._common_lookup(SpnValue.PRO_SUPPLY_AND_TEMP_DEVICE_TEMP.value, 0, None, "device_temp", True, celsius)
    
    def get_processor_temp(self) -> StatusSignal[celsius]:
        """
        Temperature of the processor
        
        This is the temperature that the processor measures itself to be at.
        Similar to Device Temperature.
        
          Minimum Value: 0.0
          Maximum Value: 255.0
          Default Value: 0
          Units: ℃
        
        Default Rates:
          CAN 2.0: 4.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: ProcessorTemp Status Signal Object
        :rtype: StatusSignal[celsius]
        """
        return self._common_lookup(SpnValue.PRO_SUPPLY_AND_TEMP_PROCESSOR_TEMP.value, 0, None, "processor_temp", True, celsius)
    
    def get_rotor_velocity(self) -> StatusSignal[rotations_per_second]:
        """
        Velocity of the motor rotor. This velocity is not affected by any
        feedback configs.
        
          Minimum Value: -512.0
          Maximum Value: 511.998046875
          Default Value: 0
          Units: rotations per second
        
        Default Rates:
          CAN 2.0: 4.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: RotorVelocity Status Signal Object
        :rtype: StatusSignal[rotations_per_second]
        """
        return self._common_lookup(SpnValue.PRO_ROTOR_POS_AND_VEL_VELOCITY.value, 0, None, "rotor_velocity", True, rotations_per_second)
    
    def get_rotor_position(self) -> StatusSignal[rotation]:
        """
        Position of the motor rotor. This position is only affected by the
        RotorOffset config.
        
          Minimum Value: -16384.0
          Maximum Value: 16383.999755859375
          Default Value: 0
          Units: rotations
        
        Default Rates:
          CAN 2.0: 4.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: RotorPosition Status Signal Object
        :rtype: StatusSignal[rotation]
        """
        return self._common_lookup(SpnValue.PRO_ROTOR_POS_AND_VEL_POSITION.value, 0, None, "rotor_position", True, rotation)
    
    def get_velocity(self) -> StatusSignal[rotations_per_second]:
        """
        Velocity of the device in mechanism rotations per second. This can be
        the velocity of a remote sensor and is affected by the
        RotorToSensorRatio and SensorToMechanismRatio configs.
        
          Minimum Value: -512.0
          Maximum Value: 511.998046875
          Default Value: 0
          Units: rotations per second
        
        Default Rates:
          CAN 2.0: 50.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: Velocity Status Signal Object
        :rtype: StatusSignal[rotations_per_second]
        """
        return self._common_lookup(SpnValue.PRO_POS_AND_VEL_VELOCITY.value, 0, None, "velocity", True, rotations_per_second)
    
    def get_position(self) -> StatusSignal[rotation]:
        """
        Position of the device in mechanism rotations. This can be the
        position of a remote sensor and is affected by the RotorToSensorRatio
        and SensorToMechanismRatio configs.
        
          Minimum Value: -16384.0
          Maximum Value: 16383.999755859375
          Default Value: 0
          Units: rotations
        
        Default Rates:
          CAN 2.0: 50.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: Position Status Signal Object
        :rtype: StatusSignal[rotation]
        """
        return self._common_lookup(SpnValue.PRO_POS_AND_VEL_POSITION.value, 0, None, "position", True, rotation)
    
    def get_acceleration(self) -> StatusSignal[rotations_per_second_squared]:
        """
        Acceleration of the device in mechanism rotations per second². This
        can be the acceleration of a remote sensor and is affected by the
        RotorToSensorRatio and SensorToMechanismRatio configs.
        
          Minimum Value: -2048.0
          Maximum Value: 2047.75
          Default Value: 0
          Units: rotations per second²
        
        Default Rates:
          CAN 2.0: 50.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: Acceleration Status Signal Object
        :rtype: StatusSignal[rotations_per_second_squared]
        """
        return self._common_lookup(SpnValue.PRO_POS_AND_VEL_ACCELERATION.value, 0, None, "acceleration", True, rotations_per_second_squared)
    
    def get_control_mode(self) -> StatusSignal[ControlModeValue]:
        """
        The active control mode of the motor controller
        
        
        Default Rates:
          CAN 2.0: 4.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: ControlMode Status Signal Object
        :rtype: StatusSignal[ControlModeValue]
        """
        return self._common_lookup(SpnValue.TALON_FX_CONTROL_MODE.value, 0, None, "control_mode", True, ControlModeValue)
    
    def get_motion_magic_is_running(self) -> StatusSignal[MotionMagicIsRunningValue]:
        """
        Check if Motion Magic® is running.  This is equivalent to checking
        that the reported control mode is a Motion Magic® based mode.
        
        
        Default Rates:
          CAN 2.0: 4.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: MotionMagicIsRunning Status Signal Object
        :rtype: StatusSignal[MotionMagicIsRunningValue]
        """
        return self._common_lookup(SpnValue.PRO_PIDSTATE_ENABLES_IS_MOTION_MAGIC_RUNNING.value, 0, None, "motion_magic_is_running", True, MotionMagicIsRunningValue)
    
    def get_device_enable(self) -> StatusSignal[DeviceEnableValue]:
        """
        Indicates if device is actuator enabled.
        
        
        Default Rates:
          CAN 2.0: 4.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: DeviceEnable Status Signal Object
        :rtype: StatusSignal[DeviceEnableValue]
        """
        return self._common_lookup(SpnValue.PRO_PIDSTATE_ENABLES_DEVICE_ENABLE.value, 0, None, "device_enable", True, DeviceEnableValue)
    
    def get_closed_loop_slot(self) -> StatusSignal[int]:
        """
        Closed loop slot in use
        
        This is the slot that the closed loop PID is using.
        
          Minimum Value: 0
          Maximum Value: 2
          Default Value: 0
          Units: 
        
        Default Rates:
          CAN 2.0: 4.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: ClosedLoopSlot Status Signal Object
        :rtype: StatusSignal[int]
        """
        return self._common_lookup(SpnValue.PRO_PIDOUTPUT_SLOT.value, 0, None, "closed_loop_slot", True, int)
    
    def get_differential_control_mode(self) -> StatusSignal[DifferentialControlModeValue]:
        """
        The active control mode of the differential controller
        
        
        Default Rates:
          CAN 2.0: 100.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: DifferentialControlMode Status Signal Object
        :rtype: StatusSignal[DifferentialControlModeValue]
        """
        return self._common_lookup(SpnValue.TALON_FX_DIFFERENTIAL_CONTROL_MODE.value, 0, None, "differential_control_mode", True, DifferentialControlModeValue)
    
    def get_differential_average_velocity(self) -> StatusSignal[rotations_per_second]:
        """
        Average component of the differential velocity of device.
        
          Minimum Value: -512.0
          Maximum Value: 511.998046875
          Default Value: 0
          Units: rotations per second
        
        Default Rates:
          CAN 2.0: 4.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: DifferentialAverageVelocity Status Signal Object
        :rtype: StatusSignal[rotations_per_second]
        """
        return self._common_lookup(SpnValue.PRO_AVG_POS_AND_VEL_VELOCITY.value, 0, None, "differential_average_velocity", True, rotations_per_second)
    
    def get_differential_average_position(self) -> StatusSignal[rotation]:
        """
        Average component of the differential position of device.
        
          Minimum Value: -16384.0
          Maximum Value: 16383.999755859375
          Default Value: 0
          Units: rotations
        
        Default Rates:
          CAN 2.0: 4.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: DifferentialAveragePosition Status Signal Object
        :rtype: StatusSignal[rotation]
        """
        return self._common_lookup(SpnValue.PRO_AVG_POS_AND_VEL_POSITION.value, 0, None, "differential_average_position", True, rotation)
    
    def get_differential_difference_velocity(self) -> StatusSignal[rotations_per_second]:
        """
        Difference component of the differential velocity of device.
        
          Minimum Value: -512.0
          Maximum Value: 511.998046875
          Default Value: 0
          Units: rotations per second
        
        Default Rates:
          CAN 2.0: 4.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: DifferentialDifferenceVelocity Status Signal Object
        :rtype: StatusSignal[rotations_per_second]
        """
        return self._common_lookup(SpnValue.PRO_DIFF_POS_AND_VEL_VELOCITY.value, 0, None, "differential_difference_velocity", True, rotations_per_second)
    
    def get_differential_difference_position(self) -> StatusSignal[rotation]:
        """
        Difference component of the differential position of device.
        
          Minimum Value: -16384.0
          Maximum Value: 16383.999755859375
          Default Value: 0
          Units: rotations
        
        Default Rates:
          CAN 2.0: 4.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: DifferentialDifferencePosition Status Signal Object
        :rtype: StatusSignal[rotation]
        """
        return self._common_lookup(SpnValue.PRO_DIFF_POS_AND_VEL_POSITION.value, 0, None, "differential_difference_position", True, rotation)
    
    def get_differential_closed_loop_slot(self) -> StatusSignal[int]:
        """
        Differential Closed loop slot in use
        
        This is the slot that the closed loop differential PID is using.
        
          Minimum Value: 0
          Maximum Value: 2
          Default Value: 0
          Units: 
        
        Default Rates:
          CAN 2.0: 4.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: DifferentialClosedLoopSlot Status Signal Object
        :rtype: StatusSignal[int]
        """
        return self._common_lookup(SpnValue.PRO_DIFF_PIDOUTPUT_SLOT.value, 0, None, "differential_closed_loop_slot", True, int)
    
    def get_bridge_output(self) -> StatusSignal[BridgeOutputValue]:
        """
        The applied output of the bridge.
        
        
        Default Rates:
          CAN 2.0: 100.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: BridgeOutput Status Signal Object
        :rtype: StatusSignal[BridgeOutputValue]
        """
        return self._common_lookup(SpnValue.PRO_MOTOR_OUTPUT_BRIDGE_TYPE_PUBLIC.value, 0, None, "bridge_output", True, BridgeOutputValue)
    
    def get_is_pro_licensed(self) -> StatusSignal[bool]:
        """
        Whether the device is Phoenix Pro licensed.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: IsProLicensed Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.VERSION_IS_PRO_LICENSED.value, 0, None, "is_pro_licensed", True, bool)
    
    def get_ancillary_device_temp(self) -> StatusSignal[celsius]:
        """
        Temperature of device from second sensor
        
        Newer versions of Talon FX have multiple temperature measurement
        methods.
        
          Minimum Value: 0.0
          Maximum Value: 255.0
          Default Value: 0
          Units: ℃
        
        Default Rates:
          CAN 2.0: 4.0 Hz
          CAN FD: 100.0 Hz (TimeSynced with Pro)
        
        :returns: AncillaryDeviceTemp Status Signal Object
        :rtype: StatusSignal[celsius]
        """
        return self._common_lookup(SpnValue.PRO_SUPPLY_AND_TEMP_DEVICE_TEMP2.value, 0, None, "ancillary_device_temp", True, celsius)
    
    def get_fault_hardware(self) -> StatusSignal[bool]:
        """
        Hardware fault occurred
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: Fault_Hardware Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.FAULT_HARDWARE.value, 0, None, "fault_hardware", True, bool)
    
    def get_sticky_fault_hardware(self) -> StatusSignal[bool]:
        """
        Hardware fault occurred
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: StickyFault_Hardware Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.STICKY_FAULT_HARDWARE.value, 0, None, "sticky_fault_hardware", True, bool)
    
    def get_fault_proc_temp(self) -> StatusSignal[bool]:
        """
        Processor temperature exceeded limit
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: Fault_ProcTemp Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.FAULT_PROC_TEMP.value, 0, None, "fault_proc_temp", True, bool)
    
    def get_sticky_fault_proc_temp(self) -> StatusSignal[bool]:
        """
        Processor temperature exceeded limit
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: StickyFault_ProcTemp Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.STICKY_FAULT_PROC_TEMP.value, 0, None, "sticky_fault_proc_temp", True, bool)
    
    def get_fault_device_temp(self) -> StatusSignal[bool]:
        """
        Device temperature exceeded limit
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: Fault_DeviceTemp Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.FAULT_DEVICE_TEMP.value, 0, None, "fault_device_temp", True, bool)
    
    def get_sticky_fault_device_temp(self) -> StatusSignal[bool]:
        """
        Device temperature exceeded limit
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: StickyFault_DeviceTemp Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.STICKY_FAULT_DEVICE_TEMP.value, 0, None, "sticky_fault_device_temp", True, bool)
    
    def get_fault_undervoltage(self) -> StatusSignal[bool]:
        """
        Device supply voltage dropped to near brownout levels
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: Fault_Undervoltage Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.FAULT_UNDERVOLTAGE.value, 0, None, "fault_undervoltage", True, bool)
    
    def get_sticky_fault_undervoltage(self) -> StatusSignal[bool]:
        """
        Device supply voltage dropped to near brownout levels
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: StickyFault_Undervoltage Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.STICKY_FAULT_UNDERVOLTAGE.value, 0, None, "sticky_fault_undervoltage", True, bool)
    
    def get_fault_boot_during_enable(self) -> StatusSignal[bool]:
        """
        Device boot while detecting the enable signal
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: Fault_BootDuringEnable Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.FAULT_BOOT_DURING_ENABLE.value, 0, None, "fault_boot_during_enable", True, bool)
    
    def get_sticky_fault_boot_during_enable(self) -> StatusSignal[bool]:
        """
        Device boot while detecting the enable signal
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: StickyFault_BootDuringEnable Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.STICKY_FAULT_BOOT_DURING_ENABLE.value, 0, None, "sticky_fault_boot_during_enable", True, bool)
    
    def get_fault_unlicensed_feature_in_use(self) -> StatusSignal[bool]:
        """
        An unlicensed feature is in use, device may not behave as expected.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: Fault_UnlicensedFeatureInUse Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.FAULT_UNLICENSED_FEATURE_IN_USE.value, 0, None, "fault_unlicensed_feature_in_use", True, bool)
    
    def get_sticky_fault_unlicensed_feature_in_use(self) -> StatusSignal[bool]:
        """
        An unlicensed feature is in use, device may not behave as expected.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: StickyFault_UnlicensedFeatureInUse Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.STICKY_FAULT_UNLICENSED_FEATURE_IN_USE.value, 0, None, "sticky_fault_unlicensed_feature_in_use", True, bool)
    
    def get_fault_bridge_brownout(self) -> StatusSignal[bool]:
        """
        Bridge was disabled most likely due to supply voltage dropping too
        low.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: Fault_BridgeBrownout Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.FAULT_TALONFX_BRIDGE_BROWNOUT.value, 0, None, "fault_bridge_brownout", True, bool)
    
    def get_sticky_fault_bridge_brownout(self) -> StatusSignal[bool]:
        """
        Bridge was disabled most likely due to supply voltage dropping too
        low.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: StickyFault_BridgeBrownout Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.STICKY_FAULT_TALONFX_BRIDGE_BROWNOUT.value, 0, None, "sticky_fault_bridge_brownout", True, bool)
    
    def get_fault_remote_sensor_reset(self) -> StatusSignal[bool]:
        """
        The remote sensor has reset.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: Fault_RemoteSensorReset Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.FAULT_TALONFX_REMOTE_SENSOR_RESET.value, 0, None, "fault_remote_sensor_reset", True, bool)
    
    def get_sticky_fault_remote_sensor_reset(self) -> StatusSignal[bool]:
        """
        The remote sensor has reset.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: StickyFault_RemoteSensorReset Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.STICKY_FAULT_TALONFX_REMOTE_SENSOR_RESET.value, 0, None, "sticky_fault_remote_sensor_reset", True, bool)
    
    def get_fault_missing_differential_fx(self) -> StatusSignal[bool]:
        """
        The remote Talon FX used for differential control is not present on
        CAN Bus.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: Fault_MissingDifferentialFX Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.FAULT_TALONFX_MISSING_DIFFERENTIAL_FX.value, 0, None, "fault_missing_differential_fx", True, bool)
    
    def get_sticky_fault_missing_differential_fx(self) -> StatusSignal[bool]:
        """
        The remote Talon FX used for differential control is not present on
        CAN Bus.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: StickyFault_MissingDifferentialFX Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.STICKY_FAULT_TALONFX_MISSING_DIFFERENTIAL_FX.value, 0, None, "sticky_fault_missing_differential_fx", True, bool)
    
    def get_fault_remote_sensor_pos_overflow(self) -> StatusSignal[bool]:
        """
        The remote sensor position has overflowed. Because of the nature of
        remote sensors, it is possible for the remote sensor position to
        overflow beyond what is supported by the status signal frame. However,
        this is rare and cannot occur over the course of an FRC match under
        normal use.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: Fault_RemoteSensorPosOverflow Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.FAULT_TALONFX_REMOTE_SENSOR_POS_OVERFLOW.value, 0, None, "fault_remote_sensor_pos_overflow", True, bool)
    
    def get_sticky_fault_remote_sensor_pos_overflow(self) -> StatusSignal[bool]:
        """
        The remote sensor position has overflowed. Because of the nature of
        remote sensors, it is possible for the remote sensor position to
        overflow beyond what is supported by the status signal frame. However,
        this is rare and cannot occur over the course of an FRC match under
        normal use.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: StickyFault_RemoteSensorPosOverflow Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.STICKY_FAULT_TALONFX_REMOTE_SENSOR_POS_OVERFLOW.value, 0, None, "sticky_fault_remote_sensor_pos_overflow", True, bool)
    
    def get_fault_over_supply_v(self) -> StatusSignal[bool]:
        """
        Supply Voltage has exceeded the maximum voltage rating of device.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: Fault_OverSupplyV Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.FAULT_TALONFX_OVER_SUPPLYV.value, 0, None, "fault_over_supply_v", True, bool)
    
    def get_sticky_fault_over_supply_v(self) -> StatusSignal[bool]:
        """
        Supply Voltage has exceeded the maximum voltage rating of device.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: StickyFault_OverSupplyV Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.STICKY_FAULT_TALONFX_OVER_SUPPLYV.value, 0, None, "sticky_fault_over_supply_v", True, bool)
    
    def get_fault_unstable_supply_v(self) -> StatusSignal[bool]:
        """
        Supply Voltage is unstable.  Ensure you are using a battery and
        current limited power supply.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: Fault_UnstableSupplyV Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.FAULT_TALONFX_UNSTABLE_SUPPLYV.value, 0, None, "fault_unstable_supply_v", True, bool)
    
    def get_sticky_fault_unstable_supply_v(self) -> StatusSignal[bool]:
        """
        Supply Voltage is unstable.  Ensure you are using a battery and
        current limited power supply.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: StickyFault_UnstableSupplyV Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.STICKY_FAULT_TALONFX_UNSTABLE_SUPPLYV.value, 0, None, "sticky_fault_unstable_supply_v", True, bool)
    
    def get_fault_reverse_hard_limit(self) -> StatusSignal[bool]:
        """
        Reverse limit switch has been asserted.  Output is set to neutral.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: Fault_ReverseHardLimit Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.FAULT_TALONFX_REVERSE_HARD_LIMIT.value, 0, None, "fault_reverse_hard_limit", True, bool)
    
    def get_sticky_fault_reverse_hard_limit(self) -> StatusSignal[bool]:
        """
        Reverse limit switch has been asserted.  Output is set to neutral.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: StickyFault_ReverseHardLimit Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.STICKY_FAULT_TALONFX_REVERSE_HARD_LIMIT.value, 0, None, "sticky_fault_reverse_hard_limit", True, bool)
    
    def get_fault_forward_hard_limit(self) -> StatusSignal[bool]:
        """
        Forward limit switch has been asserted.  Output is set to neutral.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: Fault_ForwardHardLimit Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.FAULT_TALONFX_FORWARD_HARD_LIMIT.value, 0, None, "fault_forward_hard_limit", True, bool)
    
    def get_sticky_fault_forward_hard_limit(self) -> StatusSignal[bool]:
        """
        Forward limit switch has been asserted.  Output is set to neutral.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: StickyFault_ForwardHardLimit Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.STICKY_FAULT_TALONFX_FORWARD_HARD_LIMIT.value, 0, None, "sticky_fault_forward_hard_limit", True, bool)
    
    def get_fault_reverse_soft_limit(self) -> StatusSignal[bool]:
        """
        Reverse soft limit has been asserted.  Output is set to neutral.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: Fault_ReverseSoftLimit Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.FAULT_TALONFX_REVERSE_SOFT_LIMIT.value, 0, None, "fault_reverse_soft_limit", True, bool)
    
    def get_sticky_fault_reverse_soft_limit(self) -> StatusSignal[bool]:
        """
        Reverse soft limit has been asserted.  Output is set to neutral.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: StickyFault_ReverseSoftLimit Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.STICKY_FAULT_TALONFX_REVERSE_SOFT_LIMIT.value, 0, None, "sticky_fault_reverse_soft_limit", True, bool)
    
    def get_fault_forward_soft_limit(self) -> StatusSignal[bool]:
        """
        Forward soft limit has been asserted.  Output is set to neutral.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: Fault_ForwardSoftLimit Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.FAULT_TALONFX_FORWARD_SOFT_LIMIT.value, 0, None, "fault_forward_soft_limit", True, bool)
    
    def get_sticky_fault_forward_soft_limit(self) -> StatusSignal[bool]:
        """
        Forward soft limit has been asserted.  Output is set to neutral.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: StickyFault_ForwardSoftLimit Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.STICKY_FAULT_TALONFX_FORWARD_SOFT_LIMIT.value, 0, None, "sticky_fault_forward_soft_limit", True, bool)
    
    def get_fault_remote_sensor_data_invalid(self) -> StatusSignal[bool]:
        """
        The remote sensor's data is no longer trusted. This can happen if the
        remote sensor disappears from the CAN bus or if the remote sensor
        indicates its data is no longer valid, such as when a CANcoder's
        magnet strength falls into the "red" range.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: Fault_RemoteSensorDataInvalid Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.FAULT_TALONFX_MISSING_REMOTE_SENSOR.value, 0, None, "fault_remote_sensor_data_invalid", True, bool)
    
    def get_sticky_fault_remote_sensor_data_invalid(self) -> StatusSignal[bool]:
        """
        The remote sensor's data is no longer trusted. This can happen if the
        remote sensor disappears from the CAN bus or if the remote sensor
        indicates its data is no longer valid, such as when a CANcoder's
        magnet strength falls into the "red" range.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: StickyFault_RemoteSensorDataInvalid Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.STICKY_FAULT_TALONFX_MISSING_REMOTE_SENSOR.value, 0, None, "sticky_fault_remote_sensor_data_invalid", True, bool)
    
    def get_fault_fused_sensor_out_of_sync(self) -> StatusSignal[bool]:
        """
        The remote sensor used for fusion has fallen out of sync to the local
        sensor. A re-synchronization has occurred, which may cause a
        discontinuity. This typically happens if there is significant slop in
        the mechanism, or if the RotorToSensorRatio configuration parameter is
        incorrect.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: Fault_FusedSensorOutOfSync Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.FAULT_TALONFX_FUSED_SENSOR_OUT_OF_SYNC.value, 0, None, "fault_fused_sensor_out_of_sync", True, bool)
    
    def get_sticky_fault_fused_sensor_out_of_sync(self) -> StatusSignal[bool]:
        """
        The remote sensor used for fusion has fallen out of sync to the local
        sensor. A re-synchronization has occurred, which may cause a
        discontinuity. This typically happens if there is significant slop in
        the mechanism, or if the RotorToSensorRatio configuration parameter is
        incorrect.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: StickyFault_FusedSensorOutOfSync Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.STICKY_FAULT_TALONFX_FUSED_SENSOR_OUT_OF_SYNC.value, 0, None, "sticky_fault_fused_sensor_out_of_sync", True, bool)
    
    def get_fault_stator_curr_limit(self) -> StatusSignal[bool]:
        """
        Stator current limit occured.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: Fault_StatorCurrLimit Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.FAULT_TALONFX_STATOR_CURR_LIMIT.value, 0, None, "fault_stator_curr_limit", True, bool)
    
    def get_sticky_fault_stator_curr_limit(self) -> StatusSignal[bool]:
        """
        Stator current limit occured.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: StickyFault_StatorCurrLimit Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.STICKY_FAULT_TALONFX_STATOR_CURR_LIMIT.value, 0, None, "sticky_fault_stator_curr_limit", True, bool)
    
    def get_fault_supply_curr_limit(self) -> StatusSignal[bool]:
        """
        Supply current limit occured.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: Fault_SupplyCurrLimit Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.FAULT_TALONFX_SUPPLY_CURR_LIMIT.value, 0, None, "fault_supply_curr_limit", True, bool)
    
    def get_sticky_fault_supply_curr_limit(self) -> StatusSignal[bool]:
        """
        Supply current limit occured.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: StickyFault_SupplyCurrLimit Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.STICKY_FAULT_TALONFX_SUPPLY_CURR_LIMIT.value, 0, None, "sticky_fault_supply_curr_limit", True, bool)
    
    def get_fault_using_fused_ca_ncoder_while_unlicensed(self) -> StatusSignal[bool]:
        """
        Using Fused CANcoder feature while unlicensed. Device has fallen back
        to remote CANcoder.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: Fault_UsingFusedCANcoderWhileUnlicensed Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.FAULT_TALONFX_USING_FUSED_CCWHILE_UNLICENSED.value, 0, None, "fault_using_fused_ca_ncoder_while_unlicensed", True, bool)
    
    def get_sticky_fault_using_fused_ca_ncoder_while_unlicensed(self) -> StatusSignal[bool]:
        """
        Using Fused CANcoder feature while unlicensed. Device has fallen back
        to remote CANcoder.
        
          Default Value: False
        
        Default Rates:
          CAN: 4.0 Hz
        
        :returns: StickyFault_UsingFusedCANcoderWhileUnlicensed Status Signal Object
        :rtype: StatusSignal[bool]
        """
        return self._common_lookup(SpnValue.STICKY_FAULT_TALONFX_USING_FUSED_CCWHILE_UNLICENSED.value, 0, None, "sticky_fault_using_fused_ca_ncoder_while_unlicensed", True, bool)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

    def set_control(self, request: SupportsSendRequest) -> StatusCode:
        """
        Control motor with generic control request object.

        If control request is not supported by device, this request
        will fail with StatusCode NotSupported

        :param request: Control object to request of the device
        :type request: SupportsSendRequest
        :return: StatusCode of the request
        :rtype: StatusCode
        """
        if isinstance(request, (DutyCycleOut, TorqueCurrentFOC, VoltageOut, PositionDutyCycle, PositionVoltage, PositionTorqueCurrentFOC, VelocityDutyCycle, VelocityVoltage, VelocityTorqueCurrentFOC, MotionMagicDutyCycle, MotionMagicVoltage, MotionMagicTorqueCurrentFOC, DifferentialDutyCycle, DifferentialVoltage, DifferentialPositionDutyCycle, DifferentialPositionVoltage, DifferentialVelocityDutyCycle, DifferentialVelocityVoltage, DifferentialMotionMagicDutyCycle, DifferentialMotionMagicVoltage, Follower, StrictFollower, DifferentialFollower, DifferentialStrictFollower, NeutralOut, CoastOut, StaticBrake, MusicTone, MotionMagicVelocityDutyCycle, MotionMagicVelocityTorqueCurrentFOC, MotionMagicVelocityVoltage, MotionMagicExpoDutyCycle, MotionMagicExpoVoltage, MotionMagicExpoTorqueCurrentFOC, DynamicMotionMagicDutyCycle, DynamicMotionMagicVoltage, DynamicMotionMagicTorqueCurrentFOC)):
            return self._set_control_private(request)
        return StatusCode.NOT_SUPPORTED

    
    def set_position(self, new_value: rotation, timeout_seconds: second = 0.050) -> StatusCode:
        """
        Sets the mechanism position of the device in mechanism rotations.
        
        :param new_value: Value to set to. Units are in rotations.
        :type new_value: rotation
        :param timeout_seconds: Maximum time to wait up to in seconds.
        :type timeout_seconds: second
        :returns: StatusCode of the set command
        :rtype: StatusCode
        """
    
        return self.configurator.set_position(new_value, timeout_seconds)
    
    def clear_sticky_faults(self, timeout_seconds: second = 0.050) -> StatusCode:
        """
        Clear the sticky faults in the device.
        
        This typically has no impact on the device functionality.  Instead, it
        just clears telemetry faults that are accessible via API and Tuner
        Self-Test.
        
        :param timeout_seconds: Maximum time to wait up to in seconds.
        :type timeout_seconds: second
        :returns: StatusCode of the set command
        :rtype: StatusCode
        """
    
        return self.configurator.clear_sticky_faults(timeout_seconds)
    
    def clear_sticky_fault_hardware(self, timeout_seconds: second = 0.050) -> StatusCode:
        """
        Clear sticky fault: Hardware fault occurred
        
        :param timeout_seconds: Maximum time to wait up to in seconds.
        :type timeout_seconds: second
        :returns: StatusCode of the set command
        :rtype: StatusCode
        """
    
        return self.configurator.clear_sticky_fault_hardware(timeout_seconds)
    
    def clear_sticky_fault_proc_temp(self, timeout_seconds: second = 0.050) -> StatusCode:
        """
        Clear sticky fault: Processor temperature exceeded limit
        
        :param timeout_seconds: Maximum time to wait up to in seconds.
        :type timeout_seconds: second
        :returns: StatusCode of the set command
        :rtype: StatusCode
        """
    
        return self.configurator.clear_sticky_fault_proc_temp(timeout_seconds)
    
    def clear_sticky_fault_device_temp(self, timeout_seconds: second = 0.050) -> StatusCode:
        """
        Clear sticky fault: Device temperature exceeded limit
        
        :param timeout_seconds: Maximum time to wait up to in seconds.
        :type timeout_seconds: second
        :returns: StatusCode of the set command
        :rtype: StatusCode
        """
    
        return self.configurator.clear_sticky_fault_device_temp(timeout_seconds)
    
    def clear_sticky_fault_undervoltage(self, timeout_seconds: second = 0.050) -> StatusCode:
        """
        Clear sticky fault: Device supply voltage dropped to near brownout
        levels
        
        :param timeout_seconds: Maximum time to wait up to in seconds.
        :type timeout_seconds: second
        :returns: StatusCode of the set command
        :rtype: StatusCode
        """
    
        return self.configurator.clear_sticky_fault_undervoltage(timeout_seconds)
    
    def clear_sticky_fault_boot_during_enable(self, timeout_seconds: second = 0.050) -> StatusCode:
        """
        Clear sticky fault: Device boot while detecting the enable signal
        
        :param timeout_seconds: Maximum time to wait up to in seconds.
        :type timeout_seconds: second
        :returns: StatusCode of the set command
        :rtype: StatusCode
        """
    
        return self.configurator.clear_sticky_fault_boot_during_enable(timeout_seconds)
    
    def clear_sticky_fault_bridge_brownout(self, timeout_seconds: second = 0.050) -> StatusCode:
        """
        Clear sticky fault: Bridge was disabled most likely due to supply
        voltage dropping too low.
        
        :param timeout_seconds: Maximum time to wait up to in seconds.
        :type timeout_seconds: second
        :returns: StatusCode of the set command
        :rtype: StatusCode
        """
    
        return self.configurator.clear_sticky_fault_bridge_brownout(timeout_seconds)
    
    def clear_sticky_fault_remote_sensor_reset(self, timeout_seconds: second = 0.050) -> StatusCode:
        """
        Clear sticky fault: The remote sensor has reset.
        
        :param timeout_seconds: Maximum time to wait up to in seconds.
        :type timeout_seconds: second
        :returns: StatusCode of the set command
        :rtype: StatusCode
        """
    
        return self.configurator.clear_sticky_fault_remote_sensor_reset(timeout_seconds)
    
    def clear_sticky_fault_missing_differential_fx(self, timeout_seconds: second = 0.050) -> StatusCode:
        """
        Clear sticky fault: The remote Talon FX used for differential control
        is not present on CAN Bus.
        
        :param timeout_seconds: Maximum time to wait up to in seconds.
        :type timeout_seconds: second
        :returns: StatusCode of the set command
        :rtype: StatusCode
        """
    
        return self.configurator.clear_sticky_fault_missing_differential_fx(timeout_seconds)
    
    def clear_sticky_fault_remote_sensor_pos_overflow(self, timeout_seconds: second = 0.050) -> StatusCode:
        """
        Clear sticky fault: The remote sensor position has overflowed. Because
        of the nature of remote sensors, it is possible for the remote sensor
        position to overflow beyond what is supported by the status signal
        frame. However, this is rare and cannot occur over the course of an
        FRC match under normal use.
        
        :param timeout_seconds: Maximum time to wait up to in seconds.
        :type timeout_seconds: second
        :returns: StatusCode of the set command
        :rtype: StatusCode
        """
    
        return self.configurator.clear_sticky_fault_remote_sensor_pos_overflow(timeout_seconds)
    
    def clear_sticky_fault_over_supply_v(self, timeout_seconds: second = 0.050) -> StatusCode:
        """
        Clear sticky fault: Supply Voltage has exceeded the maximum voltage
        rating of device.
        
        :param timeout_seconds: Maximum time to wait up to in seconds.
        :type timeout_seconds: second
        :returns: StatusCode of the set command
        :rtype: StatusCode
        """
    
        return self.configurator.clear_sticky_fault_over_supply_v(timeout_seconds)
    
    def clear_sticky_fault_unstable_supply_v(self, timeout_seconds: second = 0.050) -> StatusCode:
        """
        Clear sticky fault: Supply Voltage is unstable.  Ensure you are using
        a battery and current limited power supply.
        
        :param timeout_seconds: Maximum time to wait up to in seconds.
        :type timeout_seconds: second
        :returns: StatusCode of the set command
        :rtype: StatusCode
        """
    
        return self.configurator.clear_sticky_fault_unstable_supply_v(timeout_seconds)
    
    def clear_sticky_fault_reverse_hard_limit(self, timeout_seconds: second = 0.050) -> StatusCode:
        """
        Clear sticky fault: Reverse limit switch has been asserted.  Output is
        set to neutral.
        
        :param timeout_seconds: Maximum time to wait up to in seconds.
        :type timeout_seconds: second
        :returns: StatusCode of the set command
        :rtype: StatusCode
        """
    
        return self.configurator.clear_sticky_fault_reverse_hard_limit(timeout_seconds)
    
    def clear_sticky_fault_forward_hard_limit(self, timeout_seconds: second = 0.050) -> StatusCode:
        """
        Clear sticky fault: Forward limit switch has been asserted.  Output is
        set to neutral.
        
        :param timeout_seconds: Maximum time to wait up to in seconds.
        :type timeout_seconds: second
        :returns: StatusCode of the set command
        :rtype: StatusCode
        """
    
        return self.configurator.clear_sticky_fault_forward_hard_limit(timeout_seconds)
    
    def clear_sticky_fault_reverse_soft_limit(self, timeout_seconds: second = 0.050) -> StatusCode:
        """
        Clear sticky fault: Reverse soft limit has been asserted.  Output is
        set to neutral.
        
        :param timeout_seconds: Maximum time to wait up to in seconds.
        :type timeout_seconds: second
        :returns: StatusCode of the set command
        :rtype: StatusCode
        """
    
        return self.configurator.clear_sticky_fault_reverse_soft_limit(timeout_seconds)
    
    def clear_sticky_fault_forward_soft_limit(self, timeout_seconds: second = 0.050) -> StatusCode:
        """
        Clear sticky fault: Forward soft limit has been asserted.  Output is
        set to neutral.
        
        :param timeout_seconds: Maximum time to wait up to in seconds.
        :type timeout_seconds: second
        :returns: StatusCode of the set command
        :rtype: StatusCode
        """
    
        return self.configurator.clear_sticky_fault_forward_soft_limit(timeout_seconds)
    
    def clear_sticky_fault_remote_sensor_data_invalid(self, timeout_seconds: second = 0.050) -> StatusCode:
        """
        Clear sticky fault: The remote sensor's data is no longer trusted.
        This can happen if the remote sensor disappears from the CAN bus or if
        the remote sensor indicates its data is no longer valid, such as when
        a CANcoder's magnet strength falls into the "red" range.
        
        :param timeout_seconds: Maximum time to wait up to in seconds.
        :type timeout_seconds: second
        :returns: StatusCode of the set command
        :rtype: StatusCode
        """
    
        return self.configurator.clear_sticky_fault_remote_sensor_data_invalid(timeout_seconds)
    
    def clear_sticky_fault_fused_sensor_out_of_sync(self, timeout_seconds: second = 0.050) -> StatusCode:
        """
        Clear sticky fault: The remote sensor used for fusion has fallen out
        of sync to the local sensor. A re-synchronization has occurred, which
        may cause a discontinuity. This typically happens if there is
        significant slop in the mechanism, or if the RotorToSensorRatio
        configuration parameter is incorrect.
        
        :param timeout_seconds: Maximum time to wait up to in seconds.
        :type timeout_seconds: second
        :returns: StatusCode of the set command
        :rtype: StatusCode
        """
    
        return self.configurator.clear_sticky_fault_fused_sensor_out_of_sync(timeout_seconds)
    
    def clear_sticky_fault_stator_curr_limit(self, timeout_seconds: second = 0.050) -> StatusCode:
        """
        Clear sticky fault: Stator current limit occured.
        
        :param timeout_seconds: Maximum time to wait up to in seconds.
        :type timeout_seconds: second
        :returns: StatusCode of the set command
        :rtype: StatusCode
        """
    
        return self.configurator.clear_sticky_fault_stator_curr_limit(timeout_seconds)
    
    def clear_sticky_fault_supply_curr_limit(self, timeout_seconds: second = 0.050) -> StatusCode:
        """
        Clear sticky fault: Supply current limit occured.
        
        :param timeout_seconds: Maximum time to wait up to in seconds.
        :type timeout_seconds: second
        :returns: StatusCode of the set command
        :rtype: StatusCode
        """
    
        return self.configurator.clear_sticky_fault_supply_curr_limit(timeout_seconds)

