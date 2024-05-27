import copy
from dataclasses import dataclass

import phoenix6
import rev
from pint import Quantity
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d

from .sensor import SparkMaxEncoderType, SparkMaxAbsoluteEncoder
from ..abstract.motor import CoaxialDriveComponent, CoaxialAzimuthComponent
from .. import conversions, u
from ..abstract.sensor import AbsoluteEncoder


class Falcon500CoaxialDriveComponent(CoaxialDriveComponent):
    @dataclass
    class Parameters:
        wheel_circumference: Quantity
        gear_ratio: float

        max_speed: Quantity

        open_loop_ramp_rate: float
        closed_loop_ramp_rate: float

        continuous_current_limit: int
        peak_current_limit: int
        peak_current_duration: float

        neutral_mode: phoenix6.signals.NeutralModeValue

        kP: float
        kI: float
        kD: float

        kS: float
        kV: float
        kA: float

        invert_motor: bool

        def in_standard_units(self):
            data = copy.deepcopy(self)
            data.wheel_circumference = data.wheel_circumference.m_as(u.m)
            data.max_speed = data.max_speed.m_as(u.m / u.s)
            return data

        # noinspection PyPep8Naming
        def create_TalonFX_config(self) -> phoenix6.configs.TalonFXConfiguration:
            motor_config = phoenix6.configs.TalonFXConfiguration()

            supply_limit = phoenix6.configs.CurrentLimitsConfigs()
            supply_limit.supply_current_limit_enable = True
            supply_limit.supply_current_limit = self.continuous_current_limit
            supply_limit.supply_current_threshold = self.peak_current_limit
            supply_limit.supply_time_threshold = self.peak_current_duration

            motor_config.slot0.k_p = self.kP
            motor_config.slot0.k_i = self.kI
            motor_config.slot0.k_d = self.kD
            motor_config.slot0.k_s = self.kS
            motor_config.slot0.k_v = self.kV
            motor_config.slot0.k_a = self.kA
            motor_config.current_limits = supply_limit
            # motor_config.initializationStrategy = phoenix5.sensors.SensorInitializationStrategy.BootToZero
            # the above line was deprecated in phoenix6 - might cause an issue???
            motor_config.open_loop_ramps.duty_cycle_open_loop_ramp_period = (
                self.open_loop_ramp_rate
            )
            motor_config.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = (
                self.closed_loop_ramp_rate
            )

            if self.invert_motor:
                motor_config.motor_output.inverted = (
                    phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE
                )
            else:
                motor_config.motor_output.inverted = (
                    phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
                )
            motor_config.motor_output.neutral_mode = self.neutral_mode

            return motor_config

    def __init__(self, id_: int | tuple[int, str], parameters: Parameters):
        self._params = parameters.in_standard_units()

        try:
            # Unpack tuple of motor id and CAN bus id into TalonFX constructor
            self._motor = phoenix6.hardware.TalonFX(*id_)
        except TypeError:
            # Only an int was provided for id_
            self._motor = phoenix6.hardware.TalonFX(id_)

        self._config()
        self.reset()

        self._feedforward = SimpleMotorFeedforwardMeters(
            parameters.kS, parameters.kV, parameters.kA
        )

    def _config(self):
        settings = self._params.create_TalonFX_config()
        self._motor.configurator.apply(settings)

    def follow_velocity_open(self, velocity: float):
        percent_out = velocity / self._params.max_speed
        duty_cycle_out = phoenix6.controls.DutyCycleOut(percent_out)
        self._motor.set_control(duty_cycle_out)

    def follow_velocity_closed(self, velocity: float):
        converted_velocity = conversions.mps_to_falcon(
            velocity, self._params.wheel_circumference, self._params.gear_ratio
        )
        # self._motor.set(
        #     phoenix5.ControlMode.Velocity,
        #     converted_velocity,
        #     phoenix5.DemandType.ArbitraryFeedForward,
        #     self._feedforward.calculate(velocity),
        # )
        # might have been implelemted incorrectly
        velocity_duty_cycle = phoenix6.controls.VelocityDutyCycle(converted_velocity)
        self._motor.set_control(velocity_duty_cycle)

    def set_voltage(self, volts: float):
        percent_output = volts / self._motor.getBusVoltage()
        duty_cycle_out = phoenix6.controls.DutyCycleOut(percent_output)
        self._motor.set_control(duty_cycle_out)

    def reset(self):
        self._motor.set_position(0)

    @property
    def velocity(self) -> float:
        return conversions.falcon_to_mps(
            self._motor.get_velocity().value,
            self._params.wheel_circumference,
            self._params.gear_ratio,
        )

    @property
    def distance(self) -> float:
        # consider performing latency adjustment
        return conversions.falcon_to_metres(
            self._motor.get_position().value,
            self._params.wheel_circumference,
            self._params.gear_ratio,
        )

    @property
    def voltage(self) -> float:
        return self._motor.get_motor_voltage().value


class Falcon500CoaxialAzimuthComponent(CoaxialAzimuthComponent):
    @dataclass
    class Parameters:
        gear_ratio: float

        max_angular_velocity: Quantity

        ramp_rate: float

        continuous_current_limit: int
        peak_current_limit: int
        peak_current_duration: float

        neutral_mode: phoenix6.signals.NeutralModeValue

        kP: float
        kI: float
        kD: float

        invert_motor: bool

        def in_standard_units(self):
            data = copy.deepcopy(self)
            data.max_angular_velocity = data.max_angular_velocity.m_as(u.rad / u.s)
            return data

        # noinspection PyPep8Naming
        def create_TalonFX_config(self) -> phoenix6.configs.TalonFXConfiguration:
            motor_config = phoenix6.configs.TalonFXConfiguration()

            supply_limit = phoenix6.configs.CurrentLimitsConfigs()
            supply_limit.supply_current_limit_enable = True
            supply_limit.supply_current_limit = self.continuous_current_limit
            supply_limit.supply_current_threshold = self.peak_current_limit
            supply_limit.supply_time_threshold = self.peak_current_duration

            motor_config.slot0.k_p = self.kP
            motor_config.slot0.k_i = self.kI
            motor_config.slot0.k_d = self.kD
            motor_config.current_limits = supply_limit
            # motor_config.initializationStrategy = phoenix5.sensors.SensorInitializationStrategy.BootToZero

            if self.invert_motor:
                motor_config.motor_output.inverted = (
                    phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE
                )
            else:
                motor_config.motor_output.inverted = (
                    phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
                )
            motor_config.motor_output.neutral_mode = self.neutral_mode

            return motor_config

    def __init__(
        self,
        id_: int | tuple[int, str],
        azimuth_offset: Rotation2d,
        parameters: Parameters,
        absolute_encoder: AbsoluteEncoder,
    ):
        self._params = parameters.in_standard_units()

        try:
            # Unpack tuple of motor id and CAN bus id into TalonFX constructor
            self._motor = phoenix6.hardware.TalonFX(*id_)
        except TypeError:
            # Only an int was provided for id_
            self._motor = phoenix6.hardware.TalonFX(id_)

        self._absolute_encoder = absolute_encoder
        self._offset = azimuth_offset

        self._config()
        self.reset()

    def _config(self):
        settings = self._params.create_TalonFX_config()
        self._motor.configurator.apply(settings)

    def follow_angle(self, angle: Rotation2d):
        converted_angle = conversions.degrees_to_falcon(angle, self._params.gear_ratio)
        position_duty_cycle = phoenix6.controls.PositionDutyCycle(converted_angle)
        self._motor.set_control(position_duty_cycle)

    def reset(self):
        absolute_position = self._absolute_encoder.absolute_position - self._offset
        converted_position = conversions.degrees_to_falcon(
            absolute_position, self._params.gear_ratio
        )
        self._motor.set_position(converted_position)

    @property
    def rotational_velocity(self) -> float:
        return conversions.falcon_to_radps(
            self._motor.get_velocity().value, self._params.gear_ratio
        )

    @property
    def angle(self) -> Rotation2d:
        return conversions.falcon_to_degrees(
            self._motor.get_position().value, self._params.gear_ratio
        )


class NEOCoaxialDriveComponent(CoaxialDriveComponent):
    @dataclass
    class Parameters:
        wheel_circumference: Quantity
        gear_ratio: float

        max_speed: Quantity

        open_loop_ramp_rate: float
        closed_loop_ramp_rate: float

        continuous_current_limit: int
        peak_current_limit: int

        neutral_mode: rev.CANSparkMax.IdleMode

        kP: float
        kI: float
        kD: float

        kS: float
        kV: float
        kA: float

        invert_motor: bool

        def in_standard_units(self):
            data = copy.deepcopy(self)
            data.wheel_circumference = data.wheel_circumference.m_as(u.m)
            data.max_speed = data.max_speed.m_as(u.m / u.s)
            return data

    def __init__(self, id_: int, parameters: Parameters):
        self._params = parameters.in_standard_units()

        self._motor = rev.CANSparkMax(id_, rev.CANSparkMax.MotorType.kBrushless)
        self._controller = self._motor.getPIDController()
        self._encoder = self._motor.getEncoder()
        self._config()
        self.reset()

        self._feedforward = SimpleMotorFeedforwardMeters(
            parameters.kS, parameters.kV, parameters.kA
        )

    def _config(self):
        self._motor.restoreFactoryDefaults()

        self._controller.setP(self._params.kP)
        self._controller.setI(self._params.kI)
        self._controller.setD(self._params.kD)

        self._motor.setSmartCurrentLimit(self._params.continuous_current_limit)
        self._motor.setSecondaryCurrentLimit(self._params.peak_current_limit)

        self._motor.setOpenLoopRampRate(self._params.open_loop_ramp_rate)
        self._motor.setClosedLoopRampRate(self._params.closed_loop_ramp_rate)

        self._motor.setInverted(self._params.invert_motor)
        self._motor.setIdleMode(self._params.neutral_mode)

        position_conversion_factor = (
            self._params.wheel_circumference / self._params.gear_ratio
        )
        self._encoder.setPositionConversionFactor(position_conversion_factor)
        self._encoder.setVelocityConversionFactor(position_conversion_factor / 60)

    def follow_velocity_open(self, velocity: float):
        percent_out = velocity / self._params.max_speed
        self._motor.set(percent_out)

    def follow_velocity_closed(self, velocity: float):
        self._controller.setReference(
            velocity,
            rev.CANSparkMax.ControlType.kVelocity,
            arbFeedforward=self._feedforward.calculate(velocity),
        )

    def set_voltage(self, volts: float):
        self._motor.setVoltage(volts)

    def reset(self):
        self._encoder.setPosition(0)

    @property
    def velocity(self) -> float:
        return self._encoder.getVelocity()

    @property
    def distance(self) -> float:
        return self._encoder.getPosition()

    @property
    def voltage(self) -> float:
        return self._motor.getBusVoltage() * self._motor.getAppliedOutput()


class NEOCoaxialAzimuthComponent(CoaxialAzimuthComponent):
    @dataclass
    class Parameters:
        gear_ratio: float

        max_angular_velocity: Quantity

        ramp_rate: float

        continuous_current_limit: int
        peak_current_limit: int

        neutral_mode: rev.CANSparkMax.IdleMode

        kP: float
        kI: float
        kD: float

        invert_motor: bool

        def in_standard_units(self):
            data = copy.deepcopy(self)
            data.max_angular_velocity = data.max_angular_velocity.m_as(u.rad / u.s)
            return data

    def __init__(
        self,
        id_: int,
        azimuth_offset: Rotation2d,
        parameters: Parameters,
        absolute_encoder: AbsoluteEncoder | SparkMaxEncoderType,
    ):
        self._params = parameters.in_standard_units()

        self._motor = rev.CANSparkMax(id_, rev.CANSparkMax.MotorType.kBrushless)
        self._controller = self._motor.getPIDController()
        self._encoder = self._motor.getEncoder()

        # Config must be called before the absolute encoder is set up because config method
        # factory resets the SPARK MAX
        self._config()

        if isinstance(absolute_encoder, SparkMaxEncoderType):
            # Construct an AbsoluteEncoder from sensor plugged into SPARK MAX
            self._absolute_encoder = SparkMaxAbsoluteEncoder(
                self._motor, absolute_encoder
            )
        else:
            self._absolute_encoder = absolute_encoder

        self._offset = azimuth_offset

        self.reset()

    def _config(self):
        self._motor.restoreFactoryDefaults()

        self._controller.setP(self._params.kP)
        self._controller.setI(self._params.kI)
        self._controller.setD(self._params.kD)

        self._motor.setSmartCurrentLimit(self._params.continuous_current_limit)
        self._motor.setSecondaryCurrentLimit(self._params.peak_current_limit)

        self._motor.setInverted(self._params.invert_motor)
        self._motor.setIdleMode(self._params.neutral_mode)

        position_conversion_factor = 360 / self._params.gear_ratio
        self._encoder.setPositionConversionFactor(position_conversion_factor)
        self._encoder.setVelocityConversionFactor(position_conversion_factor / 60)

    def follow_angle(self, angle: Rotation2d):
        self._controller.setReference(
            angle.degrees(), rev.CANSparkMax.ControlType.kPosition
        )

    def reset(self):
        absolute_position = self._absolute_encoder.absolute_position - self._offset
        self._encoder.setPosition(absolute_position.degrees())

    @property
    def rotational_velocity(self) -> float:
        return self._encoder.getVelocity()

    @property
    def angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self._encoder.getPosition())


class DummyCoaxialComponent(CoaxialDriveComponent, CoaxialAzimuthComponent):
    """Coaxial drive or azimuth component that does nothing"""

    def __init__(self, *args):
        pass

    def follow_velocity_open(self, velocity: float):
        pass

    def follow_velocity_closed(self, velocity: float):
        pass

    def set_voltage(self, volts: float):
        pass

    def reset(self):
        pass

    @property
    def velocity(self) -> float:
        return 0

    @property
    def distance(self) -> float:
        return 0

    @property
    def voltage(self) -> float:
        return 0

    def follow_angle(self, angle: Rotation2d):
        pass

    @property
    def rotational_velocity(self) -> float:
        return 0

    @property
    def angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(0)
