from __future__ import annotations
import typing
import wpilib._wpilib
import wpimath.units
__all__ = ['OnBoardIO', 'RomiGyro', 'RomiMotor']
class OnBoardIO:
    """
    This class represents the onboard IO of the Romi
    reference robot. This includes the pushbuttons and
    LEDs.
    
    DIO 0 - Button A (input only)
    DIO 1 - Button B (input) or Green LED (output)
    DIO 2 - Button C (input) or Red LED (output)
    DIO 3 - Yellow LED (output only)
    """
    class ChannelMode:
        """
        Members:
        
          INPUT
        
          OUTPUT
        """
        INPUT: typing.ClassVar[OnBoardIO.ChannelMode]  # value = <ChannelMode.INPUT: 0>
        OUTPUT: typing.ClassVar[OnBoardIO.ChannelMode]  # value = <ChannelMode.OUTPUT: 1>
        __members__: typing.ClassVar[dict[str, OnBoardIO.ChannelMode]]  # value = {'INPUT': <ChannelMode.INPUT: 0>, 'OUTPUT': <ChannelMode.OUTPUT: 1>}
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
    kMessageInterval: typing.ClassVar[float] = 1.0
    def __init__(self, dio1: OnBoardIO.ChannelMode, dio2: OnBoardIO.ChannelMode) -> None:
        ...
    def getButtonAPressed(self) -> bool:
        """
        Gets if the A button is pressed.
        """
    def getButtonBPressed(self) -> bool:
        """
        Gets if the B button is pressed.
        """
    def getButtonCPressed(self) -> bool:
        """
        Gets if the C button is pressed.
        """
    def setGreenLed(self, value: bool) -> None:
        """
        Sets the green LED.
        """
    def setRedLed(self, value: bool) -> None:
        """
        Sets the red LED.
        """
    def setYellowLed(self, value: bool) -> None:
        """
        Sets the yellow LED.
        """
    @property
    def m_nextMessageTime(self) -> wpimath.units.seconds:
        ...
class RomiGyro:
    """
    Use a rate gyro to return the robots heading relative to a starting position.
    
    This class is for the Romi onboard gyro, and will only work in
    simulation/Romi mode. Only one instance of a RomiGyro is supported.
    """
    def __init__(self) -> None:
        ...
    def getAngle(self) -> float:
        """
        Return the actual angle in degrees that the robot is currently facing.
        
        The angle is based on integration of the returned rate form the gyro.
        The angle is continuous, that is, it will continue from 360->361 degrees.
        This allows algorithms that wouldn't want to see a discontinuity in the
        gyro output as it sweeps from 360 to 0 on the second time around.
        
        :returns: the current heading of the robot in degrees.
        """
    def getAngleX(self) -> float:
        """
        Gets the currently reported angle around the X-axis
        """
    def getAngleY(self) -> float:
        """
        Gets the currently reported angle around the X-axis
        """
    def getAngleZ(self) -> float:
        """
        Gets the currently reported angle around the X-axis
        """
    def getRate(self) -> float:
        """
        Return the rate of rotation of the gyro
        
        The rate is based on the most recent reading of the gyro.
        
        :returns: the current rate in degrees per second
        """
    def getRateX(self) -> float:
        """
        Gets the rate of turn in degrees-per-second around the X-axis
        """
    def getRateY(self) -> float:
        """
        Gets the rate of turn in degrees-per-second around the Y-axis
        """
    def getRateZ(self) -> float:
        """
        Gets the rate of turn in degrees-per-second around the Z-axis
        """
    def reset(self) -> None:
        """
        Resets the gyro
        """
class RomiMotor(wpilib._wpilib.PWMMotorController):
    """
    RomiMotor
    
    A general use PWM motor controller representing the motors on a Romi robot
    """
    def __init__(self, channel: int) -> None:
        """
        Constructor for a RomiMotor.
        
        :param channel: The PWM channel that the RomiMotor is attached to.
                        0 is left, 1 is right
        """
