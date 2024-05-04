"""
Class and units used for signals produced by Phoenix Hardware
"""

"""
Copyright (C) Cross The Road Electronics.  All rights reserved.
License information can be found in CTRE_LICENSE.txt
For support and suggestions contact support@ctr-electronics.com or file
an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
"""

from typing import TypeVar, Generic, Callable
import ctypes
from phoenix6.base_status_signal import BaseStatusSignal
from phoenix6.status_code import StatusCode
from phoenix6.phoenix_native import Native, SignalValues, ReturnValues
from phoenix6.error_reporting import report_status_code
from phoenix6.timestamp import TimestampSource
from phoenix6.units import second

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from phoenix6.hardware.device_identifier import DeviceIdentifier

T = TypeVar("T")

class StatusSignal(BaseStatusSignal, Generic[T]):
    """
    Represents a status signal with data of type T, and
    operations available to retrieve information about
    the signal.
    """

    def __init__(self, error: StatusCode, device_identifier: 'DeviceIdentifier', spn: int, report_if_old_func: Callable[[], None], generator: Callable[[], dict[int, 'StatusSignal']], signal_name: str, signal_type: type):
        """
        Construct a StatusSignal object

        :param error: Status code to construct this with. If this is not None,
                      this StatusSignal will be an error-only StatusSignal
        :type error: StatusCode
        :param device_identifier: Device Identifier for this signal
        :type device_identifier: DeviceIdentifier
        :param spn: SPN for this signal
        :type spn: int
        :param report_if_old_func: Function to call if device is too old
        :type report_if_old_func: Callable[[], None]
        :param generator: Callable function that returns a dictionary
                          mapping one signal to another
        :type generator: Callable[[], dict[int, StatusSignal]]
        :param signal_name: Name of signal
        :type signal_name: str
        :param signal_type: Type of signal for the generic
        :type signal_type: type
        """
        super().__init__(device_identifier, spn, signal_name, report_if_old_func if error is None else lambda: None)
        if error is None:
            # Error is not explicit, so this isn't an error construction
            self.__contains_underlying_types = generator is not None
            self.__basic_type_map = generator() if self.__contains_underlying_types else None
            self.__signal_type = signal_type
        else:
            self.__contains_underlying_types = False
            self.__basic_type_map = None
            self.__signal_type = float
            self._status = error

    @property
    def value(self) -> T:
        """
        Gets the value inside this StatusSignal

        :return: The value of this StatusSignal
        :rtype: T
        """
        return self.__signal_type(self._value)

    def __refresh_mappable(self, wait_for_signal: bool, timeout_seconds: second):
        if not self.__contains_underlying_types:
            return
        to_send = SignalValues()
        to_send.devicehash = self._identifier.device_hash
        to_send.spn = self._spn
        to_get = ReturnValues()
        self._status = StatusCode(Native.instance().c_ctre_phoenix6_get_signal(1, ctypes.byref(to_send), ctypes.byref(to_get), ctypes.c_char_p(bytes(self._identifier.network, 'utf-8')), 1 if wait_for_signal else 0, timeout_seconds))

    def __refresh_nonmappable(self, wait_for_signal: bool, timeout_seconds: second):
        if self.__contains_underlying_types:
            return
        to_send = SignalValues()
        to_send.devicehash = self._identifier.device_hash
        to_send.spn = self._spn
        to_get = ReturnValues()
        self._status = StatusCode(Native.instance().c_ctre_phoenix6_get_signal(1, ctypes.byref(to_send), ctypes.byref(to_get), ctypes.c_char_p(bytes(self._identifier.network, 'utf-8')), 1 if wait_for_signal else 0, timeout_seconds))

        if self._status.is_error():
            return

        self._value = to_get.outValue
        self._all_timestamps.update(
            to_get.swtimestampseconds, TimestampSource.System, True,
            to_get.hwtimestampseconds, TimestampSource.CANivore, True,
            to_get.ecutimestampseconds, TimestampSource.Device, to_get.ecutimestampseconds != 0.0
        )

    def __update_value(self, wait_for_signal: bool, timeout: second, report_error: bool):
        self._report_if_old_func()
        if self.__contains_underlying_types:
            self.__refresh_mappable(wait_for_signal, timeout)
        else:
            self.__refresh_nonmappable(wait_for_signal, timeout)

        if report_error and not self._status.is_ok():
            device = str(self._identifier) + " Status Signal " + self._name
            report_status_code(self._status, device)

    def refresh(self, report_error: bool = True) -> 'StatusSignal[T]':
        """
        Refreshes the value of this status signal.

        If the user application caches this StatusSignal object
        instead of periodically fetching it from the hardware
        device, this function must be called to fetch fresh data.

        This performs a non-blockin refresh operation. If you want
        to wait until you receive data, call wait_for_update instead.

        :param report_error: Whether to report any errors to the console, defaults to True
        :type report_error: bool, optional
        :return: Reference to itself
        :rtype: StatusSignal[T]
        """
        self.__update_value(False, 0, report_error)
        return self

    def wait_for_update(self, timeout_seconds: second, report_error: bool = True) -> 'StatusSignal[T]':
        """
        Waits up to timeout_seconds to get up-to-date status
        signal value.

        This performs a blocking refresh operation. If you want
        to non-blocking refresh the signal, call refresh instead.

        :param timeout_seconds: Maximum time to wait for a signal to update
        :type timeout_seconds: second
        :param report_error: Whether to report any errors to the console, defaults to True
        :type report_error: bool, optional
        :return: Reference to itself
        :rtype: StatusSignal[T]
        """
        self.__update_value(True, timeout_seconds, report_error)
        return self

    def as_supplier(self) -> Callable[[], T]:
        """
        Returns a lambda that calls refresh and value on this object.
        This is useful for command-based programming.

        :return: Lambda that refreshes this signal and returns it
        :rtype: Callable[[], T]
        """
        return lambda: self.refresh().value

    def __str__(self) -> str:
        """
        Gets the string representation of this object.

        Includes the value of the signal and the units associated

        :return: String representation of this object
        :rtype: str
        """
        return f"{self.value} {self.units}"
