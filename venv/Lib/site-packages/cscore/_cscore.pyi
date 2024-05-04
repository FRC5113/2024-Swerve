from __future__ import annotations
import numpy
import typing
import wpiutil
__all__ = ['AxisCamera', 'CameraServer', 'CvSink', 'CvSource', 'HttpCamera', 'ImageSink', 'ImageSource', 'MjpegServer', 'RawEvent', 'UsbCamera', 'UsbCameraInfo', 'VideoCamera', 'VideoEvent', 'VideoListener', 'VideoMode', 'VideoProperty', 'VideoSink', 'VideoSource', 'runMainRunLoop', 'runMainRunLoopTimeout', 'stopMainRunLoop']
class AxisCamera(HttpCamera):
    """
    A source that represents an Axis IP camera.
    """
    @typing.overload
    def __init__(self, name: str, host: str) -> None:
        """
        Create a source for an Axis IP camera.
        
        :param name: Source name (arbitrary unique identifier)
        :param host: Camera host IP or DNS name (e.g. "10.x.y.11")
        """
    @typing.overload
    def __init__(self, name: str, host: str) -> None:
        """
        Create a source for an Axis IP camera.
        
        :param name: Source name (arbitrary unique identifier)
        :param host: Camera host IP or DNS name (e.g. "10.x.y.11")
        """
    @typing.overload
    def __init__(self, name: str, host: str) -> None:
        """
        Create a source for an Axis IP camera.
        
        :param name: Source name (arbitrary unique identifier)
        :param host: Camera host IP or DNS name (e.g. "10.x.y.11")
        """
    @typing.overload
    def __init__(self, name: str, hosts: list[str]) -> None:
        """
        Create a source for an Axis IP camera.
        
        :param name:  Source name (arbitrary unique identifier)
        :param hosts: Array of Camera host IPs/DNS names
        """
class CameraServer:
    """
    Singleton class for creating and keeping camera servers.
    
    Also publishes camera information to NetworkTables.
    """
    kBasePort: typing.ClassVar[int] = 1181
    @staticmethod
    @typing.overload
    def addAxisCamera(host: str) -> AxisCamera:
        """
        Adds an Axis IP camera.
        
        This overload calls AddAxisCamera() with name "Axis Camera".
        
        :param host: Camera host IP or DNS name (e.g. "10.x.y.11")
        """
    @staticmethod
    @typing.overload
    def addAxisCamera(hosts: list[str]) -> AxisCamera:
        """
        Adds an Axis IP camera.
        
        This overload calls AddAxisCamera() with name "Axis Camera".
        
        :param hosts: Array of Camera host IPs/DNS names
        """
    @staticmethod
    @typing.overload
    def addAxisCamera(name: str, host: str) -> AxisCamera:
        """
        Adds an Axis IP camera.
        
        :param name: The name to give the camera
        :param host: Camera host IP or DNS name (e.g. "10.x.y.11")
        """
    @staticmethod
    @typing.overload
    def addAxisCamera(name: str, hosts: list[str]) -> AxisCamera:
        """
        Adds an Axis IP camera.
        
        :param name:  The name to give the camera
        :param hosts: Array of Camera host IPs/DNS names
        """
    @staticmethod
    def addCamera(camera: VideoSource) -> None:
        """
        Adds an already created camera.
        
        :param camera: Camera
        """
    @staticmethod
    @typing.overload
    def addServer(name: str) -> MjpegServer:
        """
        Adds a MJPEG server at the next available port.
        
        :param name: Server name
        """
    @staticmethod
    @typing.overload
    def addServer(name: str, port: int) -> MjpegServer:
        """
        Adds a MJPEG server.
        
        :param name: Server name
        :param port: Port number
        """
    @staticmethod
    @typing.overload
    def addServer(server: VideoSink) -> None:
        """
        Adds an already created server.
        
        :param server: Server
        """
    @staticmethod
    def addSwitchedCamera(name: str) -> MjpegServer:
        """
        Adds a virtual camera for switching between two streams.  Unlike the
        other addCamera methods, this returns a VideoSink rather than a
        VideoSource.  Calling SetSource() on the returned object can be used
        to switch the actual source of the stream.
        """
    @staticmethod
    def enableLogging(level: int | None = None) -> None:
        """
        Enable cscore logging
        """
    @staticmethod
    @typing.overload
    def getServer() -> VideoSink:
        """
        Get server for the primary camera feed.
        
        This is only valid to call after a camera feed has been added with
        StartAutomaticCapture() or AddServer().
        """
    @staticmethod
    @typing.overload
    def getServer(name: str) -> VideoSink:
        """
        Gets a server by name.
        
        :param name: Server name
        """
    @staticmethod
    @typing.overload
    def getVideo() -> CvSink:
        """
        Get OpenCV access to the primary camera feed.  This allows you to
        get images from the camera for image processing on the roboRIO.
        
        This is only valid to call after a camera feed has been added
        with startAutomaticCapture() or addServer().
        """
    @staticmethod
    @typing.overload
    def getVideo(camera: VideoSource) -> CvSink:
        """
        Get OpenCV access to the specified camera.  This allows you to get
        images from the camera for image processing on the roboRIO.
        
        :param camera: Camera (e.g. as returned by startAutomaticCapture).
        """
    @staticmethod
    @typing.overload
    def getVideo(camera: VideoSource, pixelFormat: VideoMode.PixelFormat) -> CvSink:
        """
        Get OpenCV access to the specified camera.  This allows you to get
        images from the camera for image processing on the roboRIO.
        
        :param camera:      Camera (e.g. as returned by startAutomaticCapture).
        :param pixelFormat: The desired pixelFormat of captured frames from the
                            camera
        """
    @staticmethod
    @typing.overload
    def getVideo(name: str) -> CvSink:
        """
        Get OpenCV access to the specified camera.  This allows you to get
        images from the camera for image processing on the roboRIO.
        
        :param name: Camera name
        """
    @staticmethod
    @typing.overload
    def getVideo(name: str, pixelFormat: VideoMode.PixelFormat) -> CvSink:
        """
        Get OpenCV access to the specified camera.  This allows you to get
        images from the camera for image processing on the roboRIO.
        
        :param name:        Camera name
        :param pixelFormat: The desired pixelFormat of captured frames from the
                            camera
        """
    @staticmethod
    def putVideo(name: str, width: int, height: int) -> CvSource:
        """
        Create a MJPEG stream with OpenCV input. This can be called to pass custom
        annotated images to the dashboard.
        
        :param name:   Name to give the stream
        :param width:  Width of the image being sent
        :param height: Height of the image being sent
        """
    @staticmethod
    def removeCamera(name: str) -> None:
        """
        Removes a camera by name.
        
        :param name: Camera name
        """
    @staticmethod
    def removeServer(name: str) -> None:
        """
        Removes a server by name.
        
        :param name: Server name
        """
    @staticmethod
    @typing.overload
    def startAutomaticCapture() -> UsbCamera:
        """
        Start automatically capturing images to send to the dashboard.
        
        You should call this method to see a camera feed on the dashboard. If you
        also want to perform vision processing on the roboRIO, use getVideo() to
        get access to the camera images.
        
        The first time this overload is called, it calls StartAutomaticCapture()
        with device 0, creating a camera named "USB Camera 0".  Subsequent calls
        increment the device number (e.g. 1, 2, etc).
        """
    @staticmethod
    @typing.overload
    def startAutomaticCapture(dev: int) -> UsbCamera:
        """
        Start automatically capturing images to send to the dashboard.
        
        This overload calls StartAutomaticCapture() with a name of "USB Camera
        {dev}".
        
        :param dev: The device number of the camera interface
        """
    @staticmethod
    @typing.overload
    def startAutomaticCapture(name: str, dev: int) -> UsbCamera:
        """
        Start automatically capturing images to send to the dashboard.
        
        :param name: The name to give the camera
        :param dev:  The device number of the camera interface
        """
    @staticmethod
    @typing.overload
    def startAutomaticCapture(name: str, path: str) -> UsbCamera:
        """
        Start automatically capturing images to send to the dashboard.
        
        :param name: The name to give the camera
        :param path: The device path (e.g. "/dev/video0") of the camera
        """
    @staticmethod
    @typing.overload
    def startAutomaticCapture(camera: VideoSource) -> MjpegServer:
        """
        Start automatically capturing images to send to the dashboard from
        an existing camera.
        
        :param camera: Camera
        """
    @staticmethod
    def waitForever() -> None:
        """
        Infinitely loops until the process dies
        """
class CvSink(ImageSink):
    """
    A sink for user code to accept video frames as OpenCV images.
    """
    @typing.overload
    def __init__(self, name: str, pixelFormat: VideoMode.PixelFormat = ...) -> None:
        """
        Create a sink for accepting OpenCV images.
        
        WaitForFrame() must be called on the created sink to get each new
        image.
        
        :param name:        Source name (arbitrary unique identifier)
        :param pixelFormat: Source pixel format
        """
    @typing.overload
    def __init__(self, name: str, processFrame: typing.Callable[[int], None], pixelFormat: VideoMode.PixelFormat = ...) -> None:
        """
        Create a sink for accepting OpenCV images in a separate thread.
        
        A thread will be created that calls WaitForFrame() and calls the
        processFrame() callback each time a new frame arrives.
        
        :param name:         Source name (arbitrary unique identifier)
        :param processFrame: Frame processing function; will be called with a
                             time=0 if an error occurred.  processFrame should call GetImage()
                             or GetError() as needed, but should not call (except in very
                             unusual circumstances) WaitForImage().
        :param pixelFormat:  Source pixel format
        """
    def grabFrame(self, image: numpy.ndarray, timeout: float = 0.225) -> tuple[int, numpy.ndarray]:
        """
        Wait for the next frame and get the image.
        Times out (returning 0) after timeout seconds.
        The provided image will have three 8-bit channels stored in BGR order.
        
        :returns: Frame time, or 0 on error (call GetError() to obtain the error
                  message); the frame time is in the same time base as wpi::Now(),
                  and is in 1 us increments.
        """
    def grabFrameNoTimeout(self, image: numpy.ndarray) -> tuple[int, numpy.ndarray]:
        """
        Wait for the next frame and get the image.  May block forever.
        The provided image will have three 8-bit channels stored in BGR order.
        
        :returns: Frame time, or 0 on error (call GetError() to obtain the error
                  message); the frame time is in the same time base as wpi::Now(),
                  and is in 1 us increments.
        """
class CvSource(ImageSource):
    """
    A source for user code to provide OpenCV images as video frames.
    """
    @typing.overload
    def __init__(self, name: str, mode: VideoMode) -> None:
        """
        Create an OpenCV source.
        
        :param name: Source name (arbitrary unique identifier)
        :param mode: Video mode being generated
        """
    @typing.overload
    def __init__(self, name: str, pixelFormat: VideoMode.PixelFormat, width: int, height: int, fps: int) -> None:
        """
        Create an OpenCV source.
        
        :param name:        Source name (arbitrary unique identifier)
        :param pixelFormat: Pixel format
        :param width:       width
        :param height:      height
        :param fps:         fps
        """
    def putFrame(self, image: numpy.ndarray) -> None:
        """
        Put an OpenCV image and notify sinks.
        
        Only 8-bit single-channel or 3-channel (with BGR channel order) images
        are supported. If the format, depth or channel order is different, use
        cv::Mat::convertTo() and/or cv::cvtColor() to convert it first.
        
        :param image: OpenCV image
        """
class HttpCamera(VideoCamera):
    """
    A source that represents a MJPEG-over-HTTP (IP) camera.
    """
    class HttpCameraKind:
        """
        HTTP camera kind.
        
        Members:
        
          kUnknown : Unknown camera kind.
        
          kMJPGStreamer : MJPG Streamer camera.
        
          kCSCore : CS Core camera.
        
          kAxis : Axis camera.
        """
        __members__: typing.ClassVar[dict[str, HttpCamera.HttpCameraKind]]  # value = {'kUnknown': <HttpCameraKind.kUnknown: 0>, 'kMJPGStreamer': <HttpCameraKind.kMJPGStreamer: 1>, 'kCSCore': <HttpCameraKind.kCSCore: 2>, 'kAxis': <HttpCameraKind.kAxis: 3>}
        kAxis: typing.ClassVar[HttpCamera.HttpCameraKind]  # value = <HttpCameraKind.kAxis: 3>
        kCSCore: typing.ClassVar[HttpCamera.HttpCameraKind]  # value = <HttpCameraKind.kCSCore: 2>
        kMJPGStreamer: typing.ClassVar[HttpCamera.HttpCameraKind]  # value = <HttpCameraKind.kMJPGStreamer: 1>
        kUnknown: typing.ClassVar[HttpCamera.HttpCameraKind]  # value = <HttpCameraKind.kUnknown: 0>
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
    @typing.overload
    def __init__(self, name: str, url: str, kind: HttpCamera.HttpCameraKind = ...) -> None:
        """
        Create a source for a MJPEG-over-HTTP (IP) camera.
        
        :param name: Source name (arbitrary unique identifier)
        :param url:  Camera URL (e.g. "http://10.x.y.11/video/stream.mjpg")
        :param kind: Camera kind (e.g. kAxis)
        """
    @typing.overload
    def __init__(self, name: str, url: str, kind: HttpCamera.HttpCameraKind = ...) -> None:
        """
        Create a source for a MJPEG-over-HTTP (IP) camera.
        
        :param name: Source name (arbitrary unique identifier)
        :param url:  Camera URL (e.g. "http://10.x.y.11/video/stream.mjpg")
        :param kind: Camera kind (e.g. kAxis)
        """
    @typing.overload
    def __init__(self, name: str, url: str, kind: HttpCamera.HttpCameraKind = ...) -> None:
        """
        Create a source for a MJPEG-over-HTTP (IP) camera.
        
        :param name: Source name (arbitrary unique identifier)
        :param url:  Camera URL (e.g. "http://10.x.y.11/video/stream.mjpg")
        :param kind: Camera kind (e.g. kAxis)
        """
    @typing.overload
    def __init__(self, name: str, urls: list[str], kind: HttpCamera.HttpCameraKind = ...) -> None:
        """
        Create a source for a MJPEG-over-HTTP (IP) camera.
        
        :param name: Source name (arbitrary unique identifier)
        :param urls: Array of Camera URLs
        :param kind: Camera kind (e.g. kAxis)
        """
    def getHttpCameraKind(self) -> HttpCamera.HttpCameraKind:
        """
        Get the kind of HTTP camera.
        
        Autodetection can result in returning a different value than the camera
        was created with.
        """
    def getUrls(self) -> list[str]:
        """
        Get the URLs used to connect to the camera.
        """
    def setUrls(self, urls: list[str]) -> None:
        """
        Change the URLs used to connect to the camera.
        """
class ImageSink(VideoSink):
    """
    A base class for single image reading sinks.
    """
    def getError(self) -> str:
        """
        Get error string.  Call this if WaitForFrame() returns 0 to determine
        what the error is.
        """
    def setDescription(self, description: str) -> None:
        """
        Set sink description.
        
        :param description: Description
        """
    def setEnabled(self, enabled: bool) -> None:
        """
        Enable or disable getting new frames.
        
        Disabling will cause processFrame (for callback-based CvSinks) to not
        be called and WaitForFrame() to not return.  This can be used to save
        processor resources when frames are not needed.
        """
class ImageSource(VideoSource):
    """
    A base class for single image providing sources.
    """
    def createBooleanProperty(self, name: str, defaultValue: bool, value: bool) -> VideoProperty:
        """
        Create a boolean property.
        
        :param name:         Property name
        :param defaultValue: Default value
        :param value:        Current value
        
        :returns: Property
        """
    def createIntegerProperty(self, name: str, minimum: int, maximum: int, step: int, defaultValue: int, value: int) -> VideoProperty:
        """
        Create an integer property.
        
        :param name:         Property name
        :param minimum:      Minimum value
        :param maximum:      Maximum value
        :param step:         Step value
        :param defaultValue: Default value
        :param value:        Current value
        
        :returns: Property
        """
    def createProperty(self, name: str, kind: VideoProperty.Kind, minimum: int, maximum: int, step: int, defaultValue: int, value: int) -> VideoProperty:
        """
        Create a property.
        
        :param name:         Property name
        :param kind:         Property kind
        :param minimum:      Minimum value
        :param maximum:      Maximum value
        :param step:         Step value
        :param defaultValue: Default value
        :param value:        Current value
        
        :returns: Property
        """
    def createStringProperty(self, name: str, value: str) -> VideoProperty:
        """
        Create a string property.
        
        :param name:  Property name
        :param value: Current value
        
        :returns: Property
        """
    def notifyError(self, msg: str) -> None:
        """
        Signal sinks that an error has occurred.  This should be called instead
        of NotifyFrame when an error occurs.
        
        :param msg: Notification message.
        """
    def setConnected(self, connected: bool) -> None:
        """
        Set source connection status.  Defaults to true.
        
        :param connected: True for connected, false for disconnected
        """
    def setDescription(self, description: str) -> None:
        """
        Set source description.
        
        :param description: Description
        """
    def setEnumPropertyChoices(self, property: VideoProperty, choices: list[str]) -> None:
        """
        Configure enum property choices.
        
        :param property: Property
        :param choices:  Choices
        """
class MjpegServer(VideoSink):
    """
    A sink that acts as a MJPEG-over-HTTP network server.
    """
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, name: str, listenAddress: str, port: int) -> None:
        """
        Create a MJPEG-over-HTTP server sink.
        
        :param name:          Sink name (arbitrary unique identifier)
        :param listenAddress: TCP listen address (empty string for all addresses)
        :param port:          TCP port number
        """
    @typing.overload
    def __init__(self, name: str, port: int) -> None:
        """
        Create a MJPEG-over-HTTP server sink.
        
        :param name: Sink name (arbitrary unique identifier)
        :param port: TCP port number
        """
    def getListenAddress(self) -> str:
        """
        Get the listen address of the server.
        """
    def getPort(self) -> int:
        """
        Get the port number of the server.
        """
    def setCompression(self, quality: int) -> None:
        """
        Set the compression for clients that don't specify it.
        
        Setting this will result in increased CPU usage for MJPEG source cameras
        as it will decompress and recompress the image instead of using the
        camera's MJPEG image directly.
        
        :param quality: JPEG compression quality (0-100), -1 for unspecified
        """
    def setDefaultCompression(self, quality: int) -> None:
        """
        Set the default compression used for non-MJPEG sources.  If not set,
        80 is used.  This function has no effect on MJPEG source cameras; use
        SetCompression() instead to force recompression of MJPEG source images.
        
        :param quality: JPEG compression quality (0-100)
        """
    def setFPS(self, fps: int) -> None:
        """
        Set the stream frames per second (FPS) for clients that don't specify it.
        
        It is not necessary to set this if it is the same as the source FPS.
        
        :param fps: FPS, 0 for unspecified
        """
    def setResolution(self, width: int, height: int) -> None:
        """
        Set the stream resolution for clients that don't specify it.
        
        It is not necessary to set this if it is the same as the source
        resolution.
        
        Setting this different than the source resolution will result in
        increased CPU usage, particularly for MJPEG source cameras, as it will
        decompress, resize, and recompress the image, instead of using the
        camera's MJPEG image directly.
        
        :param width:  width, 0 for unspecified
        :param height: height, 0 for unspecified
        """
class RawEvent:
    """
    Listener event
    """
class UsbCamera(VideoCamera):
    """
    A source that represents a USB camera.
    """
    @staticmethod
    def enumerateUsbCameras() -> list[UsbCameraInfo]:
        """
        Enumerate USB cameras on the local system.
        
        :returns: Vector of USB camera information (one for each camera)
        """
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, name: str, dev: int) -> None:
        """
        Create a source for a USB camera based on device number.
        
        :param name: Source name (arbitrary unique identifier)
        :param dev:  Device number (e.g. 0 for /dev/video0)
        """
    @typing.overload
    def __init__(self, name: str, path: str) -> None:
        """
        Create a source for a USB camera based on device path.
        
        :param name: Source name (arbitrary unique identifier)
        :param path: Path to device (e.g. "/dev/video0" on Linux)
        """
    def getInfo(self) -> UsbCameraInfo:
        """
        Get the full camera information for the device.
        """
    def getPath(self) -> str:
        """
        Get the path to the device.
        """
    def setConnectVerbose(self, level: int) -> None:
        """
        Set how verbose the camera connection messages are.
        
        :param level: 0=don't display Connecting message, 1=do display message
        """
    def setPath(self, path: str) -> None:
        """
        Change the path to the device.
        """
class UsbCameraInfo:
    """
    USB camera information
    """
    def __init__(self) -> None:
        ...
    @property
    def dev(self) -> int:
        """
        Device number (e.g. N in '/dev/videoN' on Linux)
        """
    @dev.setter
    def dev(self, arg0: int) -> None:
        ...
    @property
    def name(self) -> str:
        """
        Vendor/model name of the camera as provided by the USB driver
        """
    @name.setter
    def name(self, arg0: str) -> None:
        ...
    @property
    def otherPaths(self) -> list[str]:
        """
        Other path aliases to device (e.g. '/dev/v4l/by-id/...' etc on Linux)
        """
    @otherPaths.setter
    def otherPaths(self, arg0: list[str]) -> None:
        ...
    @property
    def path(self) -> str:
        """
        Path to device if available (e.g. '/dev/video0' on Linux)
        """
    @path.setter
    def path(self, arg0: str) -> None:
        ...
    @property
    def productId(self) -> int:
        """
        USB Product Id
        """
    @productId.setter
    def productId(self, arg0: int) -> None:
        ...
    @property
    def vendorId(self) -> int:
        """
        USB Vendor Id
        """
    @vendorId.setter
    def vendorId(self, arg0: int) -> None:
        ...
class VideoCamera(VideoSource):
    """
    A source that represents a video camera.
    """
    class WhiteBalance:
        """
        White balance.
        
        Members:
        
          kFixedIndoor : Fixed indoor white balance.
        
          kFixedOutdoor1 : Fixed outdoor white balance 1.
        
          kFixedOutdoor2 : Fixed outdoor white balance 2.
        
          kFixedFluorescent1 : Fixed fluorescent white balance 1.
        
          kFixedFlourescent2 : Fixed fluorescent white balance 2.
        """
        __members__: typing.ClassVar[dict[str, VideoCamera.WhiteBalance]]  # value = {'kFixedIndoor': <WhiteBalance.kFixedIndoor: 3000>, 'kFixedOutdoor1': <WhiteBalance.kFixedOutdoor1: 4000>, 'kFixedOutdoor2': <WhiteBalance.kFixedOutdoor2: 5000>, 'kFixedFluorescent1': <WhiteBalance.kFixedFluorescent1: 5100>, 'kFixedFlourescent2': <WhiteBalance.kFixedFlourescent2: 5200>}
        kFixedFlourescent2: typing.ClassVar[VideoCamera.WhiteBalance]  # value = <WhiteBalance.kFixedFlourescent2: 5200>
        kFixedFluorescent1: typing.ClassVar[VideoCamera.WhiteBalance]  # value = <WhiteBalance.kFixedFluorescent1: 5100>
        kFixedIndoor: typing.ClassVar[VideoCamera.WhiteBalance]  # value = <WhiteBalance.kFixedIndoor: 3000>
        kFixedOutdoor1: typing.ClassVar[VideoCamera.WhiteBalance]  # value = <WhiteBalance.kFixedOutdoor1: 4000>
        kFixedOutdoor2: typing.ClassVar[VideoCamera.WhiteBalance]  # value = <WhiteBalance.kFixedOutdoor2: 5000>
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
    def __init__(self) -> None:
        ...
    def getBrightness(self) -> int:
        """
        Get the brightness, as a percentage (0-100).
        """
    def setBrightness(self, brightness: int) -> None:
        """
        Set the brightness, as a percentage (0-100).
        """
    def setExposureAuto(self) -> None:
        """
        Set the exposure to auto aperature.
        """
    def setExposureHoldCurrent(self) -> None:
        """
        Set the exposure to hold current.
        """
    def setExposureManual(self, value: int) -> None:
        """
        Set the exposure to manual, as a percentage (0-100).
        """
    def setWhiteBalanceAuto(self) -> None:
        """
        Set the white balance to auto.
        """
    def setWhiteBalanceHoldCurrent(self) -> None:
        """
        Set the white balance to hold current.
        """
    def setWhiteBalanceManual(self, value: int) -> None:
        """
        Set the white balance to manual, with specified color temperature.
        """
class VideoEvent(RawEvent):
    """
    An event generated by the library and provided to event listeners.
    """
    def __init__(self) -> None:
        ...
    def getProperty(self) -> VideoProperty:
        """
        Returns the property associated with the event (if any).
        
        :returns: The property associated with the event (if any).
        """
    def getSink(self) -> VideoSink:
        """
        Returns the sink associated with the event (if any).
        
        :returns: The sink associated with the event (if any).
        """
    def getSource(self) -> VideoSource:
        """
        Returns the source associated with the event (if any).
        
        :returns: The source associated with the event (if any).
        """
class VideoListener:
    """
    An event listener.  This calls back to a desigated callback function when
    an event matching the specified mask is generated by the library.
    """
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, callback: typing.Callable[[VideoEvent], None], eventMask: int, immediateNotify: bool) -> None:
        """
        Create an event listener.
        
        :param callback:        Callback function
        :param eventMask:       Bitmask of VideoEvent::Kind values
        :param immediateNotify: Whether callback should be immediately called with
                                a representative set of events for the current library state.
        """
class VideoMode:
    """
    Video mode
    """
    class PixelFormat:
        """
        Members:
        
          kUnknown
        
          kMJPEG
        
          kYUYV
        
          kRGB565
        
          kBGR
        
          kGray
        
          kY16
        
          kUYVY
        """
        __members__: typing.ClassVar[dict[str, VideoMode.PixelFormat]]  # value = {'kUnknown': <PixelFormat.kUnknown: 0>, 'kMJPEG': <PixelFormat.kMJPEG: 1>, 'kYUYV': <PixelFormat.kYUYV: 2>, 'kRGB565': <PixelFormat.kRGB565: 3>, 'kBGR': <PixelFormat.kBGR: 4>, 'kGray': <PixelFormat.kGray: 5>, 'kY16': <PixelFormat.kY16: 6>, 'kUYVY': <PixelFormat.kUYVY: 7>}
        kBGR: typing.ClassVar[VideoMode.PixelFormat]  # value = <PixelFormat.kBGR: 4>
        kGray: typing.ClassVar[VideoMode.PixelFormat]  # value = <PixelFormat.kGray: 5>
        kMJPEG: typing.ClassVar[VideoMode.PixelFormat]  # value = <PixelFormat.kMJPEG: 1>
        kRGB565: typing.ClassVar[VideoMode.PixelFormat]  # value = <PixelFormat.kRGB565: 3>
        kUYVY: typing.ClassVar[VideoMode.PixelFormat]  # value = <PixelFormat.kUYVY: 7>
        kUnknown: typing.ClassVar[VideoMode.PixelFormat]  # value = <PixelFormat.kUnknown: 0>
        kY16: typing.ClassVar[VideoMode.PixelFormat]  # value = <PixelFormat.kY16: 6>
        kYUYV: typing.ClassVar[VideoMode.PixelFormat]  # value = <PixelFormat.kYUYV: 2>
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
    __hash__: typing.ClassVar[None] = None
    fps: int
    height: int
    pixelFormat: int
    width: int
    def __eq__(self, arg0: VideoMode) -> bool:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, pixelFormat_: VideoMode.PixelFormat, width_: int, height_: int, fps_: int) -> None:
        ...
    def compareWithoutFps(self, other: VideoMode) -> bool:
        ...
class VideoProperty:
    """
    A source or sink property.
    """
    class Kind:
        """
        Members:
        
          kNone : No specific property.
        
          kBoolean : Boolean property.
        
          kInteger : Integer property.
        
          kString : String property.
        
          kEnum : Enum property.
        """
        __members__: typing.ClassVar[dict[str, VideoProperty.Kind]]  # value = {'kNone': <Kind.kNone: 0>, 'kBoolean': <Kind.kBoolean: 1>, 'kInteger': <Kind.kInteger: 2>, 'kString': <Kind.kString: 4>, 'kEnum': <Kind.kEnum: 8>}
        kBoolean: typing.ClassVar[VideoProperty.Kind]  # value = <Kind.kBoolean: 1>
        kEnum: typing.ClassVar[VideoProperty.Kind]  # value = <Kind.kEnum: 8>
        kInteger: typing.ClassVar[VideoProperty.Kind]  # value = <Kind.kInteger: 2>
        kNone: typing.ClassVar[VideoProperty.Kind]  # value = <Kind.kNone: 0>
        kString: typing.ClassVar[VideoProperty.Kind]  # value = <Kind.kString: 4>
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
    def __init__(self) -> None:
        ...
    def get(self) -> int:
        """
        Returns property value.
        
        :returns: Property value.
        """
    def getChoices(self) -> list[str]:
        """
        Returns the possible values for the enum property value.
        
        This function is enum-specific.
        
        :returns: The possible values for the enum property value.
        """
    def getDefault(self) -> int:
        """
        Returns property default value.
        
        :returns: Property default value.
        """
    def getKind(self) -> VideoProperty.Kind:
        """
        Returns property kind.
        
        :returns: Property kind.
        """
    def getLastStatus(self) -> int:
        """
        Returns the last status.
        
        :returns: The last status.
        """
    def getMax(self) -> int:
        """
        Returns property maximum value.
        
        :returns: Property maximum value.
        """
    def getMin(self) -> int:
        """
        Returns property minimum value.
        
        :returns: Property minimum value.
        """
    def getName(self) -> str:
        """
        Returns property name.
        
        :returns: Property name.
        """
    def getStep(self) -> int:
        """
        Returns property step size.
        
        :returns: Property step size.
        """
    @typing.overload
    def getString(self) -> str:
        """
        Returns the string property value.
        
        This function is string-specific.
        
        :returns: The string property value.
        """
    @typing.overload
    def getString(self, buf: list[str]) -> str:
        """
        Returns the string property value as a reference to the given buffer.
        
        This function is string-specific.
        
        :param buf: The backing storage to which to write the property value.
        
        :returns: The string property value as a reference to the given buffer.
        """
    def isBoolean(self) -> bool:
        """
        Returns true if property is a boolean.
        
        :returns: True if property is a boolean.
        """
    def isEnum(self) -> bool:
        """
        Returns true if property is an enum.
        
        :returns: True if property is an enum.
        """
    def isInteger(self) -> bool:
        """
        Returns true if property is an integer.
        
        :returns: True if property is an integer.
        """
    def isString(self) -> bool:
        """
        Returns true if property is a string.
        
        :returns: True if property is a string.
        """
    def set(self, value: int) -> None:
        """
        Sets property value.
        
        :param value: Property value.
        """
    def setString(self, value: str) -> None:
        """
        Sets the string property value.
        
        This function is string-specific.
        
        :param value: String property value.
        """
class VideoSink:
    """
    A sink for video that accepts a sequence of frames.
    """
    class Kind:
        """
        Members:
        
          kUnknown : Unknown sink type.
        
          kMjpeg : MJPEG video sink.
        
          kCv : CV video sink.
        """
        __members__: typing.ClassVar[dict[str, VideoSink.Kind]]  # value = {'kUnknown': <Kind.kUnknown: 0>, 'kMjpeg': <Kind.kMjpeg: 2>, 'kCv': <Kind.kCv: 4>}
        kCv: typing.ClassVar[VideoSink.Kind]  # value = <Kind.kCv: 4>
        kMjpeg: typing.ClassVar[VideoSink.Kind]  # value = <Kind.kMjpeg: 2>
        kUnknown: typing.ClassVar[VideoSink.Kind]  # value = <Kind.kUnknown: 0>
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
    __hash__: typing.ClassVar[None] = None
    @staticmethod
    def enumerateSinks() -> list[VideoSink]:
        """
        Enumerate all existing sinks.
        
        :returns: Vector of sinks.
        """
    def __eq__(self, arg0: VideoSink) -> bool:
        ...
    def __init__(self) -> None:
        ...
    def enumerateProperties(self) -> list[VideoProperty]:
        """
        Enumerate all properties of this sink.
        """
    def getConfigJson(self) -> str:
        """
        Get a JSON configuration string.
        
        :returns: JSON configuration string
        """
    def getConfigJsonObject(self) -> wpiutil.json:
        """
        Get a JSON configuration object.
        
        :returns: JSON configuration object
        """
    def getDescription(self) -> str:
        """
        Get the sink description.  This is sink-kind specific.
        """
    def getHandle(self) -> int:
        """
        Returns the VideoSink handle.
        
        :returns: The VideoSink handle.
        """
    def getKind(self) -> VideoSink.Kind:
        """
        Get the kind of the sink.
        """
    def getLastStatus(self) -> int:
        ...
    def getName(self) -> str:
        """
        Get the name of the sink.  The name is an arbitrary identifier
        provided when the sink is created, and should be unique.
        """
    def getProperty(self, name: str) -> VideoProperty:
        """
        Get a property of the sink.
        
        :param name: Property name
        
        :returns: Property (kind Property::kNone if no property with
                  the given name exists)
        """
    def getSource(self) -> VideoSource:
        """
        Get the connected source.
        
        :returns: Connected source (empty if none connected).
        """
    def getSourceProperty(self, name: str) -> VideoProperty:
        """
        Get a property of the associated source.
        
        :param name: Property name
        
        :returns: Property (kind Property::kNone if no property with
                  the given name exists or no source connected)
        """
    @typing.overload
    def setConfigJson(self, config: str) -> bool:
        """
        Set properties from a JSON configuration string.
        
        The format of the JSON input is:
        
        ::
        
          {
              "properties": [
                  {
                      "name": property name
                      "value": property value
                  }
              ]
          }
        
        :param config: configuration
        
        :returns: True if set successfully
        """
    @typing.overload
    def setConfigJson(self, config: wpiutil.json) -> bool:
        """
        Set properties from a JSON configuration object.
        
        :param config: configuration
        
        :returns: True if set successfully
        """
    def setSource(self, source: VideoSource) -> None:
        """
        Configure which source should provide frames to this sink.  Each sink
        can accept frames from only a single source, but a single source can
        provide frames to multiple clients.
        
        :param source: Source
        """
class VideoSource:
    """
    A source for video that provides a sequence of frames.
    """
    class ConnectionStrategy:
        """
        Connection strategy.  Used for SetConnectionStrategy().
        
        Members:
        
          kConnectionAutoManage : Automatically connect or disconnect based on whether any sinks are
        connected to this source.  This is the default behavior.
        
          kConnectionKeepOpen : Try to keep the connection open regardless of whether any sinks are
        connected.
        
          kConnectionForceClose : Never open the connection.  If this is set when the connection is open,
        close the connection.
        """
        __members__: typing.ClassVar[dict[str, VideoSource.ConnectionStrategy]]  # value = {'kConnectionAutoManage': <ConnectionStrategy.kConnectionAutoManage: 0>, 'kConnectionKeepOpen': <ConnectionStrategy.kConnectionKeepOpen: 1>, 'kConnectionForceClose': <ConnectionStrategy.kConnectionForceClose: 2>}
        kConnectionAutoManage: typing.ClassVar[VideoSource.ConnectionStrategy]  # value = <ConnectionStrategy.kConnectionAutoManage: 0>
        kConnectionForceClose: typing.ClassVar[VideoSource.ConnectionStrategy]  # value = <ConnectionStrategy.kConnectionForceClose: 2>
        kConnectionKeepOpen: typing.ClassVar[VideoSource.ConnectionStrategy]  # value = <ConnectionStrategy.kConnectionKeepOpen: 1>
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
    class Kind:
        """
        Video source kind.
        
        Members:
        
          kUnknown : Unknown video source.
        
          kUsb : USB video source.
        
          kHttp : HTTP video source.
        
          kCv : CV video source.
        """
        __members__: typing.ClassVar[dict[str, VideoSource.Kind]]  # value = {'kUnknown': <Kind.kUnknown: 0>, 'kUsb': <Kind.kUsb: 1>, 'kHttp': <Kind.kHttp: 2>, 'kCv': <Kind.kCv: 4>}
        kCv: typing.ClassVar[VideoSource.Kind]  # value = <Kind.kCv: 4>
        kHttp: typing.ClassVar[VideoSource.Kind]  # value = <Kind.kHttp: 2>
        kUnknown: typing.ClassVar[VideoSource.Kind]  # value = <Kind.kUnknown: 0>
        kUsb: typing.ClassVar[VideoSource.Kind]  # value = <Kind.kUsb: 1>
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
    __hash__: typing.ClassVar[None] = None
    @staticmethod
    def enumerateSources() -> list[VideoSource]:
        """
        Enumerate all existing sources.
        
        :returns: Vector of sources.
        """
    def __eq__(self, arg0: VideoSource) -> bool:
        ...
    def __init__(self) -> None:
        ...
    def enumerateProperties(self) -> list[VideoProperty]:
        """
        Enumerate all properties of this source.
        """
    def enumerateSinks(self) -> list[VideoSink]:
        """
        Enumerate all sinks connected to this source.
        
        :returns: Vector of sinks.
        """
    def enumerateVideoModes(self) -> list[VideoMode]:
        """
        Enumerate all known video modes for this source.
        """
    def getActualDataRate(self) -> float:
        """
        Get the data rate (in bytes per second).
        
        SetTelemetryPeriod() must be called for this to be valid.
        
        :returns: Data rate averaged over the telemetry period.
        """
    def getActualFPS(self) -> float:
        """
        Get the actual FPS.
        
        SetTelemetryPeriod() must be called for this to be valid.
        
        :returns: Actual FPS averaged over the telemetry period.
        """
    def getConfigJson(self) -> str:
        """
        Get a JSON configuration string.
        
        :returns: JSON configuration string
        """
    def getConfigJsonObject(self) -> wpiutil.json:
        """
        Get a JSON configuration object.
        
        :returns: JSON configuration object
        """
    def getDescription(self) -> str:
        """
        Get the source description.  This is source-kind specific.
        """
    def getHandle(self) -> int:
        ...
    def getKind(self) -> VideoSource.Kind:
        """
        Get the kind of the source.
        """
    def getLastFrameTime(self) -> int:
        """
        Get the last time a frame was captured.
        This uses the same time base as wpi::Now().
        
        :returns: Time in 1 us increments.
        """
    def getLastStatus(self) -> int:
        ...
    def getName(self) -> str:
        """
        Get the name of the source.  The name is an arbitrary identifier
        provided when the source is created, and should be unique.
        """
    def getProperty(self, name: str) -> VideoProperty:
        """
        Get a property.
        
        :param name: Property name
        
        :returns: Property contents (of kind Property::kNone if no property with
                  the given name exists)
        """
    def getVideoMode(self) -> VideoMode:
        """
        Get the current video mode.
        """
    def isConnected(self) -> bool:
        """
        Is the source currently connected to whatever is providing the images?
        """
    def isEnabled(self) -> bool:
        """
        Gets source enable status.  This is determined with a combination of
        connection strategy and the number of sinks connected.
        
        :returns: True if enabled, false otherwise.
        """
    @typing.overload
    def setConfigJson(self, config: str) -> bool:
        """
        Set video mode and properties from a JSON configuration string.
        
        The format of the JSON input is:
        
        ::
        
          {
              "pixel format": "MJPEG", "YUYV", etc
              "width": video mode width
              "height": video mode height
              "fps": video mode fps
              "brightness": percentage brightness
              "white balance": "auto", "hold", or value
              "exposure": "auto", "hold", or value
              "properties": [
                  {
                      "name": property name
                      "value": property value
                  }
              ]
          }
        
        :param config: configuration
        
        :returns: True if set successfully
        """
    @typing.overload
    def setConfigJson(self, config: wpiutil.json) -> bool:
        """
        Set video mode and properties from a JSON configuration object.
        
        :param config: configuration
        
        :returns: True if set successfully
        """
    def setConnectionStrategy(self, strategy: VideoSource.ConnectionStrategy) -> None:
        """
        Sets the connection strategy.  By default, the source will automatically
        connect or disconnect based on whether any sinks are connected.
        
        This function is non-blocking; look for either a connection open or
        close event or call IsConnected() to determine the connection state.
        
        :param strategy: connection strategy (auto, keep open, or force close)
        """
    def setFPS(self, fps: int) -> bool:
        """
        Set the frames per second (FPS).
        
        :param fps: desired FPS
        
        :returns: True if set successfully
        """
    def setPixelFormat(self, pixelFormat: VideoMode.PixelFormat) -> bool:
        """
        Set the pixel format.
        
        :param pixelFormat: desired pixel format
        
        :returns: True if set successfully
        """
    def setResolution(self, width: int, height: int) -> bool:
        """
        Set the resolution.
        
        :param width:  desired width
        :param height: desired height
        
        :returns: True if set successfully
        """
    @typing.overload
    def setVideoMode(self, mode: VideoMode) -> bool:
        """
        Set the video mode.
        
        :param mode: Video mode
        """
    @typing.overload
    def setVideoMode(self, pixelFormat: VideoMode.PixelFormat, width: int, height: int, fps: int) -> bool:
        """
        Set the video mode.
        
        :param pixelFormat: desired pixel format
        :param width:       desired width
        :param height:      desired height
        :param fps:         desired FPS
        
        :returns: True if set successfully
        """
def _setLogger(func: typing.Callable[[int, str, int, str], None], min_level: int) -> None:
    ...
def runMainRunLoop() -> None:
    ...
def runMainRunLoopTimeout(timeoutSeconds: float) -> int:
    ...
def stopMainRunLoop() -> None:
    ...
_cleanup: typing.Any  # value = <capsule object>
