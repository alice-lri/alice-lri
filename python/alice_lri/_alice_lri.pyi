"""
Python bindings for the ALICE-LRI C++ library
"""
from __future__ import annotations
import numpy
import typing
__all__: list[str] = ['ALL_ASSIGNED', 'EMPTY_POINT_CLOUD', 'EndReason', 'ErrorCode', 'INTERNAL_ERROR', 'Interval', 'Intrinsics', 'IntrinsicsDetailed', 'MAX_ITERATIONS', 'MISMATCHED_SIZES', 'NONE', 'NO_MORE_PEAKS', 'RANGES_XY_ZERO', 'RangeImage', 'Scanline', 'ScanlineAngleBounds', 'ScanlineDetailed', 'ValueConfInterval', 'error_message', 'estimate_intrinsics', 'estimate_intrinsics_detailed', 'intrinsics_from_json_file', 'intrinsics_from_json_str', 'intrinsics_to_json_file', 'intrinsics_to_json_str', 'project_to_range_image', 'unproject_to_point_cloud']
class EndReason:
    """
    
            Reason for ending the iterative vertical fitting process.
        
    
    Members:
    
      ALL_ASSIGNED : All points assigned. This is the normal termination condition.
    
      MAX_ITERATIONS : Maximum number of iterations reached.
    
      NO_MORE_PEAKS : No more peaks found in the Hough accumulator.
    """
    ALL_ASSIGNED: typing.ClassVar[EndReason]  # value = <EndReason.ALL_ASSIGNED: 0>
    MAX_ITERATIONS: typing.ClassVar[EndReason]  # value = <EndReason.MAX_ITERATIONS: 1>
    NO_MORE_PEAKS: typing.ClassVar[EndReason]  # value = <EndReason.NO_MORE_PEAKS: 2>
    __members__: typing.ClassVar[dict[str, EndReason]]  # value = {'ALL_ASSIGNED': <EndReason.ALL_ASSIGNED: 0>, 'MAX_ITERATIONS': <EndReason.MAX_ITERATIONS: 1>, 'NO_MORE_PEAKS': <EndReason.NO_MORE_PEAKS: 2>}
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
class ErrorCode:
    """
    
            Error codes for Alice LRI operations.
        
    
    Members:
    
      NONE : No error.
    
      MISMATCHED_SIZES : Input arrays have mismatched sizes.
    
      EMPTY_POINT_CLOUD : Point cloud is empty.
    
      RANGES_XY_ZERO : At least one point has a range of zero in the XY plane.
    
      INTERNAL_ERROR : Internal error occurred.
    """
    EMPTY_POINT_CLOUD: typing.ClassVar[ErrorCode]  # value = <ErrorCode.EMPTY_POINT_CLOUD: 2>
    INTERNAL_ERROR: typing.ClassVar[ErrorCode]  # value = <ErrorCode.INTERNAL_ERROR: 4>
    MISMATCHED_SIZES: typing.ClassVar[ErrorCode]  # value = <ErrorCode.MISMATCHED_SIZES: 1>
    NONE: typing.ClassVar[ErrorCode]  # value = <ErrorCode.NONE: 0>
    RANGES_XY_ZERO: typing.ClassVar[ErrorCode]  # value = <ErrorCode.RANGES_XY_ZERO: 3>
    __members__: typing.ClassVar[dict[str, ErrorCode]]  # value = {'NONE': <ErrorCode.NONE: 0>, 'MISMATCHED_SIZES': <ErrorCode.MISMATCHED_SIZES: 1>, 'EMPTY_POINT_CLOUD': <ErrorCode.EMPTY_POINT_CLOUD: 2>, 'RANGES_XY_ZERO': <ErrorCode.RANGES_XY_ZERO: 3>, 'INTERNAL_ERROR': <ErrorCode.INTERNAL_ERROR: 4>}
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
class Interval:
    """
    
            Represents a numeric interval [lower, upper].
    
            Attributes:
                lower (float): Lower bound of the interval.
                upper (float): Upper bound of the interval.
        
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __repr__(self) -> str:
        ...
    def any_contained(self, arg0: Interval) -> bool:
        """
        Check if any part of another interval is contained in this interval.
        """
    def clamp_both(self, arg0: float, arg1: float) -> None:
        """
        Clamp both bounds to [minValue, maxValue].
        """
    def diff(self) -> float:
        """
        Get the width of the interval (upper - lower).
        """
    @property
    def lower(self) -> float:
        """
        Lower bound.
        """
    @lower.setter
    def lower(self, arg0: float) -> None:
        ...
    @property
    def upper(self) -> float:
        """
        Upper bound.
        """
    @upper.setter
    def upper(self, arg0: float) -> None:
        ...
class Intrinsics:
    """
    
            Contains intrinsic parameters for a sensor, including all scanlines.
    
            Args:
                scanline_count (int): Number of scanlines.
    
            Attributes:
                scanlines (list of Scanline): Array of scanlines describing the sensor geometry.
        
    """
    def __init__(self, scanline_count: int) -> None:
        """
        Construct with a given number of scanlines.
        """
    def __repr__(self) -> str:
        ...
    @property
    def scanlines(self) -> list[Scanline]:
        """
        List of scanlines.
        """
class IntrinsicsDetailed:
    """
    
            Detailed intrinsic parameters, including scanline details and statistics.
    
            Args:
                scanline_count (int): Number of scanlines.
                vertical_iterations (int): Number of vertical iterations performed.
                unassigned_points (int): Number of unassigned points.
                points_count (int): Total number of points.
                end_reason (EndReason): Reason for ending the process.
    
            Attributes:
                scanlines (list of ScanlineDetailed): List of detailed scanlines.
                vertical_iterations (int): Number of vertical iterations performed.
                unassigned_points (int): Number of unassigned points.
                points_count (int): Total number of points.
                end_reason (EndReason): Reason for ending the process.
        
    """
    @typing.overload
    def __init__(self, scanline_count: int) -> None:
        """
        Construct with a given number of scanlines.
        """
    @typing.overload
    def __init__(self, scanline_count: int, vertical_iterations: int, unassigned_points: int, points_count: int, end_reason: EndReason) -> None:
        """
        Full constructor with all statistics.
        """
    def __repr__(self) -> str:
        ...
    @property
    def end_reason(self) -> EndReason:
        """
        Reason for ending the process.
        """
    @end_reason.setter
    def end_reason(self, arg0: EndReason) -> None:
        ...
    @property
    def points_count(self) -> int:
        """
        Total number of points.
        """
    @points_count.setter
    def points_count(self, arg0: int) -> None:
        ...
    @property
    def scanlines(self) -> list[ScanlineDetailed]:
        """
        List of detailed scanlines.
        """
    @property
    def unassigned_points(self) -> int:
        """
        Number of unassigned points.
        """
    @unassigned_points.setter
    def unassigned_points(self, arg0: int) -> None:
        ...
    @property
    def vertical_iterations(self) -> int:
        """
        Number of vertical iterations performed.
        """
    @vertical_iterations.setter
    def vertical_iterations(self, arg0: int) -> None:
        ...
class RangeImage:
    """
    
            Represents a 2D range image with pixel data.
    
            Args:
                width (int): Image width.
                height (int): Image height.
                initial_value (float, optional): Initial value for all pixels (if provided).
    
            Attributes:
                width (int): Image width.
                height (int): Image height.
    
            Note:
                The (width, height) constructor only reserves space for pixels but does not initialize them.
                The (width, height, initial_value) constructor initializes all pixels to the given value.
        
    """
    def __array__(self, **kwargs) -> numpy.ndarray[numpy.float64]:
        ...
    def __getitem__(self, arg0: tuple) -> float:
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor (empty image).
        """
    @typing.overload
    def __init__(self, width: int, height: int) -> None:
        """
        Construct with width and height. Reserves space for pixels but does not initialize them.
        """
    @typing.overload
    def __init__(self, width: int, height: int, initial_value: float) -> None:
        """
        Construct with width, height, and initial pixel value.
        """
    @typing.overload
    def __repr__(self) -> str:
        ...
    @typing.overload
    def __repr__(self) -> str:
        ...
    def __setitem__(self, arg0: tuple, arg1: float) -> None:
        ...
    @property
    def height(self) -> int:
        """
        Get image height.
        """
    @property
    def width(self) -> int:
        """
        Get image width.
        """
class Scanline:
    """
    
            Represents a single scanline with intrinsic parameters.
    
            Attributes:
                vertical_offset (float): Vertical spatial offset of the scanline.
                vertical_angle (float): Vertical angle of the scanline.
                horizontal_offset (float): Horizontal spatial offset of the scanline.
                azimuthal_offset (float): Azimuthal offset of the scanline.
                resolution (int): Horizontal resolution of the scanline.
        
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __repr__(self) -> str:
        ...
    @property
    def azimuthal_offset(self) -> float:
        """
        Azimuthal offset.
        """
    @azimuthal_offset.setter
    def azimuthal_offset(self, arg0: float) -> None:
        ...
    @property
    def horizontal_offset(self) -> float:
        """
        Horizontal spatial offset.
        """
    @horizontal_offset.setter
    def horizontal_offset(self, arg0: float) -> None:
        ...
    @property
    def resolution(self) -> int:
        """
        Horizontal resolution.
        """
    @resolution.setter
    def resolution(self, arg0: int) -> None:
        ...
    @property
    def vertical_angle(self) -> float:
        """
        Vertical angle.
        """
    @vertical_angle.setter
    def vertical_angle(self, arg0: float) -> None:
        ...
    @property
    def vertical_offset(self) -> float:
        """
        Vertical spatial offset.
        """
    @vertical_offset.setter
    def vertical_offset(self, arg0: float) -> None:
        ...
class ScanlineAngleBounds:
    """
    
            Angle bounds for a scanline.
    
            Attributes:
                lower_line (Interval): Lower angle interval.
                upper_line (Interval): Upper angle interval.
        
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __repr__(self) -> str:
        ...
    @property
    def lower_line(self) -> Interval:
        """
        Lower angle interval.
        """
    @lower_line.setter
    def lower_line(self, arg0: Interval) -> None:
        ...
    @property
    def upper_line(self) -> Interval:
        """
        Upper angle interval.
        """
    @upper_line.setter
    def upper_line(self, arg0: Interval) -> None:
        ...
class ScanlineDetailed:
    """
    
            Detailed scanline information with uncertainty and voting statistics.
    
            Attributes:
                vertical_offset (ValueConfInterval): Vertical spatial offset with confidence interval.
                vertical_angle (ValueConfInterval): Vertical angle with confidence interval.
                horizontal_offset (float): Horizontal spatial offset.
                azimuthal_offset (float): Azimuthal offset.
                resolution (int): Horizontal resolution of the scanline.
                uncertainty (float): Estimated uncertainty.
                hough_votes (int): Number of Hough transform votes.
                hough_hash (int): Hash value for Hough voting.
                points_count (int): Number of points assigned to this scanline.
                theoretical_angle_bounds (ScanlineAngleBounds): Theoretical angle bounds for the scanline.
                vertical_heuristic (bool): Whether vertical heuristic was used.
                horizontal_heuristic (bool): Whether horizontal heuristic was used.
        
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __repr__(self) -> str:
        ...
    @property
    def azimuthal_offset(self) -> float:
        """
        Azimuthal offset.
        """
    @azimuthal_offset.setter
    def azimuthal_offset(self, arg0: float) -> None:
        ...
    @property
    def horizontal_heuristic(self) -> bool:
        """
        Whether horizontal heuristic was used.
        """
    @horizontal_heuristic.setter
    def horizontal_heuristic(self, arg0: bool) -> None:
        ...
    @property
    def horizontal_offset(self) -> float:
        """
        Horizontal offset.
        """
    @horizontal_offset.setter
    def horizontal_offset(self, arg0: float) -> None:
        ...
    @property
    def hough_hash(self) -> int:
        """
        Hash value for Hough voting.
        """
    @hough_hash.setter
    def hough_hash(self, arg0: int) -> None:
        ...
    @property
    def hough_votes(self) -> int:
        """
        Number of Hough transform votes.
        """
    @hough_votes.setter
    def hough_votes(self, arg0: int) -> None:
        ...
    @property
    def points_count(self) -> int:
        """
        Number of points assigned to this scanline.
        """
    @points_count.setter
    def points_count(self, arg0: int) -> None:
        ...
    @property
    def resolution(self) -> int:
        """
        Number of points in the scanline.
        """
    @resolution.setter
    def resolution(self, arg0: int) -> None:
        ...
    @property
    def theoretical_angle_bounds(self) -> ScanlineAngleBounds:
        """
        Theoretical angle bounds.
        """
    @theoretical_angle_bounds.setter
    def theoretical_angle_bounds(self, arg0: ScanlineAngleBounds) -> None:
        ...
    @property
    def uncertainty(self) -> float:
        """
        Estimated uncertainty.
        """
    @uncertainty.setter
    def uncertainty(self, arg0: float) -> None:
        ...
    @property
    def vertical_angle(self) -> ValueConfInterval:
        """
        Vertical angle with confidence interval.
        """
    @vertical_angle.setter
    def vertical_angle(self, arg0: ValueConfInterval) -> None:
        ...
    @property
    def vertical_heuristic(self) -> bool:
        """
        Whether vertical heuristic was used.
        """
    @vertical_heuristic.setter
    def vertical_heuristic(self, arg0: bool) -> None:
        ...
    @property
    def vertical_offset(self) -> ValueConfInterval:
        """
        Vertical offset with confidence interval.
        """
    @vertical_offset.setter
    def vertical_offset(self, arg0: ValueConfInterval) -> None:
        ...
class ValueConfInterval:
    """
    
            Value with associated confidence interval.
    
            Attributes:
                value (float): The value.
                ci (Interval): Confidence interval for the value.
        
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __repr__(self) -> str:
        ...
    @property
    def ci(self) -> Interval:
        """
        Confidence interval.
        """
    @ci.setter
    def ci(self, arg0: Interval) -> None:
        ...
    @property
    def value(self) -> float:
        """
        The value.
        """
    @value.setter
    def value(self, arg0: float) -> None:
        ...
def error_message(code: ErrorCode) -> str:
    """
            Get a human-readable error message for an error code.
    
            Args:
                code (ErrorCode): Error code.
            Returns:
                str: Error message.
    """
def estimate_intrinsics(x: list[float], y: list[float], z: list[float]) -> Intrinsics:
    """
            Estimate sensor intrinsics from point cloud coordinates given as float vectors.
    
            Args:
                x (list of float): X coordinates.
                y (list of float): Y coordinates.
                z (list of float): Z coordinates.
            Returns:
                Intrinsics: Estimated sensor intrinsics.
    """
def estimate_intrinsics_detailed(x: list[float], y: list[float], z: list[float]) -> IntrinsicsDetailed:
    """
            Estimate detailed sensor intrinsics (including algorithm execution info) from point cloud coordinates given as float vectors.
    
            Args:
                x (list of float): X coordinates.
                y (list of float): Y coordinates.
                z (list of float): Z coordinates.
            Returns:
                IntrinsicsDetailed: Detailed estimated intrinsics and statistics.
    """
def intrinsics_from_json_file(path: str) -> Intrinsics:
    """
            Load intrinsics from a JSON file.
    
            Args:
                path (str): Path to JSON file.
            Returns:
                Intrinsics: Parsed intrinsics.
    """
def intrinsics_from_json_str(json: str) -> Intrinsics:
    """
            Create intrinsics from a JSON string.
    
            Args:
                json (str): JSON string.
            Returns:
                Intrinsics: Parsed intrinsics.
    """
def intrinsics_to_json_file(intrinsics: Intrinsics, output_path: str, indent: int = -1) -> None:
    """
            Write intrinsics to a JSON file.
    
            Args:
                intrinsics (Intrinsics): Intrinsics to write.
                output_path (str): Output file path.
                indent (int, optional): Indentation for pretty printing (-1 for compact).
            Raises:
                RuntimeError: If writing fails.
    """
def intrinsics_to_json_str(intrinsics: Intrinsics, indent: int = -1) -> str:
    """
            Convert intrinsics to a JSON string.
    
            Args:
                intrinsics (Intrinsics): Intrinsics to serialize.
                indent (int, optional): Indentation for pretty printing (-1 for compact).
            Returns:
                str: JSON string.
    """
def project_to_range_image(intrinsics: Intrinsics, x: list[float], y: list[float], z: list[float]) -> RangeImage:
    """
            Project a point cloud to a range image using given intrinsics.
    
            Args:
                intrinsics (Intrinsics): Sensor intrinsics (see estimate_intrinsics).
                x (list of float): X coordinates.
                y (list of float): Y coordinates.
                z (list of float): Z coordinates.
            Returns:
                RangeImage: Projected range image.
    """
def unproject_to_point_cloud(intrinsics: Intrinsics, ri: RangeImage) -> tuple:
    """
            Unproject a range image to a 3D point cloud using given intrinsics.
    
            Args:
                intrinsics (Intrinsics): Sensor intrinsics.
                ri (RangeImage): Input range image.
            Returns:
                tuple: (x, y, z) coordinate lists.
    """
ALL_ASSIGNED: EndReason  # value = <EndReason.ALL_ASSIGNED: 0>
EMPTY_POINT_CLOUD: ErrorCode  # value = <ErrorCode.EMPTY_POINT_CLOUD: 2>
INTERNAL_ERROR: ErrorCode  # value = <ErrorCode.INTERNAL_ERROR: 4>
MAX_ITERATIONS: EndReason  # value = <EndReason.MAX_ITERATIONS: 1>
MISMATCHED_SIZES: ErrorCode  # value = <ErrorCode.MISMATCHED_SIZES: 1>
NONE: ErrorCode  # value = <ErrorCode.NONE: 0>
NO_MORE_PEAKS: EndReason  # value = <EndReason.NO_MORE_PEAKS: 2>
RANGES_XY_ZERO: ErrorCode  # value = <ErrorCode.RANGES_XY_ZERO: 3>
