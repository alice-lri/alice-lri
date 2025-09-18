"""
Python bindings for the ALICE-LRI C++ library
"""
from __future__ import annotations
import collections.abc
import numpy
import numpy.typing
import typing
__all__: list[str] = ['ALL_ASSIGNED', 'DebugIntrinsics', 'DebugScanline', 'EMPTY_POINT_CLOUD', 'EndReason', 'ErrorCode', 'INTERNAL_ERROR', 'Interval', 'Intrinsics', 'MAX_ITERATIONS', 'MISMATCHED_SIZES', 'NONE', 'NO_MORE_PEAKS', 'RANGES_XY_ZERO', 'RangeImage', 'Scanline', 'ScanlineAngleBounds', 'ValueConfInterval', 'debug_train', 'error_message', 'intrinsics_from_json_file', 'intrinsics_from_json_str', 'intrinsics_to_json_file', 'intrinsics_to_json_str', 'project_to_range_image', 'train', 'unproject_to_point_cloud']
class DebugIntrinsics:
    end_reason: EndReason
    @typing.overload
    def __init__(self, arg0: typing.SupportsInt) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: typing.SupportsInt, arg1: typing.SupportsInt, arg2: typing.SupportsInt, arg3: typing.SupportsInt, arg4: EndReason) -> None:
        ...
    def __repr__(self) -> str:
        ...
    @property
    def points_count(self) -> int:
        ...
    @points_count.setter
    def points_count(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def scanlines(self) -> list[DebugScanline]:
        ...
    @property
    def unassigned_points(self) -> int:
        ...
    @unassigned_points.setter
    def unassigned_points(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def vertical_iterations(self) -> int:
        ...
    @vertical_iterations.setter
    def vertical_iterations(self, arg0: typing.SupportsInt) -> None:
        ...
class DebugScanline:
    horizontal_heuristic: bool
    theoretical_angle_bounds: ScanlineAngleBounds
    vertical_angle: ValueConfInterval
    vertical_heuristic: bool
    vertical_offset: ValueConfInterval
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
    @property
    def azimuthal_offset(self) -> float:
        ...
    @azimuthal_offset.setter
    def azimuthal_offset(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def horizontal_offset(self) -> float:
        ...
    @horizontal_offset.setter
    def horizontal_offset(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def hough_hash(self) -> int:
        ...
    @hough_hash.setter
    def hough_hash(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def hough_votes(self) -> int:
        ...
    @hough_votes.setter
    def hough_votes(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def points_count(self) -> int:
        ...
    @points_count.setter
    def points_count(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def resolution(self) -> int:
        ...
    @resolution.setter
    def resolution(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def uncertainty(self) -> float:
        ...
    @uncertainty.setter
    def uncertainty(self, arg0: typing.SupportsFloat) -> None:
        ...
class EndReason:
    """
    Members:
    
      ALL_ASSIGNED
    
      MAX_ITERATIONS
    
      NO_MORE_PEAKS
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
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
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
    Members:
    
      NONE
    
      MISMATCHED_SIZES
    
      EMPTY_POINT_CLOUD
    
      RANGES_XY_ZERO
    
      INTERNAL_ERROR
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
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
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
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def any_contained(self, arg0: Interval) -> bool:
        ...
    def clamp_both(self, arg0: typing.SupportsFloat, arg1: typing.SupportsFloat) -> None:
        ...
    def diff(self) -> float:
        ...
    @property
    def lower(self) -> float:
        ...
    @lower.setter
    def lower(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def upper(self) -> float:
        ...
    @upper.setter
    def upper(self, arg0: typing.SupportsFloat) -> None:
        ...
class Intrinsics:
    def __init__(self, arg0: typing.SupportsInt) -> None:
        ...
    def __repr__(self) -> str:
        ...
    @property
    def scanlines(self) -> list[Scanline]:
        ...
class RangeImage:
    def __array__(self) -> numpy.typing.NDArray[numpy.float64]:
        ...
    def __getitem__(self, arg0: tuple) -> float:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, width: typing.SupportsInt, height: typing.SupportsInt) -> None:
        ...
    @typing.overload
    def __init__(self, width: typing.SupportsInt, height: typing.SupportsInt, initial_value: typing.SupportsFloat) -> None:
        ...
    @typing.overload
    def __repr__(self) -> str:
        ...
    @typing.overload
    def __repr__(self) -> str:
        ...
    def __setitem__(self, arg0: tuple, arg1: typing.SupportsFloat) -> None:
        ...
    @property
    def height(self) -> int:
        ...
    @property
    def width(self) -> int:
        ...
class Scanline:
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
    @property
    def azimuthal_offset(self) -> float:
        ...
    @azimuthal_offset.setter
    def azimuthal_offset(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def horizontal_offset(self) -> float:
        ...
    @horizontal_offset.setter
    def horizontal_offset(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def resolution(self) -> int:
        ...
    @resolution.setter
    def resolution(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def vertical_angle(self) -> float:
        ...
    @vertical_angle.setter
    def vertical_angle(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def vertical_offset(self) -> float:
        ...
    @vertical_offset.setter
    def vertical_offset(self, arg0: typing.SupportsFloat) -> None:
        ...
class ScanlineAngleBounds:
    lower_line: Interval
    upper_line: Interval
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
class ValueConfInterval:
    ci: Interval
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
    @property
    def value(self) -> float:
        ...
    @value.setter
    def value(self, arg0: typing.SupportsFloat) -> None:
        ...
@typing.overload
def debug_train(arg0: collections.abc.Sequence[typing.SupportsFloat], arg1: collections.abc.Sequence[typing.SupportsFloat], arg2: collections.abc.Sequence[typing.SupportsFloat]) -> DebugIntrinsics:
    """
    Estimate intrinsics from float vectors with debug info
    """
@typing.overload
def debug_train(arg0: collections.abc.Sequence[typing.SupportsFloat], arg1: collections.abc.Sequence[typing.SupportsFloat], arg2: collections.abc.Sequence[typing.SupportsFloat]) -> DebugIntrinsics:
    """
    Estimate intrinsics from double vectors with debug info
    """
def error_message(arg0: ErrorCode) -> str:
    """
    Get error message for error code
    """
def intrinsics_from_json_file(path: str) -> Intrinsics:
    """
    Load intrinsics from JSON file
    """
def intrinsics_from_json_str(json: str) -> Intrinsics:
    """
    Create intrinsics from JSON string
    """
def intrinsics_to_json_file(intrinsics: Intrinsics, output_path: str, indent: typing.SupportsInt = -1) -> None:
    """
    Write intrinsics to JSON file
    """
def intrinsics_to_json_str(intrinsics: Intrinsics, indent: typing.SupportsInt = -1) -> str:
    """
    Convert intrinsics to JSON string
    """
@typing.overload
def project_to_range_image(arg0: Intrinsics, arg1: collections.abc.Sequence[typing.SupportsFloat], arg2: collections.abc.Sequence[typing.SupportsFloat], arg3: collections.abc.Sequence[typing.SupportsFloat]) -> RangeImage:
    """
    Project float cloud to range image
    """
@typing.overload
def project_to_range_image(arg0: Intrinsics, arg1: collections.abc.Sequence[typing.SupportsFloat], arg2: collections.abc.Sequence[typing.SupportsFloat], arg3: collections.abc.Sequence[typing.SupportsFloat]) -> RangeImage:
    """
    Project double cloud to range image
    """
@typing.overload
def train(x: collections.abc.Sequence[typing.SupportsFloat], y: collections.abc.Sequence[typing.SupportsFloat], z: collections.abc.Sequence[typing.SupportsFloat]) -> Intrinsics:
    """
    Estimate intrinsics from float vectors
    
    Parameters:
      x: List of x coordinates
      y: List of y coordinates
      z: List of z coordinates
    Returns:
      Intrinsics
    """
@typing.overload
def train(x: collections.abc.Sequence[typing.SupportsFloat], y: collections.abc.Sequence[typing.SupportsFloat], z: collections.abc.Sequence[typing.SupportsFloat]) -> Intrinsics:
    """
    Estimate intrinsics from double vectors
    """
def unproject_to_point_cloud(arg0: Intrinsics, arg1: RangeImage) -> tuple:
    """
    Unproject range image to 3D point cloud
    """
ALL_ASSIGNED: EndReason  # value = <EndReason.ALL_ASSIGNED: 0>
EMPTY_POINT_CLOUD: ErrorCode  # value = <ErrorCode.EMPTY_POINT_CLOUD: 2>
INTERNAL_ERROR: ErrorCode  # value = <ErrorCode.INTERNAL_ERROR: 4>
MAX_ITERATIONS: EndReason  # value = <EndReason.MAX_ITERATIONS: 1>
MISMATCHED_SIZES: ErrorCode  # value = <ErrorCode.MISMATCHED_SIZES: 1>
NONE: ErrorCode  # value = <ErrorCode.NONE: 0>
NO_MORE_PEAKS: EndReason  # value = <EndReason.NO_MORE_PEAKS: 2>
RANGES_XY_ZERO: ErrorCode  # value = <ErrorCode.RANGES_XY_ZERO: 3>
