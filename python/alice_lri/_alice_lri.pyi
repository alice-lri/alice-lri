"""
Python bindings for the ALICE-LRI C++ library
"""
from __future__ import annotations
import numpy
import typing
__all__: list[str] = ['ALL_ASSIGNED', 'EMPTY_POINT_CLOUD', 'EndReason', 'ErrorCode', 'INTERNAL_ERROR', 'Interval', 'Intrinsics', 'IntrinsicsDetailed', 'MAX_ITERATIONS', 'MISMATCHED_SIZES', 'NONE', 'NO_MORE_PEAKS', 'RANGES_XY_ZERO', 'RangeImage', 'Scanline', 'ScanlineAngleBounds', 'ScanlineDetailed', 'ValueConfInterval', 'error_message', 'estimate_intrinsics', 'estimate_intrinsics_detailed', 'intrinsics_from_json_file', 'intrinsics_from_json_str', 'intrinsics_to_json_file', 'intrinsics_to_json_str', 'project_to_range_image', 'unproject_to_point_cloud']
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
    lower: float
    upper: float
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def any_contained(self, arg0: Interval) -> bool:
        ...
    def clamp_both(self, arg0: float, arg1: float) -> None:
        ...
    def diff(self) -> float:
        ...
class Intrinsics:
    def __init__(self, arg0: int) -> None:
        ...
    def __repr__(self) -> str:
        ...
    @property
    def scanlines(self) -> list[Scanline]:
        ...
class IntrinsicsDetailed:
    end_reason: EndReason
    points_count: int
    unassigned_points: int
    vertical_iterations: int
    @typing.overload
    def __init__(self, arg0: int) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: int, arg1: int, arg2: int, arg3: int, arg4: EndReason) -> None:
        ...
    def __repr__(self) -> str:
        ...
    @property
    def scanlines(self) -> list[ScanlineDetailed]:
        ...
class RangeImage:
    def __array__(self) -> numpy.ndarray[numpy.float64]:
        ...
    def __getitem__(self, arg0: tuple) -> float:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, width: int, height: int) -> None:
        ...
    @typing.overload
    def __init__(self, width: int, height: int, initial_value: float) -> None:
        ...
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
        ...
    @property
    def width(self) -> int:
        ...
class Scanline:
    azimuthal_offset: float
    horizontal_offset: float
    resolution: int
    vertical_angle: float
    vertical_offset: float
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
class ScanlineAngleBounds:
    lower_line: Interval
    upper_line: Interval
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
class ScanlineDetailed:
    azimuthal_offset: float
    horizontal_heuristic: bool
    horizontal_offset: float
    hough_hash: int
    hough_votes: int
    points_count: int
    resolution: int
    theoretical_angle_bounds: ScanlineAngleBounds
    uncertainty: float
    vertical_angle: ValueConfInterval
    vertical_heuristic: bool
    vertical_offset: ValueConfInterval
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
class ValueConfInterval:
    ci: Interval
    value: float
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
def error_message(arg0: ErrorCode) -> str:
    """
    Get error message for error code
    """
@typing.overload
def estimate_intrinsics(x: list[float], y: list[float], z: list[float]) -> Intrinsics:
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
def estimate_intrinsics(x: list[float], y: list[float], z: list[float]) -> Intrinsics:
    """
    Estimate intrinsics from double vectors
    """
@typing.overload
def estimate_intrinsics_detailed(arg0: list[float], arg1: list[float], arg2: list[float]) -> IntrinsicsDetailed:
    """
    Estimate intrinsics from float vectors with algorithm execution info
    """
@typing.overload
def estimate_intrinsics_detailed(arg0: list[float], arg1: list[float], arg2: list[float]) -> IntrinsicsDetailed:
    """
    Estimate intrinsics from double vectors with algorithm execution info
    """
def intrinsics_from_json_file(path: str) -> Intrinsics:
    """
    Load intrinsics from JSON file
    """
def intrinsics_from_json_str(json: str) -> Intrinsics:
    """
    Create intrinsics from JSON string
    """
def intrinsics_to_json_file(intrinsics: Intrinsics, output_path: str, indent: int = -1) -> None:
    """
    Write intrinsics to JSON file
    """
def intrinsics_to_json_str(intrinsics: Intrinsics, indent: int = -1) -> str:
    """
    Convert intrinsics to JSON string
    """
@typing.overload
def project_to_range_image(arg0: Intrinsics, arg1: list[float], arg2: list[float], arg3: list[float]) -> RangeImage:
    """
    Project float cloud to range image
    """
@typing.overload
def project_to_range_image(arg0: Intrinsics, arg1: list[float], arg2: list[float], arg3: list[float]) -> RangeImage:
    """
    Project double cloud to range image
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
