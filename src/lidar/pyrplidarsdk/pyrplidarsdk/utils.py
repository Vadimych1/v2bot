"""
High-performance utility functions for PyRPLIDARSDK

This module provides vectorized NumPy-based utilities for processing RPLIDAR scan data
with optimal performance and comprehensive error handling.

Copyright (C) 2025 Dexmate Inc.
Licensed under MIT License (see LICENSE for details)
"""

import math
import warnings
from typing import List, Tuple, Optional, Union, Any, Sequence
import numpy as np
from numpy.typing import NDArray

# Type aliases for better readability
FloatArray = NDArray[np.floating[Any]]
IntArray = NDArray[np.integer[Any]]
BoolArray = NDArray[np.bool_]

# Constants
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi
DEFAULT_MIN_RANGE = 0.1  # meters
DEFAULT_MAX_RANGE = 12.0  # meters  
DEFAULT_MIN_QUALITY = 10
MAX_QUALITY = 255


def _ensure_numpy_array(data: Union[List[float], NDArray], name: str = "data") -> FloatArray:
    """
    Convert input to NumPy array with validation.
    
    Args:
        data: Input data as list or numpy array
        name: Name of the data for error messages
        
    Returns:
        NumPy array with float64 dtype
        
    Raises:
        ValueError: If data is empty or contains invalid values
        TypeError: If data cannot be converted to float array
    """
    # Check for empty data - handle both lists and NumPy arrays
    if hasattr(data, '__len__') and len(data) == 0:
        raise ValueError(f"{name} cannot be empty")
    
    try:
        arr = np.asarray(data, dtype=np.float64)
        if arr.size == 0:
            raise ValueError(f"{name} cannot be empty")
        if not np.isfinite(arr).all():
            raise ValueError(f"{name} contains non-finite values (NaN or inf)")
        return arr
    except (ValueError, TypeError) as e:
        raise TypeError(f"Cannot convert {name} to numeric array: {e}") from e


def _validate_scan_data_consistency(angles: FloatArray, ranges: FloatArray, 
                                  qualities: Optional[IntArray] = None) -> None:
    """
    Validate that scan data arrays have consistent dimensions.
    
    Args:
        angles: Array of angles
        ranges: Array of ranges
        qualities: Optional array of quality values
        
    Raises:
        ValueError: If arrays have inconsistent lengths
    """
    if len(angles) != len(ranges):
        raise ValueError(f"Angles and ranges must have same length: {len(angles)} != {len(ranges)}")
    
    if qualities is not None and len(angles) != len(qualities):
        raise ValueError(f"Angles and qualities must have same length: {len(angles)} != {len(qualities)}")


def polar_to_cartesian(angles: Union[List[float], FloatArray], 
                      ranges: Union[List[float], FloatArray]) -> Tuple[FloatArray, FloatArray]:
    """
    Convert polar coordinates (angle, range) to cartesian coordinates (x, y).
    
    Uses vectorized NumPy operations for optimal performance.
    
    Args:
        angles: Angles in radians (shape: N,)
        ranges: Ranges in meters (shape: N,)
        
    Returns:
        Tuple of (x_coordinates, y_coordinates) in meters as NumPy arrays
        
    Raises:
        ValueError: If input arrays are empty or have inconsistent lengths
        TypeError: If input cannot be converted to numeric arrays
        
    Example:
        >>> angles = [0, np.pi/2, np.pi]
        >>> ranges = [1.0, 2.0, 3.0]
        >>> x, y = polar_to_cartesian(angles, ranges)
        >>> print(x)  # [1.0, 0.0, -3.0]
        >>> print(y)  # [0.0, 2.0, 0.0]
    """
    angles_arr = _ensure_numpy_array(angles, "angles")
    ranges_arr = _ensure_numpy_array(ranges, "ranges")
    
    _validate_scan_data_consistency(angles_arr, ranges_arr)
    
    # Vectorized trigonometric operations
    x_coords = ranges_arr * np.cos(angles_arr)
    y_coords = ranges_arr * np.sin(angles_arr)
    
    return x_coords, y_coords


def filter_by_range(angles: Union[List[float], FloatArray], 
                   ranges: Union[List[float], FloatArray], 
                   qualities: Union[List[int], IntArray],
                   min_range: float = DEFAULT_MIN_RANGE, 
                   max_range: float = DEFAULT_MAX_RANGE) -> Tuple[FloatArray, FloatArray, IntArray]:
    """
    Filter scan data by range limits using vectorized operations.
    
    Args:
        angles: Angles in radians
        ranges: Ranges in meters
        qualities: Quality values (0-255)
        min_range: Minimum range threshold in meters (default: 0.1)
        max_range: Maximum range threshold in meters (default: 12.0)
        
    Returns:
        Filtered tuple of (angles, ranges, qualities) as NumPy arrays
        
    Raises:
        ValueError: If min_range >= max_range or inputs are inconsistent
        TypeError: If inputs cannot be converted to numeric arrays
        
    Example:
        >>> angles = [0, 1, 2, 3]
        >>> ranges = [0.05, 1.0, 5.0, 15.0]  # One too small, one too large
        >>> qualities = [100, 150, 200, 250]
        >>> filtered = filter_by_range(angles, ranges, qualities, 0.1, 12.0)
        >>> # Returns data for indices 1 and 2 only
    """
    if min_range >= max_range:
        raise ValueError(f"min_range ({min_range}) must be less than max_range ({max_range})")
    
    angles_arr = _ensure_numpy_array(angles, "angles")
    ranges_arr = _ensure_numpy_array(ranges, "ranges")
    qualities_arr = np.asarray(qualities, dtype=np.int32)
    
    _validate_scan_data_consistency(angles_arr, ranges_arr, qualities_arr)
    
    # Vectorized range filtering
    valid_mask = (ranges_arr >= min_range) & (ranges_arr <= max_range)
    
    return angles_arr[valid_mask], ranges_arr[valid_mask], qualities_arr[valid_mask]


def filter_by_quality(angles: Union[List[float], FloatArray], 
                     ranges: Union[List[float], FloatArray], 
                     qualities: Union[List[int], IntArray],
                     min_quality: int = DEFAULT_MIN_QUALITY) -> Tuple[FloatArray, FloatArray, IntArray]:
    """
    Filter scan data by quality threshold using vectorized operations.
    
    Args:
        angles: Angles in radians
        ranges: Ranges in meters
        qualities: Quality values (0-255)
        min_quality: Minimum quality threshold (default: 10)
        
    Returns:
        Filtered tuple of (angles, ranges, qualities) as NumPy arrays
        
    Raises:
        ValueError: If min_quality is not in valid range [0, 255]
        TypeError: If inputs cannot be converted to numeric arrays
        
    Example:
        >>> angles = [0, 1, 2, 3]
        >>> ranges = [1.0, 2.0, 3.0, 4.0]
        >>> qualities = [5, 15, 25, 35]  # First one below threshold
        >>> filtered = filter_by_quality(angles, ranges, qualities, 10)
        >>> # Returns data for last 3 points only
    """
    if not 0 <= min_quality <= MAX_QUALITY:
        raise ValueError(f"min_quality must be in range [0, {MAX_QUALITY}], got {min_quality}")
    
    angles_arr = _ensure_numpy_array(angles, "angles")
    ranges_arr = _ensure_numpy_array(ranges, "ranges")
    qualities_arr = np.asarray(qualities, dtype=np.int32)
    
    _validate_scan_data_consistency(angles_arr, ranges_arr, qualities_arr)
    
    # Vectorized quality filtering
    valid_mask = qualities_arr >= min_quality
    
    return angles_arr[valid_mask], ranges_arr[valid_mask], qualities_arr[valid_mask]


def filter_by_angle_range(angles: Union[List[float], FloatArray], 
                         ranges: Union[List[float], FloatArray], 
                         qualities: Union[List[int], IntArray],
                         min_angle: float, max_angle: float,
                         angle_unit: str = "radians") -> Tuple[FloatArray, FloatArray, IntArray]:
    """
    Filter scan data by angle range.
    
    Args:
        angles: Angles in specified unit
        ranges: Ranges in meters
        qualities: Quality values (0-255)
        min_angle: Minimum angle threshold
        max_angle: Maximum angle threshold
        angle_unit: Unit of angles ("radians" or "degrees")
        
    Returns:
        Filtered tuple of (angles, ranges, qualities) as NumPy arrays
        
    Raises:
        ValueError: If angle_unit is invalid or min_angle >= max_angle
        
    Example:
        >>> # Filter to keep only front-facing measurements (±45 degrees)
        >>> filtered = filter_by_angle_range(angles, ranges, qualities, 
        ...                                 -45, 45, "degrees")
    """
    if angle_unit not in ("radians", "degrees"):
        raise ValueError(f"angle_unit must be 'radians' or 'degrees', got '{angle_unit}'")
    
    if min_angle >= max_angle:
        raise ValueError(f"min_angle ({min_angle}) must be less than max_angle ({max_angle})")
    
    angles_arr = _ensure_numpy_array(angles, "angles")
    ranges_arr = _ensure_numpy_array(ranges, "ranges")
    qualities_arr = np.asarray(qualities, dtype=np.int32)
    
    _validate_scan_data_consistency(angles_arr, ranges_arr, qualities_arr)
    
    # Convert to radians if needed
    if angle_unit == "degrees":
        working_angles = angles_arr * DEG_TO_RAD
        min_rad = min_angle * DEG_TO_RAD
        max_rad = max_angle * DEG_TO_RAD
    else:
        working_angles = angles_arr
        min_rad = min_angle
        max_rad = max_angle
    
    # Normalize angles to [0, 2π] for consistent comparison
    working_angles = np.mod(working_angles, 2 * math.pi)
    min_rad = np.mod(min_rad, 2 * math.pi)
    max_rad = np.mod(max_rad, 2 * math.pi)
    
    # Handle wrap-around case
    if min_rad <= max_rad:
        valid_mask = (working_angles >= min_rad) & (working_angles <= max_rad)
    else:  # Wraps around 0
        valid_mask = (working_angles >= min_rad) | (working_angles <= max_rad)
    
    return angles_arr[valid_mask], ranges_arr[valid_mask], qualities_arr[valid_mask]


def angles_to_degrees(angles_rad: Union[List[float], FloatArray]) -> FloatArray:
    """
    Convert angles from radians to degrees using vectorized operations.
    
    Args:
        angles_rad: Angles in radians
        
    Returns:
        Angles in degrees as NumPy array
        
    Example:
        >>> angles_deg = angles_to_degrees([0, np.pi/2, np.pi])
        >>> print(angles_deg)  # [0, 90, 180]
    """
    angles_arr = _ensure_numpy_array(angles_rad, "angles")
    return angles_arr * RAD_TO_DEG


def angles_to_radians(angles_deg: Union[List[float], FloatArray]) -> FloatArray:
    """
    Convert angles from degrees to radians using vectorized operations.
    
    Args:
        angles_deg: Angles in degrees
        
    Returns:
        Angles in radians as NumPy array
        
    Example:
        >>> angles_rad = angles_to_radians([0, 90, 180])
        >>> print(angles_rad)  # [0, π/2, π]
    """
    angles_arr = _ensure_numpy_array(angles_deg, "angles")
    return angles_arr * DEG_TO_RAD


def to_numpy_arrays(angles: Union[List[float], FloatArray], 
                   ranges: Union[List[float], FloatArray], 
                   qualities: Optional[Union[List[int], IntArray]] = None) -> Tuple[FloatArray, FloatArray, Optional[IntArray]]:
    """
    Convert lists to NumPy arrays for efficient numerical processing.
    
    Args:
        angles: Angles as list or array
        ranges: Ranges as list or array
        qualities: Optional quality values as list or array
        
    Returns:
        Tuple of NumPy arrays (angles, ranges, qualities)
        
    Example:
        >>> angles_arr, ranges_arr, qualities_arr = to_numpy_arrays(
        ...     [0, 1, 2], [1.0, 2.0, 3.0], [100, 150, 200])
    """
    angles_arr = _ensure_numpy_array(angles, "angles")
    ranges_arr = _ensure_numpy_array(ranges, "ranges")
    qualities_arr = np.asarray(qualities, dtype=np.int32) if qualities is not None else None
    
    _validate_scan_data_consistency(angles_arr, ranges_arr, qualities_arr)
    
    return angles_arr, ranges_arr, qualities_arr


def compute_scan_statistics(ranges: Union[List[float], FloatArray], 
                           qualities: Union[List[int], IntArray]) -> dict[str, float]:
    """
    Compute comprehensive statistics for scan data.
    
    Args:
        ranges: Range measurements in meters
        qualities: Quality values (0-255)
        
    Returns:
        Dictionary containing statistical measures
        
    Example:
        >>> stats = compute_scan_statistics([1, 2, 3, 4], [100, 150, 200, 250])
        >>> print(stats['mean_range'])  # 2.5
    """
    ranges_arr = _ensure_numpy_array(ranges, "ranges")
    qualities_arr = np.asarray(qualities, dtype=np.int32)
    
    if len(ranges_arr) != len(qualities_arr):
        raise ValueError("Ranges and qualities must have same length")
    
    if len(ranges_arr) == 0:
        return {
            'count': 0,
            'mean_range': 0.0,
            'std_range': 0.0,
            'min_range': 0.0,
            'max_range': 0.0,
            'mean_quality': 0.0,
            'std_quality': 0.0,
            'min_quality': 0,
            'max_quality': 0
        }
    
    return {
        'count': len(ranges_arr),
        'mean_range': float(np.mean(ranges_arr)),
        'std_range': float(np.std(ranges_arr)),
        'min_range': float(np.min(ranges_arr)),
        'max_range': float(np.max(ranges_arr)),
        'median_range': float(np.median(ranges_arr)),
        'mean_quality': float(np.mean(qualities_arr)),
        'std_quality': float(np.std(qualities_arr)),
        'min_quality': int(np.min(qualities_arr)),
        'max_quality': int(np.max(qualities_arr)),
        'median_quality': float(np.median(qualities_arr))
    }


def downsample_scan(angles: Union[List[float], FloatArray], 
                   ranges: Union[List[float], FloatArray], 
                   qualities: Union[List[int], IntArray],
                   factor: int) -> Tuple[FloatArray, FloatArray, IntArray]:
    """
    Downsample scan data by taking every nth point.
    
    Args:
        angles: Angles in radians
        ranges: Ranges in meters
        qualities: Quality values
        factor: Downsampling factor (take every nth point)
        
    Returns:
        Downsampled tuple of (angles, ranges, qualities)
        
    Raises:
        ValueError: If factor is less than 1
        
    Example:
        >>> # Keep every 3rd point
        >>> downsampled = downsample_scan(angles, ranges, qualities, 3)
    """
    if factor < 1:
        raise ValueError(f"Downsampling factor must be >= 1, got {factor}")
    
    angles_arr = _ensure_numpy_array(angles, "angles")
    ranges_arr = _ensure_numpy_array(ranges, "ranges")
    qualities_arr = np.asarray(qualities, dtype=np.int32)
    
    _validate_scan_data_consistency(angles_arr, ranges_arr, qualities_arr)
    
    # Downsample using array slicing
    indices = np.arange(0, len(angles_arr), factor)
    
    return angles_arr[indices], ranges_arr[indices], qualities_arr[indices]


def smooth_ranges(ranges: Union[List[float], FloatArray], 
                 window_size: int = 3,
                 method: str = "median") -> FloatArray:
    """
    Smooth range measurements to reduce noise.
    
    Args:
        ranges: Range measurements in meters
        window_size: Size of smoothing window (must be odd)
        method: Smoothing method ("median", "mean", or "gaussian")
        
    Returns:
        Smoothed ranges as NumPy array
        
    Raises:
        ValueError: If window_size is even or method is invalid
        
    Example:
        >>> smoothed = smooth_ranges([1, 10, 2, 3, 4], window_size=3)
        >>> # Reduces the outlier at index 1
    """
    if window_size % 2 == 0:
        raise ValueError("window_size must be odd")
    
    if method not in ("median", "mean", "gaussian"):
        raise ValueError(f"method must be 'median', 'mean', or 'gaussian', got '{method}'")
    
    ranges_arr = _ensure_numpy_array(ranges, "ranges")
    
    if len(ranges_arr) < window_size:
        warnings.warn("Array shorter than window_size, returning original array")
        return ranges_arr.copy()
    
    if method == "median":
        # Median filter for outlier removal
        from scipy.ndimage import median_filter
        return median_filter(ranges_arr, size=window_size, mode='nearest')
    elif method == "mean":
        # Moving average
        from scipy.ndimage import uniform_filter1d
        return uniform_filter1d(ranges_arr, size=window_size, mode='nearest')
    else:  # gaussian
        # Gaussian filter
        from scipy.ndimage import gaussian_filter1d
        sigma = window_size / 6.0  # Standard deviation
        return gaussian_filter1d(ranges_arr, sigma=sigma, mode='nearest')


def detect_obstacles(ranges: Union[List[float], FloatArray],
                    angles: Union[List[float], FloatArray],
                    min_size: float = 0.1,
                    max_gap: float = 0.2) -> List[Tuple[int, int, float, float]]:
    """
    Detect obstacles (contiguous regions) in scan data.
    
    Args:
        ranges: Range measurements in meters
        angles: Corresponding angles in radians
        min_size: Minimum obstacle size in meters
        max_gap: Maximum gap within obstacle in meters
        
    Returns:
        List of (start_idx, end_idx, min_range, angular_width) tuples
        
    Example:
        >>> obstacles = detect_obstacles(ranges, angles, min_size=0.2)
        >>> print(f"Found {len(obstacles)} obstacles")
    """
    ranges_arr = _ensure_numpy_array(ranges, "ranges")
    angles_arr = _ensure_numpy_array(angles, "angles")
    
    _validate_scan_data_consistency(angles_arr, ranges_arr)
    
    if len(ranges_arr) == 0:
        return []
    
    # Convert to cartesian for distance calculations
    x_coords, y_coords = polar_to_cartesian(angles_arr, ranges_arr)
    
    obstacles = []
    current_start = 0
    
    for i in range(1, len(ranges_arr)):
        # Calculate distance between consecutive points
        dx = x_coords[i] - x_coords[i-1]
        dy = y_coords[i] - y_coords[i-1]
        gap_distance = math.sqrt(dx*dx + dy*dy)
        
        if gap_distance > max_gap:
            # End of current obstacle
            if i - current_start >= 2:  # At least 2 points
                obstacle_ranges = ranges_arr[current_start:i]
                min_range = float(np.min(obstacle_ranges))
                angular_width = float(angles_arr[i-1] - angles_arr[current_start])
                
                # Check if obstacle meets minimum size requirement
                estimated_size = min_range * abs(angular_width)
                if estimated_size >= min_size:
                    obstacles.append((current_start, i-1, min_range, angular_width))
            
            current_start = i
    
    # Handle last obstacle
    if len(ranges_arr) - current_start >= 2:
        obstacle_ranges = ranges_arr[current_start:]
        min_range = float(np.min(obstacle_ranges))
        angular_width = float(angles_arr[-1] - angles_arr[current_start])
        estimated_size = min_range * abs(angular_width)
        
        if estimated_size >= min_size:
            obstacles.append((current_start, len(ranges_arr)-1, min_range, angular_width))
    
    return obstacles 