#!/usr/bin/env python3
"""
Enhanced RPLIDAR scanning example with advanced data analysis

This example demonstrates comprehensive usage of the PyRPLIDARSDK library with
vectorized utilities for high-performance data processing, filtering, and analysis.
It showcases statistics, obstacle detection, and data quality assessment.

Copyright (C) 2025 Dexmate Inc.
Licensed under MIT License
"""

import lidar.pyrplidarsdk.pyrplidarsdk as pyrplidarsdk
from lidar.pyrplidarsdk.pyrplidarsdk.utils import (
    compute_scan_statistics, filter_by_quality, filter_by_range,
    to_numpy_arrays, angles_to_degrees, detect_obstacles, 
    downsample_scan, smooth_ranges
)
import time
import sys
import argparse
import numpy as np
from typing import Optional, Tuple, List


def print_header():
    """Display the application header."""
    print("PyRPLIDARSDK Enhanced Scan Example")
    print("=" * 50)
    print("High-performance RPLIDAR interface with advanced data analysis")
    print("Featuring: Vectorized processing • Quality filtering • Obstacle detection")
    print("-" * 50)


def parse_arguments() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Enhanced RPLIDAR scanning example with advanced analysis",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python simple_scan.py --port /dev/ttyUSB0 --analysis
  python simple_scan.py --port COM3 --scans 10 --min-quality 50
  python simple_scan.py --ip 192.168.1.100 --detect-obstacles
        """
    )
    
    connection_group = parser.add_mutually_exclusive_group(required=True)
    connection_group.add_argument(
        "--port", "-p", 
        help="Serial port path (e.g., /dev/ttyUSB0, COM3)"
    )
    connection_group.add_argument(
        "--ip", "-i",
        help="IP address for UDP connection (e.g., 192.168.1.100)"
    )
    
    parser.add_argument(
        "--baudrate", "-b", 
        type=int, 
        default=1000000,
        help="Serial baudrate (default: 1000000)"
    )
    parser.add_argument(
        "--udp-port", "-u",
        type=int,
        default=8089,
        help="UDP port (default: 8089)"
    )
    parser.add_argument(
        "--scans", "-s",
        type=int,
        default=5,
        help="Number of scans to perform (default: 5)"
    )
    parser.add_argument(
        "--scan-interval", "-t",
        type=float,
        default=0.2,
        help="Interval between scans in seconds (default: 0.2)"
    )
    parser.add_argument(
        "--min-quality", "-q",
        type=int,
        default=15,
        help="Minimum quality threshold (default: 15)"
    )
    parser.add_argument(
        "--max-range", "-r",
        type=float,
        default=12.0,
        help="Maximum range in meters (default: 12.0)"
    )
    parser.add_argument(
        "--analysis", "-a",
        action="store_true",
        help="Enable detailed statistical analysis"
    )
    parser.add_argument(
        "--detect-obstacles", "-o",
        action="store_true",
        help="Enable obstacle detection"
    )
    parser.add_argument(
        "--smooth-data", "-m",
        action="store_true",
        help="Apply smoothing to reduce noise"
    )
    parser.add_argument(
        "--downsample", "-d",
        type=int,
        default=1,
        help="Downsampling factor (default: 1, no downsampling)"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Enable verbose output"
    )
    
    return parser.parse_args()


def create_driver(args: argparse.Namespace) -> pyrplidarsdk.RplidarDriver:
    """Create and configure the RPLIDAR driver."""
    if args.port:
        if args.verbose:
            print(f"Creating driver for serial port: {args.port}")
            print(f"Baudrate: {args.baudrate}")
        return pyrplidarsdk.RplidarDriver(port=args.port, baudrate=args.baudrate)
    else:
        if args.verbose:
            print(f"Creating driver for UDP connection: {args.ip}:{args.udp_port}")
        return pyrplidarsdk.RplidarDriver(ip_address=args.ip, udp_port=args.udp_port)


def connect_device(driver: pyrplidarsdk.RplidarDriver, args: argparse.Namespace) -> bool:
    """Establish connection to the RPLIDAR device."""
    connection_str = args.port if args.port else f"{args.ip}:{args.udp_port}"
    print(f"\n🔗 Connecting to RPLIDAR at {connection_str}...")
    
    if not driver.connect():
        print("❌ Failed to connect to RPLIDAR!")
        print("\n💡 Troubleshooting tips:")
        if args.port:
            print("   • Ensure the device is plugged in via USB")
            print("   • Check that the correct serial port is specified")
            print("   • Verify you have permission to access the serial port")
            print("   • Try running: sudo chmod 666 /dev/ttyUSB*")
        else:
            print("   • Ensure the device is connected to the network")
            print("   • Check that the IP address is correct")
            print("   • Verify the UDP port is not blocked by firewall")
        return False
    
    # Verify connection state
    if not driver.is_connected():
        print("❌ Connection verification failed!")
        return False
        
    print("✅ Connected successfully!")
    return True


def display_device_info(driver: pyrplidarsdk.RplidarDriver, verbose: bool = False) -> bool:
    """Display device information."""
    print("\n📋 Device Information:")
    print("-" * 25)
    
    info = driver.get_device_info()
    if not info:
        print("❌ Could not retrieve device information")
        return False
    
    print(f"   📟 Model: {info.model}")
    print(f"   🔧 Firmware: {info.firmware_version >> 8}.{info.firmware_version & 0xFF}")
    print(f"   ⚙️  Hardware: {info.hardware_version}")
    print(f"   🔢 Serial: {info.serial_number}")
    
    if verbose:
        print(f"   📊 Raw firmware value: {info.firmware_version}")
        print(f"   📊 Raw hardware value: {info.hardware_version}")
    
    return True


def check_device_health(driver: pyrplidarsdk.RplidarDriver) -> bool:
    """Check and display device health status."""
    print("\n❤️  Device Health:")
    print("-" * 20)
    
    health = driver.get_health()
    if not health:
        print("❌ Could not retrieve health information")
        return False
    
    status_emoji = {"Good": "✅", "Warning": "⚠️", "Error": "❌"}
    status_names = {0: "Good", 1: "Warning", 2: "Error"}
    status_name = status_names.get(health.status, "Unknown")
    emoji = status_emoji.get(status_name, "❓")
    
    print(f"   {emoji} Status: {status_name} (code: {health.status})")
    
    if health.error_code != 0:
        print(f"   🚨 Error Code: {health.error_code}")
        if status_name == "Error":
            print("   ⚠️  Device may require maintenance or restart")
            return False
    
    return status_name != "Error"


def process_scan_data(angles: List[float], ranges: List[float], qualities: List[int], 
                     args: argparse.Namespace) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Process scan data using vectorized utilities."""
    # Convert to NumPy arrays for efficient processing
    angles_arr, ranges_arr, qualities_arr = to_numpy_arrays(angles, ranges, qualities)
    original_count = len(angles_arr)
    
    if args.verbose:
        print(f"      📊 Raw data: {original_count} points")
    
    # Apply quality filtering
    pre_quality_count = len(angles_arr)
    angles_arr, ranges_arr, qualities_arr = filter_by_quality(
        angles_arr, ranges_arr, qualities_arr, min_quality=args.min_quality
    )
    
    if args.verbose and len(angles_arr) < pre_quality_count:
        filtered_out = pre_quality_count - len(angles_arr)
        print(f"      🔍 Filtered out {filtered_out} low-quality points")
    
    # Apply range filtering
    pre_range_count = len(angles_arr)
    angles_arr, ranges_arr, qualities_arr = filter_by_range(
        angles_arr, ranges_arr, qualities_arr, 
        min_range=0.1, max_range=args.max_range
    )
    
    if args.verbose and len(angles_arr) < pre_range_count:
        range_filtered = pre_range_count - len(angles_arr)
        total_filtered = original_count - len(angles_arr)
        print(f"      🔍 Range filtered: {range_filtered}, Total filtered: {total_filtered} points")
    
    # Apply downsampling if requested
    if args.downsample > 1:
        pre_downsample_count = len(angles_arr)
        angles_arr, ranges_arr, qualities_arr = downsample_scan(
            angles_arr, ranges_arr, qualities_arr, args.downsample
        )
        if args.verbose:
            print(f"      📉 Downsampled from {pre_downsample_count} to {len(angles_arr)} points (factor {args.downsample})")
    
    # Apply smoothing if requested
    if args.smooth_data and len(ranges_arr) > 5:
        smoothed_ranges = smooth_ranges(ranges_arr, window_size=5, method="median")
        ranges_arr = smoothed_ranges
        if args.verbose:
            print(f"      🔧 Applied median smoothing")
    
    return angles_arr, ranges_arr, qualities_arr


def analyze_scan_data(angles_arr: np.ndarray, ranges_arr: np.ndarray, 
                     qualities_arr: np.ndarray, args: argparse.Namespace) -> dict:
    """Perform comprehensive analysis of scan data."""
    analysis_results = {}
    
    if len(ranges_arr) == 0:
        return {"error": "No valid data points"}
    
    # Compute comprehensive statistics
    stats = compute_scan_statistics(ranges_arr, qualities_arr)
    analysis_results["statistics"] = stats
    
    # Detect obstacles if requested
    if args.detect_obstacles:
        obstacles = detect_obstacles(ranges_arr, angles_arr, min_size=0.15, max_gap=0.3)
        analysis_results["obstacles"] = obstacles
    
    # Analyze data quality
    high_quality_mask = qualities_arr >= 100
    medium_quality_mask = (qualities_arr >= 50) & (qualities_arr < 100)
    low_quality_mask = qualities_arr < 50
    
    analysis_results["quality_distribution"] = {
        "high_quality": int(np.sum(high_quality_mask)),
        "medium_quality": int(np.sum(medium_quality_mask)),
        "low_quality": int(np.sum(low_quality_mask))
    }
    
    # Analyze range distribution
    close_points = np.sum(ranges_arr < 1.0)
    medium_points = np.sum((ranges_arr >= 1.0) & (ranges_arr < 5.0))
    far_points = np.sum(ranges_arr >= 5.0)
    
    analysis_results["range_distribution"] = {
        "close": int(close_points),
        "medium": int(medium_points), 
        "far": int(far_points)
    }
    
    return analysis_results


def display_analysis_results(analysis: dict, scan_num: int, args: argparse.Namespace):
    """Display comprehensive analysis results."""
    if "error" in analysis:
        print(f"      ❌ Analysis error: {analysis['error']}")
        return
    
    stats = analysis["statistics"]
    
    # Basic statistics
    print(f"      📏 Points: {stats['count']}")
    print(f"      📐 Range: {stats['min_range']:.2f}m - {stats['max_range']:.2f}m")
    print(f"      📊 Mean: {stats['mean_range']:.2f}m ± {stats['std_range']:.2f}m")
    print(f"      ⭐ Quality: {stats['mean_quality']:.1f}/255 (med: {stats['median_quality']:.1f})")
    
    if args.analysis:
        print(f"      📈 Detailed Statistics:")
        print(f"         • Median range: {stats['median_range']:.2f}m")
        print(f"         • Quality std: {stats['std_quality']:.1f}")
        print(f"         • Quality range: {stats['min_quality']}-{stats['max_quality']}")
        
        # Quality distribution
        quality_dist = analysis["quality_distribution"]
        total_points = sum(quality_dist.values())
        if total_points > 0:
            print(f"      🎯 Quality Distribution:")
            print(f"         • High (≥100): {quality_dist['high_quality']} ({quality_dist['high_quality']/total_points*100:.1f}%)")
            print(f"         • Medium (50-99): {quality_dist['medium_quality']} ({quality_dist['medium_quality']/total_points*100:.1f}%)")
            print(f"         • Low (<50): {quality_dist['low_quality']} ({quality_dist['low_quality']/total_points*100:.1f}%)")
        
        # Range distribution
        range_dist = analysis["range_distribution"]
        print(f"      📏 Range Distribution:")
        print(f"         • Close (<1m): {range_dist['close']}")
        print(f"         • Medium (1-5m): {range_dist['medium']}")
        print(f"         • Far (≥5m): {range_dist['far']}")
    
    # Obstacle detection results
    if args.detect_obstacles and "obstacles" in analysis:
        obstacles = analysis["obstacles"]
        if obstacles:
            print(f"      🚧 Obstacles Detected: {len(obstacles)}")
            for i, (start_idx, end_idx, min_range, angular_width) in enumerate(obstacles[:3]):
                angular_width_deg = angular_width * 180 / np.pi
                print(f"         • Obstacle {i+1}: {min_range:.2f}m, {angular_width_deg:.1f}° wide")
            if len(obstacles) > 3:
                print(f"         • ... and {len(obstacles) - 3} more")
        else:
            print(f"      🚧 No obstacles detected")


def perform_scans(driver: pyrplidarsdk.RplidarDriver, args: argparse.Namespace) -> Tuple[bool, int, int]:
    """Perform the enhanced scanning operation."""
    print(f"\n🔄 Starting enhanced scan sequence...")
    print(f"   🎯 Filters: Quality ≥{args.min_quality}, Range ≤{args.max_range}m")
    if args.downsample > 1:
        print(f"   📉 Downsampling: Factor {args.downsample}")
    if args.smooth_data:
        print(f"   🔧 Smoothing: Enabled")
    
    if not driver.start_scan():
        print("❌ Failed to start scanning!")
        return False, 0, 0
    
    # Verify scanning state
    if not driver.is_scanning():
        print("❌ Scanning verification failed!")
        return False, 0, 0
        
    print("✅ Scanning started successfully!")
    print(f"\n📡 Performing {args.scans} scans (interval: {args.scan_interval}s):")
    
    successful_scans = 0
    total_points = 0
    all_scan_stats = []
    
    try:
        for scan_num in range(args.scans):
            print(f"\n   📊 Scan #{scan_num + 1}:")
            
            scan_data = driver.get_scan_data()
            if not scan_data:
                print("      ❌ No scan data received")
                continue
            
            angles, ranges, qualities = scan_data
            
            if len(angles) == 0:
                print("      ⚠️  No valid points received")
                continue
                
            # Process scan data using vectorized utilities
            angles_arr, ranges_arr, qualities_arr = process_scan_data(
                angles, ranges, qualities, args
            )
            
            if len(angles_arr) == 0:
                print("      ⚠️  All points filtered out")
                continue
            
            # Perform analysis
            analysis = analyze_scan_data(angles_arr, ranges_arr, qualities_arr, args)
            all_scan_stats.append(analysis)
            
            # Display results
            display_analysis_results(analysis, scan_num, args)
            
            successful_scans += 1
            total_points += len(angles_arr)
            
            # Show sample points if verbose
            if args.verbose and len(angles_arr) > 0:
                print("      🎯 Sample points (angle°, range_m, quality):")
                angles_deg = angles_to_degrees(angles_arr)
                sample_count = min(3, len(angles_arr))
                for i in range(sample_count):
                    print(f"         {angles_deg[i]:6.1f}°, {ranges_arr[i]:5.2f}m, {qualities_arr[i]:3d}")                

            
            if scan_num < args.scans - 1:  # Don't sleep after the last scan
                time.sleep(args.scan_interval)
                
    except KeyboardInterrupt:
        print("\n\n⏹️  Scan interrupted by user")
    
    return successful_scans > 0, successful_scans, total_points


def cleanup_and_summarize(driver: pyrplidarsdk.RplidarDriver, successful_scans: int, total_points: int):
    """Clean up resources and display summary."""
    print("\n🧹 Cleaning up...")
    
    if driver.is_scanning():
        driver.stop_scan()
        print("   ⏹️  Scanning stopped")
    
    if driver.is_connected():
        driver.disconnect()
        print("   🔌 Disconnected from device")
    
    print("\n📈 Enhanced Scan Summary:")
    print("-" * 30)
    print(f"   ✅ Successful scans: {successful_scans}")
    print(f"   📊 Total filtered points: {total_points}")
    if successful_scans > 0:
        print(f"   📊 Average points per scan: {total_points // successful_scans}")
        print(f"   🎯 Processing: Vectorized NumPy operations")
        print(f"   🔍 Filtering: Quality & range based")


def main() -> int:
    """Main application entry point."""
    args = parse_arguments()
    print_header()
    
    # Create driver
    driver = create_driver(args)
    
    # Connect to device
    if not connect_device(driver, args):
        return 1
    
    # Get device information
    if not display_device_info(driver, args.verbose):
        driver.disconnect()
        return 1
    
    # Check device health
    if not check_device_health(driver):
        driver.disconnect()
        return 1
    
    # Perform enhanced scans
    success, successful_scans, total_points = perform_scans(driver, args)
    
    # Clean up and show summary
    cleanup_and_summarize(driver, successful_scans, total_points)
    
    if success:
        print("\n🎉 Enhanced scan completed successfully!")
        return 0
    else:
        print("\n⚠️  Scan completed with errors")
        return 1
            


if __name__ == "__main__":
    sys.exit(main()) 