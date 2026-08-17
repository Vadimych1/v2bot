#!/usr/bin/env python3
"""
Real-time RPLIDAR data visualization with advanced analytics

This example demonstrates high-performance real-time plotting of RPLIDAR scan data 
with vectorized processing, obstacle detection, and comprehensive data analysis.
Features include filtering, smoothing, statistical analysis, and obstacle highlighting.

Requirements:
    pip install matplotlib numpy scipy

Copyright (C) 2025 Dexmate Inc.
Licensed under MIT License
"""

import pyrplidarsdk
from pyrplidarsdk.utils import (
    polar_to_cartesian, filter_by_range, filter_by_quality, filter_by_angle_range,
    compute_scan_statistics, downsample_scan, smooth_ranges, detect_obstacles,
    to_numpy_arrays, angles_to_degrees
)
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import numpy as np
import sys
import threading
import time
import argparse
from collections import deque
from typing import Optional, Tuple, List, Dict, Any


class EnhancedRplidarPlotter:
    """High-performance real-time RPLIDAR data plotter with advanced analytics."""
    
    def __init__(self, port: Optional[str] = None, ip_address: Optional[str] = None,
                 baudrate: int = 1000000, udp_port: int = 8089,
                 max_range: float = 6.0, min_quality: int = 10,
                 update_rate: int = 20, history_size: int = 3,
                 enable_obstacles: bool = False, enable_smoothing: bool = False,
                 downsample_factor: int = 1, angle_range: Optional[Tuple[float, float]] = None):
        """
        Initialize the enhanced RPLIDAR plotter.
        
        Args:
            port: Serial port path (e.g., '/dev/ttyUSB0', 'COM3')
            ip_address: IP address for UDP connection
            baudrate: Serial baudrate (default: 1000000)
            udp_port: UDP port (default: 8089)
            max_range: Maximum range to display in meters (default: 6.0)
            min_quality: Minimum quality threshold (default: 10)
            update_rate: Display update rate in Hz (default: 20)
            history_size: Number of scans to keep in history (default: 3)
            enable_obstacles: Enable obstacle detection and visualization
            enable_smoothing: Enable range smoothing
            downsample_factor: Downsampling factor for performance (default: 1)
            angle_range: Optional tuple (min_angle, max_angle) in degrees for sector scanning
        """
        self.port = port
        self.ip_address = ip_address
        self.baudrate = baudrate
        self.udp_port = udp_port
        self.max_range = max_range
        self.min_quality = min_quality
        self.update_interval = 1000 // update_rate  # Convert Hz to ms
        self.enable_obstacles = enable_obstacles
        self.enable_smoothing = enable_smoothing
        self.downsample_factor = max(1, downsample_factor)
        self.angle_range = angle_range
        
        # Driver and state
        self.driver = None
        self.running = False
        self.data_thread = None
        
        # Enhanced data storage with thread safety
        self.scan_data = deque(maxlen=history_size)
        self.obstacle_data = deque(maxlen=history_size)
        self.data_lock = threading.Lock()
        self.stats = {
            'total_scans': 0,
            'valid_scans': 0,
            'total_points': 0,
            'filtered_points': 0,
            'obstacles_detected': 0,
            'fps': 0.0,
            'processing_time': 0.0,
            'last_update': time.time()
        }
        
        # Setup matplotlib with optimized settings
        plt.style.use('dark_background')  # Better for real-time data
        self._setup_plots()
        
    def _setup_plots(self):
        """Initialize the matplotlib plots with enhanced visualization."""
        # Create figure with 3 subplots for comprehensive visualization
        self.fig = plt.figure(figsize=(18, 10))
        gs = self.fig.add_gridspec(2, 3, height_ratios=[3, 1], width_ratios=[1, 1, 1])
        
        # Main polar plot (top-left)
        self.ax1 = self.fig.add_subplot(gs[0, 0])
        self.ax1.set_title('Polar View (Cartesian Coordinates)', fontsize=12, fontweight='bold')
        self.ax1.set_xlim(-self.max_range, self.max_range)
        self.ax1.set_ylim(-self.max_range, self.max_range)
        self.ax1.set_xlabel('X (meters)', fontsize=10)
        self.ax1.set_ylabel('Y (meters)', fontsize=10)
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_aspect('equal')
        
        # Add enhanced reference elements
        for r in np.arange(1, self.max_range + 1, 1):
            circle = plt.Circle((0, 0), r, fill=False, linestyle='--', 
                              alpha=0.2, color='gray', linewidth=1)
            self.ax1.add_patch(circle)
            # Add range labels
            self.ax1.text(r, 0, f'{r}m', fontsize=8, alpha=0.6, ha='left', va='bottom')
        
        # Add angle lines and labels
        for angle in np.arange(0, 360, 45):
            x = self.max_range * np.cos(np.radians(angle))
            y = self.max_range * np.sin(np.radians(angle))
            self.ax1.plot([0, x], [0, y], '--', alpha=0.2, color='gray', linewidth=1)
            # Add angle labels
            label_x = (self.max_range * 0.9) * np.cos(np.radians(angle))
            label_y = (self.max_range * 0.9) * np.sin(np.radians(angle))
            self.ax1.text(label_x, label_y, f'{angle}°', fontsize=8, alpha=0.6, 
                         ha='center', va='center')
        
        # Add angle range indicator if specified
        if self.angle_range:
            min_ang, max_ang = self.angle_range
            for angle in [min_ang, max_ang]:
                x = self.max_range * np.cos(np.radians(angle))
                y = self.max_range * np.sin(np.radians(angle))
                self.ax1.plot([0, x], [0, y], '--', alpha=0.8, color='yellow', linewidth=2)
        
        # Initialize scatter plots with performance optimization
        self.scatter1 = self.ax1.scatter([], [], s=3, c='cyan', alpha=0.8, 
                                       edgecolors='none', rasterized=True, label='Scan Points')
        if self.enable_obstacles:
            self.obstacle_scatter = self.ax1.scatter([], [], s=8, c='red', alpha=0.9,
                                                   marker='s', edgecolors='white', linewidth=0.5,
                                                   label='Obstacles', rasterized=True)
        
        # Range vs angle plot (top-middle)
        self.ax2 = self.fig.add_subplot(gs[0, 1])
        self.ax2.set_title('Range vs Angle Profile', fontsize=12, fontweight='bold')
        self.ax2.set_xlabel('Angle (degrees)', fontsize=10)
        self.ax2.set_ylabel('Range (meters)', fontsize=10)
        self.ax2.set_xlim(0, 360)
        self.ax2.set_ylim(0, self.max_range)
        self.ax2.grid(True, alpha=0.3)
        
        # Initialize line plots
        self.line2, = self.ax2.plot([], [], 'lime', alpha=0.8, linewidth=1.5, 
                                   marker='o', markersize=1.5, markeredgewidth=0,
                                   label='Range Profile')
        if self.enable_obstacles:
            self.obstacle_line, = self.ax2.plot([], [], 'ro', alpha=0.9, markersize=4,
                                              markeredgewidth=1, markeredgecolor='white',
                                              label='Obstacles', linestyle='None')
        
        # Quality distribution plot (top-right)
        self.ax3 = self.fig.add_subplot(gs[0, 2])
        self.ax3.set_title('Quality Distribution', fontsize=12, fontweight='bold')
        self.ax3.set_xlabel('Quality Value', fontsize=10)
        self.ax3.set_ylabel('Count', fontsize=10)
        self.ax3.set_xlim(0, 255)
        self.ax3.grid(True, alpha=0.3)
        
        # Statistics and controls panel (bottom, spans all columns)
        self.ax4 = self.fig.add_subplot(gs[1, :])
        self.ax4.set_xlim(0, 1)
        self.ax4.set_ylim(0, 1)
        self.ax4.axis('off')
        
        # Add comprehensive statistics text
        self.stats_text = self.ax4.text(0.02, 0.95, '', transform=self.ax4.transAxes,
                                      fontsize=10, verticalalignment='top', fontfamily='monospace',
                                      bbox=dict(boxstyle='round,pad=0.5', facecolor='black', alpha=0.8))
        
        # Add configuration info
        config_text = (f"🎯 Config: Range≤{self.max_range}m • Quality≥{self.min_quality} • "
                      f"Downsample×{self.downsample_factor} • "
                      f"{'Smoothing✓' if self.enable_smoothing else 'Smoothing✗'} • "
                      f"{'Obstacles✓' if self.enable_obstacles else 'Obstacles✗'}")
        
        if self.angle_range:
            config_text += f" • Sector: {self.angle_range[0]}°-{self.angle_range[1]}°"
            
        self.config_text = self.ax4.text(0.02, 0.02, config_text, transform=self.ax4.transAxes,
                                       fontsize=9, verticalalignment='bottom',
                                       bbox=dict(boxstyle='round,pad=0.3', facecolor='darkblue', alpha=0.8))
        
        # Add legends
        if self.enable_obstacles:
            self.ax1.legend(loc='upper right', fontsize=8)
            self.ax2.legend(loc='upper right', fontsize=8)
        
        # Set main title first
        title = f"PyRPLIDARSDK Enhanced Real-time Visualization"
        if self.angle_range:
            title += f" (Sector: {self.angle_range[0]}°-{self.angle_range[1]}°)"
        self.fig.suptitle(title, fontsize=14, fontweight='bold', y=0.95)
        
        # Optimize layout with proper spacing for title
        plt.tight_layout(rect=[0, 0, 1, 0.93])
        
    def create_driver(self) -> bool:
        """Create and configure the RPLIDAR driver."""
        try:
            if self.ip_address:
                print(f"🌐 Creating UDP connection to {self.ip_address}:{self.udp_port}")
                self.driver = pyrplidarsdk.RplidarDriver(
                    ip_address=self.ip_address, 
                    udp_port=self.udp_port
                )
            elif self.port:
                print(f"🔌 Creating serial connection to {self.port} (baudrate: {self.baudrate})")
                self.driver = pyrplidarsdk.RplidarDriver(
                    port=self.port, 
                    baudrate=self.baudrate
                )
            else:
                print("❌ Error: Either port or IP address must be specified!")
                return False
                
            return True
            
        except Exception as e:
            print(f"❌ Error creating driver: {e}")
            return False
    
    def connect_and_setup(self) -> bool:
        """Connect to RPLIDAR and perform initial setup."""
        if not self.create_driver():
            return False
            
        connection_str = self.port if self.port else f"{self.ip_address}:{self.udp_port}"
        print(f"🔗 Connecting to RPLIDAR at {connection_str}...")
        
        if not self.driver.connect():
            print("❌ Failed to connect to RPLIDAR!")
            return False
        
        # Verify connection
        if not self.driver.is_connected():
            print("❌ Connection verification failed!")
            return False
            
        print("✅ Connected successfully!")
        
        # Get and display device info
        info = self.driver.get_device_info()
        if info:
            print(f"📟 Device: Model {info.model}, FW {info.firmware_version >> 8}.{info.firmware_version & 0xFF}")
            print(f"🔢 Serial: {info.serial_number}")
        
        # Check health
        health = self.driver.get_health()
        if health:
            status_names = {0: "Good", 1: "Warning", 2: "Error"}
            status = status_names.get(health.status, "Unknown")
            print(f"❤️  Health: {status}")
            if health.status == 2:  # Error
                print("⚠️  Warning: Device reports error status!")
        
        # Start scanning
        print("🔄 Starting scan...")
        if not self.driver.start_scan():
            print("❌ Failed to start scanning!")
            return False
            
        # Verify scanning state
        if not self.driver.is_scanning():
            print("❌ Scanning verification failed!")
            return False
            
        print("✅ Scanning started!")
        print(f"🎯 Enhanced processing enabled:")
        print(f"   • Vectorized NumPy operations")
        print(f"   • Quality filtering (≥{self.min_quality})")
        print(f"   • Range filtering (≤{self.max_range}m)")
        if self.downsample_factor > 1:
            print(f"   • Downsampling (factor: {self.downsample_factor})")
        if self.enable_smoothing:
            print(f"   • Range smoothing")
        if self.enable_obstacles:
            print(f"   • Obstacle detection")
        if self.angle_range:
            print(f"   • Sector scanning ({self.angle_range[0]}°-{self.angle_range[1]}°)")
            
        return True
    
    def process_scan_data(self, angles: List[float], ranges: List[float], 
                         qualities: List[int]) -> Tuple[np.ndarray, np.ndarray, np.ndarray, Dict[str, Any]]:
        """Process scan data using enhanced vectorized utilities."""
        processing_start = time.time()
        
        # Convert to NumPy arrays for efficient processing
        angles_arr, ranges_arr, qualities_arr = to_numpy_arrays(angles, ranges, qualities)
        original_count = len(angles_arr)
        
        # Apply quality filtering with vectorized operations
        angles_arr, ranges_arr, qualities_arr = filter_by_quality(
            angles_arr, ranges_arr, qualities_arr, min_quality=self.min_quality
        )
        
        # Apply range filtering
        angles_arr, ranges_arr, qualities_arr = filter_by_range(
            angles_arr, ranges_arr, qualities_arr, 
            min_range=0.1, max_range=self.max_range
        )
        
        # Apply angle range filtering if specified
        if self.angle_range:
            angles_arr, ranges_arr, qualities_arr = filter_by_angle_range(
                angles_arr, ranges_arr, qualities_arr,
                self.angle_range[0], self.angle_range[1], angle_unit="degrees"
            )
        
        # Apply downsampling for performance
        if self.downsample_factor > 1 and len(angles_arr) > 0:
            angles_arr, ranges_arr, qualities_arr = downsample_scan(
                angles_arr, ranges_arr, qualities_arr, self.downsample_factor
            )
        
        # Apply smoothing if enabled
        if self.enable_smoothing and len(ranges_arr) > 5:
            ranges_arr = smooth_ranges(ranges_arr, window_size=5, method="median")
        
        # Compute comprehensive statistics
        processing_results = {
            'original_points': original_count,
            'filtered_points': len(angles_arr),
            'processing_time': time.time() - processing_start
        }
        
        if len(ranges_arr) > 0:
            stats = compute_scan_statistics(ranges_arr, qualities_arr)
            processing_results.update(stats)
            
            # Detect obstacles if enabled
            if self.enable_obstacles:
                obstacles = detect_obstacles(ranges_arr, angles_arr, min_size=0.1, max_gap=0.2)
                processing_results['obstacles'] = obstacles
                self.stats['obstacles_detected'] = len(obstacles)
        
        return angles_arr, ranges_arr, qualities_arr, processing_results
    
    def data_collection_thread(self):
        """Enhanced thread function to continuously collect and process scan data."""
        scan_times = deque(maxlen=10)  # For FPS calculation
        
        while self.running and self.driver and self.driver.is_connected():
            try:
                start_time = time.time()
                
                if not self.driver.is_scanning():
                    print("⚠️  Warning: Scanning stopped unexpectedly!")
                    break
                
                scan_data = self.driver.get_scan_data()
                if scan_data:
                    angles, ranges, qualities = scan_data
                    self.stats['total_scans'] += 1
                    
                    # Process data using enhanced vectorized utilities
                    if len(angles) > 0:
                        angles_arr, ranges_arr, qualities_arr, processing_results = self.process_scan_data(
                            angles, ranges, qualities
                        )
                        
                        if len(angles_arr) > 0:  # Check if we have valid points after filtering
                            with self.data_lock:
                                self.scan_data.append((angles_arr, ranges_arr, qualities_arr))
                                
                                # Store obstacle data if available
                                if self.enable_obstacles and 'obstacles' in processing_results:
                                    self.obstacle_data.append(processing_results['obstacles'])
                            
                            self.stats['valid_scans'] += 1
                            self.stats['total_points'] += processing_results['original_points']
                            self.stats['filtered_points'] += processing_results['filtered_points']
                            self.stats['processing_time'] = processing_results['processing_time']
                
                # Calculate FPS with improved accuracy
                scan_times.append(time.time())
                if len(scan_times) > 1:
                    time_span = scan_times[-1] - scan_times[0]
                    if time_span > 0:
                        self.stats['fps'] = (len(scan_times) - 1) / time_span
                
                # Sleep to maintain target update rate with better timing
                elapsed = time.time() - start_time
                target_interval = 0.05  # Target ~20 Hz
                sleep_time = max(0, target_interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
            except Exception as e:
                print(f"⚠️  Data collection error: {e}")
                time.sleep(0.1)
    
    def animate(self, frame):
        """Enhanced animation function with comprehensive visualization."""
        try:
            with self.data_lock:
                if len(self.scan_data) == 0:
                    return self._get_artists()
                
                # Get the most recent scan data
                angles, ranges, qualities = self.scan_data[-1]
                obstacles = self.obstacle_data[-1] if (self.enable_obstacles and 
                                                     len(self.obstacle_data) > 0) else []
            
            if len(angles) == 0:
                return self._get_artists()
            
            # Convert to cartesian coordinates using vectorized operations  
            x_coords, y_coords = polar_to_cartesian(angles, ranges)
            
            # Update main polar plot with enhanced color mapping
            # Use quality-based coloring with improved colormap
            quality_norm = qualities / 255.0
            colors = plt.cm.viridis(quality_norm)  # Better colormap for data visualization
            
            self.scatter1.set_offsets(np.column_stack([x_coords, y_coords]))
            self.scatter1.set_color(colors)
            
            # Update obstacle visualization if enabled
            if self.enable_obstacles and obstacles:
                obstacle_x, obstacle_y = [], []
                obstacle_angles, obstacle_ranges = [], []
                
                for start_idx, end_idx, min_range, angular_width in obstacles:
                    # Highlight obstacle points
                    obs_idx = (start_idx + end_idx) // 2  # Center of obstacle
                    if obs_idx < len(x_coords):
                        obstacle_x.append(x_coords[obs_idx])
                        obstacle_y.append(y_coords[obs_idx])
                        obstacle_angles.append(angles[obs_idx] * 180 / np.pi)
                        obstacle_ranges.append(ranges[obs_idx])
                
                if obstacle_x:
                    self.obstacle_scatter.set_offsets(np.column_stack([obstacle_x, obstacle_y]))
                    self.obstacle_line.set_data(obstacle_angles, obstacle_ranges)
                else:
                    self.obstacle_scatter.set_offsets(np.empty((0, 2)))
                    self.obstacle_line.set_data([], [])
            
            # Update range vs angle plot with improved sorting
            angles_deg = angles_to_degrees(angles)
            
            # Sort by angle for better line plot visualization
            sort_indices = np.argsort(angles_deg)
            sorted_angles = angles_deg[sort_indices]
            sorted_ranges = ranges[sort_indices]
            
            self.line2.set_data(sorted_angles, sorted_ranges)
            
            # Update quality distribution histogram
            self.ax3.clear()
            self.ax3.hist(qualities, bins=50, alpha=0.7, color='cyan', edgecolor='black', linewidth=0.5)
            self.ax3.set_title('Quality Distribution', fontsize=12, fontweight='bold')
            self.ax3.set_xlabel('Quality Value', fontsize=10)
            self.ax3.set_ylabel('Count', fontsize=10)
            self.ax3.set_xlim(0, 255)
            self.ax3.grid(True, alpha=0.3)
            
            # Add quality statistics to the plot
            mean_quality = np.mean(qualities)
            self.ax3.axvline(mean_quality, color='red', linestyle='--', alpha=0.8, 
                           label=f'Mean: {mean_quality:.1f}')
            self.ax3.legend(fontsize=8)
            
            # Update comprehensive statistics display
            self._update_enhanced_stats_display(len(angles), qualities, ranges)
            
            return self._get_artists()
            
        except Exception as e:
            print(f"⚠️  Animation error: {e}")
            return self._get_artists()
    
    def _get_artists(self):
        """Get all matplotlib artists for animation."""
        artists = [self.scatter1, self.line2, self.stats_text]
        if self.enable_obstacles:
            artists.extend([self.obstacle_scatter, self.obstacle_line])
        return artists
    
    def _update_enhanced_stats_display(self, current_points: int, qualities: np.ndarray, ranges: np.ndarray):
        """Update the comprehensive statistics display."""
        # Calculate processing efficiency
        processing_efficiency = (self.stats['filtered_points'] / max(1, self.stats['total_points'])) * 100
        avg_points = (self.stats['filtered_points'] / max(1, self.stats['valid_scans']))
        
        # Calculate current scan statistics
        quality_stats = f"Q: {np.mean(qualities):.1f}±{np.std(qualities):.1f} (med:{np.median(qualities):.1f})"
        range_stats = f"R: {np.mean(ranges):.2f}±{np.std(ranges):.2f}m (med:{np.median(ranges):.2f}m)"
        
        # Build comprehensive statistics text
        stats_lines = [
            f"🚀 PERFORMANCE  │  FPS: {self.stats['fps']:.1f}  │  Process: {self.stats['processing_time']*1000:.1f}ms  │  Efficiency: {processing_efficiency:.1f}%",
            f"📊 SCAN DATA    │  Points: {current_points}  │  Avg/Scan: {avg_points:.0f}  │  Total Filtered: {self.stats['filtered_points']:,}",
            f"📈 STATISTICS   │  {quality_stats}  │  {range_stats}"
        ]
        
        # Add scan totals
        stats_lines.append(f"🔢 TOTALS       │  Scans: {self.stats['total_scans']}  │  Valid: {self.stats['valid_scans']}  │  Points: {self.stats['total_points']:,}")
        
        # Add obstacle information if enabled
        if self.enable_obstacles:
            stats_lines.append(f"🚧 OBSTACLES    │  Current: {self.stats['obstacles_detected']}  │  Range: {self.max_range}m  │  Quality: ≥{self.min_quality}")
        
        # Add filtering information
        filter_info = f"🔍 FILTERING    │  Range: ≤{self.max_range}m  │  Quality: ≥{self.min_quality}  │  Downsample: ×{self.downsample_factor}"
        if self.angle_range:
            filter_info += f"  │  Sector: {self.angle_range[0]}°-{self.angle_range[1]}°"
        stats_lines.append(filter_info)
        
        stats_text = '\n'.join(stats_lines)
        self.stats_text.set_text(stats_text)
    
    def start_visualization(self) -> bool:
        """Start the enhanced real-time visualization."""
        if not self.connect_and_setup():
            return False
        
        self.running = True
        
        # Start enhanced data collection thread
        self.data_thread = threading.Thread(target=self.data_collection_thread, daemon=True)
        self.data_thread.start()
        
        # Start animation with optimized settings
        print("🎬 Starting enhanced visualization...")
        print("💡 Press Ctrl+C or close the window to stop")
        print("🎯 Features: Vectorized processing • Quality coloring • Statistics • Real-time analysis")
        if self.enable_obstacles:
            print("🚧 Obstacle detection enabled")
        if self.enable_smoothing:
            print("🔧 Range smoothing enabled")
        
        try:
            self.ani = animation.FuncAnimation(
                self.fig, self.animate, 
                interval=self.update_interval,
                blit=False,  # Disabled for comprehensive visualization
                cache_frame_data=False,
                repeat=True
            )
            
            plt.show()
            
        except KeyboardInterrupt:
            print("\n⏹️  Visualization interrupted by user")
        except Exception as e:
            print(f"\n❌ Visualization error: {e}")
        finally:
            self.stop_visualization()
        
        return True
    
    def stop_visualization(self):
        """Stop the visualization and clean up resources."""
        print("\n🧹 Cleaning up...")
        self.running = False
        
        # Wait for data thread to finish
        if self.data_thread and self.data_thread.is_alive():
            self.data_thread.join(timeout=2)
        
        if self.driver:
            if self.driver.is_scanning():
                self.driver.stop_scan()
                print("   ⏹️  Scanning stopped")
            
            if self.driver.is_connected():
                self.driver.disconnect()
                print("   🔌 Disconnected from RPLIDAR")
        
        print("✅ Cleanup complete")


def parse_arguments() -> argparse.Namespace:
    """Parse command line arguments with enhanced options."""
    parser = argparse.ArgumentParser(
        description="Enhanced real-time RPLIDAR visualization with advanced analytics",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python realtime_plot.py --port /dev/ttyUSB0 --obstacles --analysis
  python realtime_plot.py --port COM3 --max-range 8 --min-quality 50 --smooth
  python realtime_plot.py --ip 192.168.1.100 --update-rate 30 --downsample 2
  python realtime_plot.py --port /dev/ttyUSB0 --sector -45 45 --obstacles
        """
    )
    
    connection_group = parser.add_mutually_exclusive_group(required=True)
    connection_group.add_argument(
        "--port", "-p", 
        help="Serial port path (e.g., /dev/ttyUSB0, COM3)"
    )
    connection_group.add_argument(
        "--ip", "-i",
        help="IP address for UDP connection"
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
        "--max-range", "-r",
        type=float,
        default=6.0,
        help="Maximum range in meters (default: 6.0)"
    )
    parser.add_argument(
        "--min-quality", "-q",
        type=int,
        default=10,
        help="Minimum quality threshold (default: 10)"
    )
    parser.add_argument(
        "--update-rate",
        type=int,
        default=20,
        help="Display update rate in Hz (default: 20)"
    )
    parser.add_argument(
        "--history-size",
        type=int,
        default=3,
        help="Number of scans to keep in history (default: 3)"
    )
    parser.add_argument(
        "--obstacles", "-o",
        action="store_true",
        help="Enable obstacle detection and visualization"
    )
    parser.add_argument(
        "--smooth", "-s",
        action="store_true",
        help="Enable range smoothing to reduce noise"
    )
    parser.add_argument(
        "--downsample", "-d",
        type=int,
        default=1,
        help="Downsampling factor for performance (default: 1, no downsampling)"
    )
    parser.add_argument(
        "--sector",
        nargs=2,
        type=float,
        metavar=('MIN_ANGLE', 'MAX_ANGLE'),
        help="Limit scanning to angle sector in degrees (e.g., --sector -45 45)"
    )
    
    return parser.parse_args()


def main() -> int:
    """Main application entry point."""
    try:
        args = parse_arguments()
        
        print("PyRPLIDARSDK Enhanced Real-time Visualization")
        print("=" * 55)
        print("🚀 High-performance vectorized processing")
        print("📊 Advanced analytics and obstacle detection")
        print("🎨 Multi-panel comprehensive visualization")
        print("-" * 55)
        
        # Determine angle range
        angle_range = tuple(args.sector) if args.sector else None
        
        # Create enhanced plotter
        plotter = EnhancedRplidarPlotter(
            port=args.port,
            ip_address=args.ip,
            baudrate=args.baudrate,
            udp_port=args.udp_port,
            max_range=args.max_range,
            min_quality=args.min_quality,
            update_rate=args.update_rate,
            history_size=args.history_size,
            enable_obstacles=args.obstacles,
            enable_smoothing=args.smooth,
            downsample_factor=args.downsample,
            angle_range=angle_range
        )
        
        # Start visualization
        success = plotter.start_visualization()
        
        return 0 if success else 1
        
    except Exception as e:
        print(f"\n💥 Unexpected error: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main()) 