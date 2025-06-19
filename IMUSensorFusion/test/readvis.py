# import websockets
# import json
# from datetime import datetime
# import argparse
# from collections import deque
# import asyncio
# import numpy as np
# import pygame
# from pygame.locals import *
# from OpenGL.GL import *
# from OpenGL.GLU import *
# from math import radians
# from scipy.spatial.transform import Rotation as R
# from numpy.linalg import inv
# import random
# import time

# # Define cube vertices, faces, and colors
# VERTICES = [
#     (-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
#     (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)
# ]

# FACES = [
#     (0, 1, 2, 3),  # Back face
#     (4, 5, 6, 7),  # Front face
#     (0, 3, 7, 4),  # Left face
#     (1, 2, 6, 5),  # Right face
#     (0, 1, 5, 4),  # Bottom face
#     (3, 2, 6, 7)   # Top face
# ]

# COLORS = [
#     (0.5, 0, 0),    # Red
#     (0, 0.5, 0),    # Green
#     (0, 0, 0.5),    # Blue
#     (0.5, 0.5, 0),  # Yellow
#     (0.5, 0, 0.5),  # Magenta
#     (0, 0.5, 0.5)   # Cyan
# ]

# def draw_cube():
#     """Draw a simple colored cube"""
#     glBegin(GL_QUADS)
#     for i, face in enumerate(FACES):
#         glColor3fv(COLORS[i])
#         for vertex in face:
#             glVertex3fv(VERTICES[vertex])
#     glEnd()
    
#     # Draw coordinate axes for reference
#     glBegin(GL_LINES)
#     # X-axis (red)
#     glColor3f(1, 0, 0)
#     glVertex3f(0, 0, 0)
#     glVertex3f(2, 0, 0)
#     # Y-axis (green)
#     glColor3f(0, 1, 0)
#     glVertex3f(0, 0, 0)
#     glVertex3f(0, 2, 0)
#     # Z-axis (blue)
#     glColor3f(0, 0, 1)
#     glVertex3f(0, 0, 0)
#     glVertex3f(0, 0, 2)
#     glEnd()

# class Cube:
#     def __init__(self, device_id=0, position=(0, 0, 0), size=1):  # Reduced size to 1
#         self.device_id = device_id
#         self.position = position
#         self.size = size
#         self.orientation = np.identity(3)  # 3x3 rotation matrix

#     def set_orientation(self, rpy):
#         """Set orientation from roll, pitch, yaw in degrees."""
#         roll, pitch, yaw = rpy  # Assumes input in degrees
#         # Convert to rotation matrix using scipy
#         rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
#         self.orientation = rotation.as_matrix()

#     def draw(self):
#         glPushMatrix()
#         glTranslatef(*self.position)

#         # Convert rotation matrix to 4x4 and apply it
#         rotation_matrix_4x4 = np.identity(4)
#         rotation_matrix_4x4[:3, :3] = self.orientation
#         glMultMatrixf(rotation_matrix_4x4.T)  # Transpose because OpenGL uses column-major order

#         # Scale the cube by size
#         glScalef(self.size, self.size, self.size)
        
#         # Draw cube
#         glBegin(GL_QUADS)
#         for i, face in enumerate(FACES):
#             glColor3fv(COLORS[i])
#             for vertex in face:
#                 glVertex3fv(VERTICES[vertex])  # No scaling here, using glScale instead
#         glEnd()
        
#         # Draw axes for this cube
#         glBegin(GL_LINES)
#         # X-axis (red)
#         glColor3f(1, 0, 0)
#         glVertex3f(0, 0, 0)
#         glVertex3f(1.5, 0, 0)
#         # Y-axis (green)
#         glColor3f(0, 1, 0)
#         glVertex3f(0, 0, 0)
#         glVertex3f(0, 1.5, 0)
#         # Z-axis (blue)
#         glColor3f(0, 0, 1)
#         glVertex3f(0, 0, 0)
#         glVertex3f(0, 0, 1.5)
#         glEnd()

#         glPopMatrix()

# class IMUData:
#     def __init__(self, sensor_id):
#         self.sensor_id = sensor_id
#         self.accel = {'x': 0, 'y': 0, 'z': 0}
#         self.gyro = {'x': 0, 'y': 0, 'z': 0}
#         self.mag = {'x': 0, 'y': 0, 'z': 0}
#         self.angles = {'yaw': 0, 'pitch': 0, 'roll': 0}
#         self.angles6 = {'yaw': 0, 'pitch': 0, 'roll': 0}
#         self.last_update = None
#         self.timestamps = deque(maxlen=100)
#         self.last_heartbeat = 0
#         self.heartbeat_timeout = 10  # seconds
#         self.connection_quality = 1.0  # 1.0 = perfect, 0.0 = poor

#     def update(self, data):
#         now = datetime.now()
#         self.timestamps.append(now)

#         # Handle heartbeat messages
#         if isinstance(data, dict) and data.get("type") == "heartbeat":
#             self.last_heartbeat = now.timestamp()
#             return
#         elif isinstance(data, dict) and data.get("type") == "heartbeat_ack":
#             return

#         # Handle optimized JSON format
#         if 'a' in data:  # New format
#             self.accel = {
#                 'x': data['a']['x'],
#                 'y': data['a']['y'],
#                 'z': data['a']['z']
#             }
#             self.gyro = {
#                 'x': data['g']['x'],
#                 'y': data['g']['y'],
#                 'z': data['g']['z']
#             }
#             self.mag = {
#                 'x': data['m']['x'],
#                 'y': data['m']['y'],
#                 'z': data['m']['z']
#             }
#             self.angles = {
#                 'yaw': data['e']['y'],
#                 'pitch': data['e']['p'],
#                 'roll': data['e']['r']
#             }
#             self.angles6 = {
#                 'yaw': data['e6']['y'],
#                 'pitch': data['e6']['p'],
#                 'roll': data['e6']['r']
#             }
#         else:  # Legacy format
#             self.accel = {
#                 'x': data['accelX'],
#                 'y': data['accelY'],
#                 'z': data['accelZ']
#             }
#             self.gyro = {
#                 'x': data['gyroX'],
#                 'y': data['gyroY'],
#                 'z': data['gyroZ']
#             }
#             self.mag = {
#                 'x': data['magX'],
#                 'y': data['magY'],
#                 'z': data['magZ']
#             }
#             self.angles = {
#                 'yaw': data['yaw'],
#                 'pitch': data['pitch'],
#                 'roll': data['roll']
#             }
#             self.angles6 = {
#                 'yaw': data['yaw6'],
#                 'pitch': data['pitch6'],
#                 'roll': data['roll6']
#             }
        
#         self.last_update = now

#     def check_connection_quality(self):
#         now = datetime.now().timestamp()
#         if self.last_heartbeat == 0:
#             return 0.0
        
#         time_since_heartbeat = now - self.last_heartbeat
#         if time_since_heartbeat > self.heartbeat_timeout:
#             return 0.0
        
#         # Calculate connection quality based on heartbeat timing
#         self.connection_quality = max(0.0, 1.0 - (time_since_heartbeat / self.heartbeat_timeout))
#         return self.connection_quality

#     def get_sample_rate(self):
#         if len(self.timestamps) < 2:
#             return 0.0
#         now = datetime.now()
#         one_sec_ago = now.timestamp() - 1.0
#         recent_samples = [ts for ts in self.timestamps if ts.timestamp() > one_sec_ago]
#         return len(recent_samples)

#     def print_data(self):
#         if self.last_update is None:
#             print(f"\nIMU {self.sensor_id}: No data received yet")
#             return

#         sample_rate = self.get_sample_rate()

#         print(f"\nIMU {self.sensor_id} Data (Last update: {self.last_update.strftime('%H:%M:%S.%f')[:-3]})")
#         print(f"Sample Rate: {sample_rate:.1f} samples/sec")
#         print("Accelerometer (g):")
#         print(f"  X: {self.accel['x']:8.3f}  Y: {self.accel['y']:8.3f}  Z: {self.accel['z']:8.3f}")
#         print("Gyroscope (deg/s):")
#         print(f"  X: {self.gyro['x']:8.3f}  Y: {self.gyro['y']:8.3f}  Z: {self.gyro['z']:8.3f}")
#         print("Magnetometer (µT):")
#         print(f"  X: {self.mag['x']:8.3f}  Y: {self.mag['y']:8.3f}  Z: {self.mag['z']:8.3f}")
#         print("Euler Angles (deg):")
#         print(f"  Yaw: {self.angles['yaw']:8.3f}  Pitch: {self.angles['pitch']:8.3f}  Roll: {self.angles['roll']:8.3f}")
#         print("Euler Angles (6-axis) (deg):")
#         print(f"  Yaw: {self.angles6['yaw']:8.3f}  Pitch: {self.angles6['pitch']:8.3f}  Roll: {self.angles6['roll']:8.3f}")


# # Function to create a dummy data source for testing visualization
# def create_dummy_data():
#     import math
#     import time
    
#     async def dummy_data_source(imu_data, frequency=10):
#         start_time = time.time()
#         while True:
#             elapsed = time.time() - start_time
#             # Create oscillating rotation angles
#             yaw = 45 * math.sin(elapsed * 0.5)
#             pitch = 30 * math.sin(elapsed * 0.7)
#             roll = 60 * math.sin(elapsed * 0.3)
            
#             # Create dummy data packet
#             data = {
#                 'accelX': math.sin(elapsed),
#                 'accelY': math.cos(elapsed),
#                 'accelZ': 1.0,
#                 'gyroX': math.sin(elapsed * 2) * 10,
#                 'gyroY': math.cos(elapsed * 2) * 10,
#                 'gyroZ': math.sin(elapsed * 3) * 10,
#                 'magX': 25 * math.sin(elapsed),
#                 'magY': 25 * math.cos(elapsed),
#                 'magZ': 25,
#                 'yaw': yaw,
#                 'pitch': pitch,
#                 'roll': roll,
#                 'yaw6': yaw * 0.8,
#                 'pitch6': pitch * 0.8,
#                 'roll6': roll * 0.8
#             }
            
#             imu_data.update(data)
#             await asyncio.sleep(1 / frequency)
    
#     return dummy_data_source


# async def connect_to_imu(uri, imu_data):
#     reconnect_delay = 1  # Start with 1 second delay
#     max_reconnect_delay = 30  # Maximum delay between reconnection attempts
#     consecutive_failures = 0
#     last_heartbeat_sent = 0
#     last_data_received = time.time()

#     while True:  # Infinite loop to keep reconnecting
#         try:
#             # Configure WebSocket for low latency and stability
#             async with websockets.connect(
#                 uri, 
#                 ping_interval=None,  # Disable automatic ping/pong
#                 ping_timeout=None,
#                 close_timeout=60,
#                 max_size=2**20,  # 1MB max message size
#                 max_queue=32,  # Limit message queue size
#                 compression=None  # Disable compression for lower latency
#             ) as websocket:
#                 print(f"Connected to {uri}")
#                 reconnect_delay = 1  # Reset delay on successful connection
#                 consecutive_failures = 0  # Reset failure counter
#                 last_heartbeat_sent = time.time()
#                 last_data_received = time.time()
                
#                 while True:
#                     try:
#                         # Send heartbeat every 2 seconds
#                         current_time = time.time()
#                         if current_time - last_heartbeat_sent >= 2:
#                             await websocket.send(json.dumps({"type": "heartbeat"}))
#                             last_heartbeat_sent = current_time

#                         # Set a timeout for receiving messages
#                         message = await asyncio.wait_for(websocket.recv(), timeout=5.0)
#                         last_data_received = time.time()
#                         data = json.loads(message)
                        
#                         # Handle connection confirmation
#                         if isinstance(data, dict) and data.get("type") == "connected":
#                             print(f"Received connection confirmation from {uri}")
#                             continue
                            
#                         imu_data.update(data)
#                     except asyncio.TimeoutError:
#                         # Check if we've received any data recently
#                         if time.time() - last_data_received > 10:  # No data for 10 seconds
#                             print(f"No data received from {uri} for 10 seconds, reconnecting...")
#                             break
#                         continue
#                     except json.JSONDecodeError:
#                         print(f"Error decoding JSON from {uri}, continuing...")
#                         await asyncio.sleep(0.1)
#                         continue
#                     except websockets.exceptions.ConnectionClosed as e:
#                         if e.code == 1000:  # Normal closure
#                             print(f"Connection to {uri} closed normally")
#                         else:
#                             print(f"Connection to {uri} closed unexpectedly: {e}")
#                         break
#                     except Exception as e:
#                         print(f"Error in websocket connection: {e}, continuing...")
#                         await asyncio.sleep(0.1)
#                         continue
        
#         except Exception as e:
#             consecutive_failures += 1
#             print(f"Error connecting to {uri}: {e}")
#             print(f"Attempting to reconnect in {reconnect_delay} seconds...")
            
#             # If we've had multiple failed connection attempts, use dummy data
#             if consecutive_failures > 5:
#                 print("Connection attempts failed, using dummy data source instead...")
#                 dummy_data_source = create_dummy_data()
#                 await dummy_data_source(imu_data)
#                 return
            
#             await asyncio.sleep(reconnect_delay)
#             # Exponential backoff with jitter for reconnect attempts
#             reconnect_delay = min(reconnect_delay * 1.5 + random.uniform(0, 1), max_reconnect_delay)


# # Separate visualization outside of asyncio for better performance
# def run_visualization(imu1_data, imu2_data):
#     # Initialize PyGame
#     pygame.init()
#     display = (800, 600)
#     pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
#     pygame.display.set_caption("IMU Visualization")
    
#     # Set up OpenGL
#     glEnable(GL_DEPTH_TEST)
#     glMatrixMode(GL_PROJECTION)
#     glLoadIdentity()
#     gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
#     glMatrixMode(GL_MODELVIEW)
#     glLoadIdentity()
#     glTranslatef(0.0, 0.0, -10)  # Move further back to see both cubes
    
#     # Create cubes for each IMU
#     cube1 = Cube(device_id=1, position=(-2.5, 0, 0))
#     cube2 = Cube(device_id=2, position=(2.5, 0, 0))
    
#     clock = pygame.time.Clock()
#     font = pygame.font.SysFont('Arial', 18)
    
#     running = True
#     while running:
#         for event in pygame.event.get():
#             if event.type == pygame.QUIT:
#                 running = False
#             if event.type == pygame.KEYDOWN:
#                 if event.key == pygame.K_ESCAPE:
#                     running = False
        
#         # Clear screen
#         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
#         glLoadIdentity()
#         glTranslatef(0.0, 0.0, -10)  # Reset view position
        
#         # Rotate view slightly for better perspective
#         glRotatef(20, 1, 0, 0)  # Rotate around X axis
        
#         # Draw world coordinate system
#         glBegin(GL_LINES)
#         # X-axis (red)
#         glColor3f(1, 0, 0)
#         glVertex3f(-5, 0, 0)
#         glVertex3f(5, 0, 0)
#         # Y-axis (green)
#         glColor3f(0, 1, 0)
#         glVertex3f(0, -5, 0)
#         glVertex3f(0, 5, 0)
#         # Z-axis (blue)
#         glColor3f(0, 0, 1)
#         glVertex3f(0, 0, -5)
#         glVertex3f(0, 0, 5)
#         glEnd()
        
#         # Update cube orientations from IMU data
#         if imu1_data.last_update:
#             cube1.set_orientation((imu1_data.angles['roll'], imu1_data.angles['pitch'], imu1_data.angles['yaw']))
#         if imu2_data.last_update:
#             cube2.set_orientation((imu2_data.angles['roll'], imu2_data.angles['pitch'], imu2_data.angles['yaw']))
        
#         # Draw cubes
#         cube1.draw()
#         cube2.draw()
        
#         # Add text info (would need to be done in 2D)
#         # Switch to 2D mode for text rendering
#         glMatrixMode(GL_PROJECTION)
#         glPushMatrix()
#         glLoadIdentity()
#         glOrtho(0, display[0], display[1], 0, -1, 1)
#         glMatrixMode(GL_MODELVIEW)
#         glPushMatrix()
#         glLoadIdentity()
        
#         # Disable depth test for text
#         glDisable(GL_DEPTH_TEST)
        
#         # Render text surfaces
#         def render_text(x, y, text, color=(255, 255, 255)):
#             text_surface = font.render(text, True, color)
#             text_data = pygame.image.tostring(text_surface, "RGBA", True)
#             text_width, text_height = text_surface.get_size()
#             glRasterPos2i(x, y)
#             glDrawPixels(text_width, text_height, GL_RGBA, GL_UNSIGNED_BYTE, text_data)
        
#         # Display IMU 1 info
#         if imu1_data.last_update:
#             render_text(10, 10, f"IMU 1 - Roll: {imu1_data.angles['roll']:.1f}, Pitch: {imu1_data.angles['pitch']:.1f}, Yaw: {imu1_data.angles['yaw']:.1f}")
#         else:
#             render_text(10, 10, "IMU 1 - No data")
        
#         # Display IMU 2 info
#         if imu2_data.last_update:
#             render_text(10, 30, f"IMU 2 - Roll: {imu2_data.angles['roll']:.1f}, Pitch: {imu2_data.angles['pitch']:.1f}, Yaw: {imu2_data.angles['yaw']:.1f}")
#         else:
#             render_text(10, 30, "IMU 2 - No data")
        
#         # Return to 3D mode
#         glEnable(GL_DEPTH_TEST)
#         glMatrixMode(GL_PROJECTION)
#         glPopMatrix()
#         glMatrixMode(GL_MODELVIEW)
#         glPopMatrix()
        
#         # Update display
#         pygame.display.flip()
#         clock.tick(60)  # Limit to 60 FPS
    
#     pygame.quit()
#     return False  # Signal to exit the main loop


# async def main():
#     parser = argparse.ArgumentParser(description='IMU WebSocket Test Client')
#     parser.add_argument('--ip', default='10.106.23.13', help='ESP32 IP address')
#     parser.add_argument('--port1', type=int, default=8080, help='Port for IMU 1 (LSM6DSOX)')
#     parser.add_argument('--port2', type=int, default=8081, help='Port for IMU 2 (ICM20948)')
#     parser.add_argument('--visual', action='store_true', help='Enable 3D visualization')
#     parser.add_argument('--demo', action='store_true', help='Use demo data without connecting to IMUs')
#     args = parser.parse_args()

#     # Create IMU data objects
#     imu1_data = IMUData(1)
#     imu2_data = IMUData(2)

#     # Create tasks list
#     tasks = []

#     # Add heartbeat task to keep the asyncio event loop running smoothly
#     async def heartbeat():
#         while True:
#             await asyncio.sleep(0.5)
    
#     tasks.append(heartbeat())

#     if args.demo:
#         # Use dummy data sources for both IMUs
#         dummy_data_source = create_dummy_data()
#         tasks.append(dummy_data_source(imu1_data, frequency=20))
#         tasks.append(dummy_data_source(imu2_data, frequency=20))
#     else:
#         # Create WebSocket URIs
#         uri1 = f"ws://{args.ip}:{args.port1}/ws"
#         uri2 = f"ws://{args.ip}:{args.port2}/ws"

#         print(f"Connecting to IMU 1 at {uri1}")
#         print(f"Connecting to IMU 2 at {uri2}")

#         # Start WebSocket connections with persistent reconnection handling
#         tasks.extend([
#             connect_to_imu(uri1, imu1_data),
#             connect_to_imu(uri2, imu2_data)
#         ])

#     # Start display task for console output
#     if not args.visual:
#         async def display_data():
#             while True:
#                 print("\033[2J\033[H", end="")  # Clear screen and move cursor to top

#                 any_data = False
#                 if imu1_data.last_update:
#                     imu1_data.print_data()
#                     any_data = True
#                 if imu2_data.last_update:
#                     imu2_data.print_data()
#                     any_data = True

#                 if not any_data:
#                     print("Waiting for data...")

#                 await asyncio.sleep(0.00001)  # Update at 10 Hz

#         tasks.append(display_data())

#     # Create a visualization controller task
#     if args.visual or args.demo:
#         # Run visualization in a separate thread
#         import threading
#         vis_thread = threading.Thread(target=run_visualization, args=(imu1_data, imu2_data))
#         vis_thread.daemon = True
#         vis_thread.start()

#     # Run all tasks concurrently
#     await asyncio.gather(*tasks)

# if __name__ == "__main__":
#     try:
#         # Add the --demo flag to test visualization without actual IMU hardware
#         # python readvis.py --visual --demo
#         asyncio.run(main())
#     except KeyboardInterrupt:
#         print("\nExiting...")
#     except Exception as e:
#         print(f"Unhandled exception: {e}")


#!/usr/bin/env python3
"""
IMU Visualization System - A modular application for IMU data processing and visualization.

This program connects to IMU sensors over WebSockets, processes orientation data,
and visualizes it in 3D using OpenGL. It features automatic reconnection,
fallback to demo data, and comprehensive error handling.
"""

import asyncio
import argparse
import json
import logging
import random
import threading
import time
from collections import deque
from datetime import datetime
from math import radians, sin, cos
from typing import Dict, List, Optional, Tuple, Any, Callable, Coroutine

import numpy as np
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from scipy.spatial.transform import Rotation as R
import websockets


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("IMU_Visualizer")


class IMUData:
    """Class for storing and processing IMU sensor data."""
    
    def __init__(self, sensor_id: int):
        """Initialize the IMU data structure.
        
        Args:
            sensor_id: Unique identifier for this IMU sensor
        """
        self.sensor_id = sensor_id
        self.accel = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.gyro = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.mag = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.angles = {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}
        self.angles6 = {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}
        self.last_update: Optional[datetime] = None
        self.timestamps = deque(maxlen=100)
        self.last_heartbeat = 0.0
        self.heartbeat_timeout = 10.0  # seconds
        self.connection_quality = 1.0  # 1.0 = perfect, 0.0 = poor

    def update(self, data: Dict[str, Any]) -> None:
        """Update IMU data with new sensor readings.
        
        Args:
            data: Dictionary containing the new IMU sensor data
        """
        now = datetime.now()
        self.timestamps.append(now)

        # Handle heartbeat messages
        if isinstance(data, dict) and data.get("type") == "heartbeat":
            self.last_heartbeat = now.timestamp()
            return
        elif isinstance(data, dict) and data.get("type") == "heartbeat_ack":
            return

        # Handle optimized JSON format
        if 'a' in data:  # New format
            self.accel = {
                'x': data['a']['x'],
                'y': data['a']['y'],
                'z': data['a']['z']
            }
            self.gyro = {
                'x': data['g']['x'],
                'y': data['g']['y'],
                'z': data['g']['z']
            }
            self.mag = {
                'x': data['m']['x'],
                'y': data['m']['y'],
                'z': data['m']['z']
            }
            self.angles = {
                'yaw': data['e']['y'],
                'pitch': data['e']['p'],
                'roll': data['e']['r']
            }
            self.angles6 = {
                'yaw': data['e6']['y'],
                'pitch': data['e6']['p'],
                'roll': data['e6']['r']
            }
        else:  # Legacy format
            self.accel = {
                'x': data['accelX'],
                'y': data['accelY'],
                'z': data['accelZ']
            }
            self.gyro = {
                'x': data['gyroX'],
                'y': data['gyroY'],
                'z': data['gyroZ']
            }
            self.mag = {
                'x': data['magX'],
                'y': data['magY'],
                'z': data['magZ']
            }
            self.angles = {
                'yaw': data['yaw'],
                'pitch': data['pitch'],
                'roll': data['roll']
            }
            self.angles6 = {
                'yaw': data['yaw6'],
                'pitch': data['pitch6'],
                'roll': data['roll6']
            }
        
        self.last_update = now

    def check_connection_quality(self) -> float:
        """Calculate and return connection quality based on heartbeat timing.
        
        Returns:
            float: Connection quality (0.0-1.0)
        """
        now = datetime.now().timestamp()
        if self.last_heartbeat == 0:
            return 0.0
        
        time_since_heartbeat = now - self.last_heartbeat
        if time_since_heartbeat > self.heartbeat_timeout:
            return 0.0
        
        # Calculate connection quality based on heartbeat timing
        self.connection_quality = max(0.0, 1.0 - (time_since_heartbeat / self.heartbeat_timeout))
        return self.connection_quality

    def get_sample_rate(self) -> float:
        """Calculate the current sample rate in samples per second.
        
        Returns:
            float: Current sample rate in Hz
        """
        if len(self.timestamps) < 2:
            return 0.0
        now = datetime.now()
        one_sec_ago = now.timestamp() - 1.0
        recent_samples = [ts for ts in self.timestamps if ts.timestamp() > one_sec_ago]
        return len(recent_samples)

    def get_orientation_rpy(self) -> Tuple[float, float, float]:
        """Get current orientation as roll, pitch, yaw values.
        
        Returns:
            Tuple[float, float, float]: Roll, pitch, yaw values in degrees
        """
        return (
            self.angles['roll'],
            self.angles['pitch'],
            self.angles['yaw']
        )
    
    def get_status_summary(self) -> Dict[str, Any]:
        """Get a summary of the IMU status for display.
        
        Returns:
            Dict: Status information including sample rate, connection quality, etc.
        """
        return {
            'id': self.sensor_id,
            'sample_rate': self.get_sample_rate(),
            'connection_quality': self.check_connection_quality(),
            'has_data': self.last_update is not None,
            'angles': self.angles,
            'angles6': self.angles6,
        }

    def __str__(self) -> str:
        """Generate a readable string representation of the IMU data.
        
        Returns:
            str: Formatted IMU data string
        """
        if self.last_update is None:
            return f"IMU {self.sensor_id}: No data received yet"

        sample_rate = self.get_sample_rate()
        
        lines = [
            f"\nIMU {self.sensor_id} Data (Last update: {self.last_update.strftime('%H:%M:%S.%f')[:-3]})",
            f"Sample Rate: {sample_rate:.1f} samples/sec",
            "Accelerometer (g):",
            f"  X: {self.accel['x']:8.3f}  Y: {self.accel['y']:8.3f}  Z: {self.accel['z']:8.3f}",
            "Gyroscope (deg/s):",
            f"  X: {self.gyro['x']:8.3f}  Y: {self.gyro['y']:8.3f}  Z: {self.gyro['z']:8.3f}",
            "Magnetometer (µT):",
            f"  X: {self.mag['x']:8.3f}  Y: {self.mag['y']:8.3f}  Z: {self.mag['z']:8.3f}",
            "Euler Angles (deg):",
            f"  Yaw: {self.angles['yaw']:8.3f}  Pitch: {self.angles['pitch']:8.3f}  Roll: {self.angles['roll']:8.3f}",
            "Euler Angles (6-axis) (deg):",
            f"  Yaw: {self.angles6['yaw']:8.3f}  Pitch: {self.angles6['pitch']:8.3f}  Roll: {self.angles6['roll']:8.3f}"
        ]
        
        return "\n".join(lines)


class CubeModel:
    """3D model of a cube with colored faces for IMU visualization."""
    
    # Define cube vertices, faces, and colors as class constants
    VERTICES = [
        (-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
        (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)
    ]

    FACES = [
        (0, 1, 2, 3),  # Back face
        (4, 5, 6, 7),  # Front face
        (0, 3, 7, 4),  # Left face
        (1, 2, 6, 5),  # Right face
        (0, 1, 5, 4),  # Bottom face
        (3, 2, 6, 7)   # Top face
    ]

    COLORS = [
        (0.5, 0, 0),    # Red
        (0, 0.5, 0),    # Green
        (0, 0, 0.5),    # Blue
        (0.5, 0.5, 0),  # Yellow
        (0.5, 0, 0.5),  # Magenta
        (0, 0.5, 0.5)   # Cyan
    ]
    
    def __init__(self, device_id: int, position: Tuple[float, float, float] = (0, 0, 0), size: float = 1.0):
        """Initialize the cube model.
        
        Args:
            device_id: Unique identifier for this cube
            position: 3D position (x, y, z) of the cube
            size: Scale factor for the cube size
        """
        self.device_id = device_id
        self.position = position
        self.size = size
        self.orientation = np.identity(3)  # 3x3 rotation matrix
        
    def set_orientation(self, rpy: Tuple[float, float, float]) -> None:
        """Set orientation from roll, pitch, yaw in degrees.
        
        Args:
            rpy: Tuple of (roll, pitch, yaw) in degrees
        """
        roll, pitch, yaw = rpy  # Assumes input in degrees
        # Convert to rotation matrix using scipy
        rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
        self.orientation = rotation.as_matrix()

    def draw(self) -> None:
        """Draw the cube with its current position and orientation."""
        glPushMatrix()
        glTranslatef(*self.position)

        # Convert rotation matrix to 4x4 and apply it
        rotation_matrix_4x4 = np.identity(4)
        rotation_matrix_4x4[:3, :3] = self.orientation
        glMultMatrixf(rotation_matrix_4x4.T)  # Transpose because OpenGL uses column-major order

        # Scale the cube by size
        glScalef(self.size, self.size, self.size)
        
        # Draw cube
        glBegin(GL_QUADS)
        for i, face in enumerate(self.FACES):
            glColor3fv(self.COLORS[i])
            for vertex in face:
                glVertex3fv(self.VERTICES[vertex])
        glEnd()
        
        # Draw axes for this cube
        glBegin(GL_LINES)
        # X-axis (red)
        glColor3f(1, 0, 0)
        glVertex3f(0, 0, 0)
        glVertex3f(1.5, 0, 0)
        # Y-axis (green)
        glColor3f(0, 1, 0)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 1.5, 0)
        # Z-axis (blue)
        glColor3f(0, 0, 1)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, 1.5)
        glEnd()

        glPopMatrix()


class IMUConnector:
    """Manages WebSocket connections to IMU sensors."""
    
    def __init__(self, uri: str, imu_data: IMUData):
        """Initialize the connector.
        
        Args:
            uri: WebSocket URI to connect to
            imu_data: IMUData instance to update with received data
        """
        self.uri = uri
        self.imu_data = imu_data
        self.reconnect_delay = 1.0  # Start with 1 second delay
        self.max_reconnect_delay = 30.0  # Maximum delay between reconnection attempts
        self.consecutive_failures = 0
        self.last_heartbeat_sent = 0.0
        self.last_data_received = time.time()
        
    async def run(self) -> None:
        """Main connection loop that handles reconnection and data processing."""
        while True:  # Infinite loop to keep reconnecting
            try:
                # Configure WebSocket for low latency and stability
                async with websockets.connect(
                    self.uri, 
                    ping_interval=None,  # Disable automatic ping/pong
                    ping_timeout=None,
                    close_timeout=60,
                    max_size=2**20,  # 1MB max message size
                    max_queue=32,  # Limit message queue size
                    compression=None  # Disable compression for lower latency
                ) as websocket:
                    logger.info(f"Connected to IMU at {self.uri}")
                    self.reconnect_delay = 1.0  # Reset delay on successful connection
                    self.consecutive_failures = 0  # Reset failure counter
                    self.last_heartbeat_sent = time.time()
                    self.last_data_received = time.time()
                    
                    while True:
                        try:
                            # Send heartbeat every 2 seconds
                            current_time = time.time()
                            if current_time - self.last_heartbeat_sent >= 2:
                                await websocket.send(json.dumps({"type": "heartbeat"}))
                                self.last_heartbeat_sent = current_time

                            # Set a timeout for receiving messages
                            message = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                            self.last_data_received = time.time()
                            data = json.loads(message)
                            
                            # Handle connection confirmation
                            if isinstance(data, dict) and data.get("type") == "connected":
                                logger.info(f"Received connection confirmation from {self.uri}")
                                continue
                                
                            self.imu_data.update(data)
                        except asyncio.TimeoutError:
                            # Check if we've received any data recently
                            if time.time() - self.last_data_received > 10:  # No data for 10 seconds
                                logger.warning(f"No data received from {self.uri} for 10 seconds, reconnecting...")
                                break
                            continue
                        except json.JSONDecodeError:
                            logger.warning(f"Error decoding JSON from {self.uri}, continuing...")
                            await asyncio.sleep(0.1)
                            continue
                        except websockets.exceptions.ConnectionClosed as e:
                            if e.code == 1000:  # Normal closure
                                logger.info(f"Connection to {self.uri} closed normally")
                            else:
                                logger.warning(f"Connection to {self.uri} closed unexpectedly: {e}")
                            break
                        except Exception as e:
                            logger.error(f"Error in websocket connection: {e}")
                            await asyncio.sleep(0.1)
                            continue
            
            except Exception as e:
                self.consecutive_failures += 1
                logger.error(f"Error connecting to {self.uri}: {e}")
                logger.info(f"Attempting to reconnect in {self.reconnect_delay:.1f} seconds...")
                
                # If we've had multiple failed connection attempts, switch to fallback
                if self.consecutive_failures > 5:
                    logger.warning(f"Multiple connection failures to {self.uri}, switching to fallback mode")
                    return
                
                await asyncio.sleep(self.reconnect_delay)
                # Exponential backoff with jitter for reconnect attempts
                self.reconnect_delay = min(self.reconnect_delay * 1.5 + random.uniform(0, 1), self.max_reconnect_delay)


class DummyDataGenerator:
    """Generates simulated IMU data for testing or when hardware is unavailable."""
    
    def __init__(self, imu_data: IMUData, frequency: float = 20.0):
        """Initialize the dummy data generator.
        
        Args:
            imu_data: IMUData instance to update with simulated data
            frequency: Update frequency in Hz
        """
        self.imu_data = imu_data
        self.frequency = frequency
        self.start_time = time.time()
        
    async def run(self) -> None:
        """Run the dummy data generator."""
        logger.info(f"Starting dummy data generator for IMU {self.imu_data.sensor_id} at {self.frequency} Hz")
        
        while True:
            elapsed = time.time() - self.start_time
            
            # Create oscillating rotation angles
            yaw = 45 * sin(elapsed * 0.5)
            pitch = 30 * sin(elapsed * 0.7)
            roll = 60 * sin(elapsed * 0.3)
            
            # Create dummy data packet
            data = {
                'accelX': sin(elapsed),
                'accelY': cos(elapsed),
                'accelZ': 1.0,
                'gyroX': sin(elapsed * 2) * 10,
                'gyroY': cos(elapsed * 2) * 10,
                'gyroZ': sin(elapsed * 3) * 10,
                'magX': 25 * sin(elapsed),
                'magY': 25 * cos(elapsed),
                'magZ': 25,
                'yaw': yaw,
                'pitch': pitch,
                'roll': roll,
                'yaw6': yaw * 0.8,
                'pitch6': pitch * 0.8,
                'roll6': roll * 0.8
            }
            
            self.imu_data.update(data)
            await asyncio.sleep(1 / self.frequency)


class ConsoleView:
    """Displays IMU data in the console."""
    
    def __init__(self, imu_data_list: List[IMUData]):
        """Initialize the console view.
        
        Args:
            imu_data_list: List of IMUData instances to display
        """
        self.imu_data_list = imu_data_list
    
    async def run(self) -> None:
        """Run the console view update loop."""
        while True:
            # Clear screen (works on most terminals)
            print("\033[2J\033[H", end="")
            
            any_data = False
            for imu_data in self.imu_data_list:
                if imu_data.last_update:
                    print(imu_data)
                    any_data = True
            
            if not any_data:
                print("Waiting for data from any IMU...")
            
            await asyncio.sleep(0.1)  # Update at 10 Hz


class OpenGLVisualizer:
    """3D visualization of IMU orientation data using OpenGL."""
    
    def __init__(self, imu_data_list: List[IMUData], width: int = 800, height: int = 600):
        """Initialize the OpenGL visualizer.
        
        Args:
            imu_data_list: List of IMUData instances to visualize
            width: Width of the visualization window
            height: Height of the visualization window
        """
        self.imu_data_list = imu_data_list
        self.width = width
        self.height = height
        self.running = False
        self.cubes = []
        
        # Calculate positions for multiple IMUs
        spacing = 5.0 / max(len(imu_data_list), 1)
        for i, imu_data in enumerate(imu_data_list):
            position = (-2.5 + i * spacing, 0, 0)
            self.cubes.append(CubeModel(device_id=imu_data.sensor_id, position=position))
    
    def setup_opengl(self) -> None:
        """Set up OpenGL rendering context."""
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, (self.width / self.height), 0.1, 50.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -10)
    
    def draw_world_axes(self) -> None:
        """Draw the world coordinate system axes."""
        glBegin(GL_LINES)
        # X-axis (red)
        glColor3f(1, 0, 0)
        glVertex3f(-5, 0, 0)
        glVertex3f(5, 0, 0)
        # Y-axis (green)
        glColor3f(0, 1, 0)
        glVertex3f(0, -5, 0)
        glVertex3f(0, 5, 0)
        # Z-axis (blue)
        glColor3f(0, 0, 1)
        glVertex3f(0, 0, -5)
        glVertex3f(0, 0, 5)
        glEnd()
    
    def render_text(self, x: int, y: int, text: str, font, color: Tuple[int, int, int] = (255, 255, 255)) -> None:
        """Render text on the OpenGL surface.
        
        Args:
            x: X position in screen coordinates
            y: Y position in screen coordinates
            text: Text to render
            font: PyGame font to use
            color: RGB color tuple
        """
        text_surface = font.render(text, True, color)
        text_data = pygame.image.tostring(text_surface, "RGBA", True)
        text_width, text_height = text_surface.get_size()
        glRasterPos2i(x, y)
        glDrawPixels(text_width, text_height, GL_RGBA, GL_UNSIGNED_BYTE, text_data)
    
    def run(self) -> None:
        """Run the visualization loop."""
        # Initialize PyGame
        pygame.init()
        display = (self.width, self.height)
        pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
        pygame.display.set_caption("IMU Visualization")
        
        # Set up OpenGL
        self.setup_opengl()
        
        clock = pygame.time.Clock()
        font = pygame.font.SysFont('Arial', 18)
        
        self.running = True
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self.running = False
            
            # Clear screen
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glLoadIdentity()
            glTranslatef(0.0, 0.0, -10)  # Reset view position
            
            # Rotate view slightly for better perspective
            glRotatef(20, 1, 0, 0)  # Rotate around X axis
            
            # Draw world coordinate system
            self.draw_world_axes()
            
            # Update and draw each cube with its IMU data
            for i, (cube, imu_data) in enumerate(zip(self.cubes, self.imu_data_list)):
                if imu_data.last_update:
                    cube.set_orientation(imu_data.get_orientation_rpy())
                cube.draw()
            
            # Switch to 2D mode for text rendering
            glMatrixMode(GL_PROJECTION)
            glPushMatrix()
            glLoadIdentity()
            glOrtho(0, display[0], display[1], 0, -1, 1)
            glMatrixMode(GL_MODELVIEW)
            glPushMatrix()
            glLoadIdentity()
            
            # Disable depth test for text
            glDisable(GL_DEPTH_TEST)
            
            # Display IMU info
            y_offset = 10
            for i, imu_data in enumerate(self.imu_data_list):
                if imu_data.last_update:
                    text = f"IMU {imu_data.sensor_id} - Roll: {imu_data.angles['roll']:.1f}, "
                    text += f"Pitch: {imu_data.angles['pitch']:.1f}, Yaw: {imu_data.angles['yaw']:.1f}"
                    self.render_text(10, y_offset, text, font)
                else:
                    self.render_text(10, y_offset, f"IMU {imu_data.sensor_id} - No data", font)
                y_offset += 20
            
            # Return to 3D mode
            glEnable(GL_DEPTH_TEST)
            glMatrixMode(GL_PROJECTION)
            glPopMatrix()
            glMatrixMode(GL_MODELVIEW)
            glPopMatrix()
            
            # Update display
            pygame.display.flip()
            clock.tick(60)  # Limit to 60 FPS
        
        pygame.quit()


class IMUVisualizationSystem:
    """Main application class that coordinates IMU data collection and visualization."""
    
    def __init__(self):
        """Initialize the IMU visualization system."""
        self.imu_data_list = []
        self.tasks = []
        self.visualizer = None
        self.console_view = None
    
    def parse_arguments(self) -> argparse.Namespace:
        """Parse command line arguments.
        
        Returns:
            argparse.Namespace: Parsed arguments
        """
        parser = argparse.ArgumentParser(description='IMU WebSocket Test Client')
        parser.add_argument('--ip', default='10.106.23.138', help='ESP32 IP address')
        parser.add_argument('--port1', type=int, default=8080, help='Port for IMU 1 (LSM6DSOX)')
        parser.add_argument('--port2', type=int, default=8081, help='Port for IMU 2 (ICM20948)')
        parser.add_argument('--visual', action='store_true', help='Enable 3D visualization')
        parser.add_argument('--demo', action='store_true', help='Use demo data without connecting to IMUs')
        parser.add_argument('--debug', action='store_true', help='Enable debug logging')
        return parser.parse_args()
    
    def setup(self, args: argparse.Namespace) -> None:
        """Set up the system based on command line arguments.
        
        Args:
            args: Parsed command line arguments
        """
        # Configure logging level
        if args.debug:
            logger.setLevel(logging.DEBUG)
        
        # Create IMU data objects
        self.imu_data_list = [IMUData(1), IMUData(2)]
        
        # Add heartbeat task to keep the asyncio event loop running smoothly
        async def heartbeat():
            while True:
                await asyncio.sleep(0.5)
        
        self.tasks.append(heartbeat())
        
        # Configure data sources based on arguments
        if args.demo:
            # Use dummy data sources for both IMUs
            for i, imu_data in enumerate(self.imu_data_list):
                generator = DummyDataGenerator(imu_data, frequency=20)
                self.tasks.append(generator.run())
                logger.info(f"Using dummy data for IMU {i+1}")
        else:
            # Create WebSocket URIs
            uris = [
                f"ws://{args.ip}:{args.port1}/ws",
                f"ws://{args.ip}:{args.port2}/ws"
            ]

            # Start WebSocket connections
            for i, (uri, imu_data) in enumerate(zip(uris, self.imu_data_list)):
                logger.info(f"Connecting to IMU {i+1} at {uri}")
                connector = IMUConnector(uri, imu_data)
                self.tasks.append(connector.run())
        
        # Configure visualization based on arguments
        if args.visual or args.demo:
            self.visualizer = OpenGLVisualizer(self.imu_data_list)
        else:
            self.console_view = ConsoleView(self.imu_data_list)
            self.tasks.append(self.console_view.run())
    
    async def run_async(self) -> None:
        """Run the async tasks of the system."""
        # Start visualization in a separate thread if configured
        if self.visualizer:
            vis_thread = threading.Thread(target=self.visualizer.run)
            vis_thread.daemon = True
            vis_thread.start()
        
        # Run all async tasks
        try:
            await asyncio.gather(*self.tasks)
        except asyncio.CancelledError:
            logger.info("Tasks cancelled")
    
    def run(self) -> None:
        """Run the complete IMU visualization system."""
        args = self.parse_arguments()
        self.setup(args)
        
        try:
            asyncio.run(self.run_async())
        except KeyboardInterrupt:
            logger.info("Application terminated by user")
        except Exception as e:
            logger.exception(f"Unhandled exception: {e}")


def main():
    """Entry point for the IMU visualization application."""
    system = IMUVisualizationSystem()
    system.run()


if __name__ == "__main__":
    main()
    main()