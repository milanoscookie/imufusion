import websockets
import json
from datetime import datetime
import argparse
from collections import deque
import asyncio

class IMUData:
    def __init__(self, sensor_id):
        self.sensor_id = sensor_id
        self.accel = {'x': 0, 'y': 0, 'z': 0}
        self.gyro = {'x': 0, 'y': 0, 'z': 0}
        self.mag = {'x': 0, 'y': 0, 'z': 0}
        self.angles = {'yaw': 0, 'pitch': 0, 'roll': 0}
        self.angles6 = {'yaw': 0, 'pitch': 0, 'roll': 0}
        self.last_update = None
        self.timestamps = deque(maxlen=100)  # Store recent timestamps to calculate samples/sec

    def update(self, data):
        now = datetime.now()
        self.timestamps.append(now)

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

    def get_sample_rate(self):
        if len(self.timestamps) < 2:
            return 0.0
        now = datetime.now()
        one_sec_ago = now.timestamp() - 1.0
        recent_samples = [ts for ts in self.timestamps if ts.timestamp() > one_sec_ago]
        return len(recent_samples)

    def print_data(self):
        if self.last_update is None:
            print(f"\nIMU {self.sensor_id}: No data received yet")
            return

        sample_rate = self.get_sample_rate()

        print(f"\nIMU {self.sensor_id} Data (Last update: {self.last_update.strftime('%H:%M:%S.%f')[:-3]})")
        print(f"Sample Rate: {sample_rate:.1f} samples/sec")
        print("Accelerometer (g):")
        print(f"  X: {self.accel['x']:8.3f}  Y: {self.accel['y']:8.3f}  Z: {self.accel['z']:8.3f}")
        print("Gyroscope (deg/s):")
        print(f"  X: {self.gyro['x']:8.3f}  Y: {self.gyro['y']:8.3f}  Z: {self.gyro['z']:8.3f}")
        print("Magnetometer (µT):")
        print(f"  X: {self.mag['x']:8.3f}  Y: {self.mag['y']:8.3f}  Z: {self.mag['z']:8.3f}")
        print("Euler Angles (deg):")
        print(f"  Yaw: {self.angles['yaw']:8.3f}  Pitch: {self.angles['pitch']:8.3f}  Roll: {self.angles['roll']:8.3f}")
        print("Euler Angles (6-axis) (deg):")
        print(f"  Yaw: {self.angles6['yaw']:8.3f}  Pitch: {self.angles6['pitch']:8.3f}  Roll: {self.angles6['roll']:8.3f}")


async def connect_to_imu(uri, imu_data):
    try:
        async with websockets.connect(uri) as websocket:
            print(f"Connected to {uri}")
            while True:
                try:
                    message = await websocket.recv()
                    data = json.loads(message)
                    imu_data.update(data)
                except json.JSONDecodeError:
                    print(f"Error decoding JSON from {uri}")
                except websockets.exceptions.ConnectionClosed:
                    print(f"Connection to {uri} closed")
                    break
    except Exception as e:
        print(f"Error connecting to {uri}: {e}")

async def main():
    parser = argparse.ArgumentParser(description='IMU WebSocket Test Client')
    parser.add_argument('--ip', default='10.0.0.190', help='ESP32 IP address')
    parser.add_argument('--port1', type=int, default=8080, help='Port for IMU 1 (LSM6DSOX)')
    parser.add_argument('--port2', type=int, default=8081, help='Port for IMU 2 (ICM20948)')
    args = parser.parse_args()

    # Create IMU data objects
    imu1_data = IMUData(1)
    imu2_data = IMUData(2)

    # Create WebSocket URIs
    uri1 = f"ws://{args.ip}:{args.port1}/ws"
    uri2 = f"ws://{args.ip}:{args.port2}/ws"

    # Start WebSocket connections
    tasks = [
        connect_to_imu(uri1, imu1_data),
        connect_to_imu(uri2, imu2_data)
    ]

    # Start display task
    async def display_data():
        while True:
            print("\033[2J\033[H", end="")  # Clear screen and move cursor to top

            any_data = False
            if imu1_data.last_update:
                imu1_data.print_data()
                any_data = True
            if imu2_data.last_update:
                imu2_data.print_data()
                any_data = True

            if not any_data:
                print("Waiting for data...")

            await asyncio.sleep(0.00001)  

    tasks.append(display_data())

    # Run all tasks concurrently
    await asyncio.gather(*tasks)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nExiting...")
