#!/usr/bin/env python3
import sys
import glob
import rclpy
from rclpy.node import Node
import subprocess

class FindEsp32:
    def __init__(self):
        self.pyudev = self.install_pyudev()
        self.VENDOR_ID = "10c4"
        self.MODEL_ID = "CP2102_USB_to_UART_Bridge_Controller"
        self.esp32_devices = []
        self.ttyusb_devices = glob.glob('/dev/ttyUSB*')
        self.context = self.pyudev.Context() if self.pyudev else None
        self.find_esp32_devices()

    def find_esp32_devices(self)->str:
        if not self.pyudev or not self.context:
            return " "
        if not self.ttyusb_devices:
            print("No /dev/ttyUSB* devices found.")
        else:
            for device_path in self.ttyusb_devices:
                try:
                    device = self.pyudev.Devices.from_device_file(self.context, device_path)
                    vendor_id = device.get('ID_VENDOR_ID')
                    MODEL = device.get('ID_MODEL')
                    if vendor_id == self.VENDOR_ID and MODEL == self.MODEL_ID:
                        self.esp32_devices.append(device_path)
                except self.pyudev.DeviceNotFoundError:
                    pass

        if not self.esp32_devices:
            return " "
        else:
            for dev in self.esp32_devices:
                return dev

    def install_pyudev(self):
        try:
            import pyudev
            return pyudev
        except ImportError:
            print("pyudev not found. Please install it (e.g., 'sudo apt install python3-pyudev') and rerun.")
            return None

class MicroRosAgentLauncher(Node):
    def __init__(self):
        super().__init__('micro_ros_agent_launcher')
        # Declare and get parameters
        find = FindEsp32()

        if len(find.esp32_devices) > 0:
            self.esp32_device = find.esp32_devices[0]
        else:
            self.esp32_device = " "

        self.declare_parameter('baud_rate', 115200)
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.launch_micro_ros_agent()

    def launch_micro_ros_agent(self):
        if self.esp32_device != " ":
            command = [
                "sudo","docker", "run", "--rm",
                "--net=host",
                f"--device={self.esp32_device}",
                "microros/micro-ros-agent:jazzy",
                "serial", "--dev", self.esp32_device , "-b", str(self.baud_rate)
            ]
            self.get_logger().info(f"Executing command: {' '.join(command)}")
        else:
            command = ["echo" ,"."]
            self.get_logger().warn(f"No ESP32 devices found. Skipp.ing Micro-ROS agent launch")


        try:
            subprocess.Popen(command)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Command failed with error: {e}")

def remove_all_docker_containers():
    try:
        # Get all container IDs
        container_ids = subprocess.check_output(
            ["sudo", "docker", "ps", "-a", "-q"], text=True
        ).strip()
        
        if container_ids:
            # Remove all containers
            subprocess.Popen(
                ["sudo", "docker", "rm", "-f"] + container_ids.split(),
            )
            print("All containers have been removed successfully.")
        else:
            print("No containers to remove.")
    except subprocess.CalledProcessError as e:
        print(f"An error occurred: {e}")

def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = MicroRosAgentLauncher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Shutting down node.')
    finally:
        remove_all_docker_containers()
        if node:
            node.destroy_node()
        #kill the docker container
        rclpy.shutdown()

if __name__ == '__main__':
    main()