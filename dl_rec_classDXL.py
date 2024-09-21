import os
import re
import serial
import serial.tools.list_ports
from dynamixel_sdk import *
from typing import List, Optional, Union, Any
from enum import Enum

import time
import ctypes
from graphics.plot import Plotter
from threading import Thread, Event
import random
from rich import print as rprint
from PyQt5.QtWidgets import QApplication, QWidget, QHBoxLayout, QVBoxLayout, QRadioButton, QPushButton, QListWidget, QListWidgetItem, QSpacerItem, QSizePolicy, QLabel, QFileDialog
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt
import wave, struct

class config_dev():

    def __init__(self) -> None:
        self.devices_list = []

    def get_device_info(self):
        """Method to get device info from user input with proper validation."""

        # Validate device ID
        while True:
            try:
                user_dxl_id = int(input("Enter device ID (1-254), if dont know - 0: "))
                if 1 <= user_dxl_id < 255:  # Device ID must be within valid range
                    dxl_id = user_dxl_id
                    break
                elif user_dxl_id == 0:
                    dxl_id = None
                    break
                else:
                    print("Invalid input! Device ID must be between 1 and 254.")
            except ValueError:
                print("Invalid input! Please enter a valid integer for device ID.")

        # Validate baudrate input
        while True:
            try:
                user_dxl_bdrate = int(input("Enter baudrate (e.g., 57600), if dont know - 0: "))
                if user_dxl_bdrate > 0:
                    baudrate = user_dxl_bdrate
                    break
                elif user_dxl_bdrate == 0:
                    baudrate = None
                    break
                else:
                    print("Invalid input! Baudrate must be a positive integer.")
            except ValueError:
                print("Invalid input! Please enter a valid integer for baudrate.")
        
        # Validate protocol version
        while True:
            try:
                user_protocol_v = float(input("Enter protocol version (e.g., 2.0), if dont know - 0: "))
                if user_protocol_v == 1.0 or user_protocol_v == 2.0:
                    protocol_version = user_protocol_v
                    break
                elif user_protocol_v == 0.0:
                    protocol_version = None
                    break
                else:
                    print("Invalid input! Protocol version must be a positive number.")
            except ValueError:
                print("Invalid input! Please enter a valid number for protocol version.")
        
        # Input for COM port
        while True:
            try:
                user_port = int(input("Enter COM port only last 4 digits (e.g., 0001): "))
                if 0 < user_port < 9999:
                    formatted_port = str(user_port).zfill(4)
                    port = '/dev/cu.usbserial-{}'.format(formatted_port)
                    break
                elif user_port == 0:
                    port = None
                    break
                else:
                    print("Invalid input! Protocol version must be a positive number.")
            except ValueError:
                print("Invalid input! Please enter a valid number for protocol version.")
    

        device = {
        "dxl_id": dxl_id,
        "baudrate": baudrate,
        "protocol_version": protocol_version,
        "port": port
        }

        self.devices_list.append(device)

    def __call__(self, *args: Any, **kwds: Any) -> Any:
        return self.devices_list

# atleast ask for two and more device not need, then work 
class DXL_device_list():

    def __init__(self, dxl_id=None, baudrate=None, protocol_version=None, device_list=None, port_timeout=100, **kwargs) -> None:
        """
        Initialize the DXL_device instance with a list of devices.
        
        Parameters:
            device_list (list of dict): List of dictionaries where each dictionary contains
                                         device information such as 'dxl_id', 'port', 'baudrate', and 'protocol_version'.
        """
        self.device_id = dxl_id
        self.baudrate = baudrate
        self.protocol_version = protocol_version
        self.port_timeout = port_timeout
        self.port_handler = None
        self.packet_handler = None

        self.devices = device_list if device_list is not None else []

        # add attr
        for key, value in kwargs.items():
            setattr(self, key, value)

    def _parser_param_list(self):
        for device_info in self.devices:
            # Update attributes if they are present in the dictionary
            if 'dxl_id' in device_info:
                self.device_id = device_info['dxl_id']
            if 'baudrate' in device_info:
                self.baudrate = device_info['baudrate']
            if 'protocol_version' in device_info:
                self.protocol_version = device_info['protocol_version']
            if 'port' in device_info:
                self.port_handler = PortHandler(device_info['port'])
            print(f"Updated with: {device_info}")

    def detect_device(self):
        """Attempts to detect the ID of the connected device."""
        for i in range(2):
            outping_data, dxl_comm_result, dxl_error = self.packet_handler.ping(self.port_handler, self.device_id)
            if dxl_comm_result == COMM_SUCCESS:
                print(f"Device detected! ID: {self.device_id}")
                return outping_data
        return None
    
    def _scan_ports(self):
        """
        Scan all available COM ports and return a list of matching ports.
        
        Returns:
            list of str: List of serial port names.
        """
        pattern = re.compile(r'^/dev/cu\.usbserial.*')
        matching_ports = [port.device for port in serial.tools.list_ports.comports() if pattern.match(port.device)]
        return sorted(matching_ports)
    
    def _try_connection(self, port, baudrate_option, protocol_option):
        """
        Attempt to connect to a device on the given port with the specified baudrate and protocol version.
        
        Parameters:
            port (str): COM port.
            baudrate_option (int): Baudrate to try.
            protocol_option (float): Protocol version to try.
        
        Returns:
            bool: True if connection is successful, otherwise False.
        """
        try:
            ser = serial.Serial(port=port, baudrate=baudrate_option, timeout=self.port_timeout)
            self.port_handler = PortHandler(ser.name)
            self.packet_handler = PacketHandler(protocol_option)
            self.port_handler.setBaudRate(baudrate_option)
            self.port_handler.openPort()

            if self.device_id:
                if self._detect_device():
                    self.baudrate = baudrate_option
                    self.protocol_version = protocol_option
                    return True

            # Try known IDs and full ID range if needed
            if self._try_known_and_full_id_range():
                self.baudrate = baudrate_option
                self.protocol_version = protocol_option
                return True

        except (serial.SerialException, Exception) as e:
            print(f"Error: {e}")
        finally:
            if self.port_handler:
                self.port_handler.closePort()
                
        return False

    def _try_known_and_full_id_range(self):
        """
        Attempt to find a device by trying known IDs and then a full range of IDs.
        
        Returns:
            bool: True if a device is found, otherwise False.
        """
        possible_ids = [171]  # Known IDs to try first

        for dxl_id_option in possible_ids:
            self.device_id = dxl_id_option
            if self._detect_device():
                possible_ids.append(self.device_id)
                return True
            
        # remaining_ids = set(range(1, 255)) - set(possible_ids)

        for dxl_id_option in set(range(1, 255)) - set(possible_ids):
            self.device_id = dxl_id_option
            if self._detect_device():
                possible_ids.append(self.device_id)
                return True

        return False
    
    def _detect_device(self):
        """
        Detect if a device with the current ID is connected.
        
        Returns:
            bool: True if device is detected, otherwise False.
        """
        # Implement the logic to detect the device
        return True  # Placeholder implementation

    def connect_to_DXL_device(self):
        """
        Universal connection method.
        - If user knows device parameters, they can pass them as arguments.
        - If any parameter is unknown, the function will try to auto-detect it.
        
        Returns:
            bool: True if connection is successful, otherwise False.
        """
        ports = self._scan_ports()
        # baudrates = [self.baudrate] if self.baudrate else [9600, 57600, 115200, 1000000]
        baudrates = [9600, 57600, 115200, 1000000]
        # protocol_versions = [self.protocol_version] if self.protocol_version else [1.0, 2.0]
        protocol_versions = [1.0, 2.0]

        for port in ports:
            print(f"Trying port: {port}")
            for baudrate_option in baudrates:
                for protocol_option in protocol_versions:
                    print(f"Trying Protocol Version: {protocol_option} at Baudrate: {baudrate_option}")
                    if self._try_connection(port, baudrate_option, protocol_option):
                        print(f"Connection successful on port {port} with Baudrate {self.baudrate} and Protocol Version {self.protocol_version}")
                        return True

        print("Device not found")
        return False


    def __call__(self, *args: Any, **kwds: Any) -> Any:
        """
        Display information about each device when the instance is called.
        """
        if not self.devices:
            print("No devices connected.")
        else:
            for index, device in enumerate(self.devices, start=1):
                if isinstance(device, dict):  # Ensure that each item is a dictionary
                    print(f"Device {index}:")
                    print(f"  ID: {device['dxl_id']}")
                    print(f"  Baudrate: {device['baudrate']}")
                    print(f"  Protocol Version: {device['protocol_version']}")
                    print(f"  Port: {device['port']}")
     
# Define communication statuses as an enum
class CommunicationStatus(Enum):
    SUCCESS = 0
    RX_TIMEOUT = -3001
    CRC_ERROR = -3002
    BUSY = -3003

# Define device errors as an enum
class DeviceError(Enum):
    VOLTAGE_ERROR = 1
    OVERHEATING = 2
    MOTOR_OVERLOAD = 3

class DXL_device:
    def __init__(self, dxl_id: Optional[int] = None, baudrate: Optional[int] = None, 
                 protocol_version: Optional[float] = None, port_timeout: int = 100) -> None:
        """
        Initialize DXL_device instance.
        """
        self.dxl_id = dxl_id
        self.baudrate = baudrate
        self.protocol_ver = protocol_version
        self.port_timeout = port_timeout
        self.port_handler = None
        self.packet_handler = None
        self.serial_connection = None  # For keep serial.Serial

    def _scan_ports(self, pattern: str = r'^/dev/cu\.usbserial.*') -> List[str]:
        """
        Scan all available COM ports based on the provided pattern.

        Parameters:
            pattern (str): Regular expression pattern to match port names.

        Returns:
            List of matching serial port names.
        """
        compiled_pattern = re.compile(pattern)
        return sorted([port.device for port in serial.tools.list_ports.comports() if compiled_pattern.match(port.device)])

    def _find_device(self, dxl_option: int, baudrate_option: int, protocol_option: float, port: str) -> bool:
        """
        Attempt to find and connect to the device using the provided ID, baudrate, and protocol version.

        Parameters:
            dxl_option (int): Device ID to try.
            baudrate_option (int): Baudrate to try.
            protocol_option (float): Protocol version to try.
            port (str): Serial port to try.

        Returns:
            bool: True if connection is successful, otherwise False.
        """
        print(f"Trying device ID: {dxl_option}")
        self.dxl_id = dxl_option  # Update device ID
        if self._detect_device():
            self.baudrate = baudrate_option
            self.protocol_ver = protocol_option
            print(f"Connection successful on port {port} with Baudrate {self.baudrate} and Protocol Version {self.protocol_ver}")
            return True
        return False

    def _detect_device(self) -> Optional[dict]:
        """
        Attempts to detect the device by pinging it with the current ID.

        Returns:
            dict: Device information if found, otherwise None.
        """
        for i in range(2):  # Try pinging twice
            outping_data, dxl_comm_result, dxl_error = self.packet_handler.ping(self.port_handler, self.dxl_id)
            if dxl_comm_result == COMM_SUCCESS:
                print(f"Device detected! ID: {self.dxl_id}")
                return outping_data
        print(f"Not device detected! ID: {self.dxl_id}")
        return None

    def ping_device(self) -> None:
        """
        Continuously ping the device until a successful response is received.
        """
        while True:
            outping_data, dxl_comm_result, dxl_error = self.packet_handler.ping(self.port_handler, self.dxl_id)
            if dxl_comm_result == COMM_SUCCESS:
                print(f"Ping successful: {outping_data}")
                break

    def connect_device(self) -> bool:
        """
        Universal connection method.
        If device parameters (ID, baudrate, protocol version) are unknown, try to auto-detect them.

        Returns:
            bool: True if connection is successful, otherwise False.
        """
        ports = self._scan_ports()
        baudrates = [self.baudrate] if self.baudrate else [9600, 57600, 115200, 1000000]
        protocol_versions = [self.protocol_ver] if self.protocol_ver else [1.0, 2.0]

        for port in ports:
            print(f"Trying port: {port}")
            for baudrate_option in baudrates:
                if self._attempt_connection(port, baudrate_option, protocol_versions):
                    return True
            print(f"No device on port {port}")
            if self.port_handler:
                self.port_handler.closePort()
            self.close_used_serPort()
        print("Device not found")
        return False

    def _attempt_connection(self, port: str, baudrate_option: int, protocol_versions: List[float]) -> bool:
        """
        Attempt connection with a specific baudrate and list of protocol versions.

        Parameters:
            port (str): Serial port.
            baudrate_option (int): Baudrate to use.
            protocol_versions (List[float]): List of protocol versions to try.

        Returns:
            bool: True if connection is successful, otherwise False.
        """
        try:
            # Create serial connection
            self.serial_connection = serial.Serial(port=port, baudrate=baudrate_option, timeout=self.port_timeout)

            # Initialize PortHandler and PacketHandler
            self.port_handler = PortHandler(self.serial_connection.name)

            for protocol_option in protocol_versions:
                print(f"Trying Protocol Version: {protocol_option} at Baudrate: {baudrate_option}")
                self.packet_handler = PacketHandler(protocol_option)
                self.port_handler.setBaudRate(baudrate_option)
                self.port_handler.openPort()

                if self.dxl_id:
                    if self._find_device(self.dxl_id, baudrate_option, protocol_option, port):
                        return True
                    # need back in protocol itter
                    continue
                
                for dxl_id_option in range(1, 255):
                    if self._find_device(dxl_id_option, baudrate_option, protocol_option, port):
                        return True      

        except serial.SerialException as e:
            print(f"Serial error on port {port}: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")
        # finally:
        #     self.close_used_serPort()  # Ensure the port is closed in case of an exception
        return False
    
    def close_used_serPort(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print("Port closed successfully.")

    def __call__(self) -> str:
        """
        Return a string representation of the device parameters.
        """
        return (f"Your DXL device has parameters: ID - {self.dxl_id}, "
                f"Baudrate - {self.baudrate}, Protocol version - {self.protocol_ver}, "
                f"Timeout - {self.port_timeout}, "
                f"Port handler  - {self.port_handler} & Packet handler - {self.packet_handler}")

class Sensor:
    def __init__(self, sensor_id: Optional[int] = None, sensor_range: Optional[str] = None, 
                 dxl_id_dev: Optional[int] = None, port_handler: Optional[Any] = None, 
                 packet_handler: Optional[Any] = None) -> None:
        if port_handler is None or packet_handler is None:
            raise ValueError("Port handler or Packet handler must be initialized.")
    
        self.sns_id = sensor_id
        self.sns_range = sensor_range
        self.dxl_id_device = dxl_id_dev
        self.port_handler = port_handler
        self.packet_handler = packet_handler

    def _read_data_elif(self, register_id: int, byte_count: int = 1) -> int:
        """
        Reads a specified number of bytes from a given register.

        Parameters:
        register_id (int): The register ID from which to read data.
        byte_count (int): The number of bytes to read (1, 2, or 4).

        Returns:
        int: The data read from the register.
        """
        while True:
            time.sleep(0.05)
            if byte_count == 1:
                data_from_reg, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, self.dxl_id_device, register_id)
            elif byte_count == 2:
                data_from_reg, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.dxl_id_device, register_id)
            elif byte_count == 4:
                data_from_reg, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, self.dxl_id_device, register_id)
            else:
                raise ValueError(f"Unsupported byte count: {byte_count}")
        
            if CommunicationStatus(dxl_comm_result) != CommunicationStatus.SUCCESS:
                comm_error_msg = self.packet_handler.getTxRxResult(dxl_comm_result)
                print(f"Communication error on register {register_id}: {comm_error_msg}")
            else:
                return data_from_reg
            
    def _read_data(self, register_id: int, byte_count: int = 1) -> int:
        """
        Reads a specified number of bytes from a given register.

        Parameters:
        register_id (int): The register ID from which to read data.
        byte_count (int): The number of bytes to read (1, 2, or 4).

        Returns:
        int: The data read from the register.
        """
        while True:
            time.sleep(0.05)
            match byte_count:
                case 1:
                    data_from_reg, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, self.dxl_id_device, register_id)
                case 2:
                    data_from_reg, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.dxl_id_device, register_id)
                case 4:
                    data_from_reg, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, self.dxl_id_device, register_id)
                case default:
                    data_from_reg, dxl_comm_result, dxl_error = self.packet_handler.readTxRx(self.port_handler, self.dxl_id_device, register_id, byte_count)
        
            if CommunicationStatus(dxl_comm_result) != CommunicationStatus.SUCCESS:
                comm_error_msg = self.packet_handler.getTxRxResult(dxl_comm_result)
                print(f"Communication error on register {register_id}: {comm_error_msg}")
            else:
                return data_from_reg

    def _read_data_universal(self, register_id: int, byte_count: int = 1) -> int:
        """
        Reads a specified number of bytes from a given register.

        Parameters:
        register_id (int): The register ID from which to read data.
        byte_count (int): The number of bytes to read (1, 2, or 4).

        Returns:
        int: The data read from the register.
        """
        while True:
            time.sleep(0.05)
            data_from_reg, dxl_comm_result, dxl_error = self.packet_handler.readTxRx(self.port_handler, self.dxl_id_device, register_id, byte_count)
        
            if CommunicationStatus(dxl_comm_result) != CommunicationStatus.SUCCESS:
                comm_error_msg = self.packet_handler.getTxRxResult(dxl_comm_result)
                print(f"Communication error on register {register_id}: {comm_error_msg}")
            else:
                return data_from_reg
            
    def _write_data_elif(self, register_id: int, data_to_write, byte_count: int = 1):
        """
        Writes a specified number of bytes to a given register.
        """
        while True:
            time.sleep(0.05)
            if byte_count == 1:
                dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id_device, register_id, data_to_write)
            elif byte_count == 2:
                dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, self.dxl_id_device, register_id, data_to_write)
            elif byte_count == 4:
                dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id_device, register_id, data_to_write)
            else:
                raise ValueError(f"Unsupported byte count: {byte_count}")
        
            if CommunicationStatus(dxl_comm_result) != CommunicationStatus.SUCCESS:
                comm_error_msg = self.packet_handler.getTxRxResult(dxl_comm_result)
                print(f"Communication error on register {register_id}: {comm_error_msg}")
            else:
                return True
            
    def _write_data(self, register_id: int, data_to_write, byte_count: int = 1):
        """
        Writes a specified number of bytes to a given register.
        """
        while True:
            time.sleep(0.05)
            match byte_count:
                case 1:
                    dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id_device, register_id, data_to_write)
                case 2:
                    dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, self.dxl_id_device, register_id, data_to_write)
                case 4:
                    dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id_device, register_id, data_to_write)
                case default:
                    dxl_comm_result, dxl_error = self.packet_handler.writeTxRx(self.port_handler, self.dxl_id_device, register_id, data_to_write, byte_count)
        
            if CommunicationStatus(dxl_comm_result) != CommunicationStatus.SUCCESS:
                comm_error_msg = self.packet_handler.getTxRxResult(dxl_comm_result)
                print(f"Communication error on register {register_id}: {comm_error_msg}")
            else:
                return True

    def _write_data_universal(self, register_id: int, data_to_write, byte_count: int = 1):
        """
        Writes a specified number of bytes to a given register.
        """
        while True:
            time.sleep(0.05)
            dxl_comm_result, dxl_error = self.packet_handler.writeTxRx(self.port_handler, self.dxl_id_device, register_id, data_to_write, byte_count)
            if CommunicationStatus(dxl_comm_result) != CommunicationStatus.SUCCESS:
                comm_error_msg = self.packet_handler.getTxRxResult(dxl_comm_result)
                print(f"Communication error on register {register_id}: {comm_error_msg}")
            else:
                return True

    def _choose_sns(self,found_sns) -> bool:
        """Prompt user to select a sensor from the found sensors."""
        if found_sns:
            print("Sensors found:")
            for sns_id, port_name in found_sns.items():
                print(f"Sensor ID: {sns_id} on {port_name}")
        
            while True:
                selected_id = int(input("Enter the Sensor ID you want to select: "))
                try:
                    if selected_id in found_sns:
                        self.sns_id = selected_id
                        self.sns_port = found_sns[selected_id]  # Corrected: Set the sensor port based on ID
                        print(f"Selected Sensor {self.sns_id} on {self.sns_port}.")
                        return True
                    else:
                        print("Selected Sensor ID is not valid.")
                except ValueError:
                    print("Invalid input. Please enter a numeric Sensor ID.")
        else:
            print("No sensors found.")
            return False

    def _find_sns_port(self):
        """Find the port where the sensor with the desired ID is connected."""
        print(f"Checking where sensors is connected.")
        found_sensors = {}
        # Port related constants
        ports_sns = {37:"Port_1", 39:"Port_2", 41:"Port_3", 43:"Port_4"}
        for reg_port_sns_option, name_port_sns_option  in zip(ports_sns.keys(), ports_sns.values()): 
            cur_sns_id = self._read_data(reg_port_sns_option, byte_count=2)
            print(f"Name {name_port_sns_option} -> Ports register - {reg_port_sns_option} -> SNS_ID - {cur_sns_id}")
            if self.sns_id == cur_sns_id:
                self.sns_port = reg_port_sns_option
                print(f"Sensor {self.sns_id} found on {name_port_sns_option}.")
                return True
            # if self.sns_id is None:
                # Collect all found sensors
            if cur_sns_id not in found_sensors and cur_sns_id != 0:
                found_sensors[cur_sns_id] = name_port_sns_option
                # print(found_sensors)

        if not self._choose_sns(found_sensors):
            return False
        else: 
            return True
        
    def _set_range_binary(self, nums_sns=1) -> bool:
        """
        Set the range for the sensor by activating the binary bits that correspond to enabled ranges.
        """
        print(f"Activate Sensor ID {self.sns_id} - Range: {self.sns_range} ")
        binary_representation = bin(self.sns_range)[2:].zfill(10)  # pad to 10 bits if necessary
        # print(binary_representation)
        cur_register_id = 60
        for index, bit in enumerate(reversed(binary_representation), start=0):
            if bit == "1":
                # Example: Activate sensor for these ranges
                self._write_data(register_id=cur_register_id, data_to_write=self.sns_id)
                # print(index)
                self._write_data(register_id=cur_register_id + 1, data_to_write=index)
                cur_register_id += 2
        return True
    
    def _set_range(self, nums_sns=1) -> bool:
        """
        Set the range for the sensor by activating the binary bits that correspond to enabled ranges.
        """
        print(f"Activate Sensor ID {self.sns_id} - Range: {self.sns_range} ")
        # print(binary_representation)
        cur_register_id = 60
        # range_num_str = str(self.sns_range)
        for range in self.sns_range:
                # Example: Activate sensor for these ranges
                self._write_data(register_id=cur_register_id, data_to_write=self.sns_id)
                # print(range)
                self._write_data(register_id=cur_register_id + 1, data_to_write=int(range)-1)
                cur_register_id += 2
        return True
    
    def _unset_range_binary(self, nums_sns=1) -> bool:
        """
        Unset the range for the sensor by activating the binary bits that correspond to enabled ranges.
        """
        print(f"Deactivate Sensor ID {self.sns_id} - Range: {self.sns_range} ")
        binary_representation = bin(self.sns_range)[2:].zfill(10)  # pad to 10 bits if necessary
        # print(binary_representation)
        cur_register_id = 60
        for index, bit in enumerate(reversed(binary_representation), start=0):
            if bit == "1":
                # Example: Activate sensor for these ranges
                self._write_data(register_id=cur_register_id, data_to_write=0)
                self._write_data(register_id=cur_register_id + 1, data_to_write=0)
                cur_register_id += 2
        return True

    def _unset_range(self, nums_sns=1) -> bool:
        """
        Set the range for the sensor by activating the binary bits that correspond to enabled ranges.
        """
        print(f"Deactivate Sensor ID {self.sns_id} - Range: {self.sns_range} ")
        # print(binary_representation)
        cur_register_id = 60
        # range_num_str = str(self.sns_range)
        for range in self.sns_range:
                # Example: Activate sensor for these ranges
                self._write_data(register_id=cur_register_id, data_to_write=0)
                # print(index)
                self._write_data(register_id=cur_register_id + 1, data_to_write=0)
                cur_register_id += 2
        return True

    def _start_meas(self) -> bool:
        """Start Measuring sensor"""
        print(f"Start measuring")
        self._write_data(register_id=24, data_to_write=1)
        return True
    
    def _stop_meas(self) -> bool:
        """Stop Measuring sensor"""
        print(f"Stop measuring")
        self._write_data(register_id=24, data_to_write=0)
        return True

    def _check_data_written(self) -> bool:
        # range_num_str = str(self.sns_range)
        reg_en_id = 60
        reg_en_id_r = 61
        for index, num in enumerate(self.sns_range):
            time.sleep(0.05)
            data_en_sns = self._read_data(register_id=reg_en_id)
            print(f"DX_ENABLE_SENSOR_ID_{index}_ID {data_en_sns}")
            reg_en_id += 2
            data_en_sns_r = self._read_data(register_id=reg_en_id_r)
            print(f"DX_ENABLE_SENSOR_ID_{index}_RANGE {data_en_sns_r}")
            reg_en_id_r += 2
        data_status_meas = self._read_data(register_id=24)
        print(f"DX_MEAS_START_STOP - {data_status_meas}")
        return True

    def activate_sns_measure(self) -> bool:
        """Activate measuring desiring sns"""
        
        if self._find_sns_port():
            if self._set_range():
                if self._start_meas():
                    # self._check_data_written()
                    return True
        return False

    def deactivate_sns_measure(self) -> bool:
        """Activate measuring desiring sns"""
        if self._find_sns_port():
            if self._unset_range():
                if self._stop_meas():
                    # self._check_data_written()
                    return True
        return False

    def read_sns_results_manual(self)-> list:
        """Start get data from regs desiring sns"""
        data_read = []
        for pair_n in range(2):
            time.sleep(1)
            # if self._read_data(register_id=85, byte_count=1) == 128:
            data_85 = self._read_data(register_id=85, byte_count=1)
            print(f"From register DX_SENSORS_DATA_FIRST read: {data_85}")
            if data_85:
                data_read.append(self._read_data(register_id=85+1, byte_count=2))
            data_88 = self._read_data(register_id=88, byte_count=1)
            print(f"From register DX_SENSORS_DATA_FIFTH read: {data_88}")
            if data_88:
                data_read.append(self._read_data(register_id=88+1, byte_count=2))
                print(f"Data {pair_n} iter written")
        print(f"Data taken {data_read}")
        return data_read
    
    def read_sns_results(self, count_bytes_res = 2)-> list:
        """Start get data from regs desiring sns"""
        data_read = []
        # range_num_str = str(self.sns_range)
        # nums of itterarion от self.range
        for pair_n in range(2):
            # print(f"Count of take measures {pair_n}")
            # Initialize register addresses
            reg_status = 85
            reg_value = 86
            # List to store data for the current iteration
            current_data = []
            for num in self.sns_range:
                time.sleep(0.5)
                # Read the status from the current register
                data_status = self._read_data(register_id=reg_status, byte_count=1)
                # print(f"From register DX_SENSORS_DATA_{num} read: {data_status}")
                # Define the register for the value based on the status register
                reg_status = reg_value + count_bytes_res
                # print(f"reg_status -- {reg_status}")
                if data_status:
                    time.sleep(0.2)
                    # If status is read successfully, read the value + append
                    current_data.append(self._read_data(register_id=reg_value, byte_count=count_bytes_res))
                # Move to the next range's registers
                reg_value = reg_status + 1
                # print(f"reg_value -- {reg_value}")
            # If more than one digit in the range, group data into tuples
            if len(self.sns_range) > 1:
                # Group data into tuples of pairs
                data_read.extend(tuple(current_data[i:i+2]) for i in range(0, len(current_data), 2))
            else:
                # Append single values to the list
                data_read.extend(current_data)

            # print(f"Data {pair_n} iter written")
        
        print(f"Data taken {data_read}")
        return data_read

    def __call__(self, *args: Any, **kwds: Any) -> str:
        """
        Return a string representation of the device parameters.
        """
        return (f"Your Sensor has parameters: ID - {self.sns_id}, "
                f"Sensor range - {self.sns_range}")

class DataManager:
    def __init__(self, filename="results_term_compens.txt"):
        self.filename = filename

    def write_data(self, sensor_id: Optional[int] = None, sensor_range: Optional[str] = None, data: Optional[list] = None): 
        """Write sensor data to a file."""
        file_exists = os.path.isfile(self.filename)
        with open(self.filename, 'a') as file:
            if not file_exists:
                file.write("Sensor Data Pairs\n=================\n\n")
            for index, pair in enumerate(data):
                if index == 0:
                    file.write(f"SENSOR is active: {sensor_id}\n")
                    file.write(f"SENSORs Range is/are {sensor_range}\n")
                # If tuples in list
                if isinstance(pair, tuple) and len(pair) == 2:
                    file.write(f"Pair {index+1}: {pair[0]:>6}, {pair[1]:>6}\n")
                else:
                    # print(f"Not tuples data at index {index+1}: {pair}")
                    file.write(f"Just value tick {index+1}: {pair}\n")
            file.write("\nEND OF DATA\n\n")

    def verify_data(self):
        """Verify the data by checking the control string."""
        if not os.path.isfile(self.filename):
            return False
        with open(self.filename, 'r') as file:
            # print(f"Why?")
            return file.readlines()[-2].strip() == "END OF DATA"

class PlotterManager:
    def __init__(self, data: list, labels: list, max_mins: list, show_legend: bool=False, title: str=None, subplots: list=None):
        self.data = data  # Sensor data buffer
        self.labels = labels  # Labels for the plots
        self.max_mins = max_mins  # Min/Max values for the y-axis scaling
        self.subplots = subplots if subplots else [len(data)]  # Define subplots or use default
        self.show_legend = show_legend  # Whether to show the legend
        self.title = title  # Title for the plot
        self.plotter_stop_event = Event()  # Event to signal stopping the plotter thread
        self.plotter = None  # Plotter instance to be initialized later

    def start(self, packetHandler, portHandler, sample_size=1024):
        """
        Starts the data collection and plotting process.
        :param packetHandler: Communication handler for the device
        :param portHandler: Port handler for the device
        :param sample_size: Number of samples to collect per update
        """
        
        self.plotter = Plotter(self.data, self.labels, self.max_mins, show_legend=self.show_legend, title=self.title, sublots=self.subplots)
        plotter_thread = Thread(target=self.plotter_proc, args=(packetHandler, portHandler, sample_size, self.plotter_stop_event))
        plotter_thread.start()  # Start the background thread for data collection
        print(f"Start build graphics")
        self.visual_process(self.plotter)  # Start the plotting in the main thread

    def plotter_proc(self, packetHandler, portHandler, sample_size, stop_event: Event):
        """
        Background process to collect data from sensors.
        :param packetHandler: Communication handler
        :param portHandler: Port handler
        :param sample_size: Number of samples to collect per update
        :param stop_event: Event to signal when to stop the process
        """
        time.sleep(0.1)
        while not stop_event.is_set():
            for frame_num in range(sample_size):
                # Collect data from the device (sensor)
                while True:
                    time.sleep(0.005)
                    data, dxl_comm_result, dxl_error = packetHandler.readTxRx(portHandler, 171, 85, 6)
                    if dxl_comm_result == COMM_SUCCESS:
                        break

                # Process the raw data
                bin_data_1 = int((data[1] | (data[2] << 8)))
                bin_data_2 = int((data[4] | (data[5] << 8)))

                signed_value_1 = int.from_bytes(bin_data_1.to_bytes(2, byteorder='big'), byteorder='big', signed=True)
                signed_value_2 = int.from_bytes(bin_data_2.to_bytes(2, byteorder='big'), byteorder='big', signed=True)

                # Update the data buffer
                self.data[0][frame_num] = signed_value_1
                self.data[1][frame_num] = signed_value_2

            # Signal the plotter to update
            self.plotter.upd_cnt = 0

    def visual_process(self, plotter: Plotter):
        """
        Starts the animation process in the main thread to visualize the data.
        :param plotter: Plotter instance
        """
        plotter.animate()  # This will handle plt.show() in the main thread
        print("Plotting stopped")
        self.plotter_stop_event.set()  # Signal the plotter thread to stop when plotting ends

    def __call__(self, *args: Any, **kwds: Any) -> Any:
        pass


class Application:
    def __init__(self, dxl_id: Optional[int] = None, baudrate: Optional[int] = 9600,
        protocol_version: Optional[float] = 2.0, port_timeout: int = 100,
        sensor_id: Optional[int] = None, sensor_range: Optional[str] = None,
        filename: Optional[str] = None,
        mode: Optional[str] = None) -> None:
        """
        Initializes the Application with device and sensor configurations.
        
        Parameters:
            dxl_id (Optional[int]): Device ID for DXL_device.
            baudrate (Optional[int]): Baudrate for serial communication.
            protocol_version (Optional[float]): Protocol version for communication.
            port_timeout (int): Timeout for the serial port in milliseconds.
            sensor_id (Optional[int]): Sensor ID to activate.
            sensor_range (Optional[str]): Range configuration for the sensor.
            filename (Optional[str]): Filename for data writing.
            mode (Optional[str]): Operation mode ('write' or 'plotting').
        """
        self.dxl_id_devs = dxl_id
        self.baudrates = baudrate
        self.protocols = protocol_version
        self.port_timeout = port_timeout

        self.sns_ids = sensor_id
        self.sns_ranges = sensor_range

        self.file_names = filename

        self.mode = mode
        self.port_handler = None
        self.packet_handler = None
        self.serial_connection = None  # For keeping the serial.Serial connection

        # Initialize device and sensor to None
        self.devices = None
        self.sensors = None
        self.data_manager = None
        self.plotter_manager = None

    def run(self):
        """
        Executes the main application logic based on the selected mode.
        Ensures that resources are cleaned up gracefully upon completion or error.
        """
        try:
            self._connect_device()
            self._initialize_sensor()

            if self.sensors.activate_sns_measure():
                if self.mode == "write":
                    self._write_mode()
                elif self.mode == "plotting":
                    self._plotting_mode()
                else:
                    print(f"Unknown mode: {self.mode}. Exiting.")
            else:
                print("Failed to activate sensor measurements.")

        except Exception as e:
            print(f"An unexpected error occurred: {e}")
        finally:
            self._deactivate_and_cleanup()
            print("Application finished.")

    def _connect_device(self):
        """
        Connects to the DXL device using the provided configurations.
        """
        print("Connecting to DXL device...")
        self.devices = DXL_device(
            dxl_id=self.dxl_id_devs,
            baudrate=self.baudrates,
            protocol_version=self.protocols,
            port_timeout=self.port_timeout
        )
        time.sleep(1)  # Wait for the device to initialize

        if self.devices.connect_device():
            print("Device connected successfully.")
        else:
            raise ConnectionError("Failed to connect to the DXL device.")

    def _initialize_sensor(self):
        """
        Initializes the sensor with the connected device's handlers.
        """
        print("Initializing sensor...")
        self.sensors = Sensor(
            sensor_id=self.sns_ids,
            sensor_range=self.sns_ranges,
            dxl_id_dev=self.dxl_id_devs,
            port_handler=self.devices.port_handler,
            packet_handler=self.devices.packet_handler
        )

    def _write_mode(self):
        """
        Handles the 'write' mode: reads data from sensors and writes it to a file.
        """
        print("Running in 'write' mode...")
        res_sns = self.sensors.read_sns_results()

        if res_sns:
            self.data_manager = DataManager(filename=self.file_names)
            self.data_manager.write_data(
                sensor_id=self.sensors.sns_id,
                sensor_range=self.sensors.sns_range,
                data=res_sns
            )

            if self.data_manager.verify_data():
                print("Data successfully written and verified.")
            else:
                print("Data verification failed.")
        else:
            print("No sensor results to write.")

    def _plotting_mode(self):
        """
        Handles the 'plotting' mode: collects sensor data and visualizes it.
        """
        print("Running in 'plotting' mode...")
        # Initialize data buffer and plot settings
        data_buff = [[None] * 1024 for _ in range(2)]  # Assuming two sensors
        plot_legend = [f"Sensor {i+1}" for i in range(len(data_buff))]
        max_mins = [[-2000, 2000] for _ in range(len(data_buff))]
        subplots = [2]  # Adjust based on the number of sensors or requirements

        # Initialize PlotterManager
        self.plotter_manager = PlotterManager(
            data=data_buff,
            labels=plot_legend,
            max_mins=max_mins,
            # show_legend=True,
            # title="Sensor Data Visualization",
            subplots=subplots
        )

        # Start plotting and data acquisition
        self.plotter_manager.start(
            packetHandler=self.devices.packet_handler,
            portHandler=self.devices.port_handler,
            sample_size=1024
        )

    def _deactivate_and_cleanup(self):
        """
        Deactivates sensor measurements and closes the serial port.
        Ensures all resources are properly released.
        """
        print("Deactivating sensor measurements...")
        if self.sensors:
            try:
                self.sensors.deactivate_sns_measure()
                print("Sensor deactivated successfully.")
            except Exception as e:
                print(f"Error deactivating sensor: {e}")

        print("Closing serial port...")
        if self.devices:
            try:
                self.devices.close_used_serPort()
                print("Serial port closed successfully.")
            except Exception as e:
                print(f"Error closing serial port: {e}")


    
def main(args: list):
    # Define configurations
    DXL_ID = 171            # Replace with your actual device ID
    BAUDRATE = 115200
    PROTOCOL_VER = 2.0
    PORT_TIM = 100          # milliseconds

    SENSOR_ID = 3           # Set to None to allow selection
    SENSOR_RANGE = "1"      # Replace with actual range configuration
    FILENAME = "results_term_compens.txt"
    MODE = "plotting"       # "write" or "plotting"

    app = Application(dxl_id=DXL_ID, baudrate=BAUDRATE, protocol_version=PROTOCOL_VER, sensor_id=SENSOR_ID, sensor_range=SENSOR_RANGE, filename=FILENAME, mode=MODE)

    app.run()

if __name__ == '__main__':
    main(sys.argv)