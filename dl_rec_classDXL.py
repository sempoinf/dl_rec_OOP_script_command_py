import os
import re
import serial
import serial.tools.list_ports
#from serial.serialutil import SerialException  # Correct way to import SerialException
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

# at least ask for two and more device not need, then work 
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

    def _scan_ports(self, pattern: str = r'^/dev/cu.usbmodem*') -> List[str]:
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

    def _get_port_pattern(self, platform_name: str) -> str:
        """Return pattern of port depend os"""
        patterns = {
            "win": "^/dev/tty*",
            "linux": "^/dev/cu.*",
            "darwin": "^/dev/cu.usbmodem*"
        }
        # find key 
        for key, pattern in patterns.items():
            if platform_name.startswith(key):
                return pattern
        return None

    def connect_device(self) -> bool:
        """
        Universal connection method.
        If device parameters (ID, baudrate, protocol version) are unknown, try to auto-detect them.

        Returns:
            bool: True if connection is successful, otherwise False.
        """
        os_name = sys.platform
        port_pattern = self._get_port_pattern(platform_name = os_name)
        ports = self._scan_ports(pattern=port_pattern)
        baudrates = [self.baudrate] if self.baudrate else (9600, 57600, 115200, 1000000)
        protocol_versions = [self.protocol_ver] if self.protocol_ver else (1.0, 2.0)

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
                        # print(f"Port: {self.port_handler}\n Packet: {self.packet_handler}\n")
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
    def __init__(self, sensor_id: Optional[list] = None, sensor_range: Optional[list] = None, 
                 dxl_id_dev: Optional[int] = None, port_handler: Optional[Any] = None, 
                 packet_handler: Optional[Any] = None) -> None:
        if port_handler is None or packet_handler is None:
            raise ValueError("Port handler or Packet handler must be initialized.")
    
        self.sns_id = sensor_id
        self.sns_range = sensor_range
        self.dxl_id_device = dxl_id_dev
        self.port_handler = port_handler
        self.packet_handler = packet_handler

        # Hidden parameters
        self.sns_port = [] # mb dont need
        self.flag_activate_sns = {}
       
    def _read_data(self, register_id: int, byte_count: int=1) -> int:
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

    def _read_data_universal(self, register_id: int, byte_count: int=1) -> int:
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
            
    def _write_data(self, register_id: int, data_to_write, byte_count: int=1):
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

    def _write_data_universal(self, register_id: int, data_to_write, byte_count: int=1):
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
    
    def _rewrite_list_sns_id(self):
        """Update sns_id to include only active sensors."""
        # Filter sns_id based on active sensors in flag_activate_sns
        self.sns_id = [sensor_id for sensor_id, status in self.flag_activate_sns.items() if status]

    def _choose_sns(self, found_sns: dict) -> bool:
        """Prompt user to select a sensor from the found sensors."""
        if not found_sns:
            print("No sensors found.")
            return False

        print("Sensors found:")
        for sns_id, port_name in found_sns.items():
            print(f"Sensor ID: {sns_id} on {port_name}")

        self.sns_id = []  # Reset sns_id list

        while True:
            selected_id = input("Enter the Sensor ID you want to select (comma-separated), or press Enter to skip: ")
        
            if selected_id.strip() == '':
                print("No sensor selected. Skipping sensor selection.")
                return True

            try:
                sensor_ids = [int(id.strip()) for id in selected_id.split(',')] # 'space' or , or ,' '
                print("Selected Sensor IDs:", sensor_ids)

                # Process each selected sensor ID
                for sns_id in sensor_ids:
                    if sns_id in found_sns:
                        self.sns_id.append(sns_id)  # Add sensor ID to the list
                        print(f"Selected Sensor {sns_id} on {found_sns[sns_id]}.")
                        self.flag_activate_sns[sns_id] = True  # Mark sensor as active
                    else:
                        print(f"Sensor ID {sns_id} is not valid.")
            
                # Exit loop if any valid sensor was selected
                if self.sns_id:
                    return True
                else:
                    print("No valid sensor selected. Please try again.")
            except ValueError:
                print("Invalid input. Please enter numeric Sensor IDs.")

    def _find_sns_port(self) -> bool:
        """Find the port where the sensor with the desired ID is connected."""
        print("Checking where sensors are connected.")
    
        found_sensors = {}
        self.flag_activate_sns = {sns: False for sns in self.sns_id}  # Initialize all sensors as inactive
        self.sns_port = []  # Reset sensor port list

        # Define available ports and register addresses for sensor IDs
        ports_sns = {37: "Port_1", 39: "Port_2", 41: "Port_3", 43: "Port_4"}
        
        # Scan ports to find connected sensors
        for reg_port_sns_option, name_port_sns_option in ports_sns.items():
            cur_sns_id = self._read_data(reg_port_sns_option, byte_count=2)
            print(f"Checking {name_port_sns_option} -> Found Sensor ID: {cur_sns_id}")

            # If sensor is found on the port, activate and record it
            if cur_sns_id in self.sns_id:
                self.sns_port.append(reg_port_sns_option)
                print(f"Sensor {cur_sns_id} found on {name_port_sns_option}.")
                self.flag_activate_sns[cur_sns_id] = True
            elif cur_sns_id != 0:
                found_sensors[cur_sns_id] = name_port_sns_option
                
        # Allow user to select from found sensors if necessary
        if found_sensors:
            if not self._choose_sns(found_sensors):
                return False

        # Update sns_id to only include active sensors
        self._rewrite_list_sns_id()

        # Return True if at least one sensor is active
        return any(self.flag_activate_sns.values())

    def _set_range(self) -> bool:
        """
        Set the range for the sensor by activating the binary bits that correspond to enabled ranges.
        """
        print(f"Activate Sensor ID/s {self.sns_id} - Range/s: {self.sns_range} ")
        cur_register_id = 60
        # range_num_str = str(self.sns_range)
        for sns in self.sns_id:
            # print(sns)
            for range in self.sns_range:
                # Example: Activate sensor for these ranges
                self._write_data(register_id=cur_register_id, data_to_write=sns)
                # print(range)
                self._write_data(register_id=cur_register_id + 1, data_to_write=int(range)-1) # -1 to decrease range 
                cur_register_id += 2
        return True
    
    def _unset_range(self) -> bool:
        """
        Set the range for the sensor by activating the binary bits that correspond to enabled ranges.
        """
        print(f"Deactivate Sensor ID {self.sns_id} - Range: {self.sns_range} ")
        # print(binary_representation)
        cur_register_id = 60
        # range_num_str = str(self.sns_range)
        for sns in self.sns_id:
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

    def _check_data_written(self) -> dict:
        """Check if the data is written correctly to the registers."""
        reg_en_id, reg_en_id_r = 60, 61
        reg_data_written = {}

        for index, sns in enumerate(self.sns_id):
            for rnge in self.sns_range:
                time.sleep(0.05)
                data_en_sns = self._read_data(register_id=reg_en_id)
                data_en_sns_r = self._read_data(register_id=reg_en_id_r)

                print(f"\nDX_ENABLE_SENSOR_ID_{index}_ID {data_en_sns}")
                print(f"DX_ENABLE_SENSOR_ID_{index}_RANGE {data_en_sns_r}")

                reg_en_id += 2
                reg_en_id_r += 2
                reg_data_written[data_en_sns] = data_en_sns_r

        reg_data_written['start'] = self._read_data(register_id=24)
        print(f"DX_MEAS_START_STOP - {reg_data_written['start']}\n")

        return reg_data_written
    
    def _comp_data_activate(self, data: dict) -> bool:
        """Compare data for sensor activation and trigger start measurement if valid."""
        # print(data)
        # all_snss_in_data = all(sns in data for sns in self.sns_id)

        if not all(sns in data for sns in self.sns_id):
            self._set_range()
        else:
            for sns in self.sns_id:
                for range in self.sns_range:
                    # print(1)
                    if isinstance(data[sns], (list, tuple)):
                        if range-1 not in data[sns]:
                            # print(2)
                            self._set_range()
                    else: 
                        if range-1 != data[sns]:
                            self._set_range()

        last_key = next(reversed(data))
        if last_key == 'start' and data[last_key] == 1:
            return True
        else: return self._start_meas()
     
    def _comp_data_deactivate(self, data: dict) -> bool:
        """Deactivate sensors if measurement is active."""
    
        if data.get('start') == 1:
            self._stop_meas()
            data.pop('start', None)

            for sns, ranges in data.items():
                if isinstance(ranges, (list, tuple)):
                    for rnge in ranges:
                        if sns != 0 or rnge != 0:
                            return self._unset_range()
                elif ranges != 0: return self._unset_range()
        return True
       
    def activate_sns_measure(self) -> bool:
        """Activate measuring desiring sns"""
        if self._find_sns_port():
            while True:  # Check twice
                # print('\nstart')
                data_w_regs = self._check_data_written()
                # print('after read data\n')
                if self._comp_data_activate(data_w_regs):
                    return True
        return False

    def deactivate_sns_measure(self) -> bool:
        """Deactivate measuring desiring sns"""
        data_w_regs = self._check_data_written()
        while True:
            if self._comp_data_deactivate(data_w_regs):
                break
        self.flag_activate_sns = False
        return True
       
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
    
    def _tryhard(self)-> None:
        reg_value = 85
        for i in range(6):
            time.sleep(0.5)
            # print(i)
            data_val = self._read_data(register_id=reg_value + i, byte_count=1)
            print(f"Value from {reg_value + i} {data_val}")

    def read_sns_results(self, count_of_measure: int=2, count_bytes_res: int=2)-> list:
        """Start get data from regs desiring sns"""
        REG_STATUS = 84
        # print(f"reg_status start sns -- {reg_status}")
        data_read = []
        data_status = self._read_data(register_id=REG_STATUS, byte_count=1)
        print(f"From register DX_SENSORS_STATUS read: {data_status}")
        # nums of itterarion от self.range
        for pair_n in range(count_of_measure):
            # Read the status from the current register
            if data_status == 128:
                # Initialize register addresses
                REG_VALUE = 85
                # If status is read successfully, read the value + append
                for sns in self.sns_id:
                    time.sleep(0.5)
                    # List to store data for the current iteration
                    current_data = []
                    print(f"Sensor {sns}")
                    for num in self.sns_range:
                        time.sleep(0.5)
                        sns_val = self._read_data_universal(register_id=REG_VALUE, byte_count=count_bytes_res)
                        bin_data = int((sns_val[0] | (sns_val[1] << 8)))
                        signed_value = int.from_bytes(bin_data.to_bytes(2, byteorder='big'), byteorder='big', signed=True)
                        # print(signed_value)
                        # verify_value = signed_value if signed_value > min(min_max) and signed_value < max(min_max) else None
                        # current_data.append(verify_value)
                        current_data.append(signed_value)
                        # Define the register for the value based on the status register
                        # Move to the next range's registers
                        REG_VALUE += count_bytes_res
                        # print(reg_value)
                    
                    # If more than one digit in the range, group data into tuples
                    if len(self.sns_range) > 1:
                        # Group data into tuples of pairs
                        data_read.extend(tuple(current_data[i:i+2]) for i in range(0, len(current_data), 2))
                    else:
                        # Append single values to the list
                        data_read.extend(current_data)
                    print(f"Sensor value all range: {current_data}")
                    # input()
                    
        print(f"Data taken: {data_read}")
        return data_read

    def read_packet_results(self, count_of_elems: int=2, count_bytes_res: int=2)-> list:
        """Start get data from regs desiring sns"""
        # REG_STATUS = 84
        data_read = []
        # data_status = self._read_data(register_id=REG_STATUS, byte_count=1)
        # nums of itterarion от self.range
        REG_VALUE = 396
        for cnt in range(count_of_elems):
            # Read the status from the current register
            # if data_status == 128:
                # Initialize register addresses
                # If status is read successfully, read the value + append
                # time.sleep(0.01)
                sns_val = self._read_data_universal(register_id=REG_VALUE, byte_count=count_bytes_res)
                bin_data = int((sns_val[0] | (sns_val[1] << 8)))
                signed_value = int.from_bytes(bin_data.to_bytes(2, byteorder='big'), byteorder='big', signed=True)
                # print(signed_value)
                data_read.append(signed_value)
                REG_VALUE += count_bytes_res
        print(f"Data taken: {data_read}")
        return data_read
    
    def __call__(self, *args: Any, **kwds: Any) -> str:
        """
        Return a string representation of the device parameters.
        """
        return (f"Your Sensor has parameters: ID - {self.sns_id}, "
                f"Sensor range - {self.sns_range}")

class DataManager:
    def __init__(self, filename: Optional[str] = None):
        self.filename = filename

    def write_data(self, sensor_id: Optional[list] = None, sensor_range: Optional[list] = None, count_of_measure: Optional[int] = 1, name_exp: Optional[str]  = None, data: Optional[list] = None): 
        """Write sensor data to a file."""
        file_exists = os.path.isfile(self.filename)
        with open(self.filename, 'a') as file:
            if not file_exists:
                file.write("Sensor Data Pairs\n=================\n\n")
            for index, pair in enumerate(data):
                if index == 0:
                    file.write(f"SENSOR is active: {', '.join(map(str, sensor_id))}\n")
                    file.write(f"SENSORs Range is/are {', '.join(map(str, sensor_range))}\n")
                    file.write(f"Count of measure - {count_of_measure}\n")
                    file.write(f"{name_exp}\n")
                # If tuples in list
                if isinstance(pair, tuple):
                    formatted_values = ', '.join([f"{value:>6} mV" for value in pair])
                    file.write(f"Index values - {index+1}: {formatted_values}\n")
                else:
                    file.write(f"Just value tick {index+1}: {pair} mV\n")
            file.write("\nEND OF DATA\n\n")

    def verify_data(self):
        """Verify the data by checking the control string."""
        if not os.path.isfile(self.filename):
            return False
        with open(self.filename, 'r') as file:
            return file.readlines()[-2].strip() == "END OF DATA"
        
    def __call__(self, *args: Any, **kwds: Any) -> Any:
        """
        Return a string representation of the file parameters.
        """
        return (f"Your Sensor has parameters: ID - {self.filename}")

class PlotterManager:
    def __init__(self, dxl_id: Optional[int], data: dict, labels: list, max_mins: list, show_legend: bool=False, title: str=None, subplots: list=None):
        self.dxl_id = dxl_id # 
        self.data = data  # Sensor data buffer
        self.labels = labels  # Labels for the plots
        self.max_mins = max_mins  # Min/Max values for the y-axis scaling
        self.subplots = subplots if subplots else [len(data)]  # Define subplots or use default
        self.show_legend = show_legend  # Whether to show the legend
        self.title = title  # Title for the plot
        self.plotter_stop_event = Event()  # Event to signal stopping the plotter thread
        self.plotter = None  # Plotter instance to be initialized later

    def start(self, packetHandler, portHandler, all_sns: Optional[int], all_ranges: Optional[int], sample_size: int=1024):
        """
        Starts the data collection and plotting process.
        :param packetHandler: Communication handler for the device
        :param portHandler: Port handler for the device
        :param sample_size: Number of samples to collect per update
        """
        self.plotter = Plotter(data=self.data, max_mins=self.max_mins)
        plotter_thread = Thread(target=self.plotter_proc, args=(packetHandler, portHandler, all_sns, all_ranges, sample_size, self.plotter_stop_event))
        # print(2)
        plotter_thread.start()  # Start the background thread for data collection
        print(f"Start build graphics")
        self.visual_process(self.plotter)  # Start the plotting in the main thread

    def _read_data_sns(self, packet_handler, port_handler, register_id: int, byte_count: int=1) -> int:
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
                    data_from_reg, dxl_comm_result, dxl_error = packet_handler.read1ByteTxRx(port_handler, self.dxl_id, register_id)
                case 2:
                    data_from_reg, dxl_comm_result, dxl_error = packet_handler.read2ByteTxRx(port_handler, self.dxl_id, register_id)
                case 4:
                    data_from_reg, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(port_handler, self.dxl_id, register_id)
                case default:
                    data_from_reg, dxl_comm_result, dxl_error = packet_handler.readTxRx(port_handler, self.dxl_id, register_id, byte_count)
        
            if CommunicationStatus(dxl_comm_result) != CommunicationStatus.SUCCESS:
                comm_error_msg = packet_handler.getTxRxResult(dxl_comm_result)
                print(f"Communication error on register {register_id}: {comm_error_msg}")
            else:
                return data_from_reg
        
    def plotter_proc(self, packetHandler, portHandler, snss: Optional[list[int]], ranges: Optional[list[int]], sample_size: Optional[int], stop_event: Event):
        """
        Background process to collect data from sensors.
        :param packetHandler: Communication handler
        :param portHandler: Port handler
        :param snss: List of sensors (defines how many sensors to read from)
        :param ranges: List of ranges for each sensor
        :param sample_size: Number of samples to collect per update
        :param stop_event: Event to signal when to stop the process
        """
        time.sleep(0.1)
        DX_SENSORS_DATA_FIRST = 85  # First register to read from
        DEFAULT_BYTE_READ = 2

        while not stop_event.is_set():
            # rewrite in normal way
            for frame_num in range(sample_size):
                index = 0
                for sns_index, sns in enumerate(snss):
                    data_buff_r = {}
                    for rnge_index, rnge in enumerate(ranges):
                        start_address = DX_SENSORS_DATA_FIRST + (index * DEFAULT_BYTE_READ)
                        # Read data for each sensor based on bytes_to_call
                        data = self._read_data_sns(packet_handler=packetHandler, port_handler=portHandler, register_id=start_address, byte_count=DEFAULT_BYTE_READ)
                        if isinstance(data, list): 
                            if len(data) == 2:
                        # Process the raw data (extracting status and 2-byte sensor value)
                                bin_data = int((data[0] | (data[1] << 8)))  # Merge two bytes into a single value
                                signed_value = int.from_bytes(bin_data.to_bytes(2, byteorder='big'), byteorder='big', signed=True)
                        else:
                            signed_value = data  # Handle case where data is not a list or invalid
                        # Update the data buffer dynamically based on number of sensors
                        self.data[sns][rnge][frame_num] = signed_value
                        index += 1

            # Signal the plotter to update (if needed)
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
        """
        Return a string representation of the file parameters.
        """
        return (f"Your Plot has parameters: Sensor data buffer - {self.data}, "
                f"Labels for the plots - {self.labels}, "
                f"Min/Max values for the y-axis scaling - {self.max_mins}, "
                f"Define subplots or use default - {self.subplots}, "
                f"Whether to show the legend - {self.show_legend}, "
                f"Title for the plot - {self.title}"
                )

class Application:
    def __init__(self, dxl_id: Optional[int] = None, baudrate: Optional[int] = None,
        protocol_version: Optional[float] = None, port_timeout: int = 100,
        sensor_id: Optional[list] = None, sensor_range: Optional[list] = None,
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
                time.sleep(1)
                if self.mode == "writing":
                    self._writing_mode()
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
            print("Device connected successfully.\n")
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

    def _writing_mode(self):
        """
        Handles the 'writing' mode: reads data from sensors and writes it to a file.
        """
        print("Running in 'writing' mode...")
        COUNT_MEASURE = 64
        # max_mins = (100, 3500)
        # list_exp = ['0%', '1%', 'Vstill', '2%', '3%', '4%', 'Vstill', '5%', '10%', '20%', '50%', 'Vstill']
        list_exp_100Hz = ['Vstill', '500Hz', 'Vstill']
        for i in list_exp_100Hz:
            input(f"Put It down for {i}")
            time.sleep(10)
            res_sns = self.sensors.read_packet_results(count_of_elems=COUNT_MEASURE)
            if res_sns:
                self.data_manager = DataManager(filename=self.file_names)
                self.data_manager.write_data(
                sensor_id=self.sensors.sns_id,
                sensor_range=self.sensors.sns_range,
                count_of_measure = COUNT_MEASURE,
                name_exp = i,
                data=res_sns
                )
                if self.data_manager.verify_data():
                    print("Data successfully written and verified.\n")
                else:
                    print("Data verification failed.")
            else:
                print("No sensor results to write.")
            # if any(chr.isdigit() for chr in i): time.sleep(90)
            
    def _plotting_mode(self):
        """
        Handles the 'plotting' mode: collects sensor data and visualizes it.
        """
        print("Running in 'plotting' mode...")
        # Initialize data buffer and plot settings
        sample_size = 1024
        data_buff_sns_r = {sns: {rnge: [None] * sample_size for rnge in self.sns_ranges} for sns in self.sns_ids}
        # data_buff_sns = [[None] * 1024 for _, _ in enumerate(self.sns_ids)]
        # for all ranges in one sns
        # data_buff_ranges = [[None] * 1024 for _, _ in enumerate(self.sns_ranges)]
        
        plot_legend = [f"Sensor {i+1}" for i in range(len(data_buff_sns_r))]
        max_mins = [[100, 3400] for _ in range(len(data_buff_sns_r))]
        subplots = [len(self.sns_ids)]  # Adjust based on the number of sensors or requirements

        # Initialize PlotterManager
        self.plotter_manager = PlotterManager(
            dxl_id=self.dxl_id_devs,
            data=data_buff_sns_r,
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
            all_sns=self.sns_ids,
            all_ranges=self.sns_ranges,
            sample_size=sample_size
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

    SENSOR_ID = [46]           # Set to None to allow selection
    SENSOR_RANGE = [1]      # Replace with actual range configuration
    FILENAME = '../dl_rec_read_data_py/res/results_500Hz.txt'
    MODE = "plotting"       # "writing" or "plotting"

    app = Application(dxl_id=DXL_ID, baudrate=BAUDRATE, 
                      protocol_version=PROTOCOL_VER, sensor_id=SENSOR_ID, 
                      sensor_range=SENSOR_RANGE, filename=FILENAME, mode=MODE)

    app.run()

if __name__ == '__main__':
    main(sys.argv)