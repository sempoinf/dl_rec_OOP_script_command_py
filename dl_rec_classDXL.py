import os
import re
import serial
import serial.tools.list_ports
from dynamixel_sdk import *
from typing import List, Optional, Union, Any
from enum import Enum


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
        # try:
            # # Используем контекстный менеджер для автоматического закрытия порта
            # with serial.Serial(port=port, baudrate=baudrate_option, timeout=self.port_timeout) as ser:
            #     self.port_handler = PortHandler(ser.name)
        ser = None
        try:
            # Create serial connection
            ser = serial.Serial(port=port, baudrate=baudrate_option, timeout=100)

            # Initialize PortHandler and PacketHandler
            self.port_handler = PortHandler(ser.name)

            for protocol_option in protocol_versions:
                print(f"Trying Protocol Version: {protocol_option} at Baudrate: {baudrate_option}")
                self.packet_handler = PacketHandler(protocol_option)
                self.port_handler.setBaudRate(baudrate_option)
                self.port_handler.openPort()

                if self.dxl_id:
                    if self._find_device(self.dxl_id, baudrate_option, protocol_option, port):
                        return True
                else:
                    for dxl_id_option in range(1, 255):
                        if self._find_device(dxl_id_option, baudrate_option, protocol_option, port):
                            return True


        except serial.SerialException as e:
            print(f"Serial error on port {port}: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")
  
        return False

    def __call__(self) -> str:
        """
        Return a string representation of the device parameters.
        """
        return (f"Your DXL device has parameters: ID - {self.dxl_id}, "
                f"Baudrate - {self.baudrate}, Protocol version - {self.protocol_ver}, "
                f"Timeout - {self.port_timeout}, "
                f"Port handler  - {self.port_handler} & Packet handler - {self.packet_handler}")

class Sensor:
    def __init__(self, sensor_id: Optional[int] = None, sensor_range: Optional[int] = None, 
                 dxl_id_dev: Optional[int] = None, port_handler: Optional[Any] = None, 
                 packet_handler: Optional[Any] = None) -> None:
        if port_handler is None or packet_handler is None:
            raise ValueError("Port handler or Packet handler must be initialized.")
    
        self.sns_id = sensor_id
        self.sns_range = sensor_range
        self.dxl_id_device = dxl_id_dev
        self.port_handler = port_handler
        self.packet_handler = packet_handler

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
            time.sleep(0.5)
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

    def _write_data(self, register_id, data_to_write, byte_count=1):
        """
        Writes a specified number of bytes to a given register.
        """
        while True:
            time.sleep(0.5)
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

    def _find_sns_port(self):
        """Find the port where the sensor with the desired ID is connected."""
        print(f"Checking where sensor {self.sns_id} is connected.")
        # Port related constants
        # for port_sns_option in range(37, 44, 2):
        ports_sns = {37:"Port1", 39:"Port_2", 41:"Port_3", 43:"Port_4"}
        for reg_port_sns_option, name_port_sns_option  in zip(ports_sns.keys(), ports_sns.values()): 
            if self.sns_id == self._read_data(reg_port_sns_option):
                self.sns_port = reg_port_sns_option
                print(f"Sensor {self.sns_id} found on {name_port_sns_option}.")

    def _set_range(self, nums_sns=1) -> bool:
        """
        Set the range for the sensor by activating the binary bits that correspond to enabled ranges.
        """
        print(f"Activate Sensor ID {self.sns_id} - Range: {self.sns_range} ")
        binary_representation = bin(self.sns_range)[2:].zfill(10)  # pad to 10 bits if necessary
        cur_register_id = 60
        for index, bit in enumerate(reversed(binary_representation)):
            if bit == "1":
                # Example: Activate sensor for these ranges
                self._write_data(register_id=cur_register_id, data_to_write=self.sns_id)
                self._write_data(register_id=cur_register_id + 1, data_to_write=index)
                cur_register_id += 2
        return True

    def _start_meas(self) -> bool:
        """Start Measuring sensor"""
        print(f"Start measuring")
        self._write_data(register_id=24, data_to_write=1)
        return True

    def _check_data_written(self):
        pass

    def activate_sns_measure(self) -> bool:
        """Activate measuring desiring sns"""
        # self._find_sns_port()
        if self._set_range():
            if self._start_meas():
                return True
        else: False

    def read_sns_results(self)-> list:
        """Start get data from regs desiring sns"""
        data_read = []
        for pair_n in range(3):
            time.sleep(1)
            if self._read_data(register_id=85, byte_count=1) == 128:
                data_read.append(self._read_data(register_id=85+1, byte_count=1))
                print(f"Data {pair_n} iter written")
        return data_read


    def __call__(self, *args: Any, **kwds: Any) -> str:
        """
        Return a string representation of the device parameters.
        """
        return (f"Your Sensor has parameters: ID - {self.sns_id}, "
                f"Sensor range - {self.sns_range}")

class Data_manager():
    def __init__(self, filename="results_term_compens.txt"):
        self.filename = filename

    def write_data(self, data): 
        """Write sensor data to a file."""
        file_exists = os.path.isfile(self.filename)
        with open(self.filename, 'a') as file:
            if not file_exists:
                file.write("Sensor Data Pairs\n=================\n\n")
            for index, pair in enumerate(data):
                file.write(f"Pair {index+1}: {pair[0]:>6}, {pair[1]:>6}\n")
            file.write("\nEND OF DATA\n")

    def verify_data(self):
        """Verify the data by checking the control string."""
        if not os.path.isfile(self.filename):
            return False
        with open(self.filename, 'r') as file:
            return file.readlines()[-1].strip() == "END OF DATA"


def main():
    dxl_rec = DXL_device(dxl_id=171, baudrate=115200, protocol_version=2.0)
    # print(dxl_rec())
    if dxl_rec.connect_device():
        sns_ethanol = Sensor(sensor_id=46, sensor_range=1, dxl_id_dev=dxl_rec.dxl_id ,port_handler=dxl_rec.port_handler, packet_handler=dxl_rec.packet_handler)
        print(sns_ethanol()) 
        if sns_ethanol.activate_sns_measure():
            res_sns = sns_ethanol.read_sns_results()
            if res_sns:
                data_manager = Data_manager()
                data_manager.write_data(res_sns)
                if data_manager.verify_data():
                    print("Data successfully written.")
                else:
                    print("Data verification failed.")
            else: print("Not results.")
        else: print("Not activated.")
    else: print("Not connected.")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nScript interrupted by user.")