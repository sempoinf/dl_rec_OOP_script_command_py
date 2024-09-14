import re
import serial
import serial.tools.list_ports
from dynamixel_sdk import *

DX_DXL_ID = 3                  # RW
DX_BAUD_RATE = 4               # RW
DX_RETURN_DELAY_TIME = 5       # RW

class DXL_device():
    dxl_id = None
    baudrate = None
    protocol_ver = None

    def __init__(self, port_timeout=100) -> None:
        self.dxl_id = None
        self.baudrate = None
        self.protocol_ver = None
        self.port_handler = None
        self.packet_handler = None
        self.port_timeout = port_timeout


    def detect_device(self):
        """Attempts to detect the ID of the connected device."""
        for i in range(2):
            outping_data, dxl_comm_result, dxl_error = self.packet_handler.ping(self.port_handler, self.device_id)
            if dxl_comm_result == COMM_SUCCESS:
                print(f"Device detected! ID: {self.device_id}")
                return outping_data
        return None
    
    # def _

    def connect_to_DXL_device(self, dxl_id=None, baudrate=None, protocol_version=None):
        """
        Universal connection method.
        - If user knows device parameters (ID, baudrate, protocol version), they can pass them as arguments.
        - If any parameter is unknown, the function will try to auto-detect it.
        
        Parameters:
        - dxl_id (int): Known device ID (optional).
        - baudrate (int): Known baudrate (optional).
        - protocol_version (float): Known protocol version (optional).
        
        Returns:
        True if connection is successful, otherwise False.
        """
        # If user provides device ID, use it; otherwise, set to None
        self.device_id = dxl_id if dxl_id is not None else None

        # List of baudrates and protocol versions if they are not provided by user
        baudrates = [baudrate] if baudrate else [9600, 57600, 115200, 1000000]
        protocol_versions = [protocol_version] if protocol_version else [1.0, 2.0]

        # Define the pattern to match specific serial ports (adjust pattern for your system)
        pattern = re.compile(r'^/dev/cu\.usbserial.*')
        # Define the pattern to match specific serial ports (adjust pattern for your system)
        for port in sorted(serial.tools.list_ports.comports(), key=lambda p: p.device):
            # print(f"{port}")
            if pattern.match(port.device):
                print(f"Trying port: {port.device}")
                for baudrate_option in baudrates:
                    try:
                        # Attempt to connect at this baudrate
                        ser = serial.Serial(port=port.device, baudrate=baudrate_option, timeout=self.port_timeout)
                        self.port_handler = PortHandler(ser.name)
                        
                        for protocol_option in protocol_versions:
                            try:
                                print(f"Trying Protocol Version: {protocol_option} at Baudrate: {baudrate_option}")
                                self.packet_handler = PacketHandler(protocol_option)
                                self.port_handler.setBaudRate(baudrate_option)
                                self.port_handler.openPort()
                                
                                # Try Know IDs list 
                                possible_ids = [170]                            
                                for dxl_id_option in possible_ids:     
                                    print(f"Trying ID: {dxl_id_option}")
                                    self.device_id = dxl_id_option
                                    # Ping the device to check if we can communicate
                                    if self.detect_device():
                                        #print(f"Device found with ID {dxl_id_option}")
                                        self.baudrate = baudrate_option
                                        self.protocol_version = protocol_option
                                        print(f"Connection successful on port {port.device} with Baudrate {self.baudrate} and Protocol Version {self.protocol_version}")
                                        return True

                                # If not match try range all IDs    
                                print("Known IDs not successful. Scanning full ID range...")
                                # subtract IDs that already checked
                                remaining_ids = set(range(1, 255)) - set(possible_ids)
                                # toss left IDs
                                for dxl_id_option in remaining_ids:
                                    print(f"Trying ID: {dxl_id_option}") 
                                    self.device_id = dxl_id_option   
                                    # Ping the device to check if we can communicate
                                    if self.detect_device():
                                        #print(f"Device found with ID {dxl_id_option}")
                                        self.baudrate = baudrate_option
                                        self.protocol_version = protocol_option
                                        # add new ID in List
                                        possible_ids.append(self.device_id)
                                        print(f"List of Known IDs - {possible_ids}")
                                        
                                        print(f"Connection successful on port {port.device} with Baudrate {self.baudrate} and Protocol Version {self.protocol_version}")
                                        return True
                                    
                            except Exception as e:
                                print(f"Protocol version {protocol_version} failed: {e}")
                                continue
                    except serial.SerialException as e:
                        print(f"Serial error: {e}")
                    except Exception as e:
                        print(f"Unexpected error: {e}")
                    finally:
                        self.port_handler.closePort()  # Ensure port is closed if it was opened
                        
        print("Device not found")
        return False        
                                          
    def __call__(self):
	    return f"Your DXL device has parametrs: ID - {self.dxl_id}, Baudrate - {self.baudrate}, Protocol version - {self.protocol_ver}"
                                 
    
def main():
    # string_input = input("Enter ID, Baudrate and Protocol version your device: ")
    # list_pars_device = string_input.split()
    # print(list_pars_device)
    # dxl_rec = DXL_device(171, 115200, 2.0)
    # print(f"{dxl_rec()}")
    
    device_comm = DXL_device()
    print(f"{device_comm()}")
    device_comm.connect_to_DXL_device()
    print(f"{device_comm()}")

if __name__ == '__main__':
	main()