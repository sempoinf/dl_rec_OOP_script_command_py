#!/usr/bin/env python3
from dataclasses import dataclass
import sys
import os
import serial
import serial.tools.list_ports
import time
import ctypes
from plot import Plotter
from threading import Thread
from threading import Event
from dynamixel_sdk import *
import random
from rich import print as rprint
from PyQt5.QtWidgets import QApplication, QWidget, QHBoxLayout, QVBoxLayout, QRadioButton, QPushButton, QListWidget, QListWidgetItem, QSpacerItem, QSizePolicy, QLabel, QFileDialog
from PyQt5.QtGui import QFont
from PyQt5.QtCore import Qt
import wave, struct

DXL_ID = 171
BAUDRATE = 115200
PROTOCOL_VERSION = 2.0
DEF_TMO = 100
SAMPLE_SIZE = 1024

data_buff = [[None] * 1024, [None] * 1024]
plot_legend = ["Sensor 1", "Sensor 2"]
max_mins = [[0, +4000]]
sublots = [2]

plotter = Plotter(data_buff, plot_legend, max_mins, sublots=sublots)

### REGISTERS

DX_MEAS_START_STOP = 24

DX_TEMP_PORT_ID = 25
DX_LIGHT_PORT_ID = 27
DX_HUM_PORT_ID = 29
DX_PRESS_PORT_ID = 31
DX_PULSE_PORT_ID = 33
DX_UV_PORT_ID = 35

DX_PORT1_SNS_ID = 37
DX_PORT2_SNS_ID = 39
DX_PORT3_SNS_ID = 41
DX_PORT4_SNS_ID = 43
DX_PORT1S_SNS_ID = 45
DX_PORT2S_SNS_ID = 47
DX_PORT3S_SNS_ID = 49
DX_PORT4S_SNS_ID = 51

DX_SENSORS_STATUS = 53

DX_ENABLE_SENSOR_ID_0_ID = 60
DX_ENABLE_SENSOR_ID_0_RANGE = 61

DX_ENABLE_SENSOR_ID_1_ID = 62
DX_ENABLE_SENSOR_ID_1_RANGE = 63

DX_ENABLE_SENSOR_ID_2_ID = 64
DX_ENABLE_SENSOR_ID_2_RANGE = 65

DX_ENABLE_SENSOR_ID_3_ID = 66
DX_ENABLE_SENSOR_ID_3_RANGE = 67

DX_ENABLE_SENSOR_ID_4_ID = 68
DX_ENABLE_SENSOR_ID_4_RANGE = 69

DX_ENABLE_SENSOR_ID_5_ID = 70
DX_ENABLE_SENSOR_ID_5_RANGE = 71

DX_ENABLE_SENSOR_ID_6_ID = 72
DX_ENABLE_SENSOR_ID_6_RANGE = 73

DX_ENABLE_SENSOR_ID_7_ID = 74
DX_ENABLE_SENSOR_ID_7_RANGE = 75

DX_ENABLE_SENSOR_ID_8_ID = 76
DX_ENABLE_SENSOR_ID_8_RANGE = 77

DX_ENABLE_SENSOR_ID_9_ID = 78
DX_ENABLE_SENSOR_ID_9_RANGE = 79

DX_SENSORS_DATA_FIRST = 85

def connect_dev(id, baudrate):
    ports = serial.tools.list_ports.comports()
    for port in sorted(ports):
        print("Trying port: ", port.device)
        try:
            ser = serial.Serial(port = port.device, baudrate = baudrate, timeout=DEF_TMO)
            portHandler = PortHandler(ser.name)
            packetHandler = PacketHandler(PROTOCOL_VERSION)
            portHandler.setBaudRate(BAUDRATE)
            # portHandler.setPacketTimeout(DEF_TMO)
            portHandler.openPort()

            buff, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, id)

            if dxl_comm_result == COMM_SUCCESS:
                print("Recorder found!\r\n")
                return portHandler, packetHandler
            else:
                print("No recorder on port")
                portHandler.closePort()
                ser.close()
            # time.sleep(0.2)
        except:
            pass
    
    print("\r\nDevice not found")
    return False, False


def visual_proc(plotter: Plotter):
    # try:
        plotter.animate()
    # finally:
        print("Plotting stopped")
        return

def plotter_proc(packetHandler, portHandler, sample_size, stop_event: Event):
    # try:
        time.sleep(0.1)
        while True:
            
            for frames_num in range(sample_size):
                while True:
                    time.sleep(0.005)
                    data, dxl_comm_result, dxl_error = packetHandler.readTxRx(portHandler, DXL_ID , DX_SENSORS_DATA_FIRST, 6)
                    if dxl_comm_result == COMM_SUCCESS:
                        break

                bin_data_1 = int((data[1] | (data[2] << 8)))
                bin_data_2 = int((data[4] | (data[5] << 8)))

                signed_value_1 = int.from_bytes(bin_data_1.to_bytes(2, byteorder='big'), byteorder='big', signed=True)
                signed_value_2 = int.from_bytes(bin_data_2.to_bytes(2, byteorder='big'), byteorder='big', signed=True)

                data_buff[0][frames_num] = signed_value_1
                data_buff[1][frames_num] = signed_value_2

                if stop_event.is_set():
                    print("Stopped")
                    return

            plotter.upd_cnt = 0

    # finally:
    #     return

def sensor_stop_measure(portHandler, packetHandler):
    while True:
        time.sleep(0.005)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID , DX_MEAS_START_STOP, 0)
        if dxl_comm_result == COMM_SUCCESS:
            break
    print("Measure stopped")

def sensor_start_measure(portHandler, packetHandler):
    while True:
        time.sleep(0.005)
        data, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID , DX_PORT1_SNS_ID)
        if dxl_comm_result == COMM_SUCCESS:
            break
    
    sensor_id_1 = data
    print(f"Selected sensor with ID {sensor_id_1}")

    while True:
        time.sleep(0.005)
        data, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID , DX_PORT2_SNS_ID)
        if dxl_comm_result == COMM_SUCCESS:
            break
    
    sensor_id_2 = data
    print(f"Selected sensor with ID {sensor_id_2}")

    while True:
        time.sleep(0.005)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID , DX_ENABLE_SENSOR_ID_0_ID, sensor_id_1)
        if dxl_comm_result == COMM_SUCCESS:
            break
    print("Sensor ID selected")

    while True:
        time.sleep(0.005)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID , DX_ENABLE_SENSOR_ID_0_RANGE, 0)
        if dxl_comm_result == COMM_SUCCESS:
            break
    print("Sensor 0 range selected")


    while True:
        time.sleep(0.005)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID , DX_ENABLE_SENSOR_ID_1_ID, sensor_id_2)
        if dxl_comm_result == COMM_SUCCESS:
            break
    print("Sensor ID selected")

    while True:
        time.sleep(0.005)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID , DX_ENABLE_SENSOR_ID_1_RANGE, 0)
        if dxl_comm_result == COMM_SUCCESS:
            break
    print("Sensor 0 range selected")

    while True:
        time.sleep(0.005)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID , DX_MEAS_START_STOP, 1)
        if dxl_comm_result == COMM_SUCCESS:
            break
    print("Measure started")


def main(args: list):
    portHandler, packetHandler = connect_dev(DXL_ID, BAUDRATE)
    time.sleep(1)
    if not (portHandler and packetHandler):
        quit()

    sensor_start_measure(portHandler, packetHandler)

    plotter_stop = Event()
    visual_stop = Event()

    plotter_thread = Thread(target=plotter_proc, args=(packetHandler, portHandler, SAMPLE_SIZE, plotter_stop))
    plotter_thread.start()

    visual_proc(plotter)

    plotter_stop.set()

    sensor_stop_measure(portHandler, packetHandler)
    print("Finish")
    quit()


if __name__ == '__main__':
    main(sys.argv)

