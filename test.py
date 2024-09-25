import sys


def main(args: list):
    # Define configurations
    DXL_ID = 171            # Replace with your actual device ID
    BAUDRATE = 115200
    PROTOCOL_VER = 2.0
    PORT_TIM = 100          # milliseconds

    SENSOR_ID = [46]           # Set to None to allow selection
    SENSOR_RANGE = [1, 2]      # Replace with actual range configuration
    FILENAME = "results_term_compens.txt"
    MODE = "writing"       # "writing" or "plotting"

    app = Application(dxl_id=DXL_ID, baudrate=BAUDRATE, protocol_version=PROTOCOL_VER, sensor_id=SENSOR_ID, sensor_range=SENSOR_RANGE, filename=FILENAME, mode=MODE)

    app.run()

if __name__ == '__main__':
    main(sys.argv)