import serial
import argparse
import time
import os
import sys

global lastBeaconInfo

def parse_command_line():

    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--port',         action='store',      help="COM port")
    parser.add_argument('-v', '--verbose',          action='store_true', help="Enable verbose printing to log")
    return parser.parse_args()


class SerialReaderWriter():
    def __init__(self, port, baudrate=115200):
        self._port = port
        self._baudrate = baudrate
        try:
            self._serial = serial.Serial(self._port, baudrate, timeout=0.025)
        except:
            print("Failed to connect to {}\n".format(self._port))
            if sys.platform == 'linux':
                print("You might have to run as root on Linux\n")    
        # Add delay as workaround to make sure port is open
        time.sleep(0.3)
        self._serial.flushInput()
        self._serial.flushOutput()
        self._serial.write(bytearray('\n', encoding='utf-8'))

    def close(self):
        self._serial.flushInput()
        self._serial.flushOutput()
        time.sleep(0.1)
        self._serial.close()

    def write(self, cmd):
        self._serial.write(bytearray(source=str(cmd) + '\n', encoding='utf-8'))
        # Add blocking delay to make program not crash
        time.sleep(0.01)
        
    def flush(self):
        self._serial.flushInput()
        self._serial.flushOutput()

    
    def read(self):
        # Emulate a do-while loop
        readBuffer = b''
        chunkSize = 4096
        while True:
            # Read in chunks. Each chunk will wait as long as specified by
            # timeout. Increase chunkSize to fail quicker
            byteChunk = self._serial.read(chunkSize)
            readBuffer += byteChunk
            # print(str(byteChunk))
            if not len(byteChunk) == chunkSize:
                break
        # Make string object instead of byte object of serial reading
        readBuffer = str(readBuffer)
        
        # Modify string to use new line char instead of two separate \ + n chars
        # Also remove carrige return
        readBuffer = readBuffer.replace("\\n", "\n")
        readBuffer = readBuffer.replace("\\r", "")
        
        # Split string into array of lines
        raw_lines = iter(readBuffer.splitlines())
        refinedLines = str()
        for line in raw_lines:
            if ("usb_cli" in line) or (line.strip() == '\n'):
                continue
            else:
                refinedLines = refinedLines + line + '\n'
        # print(refinedLines)
        refinedLines = refinedLines.splitlines()
        return refinedLines


def printBeaconFromMyCompany(lines):
    global lastBeaconInfo
    for line in lines:
        csv = line.split(",")
        if len(csv) > 9:
            # company ID is located at position 8 and 9 in our custom beacon
            if csv[8].strip() == "0x59" and  csv[9].strip() == "0x00":
                # Workaround since some lines are shorter due to serial bugs.
                # We know that our custom beacons have full length 32 after converted to csv
                if len(csv) == 32: 
                    if not (line == lastBeaconInfo):
                        print(line)
                        lastBeaconInfo = line
        


def main():
    global lastBeaconInfo
    lastBeaconInfo = str()
    args = parse_command_line()

    cmd = SerialReaderWriter(args.port)
    cmd.write('advertise off')

    # cmd.write("scan on")
    # printBeaconFromMyCompany(cmd.read())
    # printBeaconFromMyCompany(cmd.read())
    # printBeaconFromMyCompany(cmd.read())

    # cmd.flush()
    # cmd.close()
    # print("End of program")
    # exit()
    cmd.write("scan on")
    try:
        while True:
            cmd.write("device_details")
            printBeaconFromMyCompany(cmd.read())
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exit program")
        cmd.close()
    

if __name__ == "__main__":
    # execute only if run as a script
    main()