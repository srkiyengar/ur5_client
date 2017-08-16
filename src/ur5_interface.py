__author__ = 'srkiyengar'
import logging
import logging.handlers
from datetime import datetime
import tcp_client as tp
import time
import struct
import codecs

#logger
LOG_LEVEL = logging.DEBUG
LOG_FILENAME = 'ur5' + datetime.now().strftime('%Y-%m-%d---%H:%M:%S')

PORT_30003 = 30003
HOST = '192.168.10.2'

if __name__ == '__main__':

    # Set up a logger with output level set to debug; Add the handler to the logger
    my_logger = logging.getLogger("UR5_Logger")
    my_logger.setLevel(LOG_LEVEL)
    handler = logging.handlers.RotatingFileHandler(LOG_FILENAME, maxBytes=6000000, backupCount=5)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    my_logger.addHandler(handler)
    # end of logfile preparation Log levels are debug, info, warn, error, critical



    s = tp.ur5_connection(HOST, PORT_30003)
    time.sleep(1.00)

    my_logger.info("reading packets from UR5")
    packet_1 = s.recv(4)                                # Read first 4 bytes

    total_length = struct.unpack('!I',packet_1)[0]      # Total length of the packets following the first 4 bytes
    packet_2 = s.recv(total_length)
    packet_len = len(packet_2)
    if packet_len != total_length:
        my_logger.info("Warning !!!! - Should have read {} bytes but got only {} bytes from UR5",)

    tool_pos = packet_2[440:488]                        # 48 bytes consisting of 8 bytes of x,y,z,rx,ry,rz)
    x,y,z,Rx,Ry,Rz = struct.unpack('!dddddd',tool_pos)

    s.close_ur5_connection()

    s = tp.ur5_connection(HOST, PORT_30003)
    my_logger.info("reading packets from UR5")
    packet_1 = s.recv(4)                                # Read first 4 bytes

    total_length = struct.unpack('!I',packet_1)[0]      # Total length of the packets following the first 4 bytes
    packet_2 = s.recv(total_length)
    packet_len = len(packet_2)
    if packet_len != total_length:
        my_logger.info("Warning !!!! - Should have read {} bytes but got only {} bytes from UR5",)

    tool_pos = packet_2[440:488]                        # 48 bytes consisting of 8 bytes of x,y,z,rx,ry,rz)
    x1,y1,z1,Rx1,Ry1,Rz1 = struct.unpack('!dddddd',tool_pos)


    s.close_ur5_connection()

    my_logger.info("Completed")