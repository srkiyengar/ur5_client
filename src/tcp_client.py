__author__ = 'srkiyengar'

import socket
import logging
import struct
from datetime import datetime
import time


LOG_LEVEL = logging.DEBUG

# Set up a logger with output level set to debug; Add the handler to the logger
my_logger = logging.getLogger("UR5_Logger")


class make_connection:

    def __init__(self, sock=None):
        if sock is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(10)
        else:
            self.sock = sock
        self.connection = 0

    def connect(self, host, port):
        try:
            self.sock.connect((host,port))
            self.link = 1
        except socket.timeout:
            my_logger.info("Socket time out error")
            self.link = 0

    def end_socket(self):
        #self.sock.shutdown(self.sock)
        self.sock.close()

    def send_data(self, msg):

        message_len = len(msg)
        total = 0
        while total < message_len:
            sent = self.sock.send(msg[total:])
            if sent == 0:
                raise RuntimeError("socket connection broken")
            total = total + sent

    def receive_data(self):

        received_data = []
        how_many = 4098         #arbitary as we don't know the response from UR-5
        bytes_recd = 0
        incoming_data = 1
        while (incoming_data):
            chunk = self.sock.recv((how_many - bytes_recd), 2048)
            if chunk == '':
                incoming_data = 0
            received_data.append(chunk)
            bytes_recd = bytes_recd + len(chunk)
        return ''.join(received_data)


class command_ur5:

    def __init__(self, host, port=5000):

        self.my_connection = make_connection()
        self.my_connection.connect(host,port)

    def send_command(self,command_string):
        command_string = command_string + '\n'
        self.my_connection.send_data(command_string)
        return self.my_connection.receive_data()

    def receive_command_response(self):
        self.my_connection.receive_data()

    def close_ur5_connection(self):
        self.my_connection.end_socket()




