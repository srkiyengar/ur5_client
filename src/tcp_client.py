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

    def connect(self, host, port):
        try:
            self.sock.connect((host,port))
            self.link = 1
            self.socket_host = host
            self.socket_port = port
        except socket.timeout:
            my_logger.info("Socket time out error")
            self.link = 0

    def end_socket(self):
        #self.sock.shutdown(self.sock)
        self.sock.close()

    def send_data(self, msg):

        if(self.link == 0):
            my_logger.info("Socket does not have an establsihed connection to a host/port")
            return 0
        message_len = len(msg)
        total = 0
        while total < message_len:
            sent = self.sock.send(msg[total:])
            if sent == 0:
                raise RuntimeError("socket connection broken")
            total = total + sent
        my_logger.info("Socket write - Length {} sent through socket to host - {}/port - {}",
                       total,self.socket_host,self.socket_port)

    def receive_data(self,how_many):

        received_data = ''
        bytes_recd = 0
        incoming_data = 1
        bytes_remaining = how_many-bytes_recd
        while (incoming_data and bytes_remaining):
            chunk = self.sock.recv(bytes_remaining, 2048)
            if chunk == '':
                incoming_data = 0
            received_data = received_data + chunk
            bytes_recd = bytes_recd + len(chunk)
            bytes_remaining = how_many-bytes_recd
        return received_data

class ur5_connection:

    def __init__(self, host, port):

        self.my_socket = make_connection()
        self.my_socket.connect(host,port)


    def recv(self,no_bytes):
        return self.my_socket.receive_data(no_bytes)


    def close_ur5_connection(self):
        self.my_socket.end_socket()




