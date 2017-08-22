__author__ = 'srkiyengar'

import socket
import logging
import struct
from datetime import datetime
import time


LOG_LEVEL = logging.DEBUG
PORT_SECONDARY_CLIENT = 30002
PORT_REALTIME_CLIENT = 30003

# Set up a logger with output level set to debug; Add the handler to the logger
my_logger = logging.getLogger("UR5_Logger")


class make_connection:

    def __init__(self, sock=None):
        if sock is None:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            except socket.error as e:
                my_logger.info("Socket Error: {} ".format(e))
                raise Exception('Could not create socket\n')

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
            my_logger.info("Socket Timeout: No socket connection to Host {} Port {}".format(host,port))
            self.link = 0
            raise RuntimeError("Failure of socket connection to Host {} Port {}".format(host,port))

    def end_socket(self):
        #self.sock.shutdown(self.sock)
        self.sock.close()

    def send_data(self, msg):

        if(self.link == 0):
            my_logger.info("Socket does not have an established connection to a host/port")
            return 0
        message_len = len(msg)
        total = 0
        while total < message_len:
            sent = self.sock.send(msg[total:])
            if sent == 0:
                raise RuntimeError("socket connection broken")
            total = total + sent
        my_logger.info("Socket write: {} \nLength {} sent through socket to host - {} and port - {}".
                       format(msg, total,self.socket_host,self.socket_port))
        return message_len

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

class ur5_connector:

    def __init__(self, host, port):

        if port == PORT_REALTIME_CLIENT:
            self.type = 1
        elif port == PORT_SECONDARY_CLIENT:
            self.type = 2
        else:
            self.type = 0

        if (self.type != 0):
            self.my_socket = make_connection()
            self.my_socket.connect(host,port)

    def recv(self,no_bytes):
        return self.my_socket.receive_data(no_bytes)

    def send(self,my_str):
        return self.my_socket.send_data(my_str)



    def close(self):
        self.my_socket.end_socket()






