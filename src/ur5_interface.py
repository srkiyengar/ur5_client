__author__ = 'srkiyengar'
import logging
import logging.handlers
from datetime import datetime
import tcp_client as tp

#logger
LOG_LEVEL = logging.DEBUG
LOG_FILENAME = 'ur5' + datetime.now().strftime('%Y-%m-%d---%H:%M:%S')


if __name__ == '__main__':

    # Set up a logger with output level set to debug; Add the handler to the logger
    my_logger = logging.getLogger("UR5_Logger")
    my_logger.setLevel(LOG_LEVEL)
    handler = logging.handlers.RotatingFileHandler(LOG_FILENAME, maxBytes=6000000, backupCount=5)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    my_logger.addHandler(handler)
    # end of logfile preparation Log levels are debug, info, warn, error, critical


    ur5_connection = tp.command_ur5('192.168.10.2', 30002)
    command_response = ur5_connection.send_command("get_actual_tcp_pose\n")

    r = 7