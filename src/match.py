__author__ = 'srkiyengar'

import os
import shutil
import logging.handlers
from datetime import datetime


LOG_LEVEL = logging.DEBUG

# Set up a logger with output level set to debug; Add the handler to the logger
my_logger = logging.getLogger("UR5_Logger")

my_dir = "../trials"

class match:

    def __init__(self,dname):

        # create directories if they don't exisit
        if not os.path.exists(dname):
            raise ("The directory {} does not exist".format(dname))
            sys.exit(1)
        else:
            self.dname = dname
            self.ur5_files = []
            self.npy_files = []
            self.id = []


    def pickup_files(self):
        # read files from raw_data directory
        a = os.walk(self.dname)
        all_files = a.next()[2]
        for name in all_files:
            if "ur5" in name:
                self.ur5_files.append(name)
            elif "npy" in name and "color" in name:
                self.npy_files.append(name)
        return


    def pair_files(self):
        for name in self.ur5_files:
            for image_file in self.npy_files:
                if name[0:6] == image_file[0:6]:
                    self.id.append(name[0:6])
                    break
        return


if __name__ == "__main__":

    a = match(my_dir)
    a.pickup_files()
    a.pair_files()
    print(a.id)



