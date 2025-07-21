import glob

from PhaseDet2 import *

dev = PhaseDet2()

ports = dev.search_com()
dev.open(comport=ports[0])

path = ["../../Firmware/PhaseDet2_app/Debug/"]

firmware_bin_name = 'PhaseDet2_app-*.bin'

# Check all paths and try to find given file
for p in path:
    files = glob.glob(p + firmware_bin_name)
    if files:
        files.sort()
        if len(files) >= 1:
            dev.upgrade_firmware(files[-1])



