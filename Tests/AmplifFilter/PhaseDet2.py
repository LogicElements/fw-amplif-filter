from lecore.Devices.LeBinDev import *


class PhaseDet2(LeBinDev):

    def __init__(self, com=None, reg_map=None):
        """
        Phase detector 2 device

        The respective parameters allow to redefine default settings. In most cases, it is not necessary.
        :param com: Communication settings file override
        :param reg_map: Register map definition file override
        """
        super().__init__()
        # Get path to RFTB directory
        self._path = os.path.dirname(os.path.abspath(__file__)) + "/"
        print(f"PATH PHASE_DET_2 = {self._path}")
        # Set default register map
        self._def_regs = "PhaseDet2_registers.json"
        self._def_usb = {'vid': 1155, 'pid': 22336, 'sn': "PHASEDETECTOR2"}

        # Initialize all internal structures
        self._com(com)
        self._reg_map(reg_map)
        self._upgrade()
        self._visual()
