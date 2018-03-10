from __future__ import print_function
import sys
import serial
from librepilot.uavtalk import uavtalk, objectManager, connectionManager, uavobject


class Uav:

    def __init__(self, serial_port, baud_rate=57600):
        self.serial = serial.Serial(serial_port, baudrate=baud_rate, timeout=0.5)
        if not self.serial.isOpen():
            raise IOError("Could not open serial port {}".format(serial_port))

        self.uavtalk = uavtalk.UavTalk(self.serial, logFile=None)
        self.object_manager = objectManager.ObjManager(self.uavtalk)
        self.object_manager.importDefinitions()

        self.uavtalk.start()

        self.connection_manager = connectionManager.ConnectionManager(self.uavtalk, self.object_manager)
        self.connection_manager.connect()

        self.object_manager.requestAllObjUpdate()

        # print("Request fast periodic updates for AttitudeState")
        self.object_manager.getObjByName("AttitudeState").metadata.telemetryUpdateMode = \
            uavobject.UAVMetaDataObject.UpdateMode.PERIODIC
        self.object_manager.getObjByName("AttitudeState").metadata.telemetryUpdatePeriod.value = 50
        self.object_manager.getObjByName("AttitudeState").metadata.updated()

        print("Install Observer for AttitudeState updates\n")
        self.object_manager.regObjectObserver(self.object_manager.getObjByName("AttitudeState"),
                                              self, "_onAttitudeUpdate")

    def _onAttitudeUpdate(self, args):
        print(args)

