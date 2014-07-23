import serial
import struct
import time
import RobotRaconteur as RR
import thread
import threading
import numpy
import traceback

# w = RobotRaconteur.Connect('tcp://localhost:3400/{0}/WheelChair')
#Port names and NodeID of this service
serial_port_name="/dev/ttyACM0"
#This NodeID must be different for every instance.  Change this to a new value.
wheelChair_nodeid="{8519ee12-eb36-4f72-b434-4284f7dba8e8}"

wheelChair_servicedef="""
#Service to provide sample interface to the iRobot Create
service WheelChair_interface

option version 0.3

object WheelChair
    function void Drive(uint8 velocity, int8 axis)
    function void Stop()
end object
"""


class WheelChair_impl(object):
    def __init__(self):
        self._lock=threading.RLock()
        
    def Drive(self,value,axis):
        with self._lock:
            if axis == -1:
                dat=struct.pack("BB",48,value) 
                self._serial.write(dat)
            if axis == 1:
                dat=struct.pack("BB",50,value)
                self._serial.write(dat)
	    print value
	    print axis

    def Stop(self):
        with self._lock:
            dat=struct.pack("BB",49,0)
            self._serial.write(dat)

    def Shutdown(self):
        with self._lock:
            self._serial.close()

    def Init(self, port):
        with self._lock:
            self._serial=serial.Serial(port="/dev/ttyACM0",baudrate=115200)
            


def main():

    #Enable numpy
    RR.RobotRaconteurNode.s.UseNumPy=True

    #Set the node id and node id
    RR.RobotRaconteurNode.s.NodeID=RR.NodeID(wheelChair_nodeid)
    RR.RobotRaconteurNode.s.NodeName="WheelChairServer"

    #Initialize the object in the service
    obj=WheelChair_impl()
    obj.Init(serial_port_name)



    #Create the transport, register it, and start the server
    t=RR.TcpTransport()
    RR.RobotRaconteurNode.s.RegisterTransport(t)
    t.StartServer(3400) #random port, any unused port is fine
    t.EnableNodeAnnounce(RR.IPNodeDiscoveryFlags_NODE_LOCAL | RR.IPNodeDiscoveryFlags_LINK_LOCAL | RR.IPNodeDiscoveryFlags_SITE_LOCAL)

    #Register the service type and the service
    RR.RobotRaconteurNode.s.RegisterServiceType(wheelChair_servicedef)
    RR.RobotRaconteurNode.s.RegisterService("WheelChair","WheelChair_interface.WheelChair",obj)

    #Wait for the user to stop the server
    raw_input("Server started, press enter to quit... use  w = RobotRaconteur.Connect('tcp://localhost:3400/{0}/WheelChair')")

    #Shutdown
    obj.Shutdown()

    #You MUST shutdown or risk segfault...
    RR.RobotRaconteurNode.s.Shutdown()

if __name__ == '__main__':
    main()
