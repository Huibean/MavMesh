from pymavlink import mavutil
import threading
import time

class MavMesh(object):

    def __init__(self, device):
        self.mav = mavutil.mavlink_connection('0.0.0.0:14552')

    def run(self):
        self.mav.wait_heartbeat()
        print("vehicle connected")
        self.receive_thread = threading.Thread(target=self.receive_loop, args=())
        self.receive_thread.start()

    def exit(self):
        self.receive_thread.join()

    def receive_loop(self):
        while True:
            msg = self.mav.recv_msg()

    @property
    def location(self):
        return master.mav.location()

    @property
    def current_vehicle_id(self):
        return self.mav.sysid

    @property
    def mode(self):
        return self.mav.flightmode

    def set_mode(self, name):
        self.mav.set_mode(name)

    @property
    def is_armed(self):
        return bool(self.mav.motors_armed())

    def arm(self):
        self.mav.arducopter_arm()

    def disarm(self):
        self.mav.arducopter_disarm()

    def select_vehicle(self, id):
        print(f"select vehicle {id}")
        self.mav.sysid = id

    def enter_broadcast_mode(self):
        print("entering broadcast mode...")
        self.mav.sysid = 0

    def takeoff(self, altitude):
        self.set_mode("GUIDED")
        time.sleep(0.1)
        self.mav.arducopter_arm()
        time.sleep(0.1)
        self.mav.mav.command_long_send(self.mav.sysid, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)

    def rtl(self):
        self.set_mode("RTL")

    def land(self):
        self.set_mode("LAND")
