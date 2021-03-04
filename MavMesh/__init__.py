from pymavlink import mavutil
import threading
import time
mavlink = mavutil.mavlink

class MavMesh(object):

    def __init__(self, device):
        self.mav = mavutil.mavlink_connection(
                device,
                autoreconnect=True,
                force_connected=True,
                source_system = 0,
                source_component = 0,
                )
        self._exit = False

    def run(self, callbacks={}):
        print("Connecting to vehicle...")
        self.send_heartbeat()
        self.mav.wait_heartbeat()
        self.mode_mapping = self.mav.mode_mapping()
        print("Vehicle connected")
        self.receive_thread = threading.Thread(target=self.receive_loop, args=(callbacks,))
        self.receive_thread.start()

    def exit(self):
        self._exit = True
        self.receive_thread.join()
    
    def send_heartbeat(self):
        self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                    0,
                                    0,
                                    0)

    def receive_loop(self, callbacks):
        print("callbacks:", callbacks)
        last_heartbeat = time.time()
        interesting_messages = list(callbacks.keys())
        while not self._exit:
            msg = self.mav.recv_match(type=interesting_messages, timeout=1, blocking=True)
            if msg:
                callbacks[msg.get_type()](msg)

            if time.time() - last_heartbeat > 0.2:
                self.send_heartbeat()
                last_heartbeat = time.time()

    @property
    def location(self):
        return self.mav.location()

    @property
    def current_vehicle_id(self):
        return self.mav.sysid

    @property
    def mode(self):
        return self.mav.flightmode

    def set_mode(self, name):
        mode = self.mode_mapping[name]
        self.mav.set_mode_apm(mode)

    @property
    def is_armed(self):
        return bool(self.mav.motors_armed())

    def arm(self):
        self.mav.arducopter_arm()

    def disarm(self):
        self.mav.arducopter_disarm()

    def select_vehicle(self, id):
        print(f"select vehicle {id}")
        self.mav.target_system = id

    def enter_broadcast_mode(self):
        print("entering broadcast mode...")
        self.mav.sysid = 0

    def send(self, msg):
        self.mav.mav.send(msg)

    def send_takeoff_command(self, altitude):
        self.mav.mav.command_long_send(self.mav.sysid, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)

    def send_ned_command(self, north, east, down):
        self.mav.mav.set_position_target_local_ned_send(0, self.mav.sysid, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000, north, east, down, 0, 0, 0, 0, 0, 0, 0, 0)

    def select_broadcast_mode_mapping_copter(self):
        map = mavutil.mode_mapping_acm
        inv_map = dict((a, b) for (b, a) in map.items())
        self.mode_mapping = inv_map

    def takeoff(self, altitude):
        self.mav.arducopter_arm()
        time.sleep(0.1)
        self.send_takeoff_command(altitude)

    def rtl(self):
        self.set_mode("RTL")

    def land(self):
        self.set_mode("LAND")
