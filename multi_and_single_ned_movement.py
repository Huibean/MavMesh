from MavMesh import MavMesh
import time
from pymavlink import mavutil

system = MavMesh('0.0.0.0:14552')
system.run()

system.enter_broadcast_mode()
system.select_broadcast_mode_mapping_copter()
system.set_mode('GUIDED')
time.sleep(1)
system.takeoff(30)
time.sleep(10)

for i in range(2):
    system.send_ned_command(10, 10, -40)
    time.sleep(10)

    system.send_ned_command(0, 0, -40)
    time.sleep(10)

system.select_vehicle(1)
system.send_ned_command(10, 0, -40)
system.select_vehicle(2)
system.send_ned_command(-10, 0, -40)
time.sleep(10)
system.enter_broadcast_mode()
system.send_ned_command(0, 0, -40)
time.sleep(10)

system.land()

time.sleep(1)
system.exit()
