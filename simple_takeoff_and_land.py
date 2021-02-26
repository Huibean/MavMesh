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

system.land()
time.sleep(1)
system.exit()
