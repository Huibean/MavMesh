from MavMesh import MavMesh
import time
from pymavlink import mavutil

system = MavMesh('0.0.0.0:14552')
system.run()

system.enter_broadcast_mode()
system.select_broadcast_mode_mapping_copter()

while True:
    for i in range(3):
        system.mav.sysid = i
        print(f'target_system: {system.mav.target_system}, target_component: {system.mav.target_component}')
        system.set_mode('GUIDED')
        time.sleep(1)
        system.set_mode('LAND')
        time.sleep(1)
        system.set_mode('RTL')
        time.sleep(1)
        system.set_mode('ALT_HOLD')
        time.sleep(1)
        system.set_mode('ACRO')
        time.sleep(1)
