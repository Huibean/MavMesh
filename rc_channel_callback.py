from MavMesh import MavMesh
import time
from pymavlink import mavutil

def rc_channels_msg_callback(value):
    print("rc_channels:", value.chan1_raw, value.chan2_raw, value.chan3_raw, value.chan4_raw)
    if value.chan1_raw > 1700:
        print("chan1_raw > 1700")

    if value.chan2_raw > 1700:
        print("chan2_raw > 1700")

mavlink_callbacks = {
    'RC_CHANNELS': rc_channels_msg_callback,
}


system = MavMesh('0.0.0.0:14552')
system.run(callbacks=mavlink_callbacks)

system.enter_broadcast_mode()
system.select_broadcast_mode_mapping_copter()
system.select_vehicle(1)
