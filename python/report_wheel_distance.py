# read a couple of wheel distance samples
from pymavlink import mavutil
import time

# Connect to the autopilot via UDP (PicoW)
# master = mavutil.mavlink_connection('udp:192.168.4.16:14550')
# master = mavutil.mavlink_connection('udp:192.168.4.16:14550', source_system=255)
master = mavutil.mavlink_connection('udp:192.168.4.1:14550', source_system=255, input=False)
# master = mavutil.mavlink_connection('udpout:192.168.4.1:14550', source_system=255)
# master = mavutil.mavlink_connection('udpout:192.168.4.1:14550')

# Wait for the heartbeat
print("Waiting for heartbeat...")
master.wait_heartbeat(timeout = 3)
print(f"Heartbeat received from system (system {master.target_system} component {master.target_component})")

def send_command(command, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        command,
        0,  # confirmation
        param1, param2, param3, param4, param5, param6, param7
    )
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack is not None:
        print(f"Command {command} acknowledged: {mavutil.mavlink.enums['MAV_RESULT'][ack.result].name}")
        return ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    else:
        print(f"Command {command} not acknowledged")
        return False

# request wheel distance message

# master.mav.command_long_send(
#         master.target_system,
#         master.target_component,
#         mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
#         0,  # confirmation
#         mavutil.mavlink.MAVLINK_MSG_ID_WHEEL_DISTANCE,
#         1000000, # interval in us, here, 1Hz
#         0,0,0,0,0 # not used params
#     )



def print_wheel_distance(duration):
    end_time = time.time() + duration
    while time.time() < end_time:
        msg = master.recv_match(type='WHEEL_DISTANCE', blocking=True, timeout=2)
        if msg is not None:
            print(f"Wheel distances: {msg.distance}")

print(" Monitoring wheel distance for 10 seconds...")
print_wheel_distance(10)  # Monitor wheel distance for 10 seconds

