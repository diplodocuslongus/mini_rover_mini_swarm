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
master.wait_heartbeat(timeout = 5)
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

def set_gps_global_origin(latitude, longitude, altitude):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,  # Confirmation
        1,  # Use current position: 1 (true)
        0, 0, 0,  # Unused parameters
        latitude / 1e7,  # Latitude in degrees (scaled by 1e7)
        longitude / 1e7,  # Longitude in degrees (scaled by 1e7)
        altitude  # Altitude in meters
    )

def arm():
    print("Arming...")
    if not send_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1):
        print("Failed to arm. Check safety switch, RC, and sensors.")
        return False
    return True

def disarm():
    print("Disarming...")
    if not send_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0):
        print("Failed to disarm. Check safety switch, RC, and sensors.")
        return False
    return True

def print_wheel_distance(duration):
    end_time = time.time() + duration
    while time.time() < end_time:
        msg = master.recv_match(type='WHEEL_DISTANCE', blocking=True, timeout=1)
        if msg is not None:
            print(f"Wheel distances: {msg.distance}")

# Set GPS origin
# print(f"Setting gps origin")
# set_gps_global_origin(-24774307,121045573, 100)
# time.sleep(1)  # Wait for the command to be processed
print(f"Done.")

# Arm the rover
# if not arm():
#     exit(1)

# Wait a moment for mode change
time.sleep(1)

# Send movement command: 1m forward at 0.2 m/s (adjust sign if wheels spin backward)
print("Sending movement command...")
def send_pos(pos):
    master.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        # mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b110111111000,  # type_mask: Use position (x) and velocity (vx)
        pos, 0, 0,  # x, y, z positions (meters)
        0, 0, 0,  # x, y, z velocities (m/s)
        0, 0, 0,  # x, y, z accelerations (not used)
        0, 0  # yaw, yaw_rate (not used)
    )
def send_pos_vel(pos,vel):
    master.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        # mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b110111100111,  # type_mask: Use position (x) and velocity (vx)
        pos, 0, 0,  # x, y, z positions (meters)
        vel, 0, 0,  # x, y, z velocities (m/s)
        0, 0, 0,  # x, y, z accelerations (not used)
        0, 0  # yaw, yaw_rate (not used)
    )

def send_vel(vx,vy,vz,duration,cmd_send_rate):
    end_time = time.time() + duration
    while time.time() < end_time:
        master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            # mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # type_mask: Use velocity (vx)
            0, 0, 0,  # x, y, z positions (meters)
            vx, vy, vz,  # x, y, z velocities (m/s)
            0, 0, 0,  # x, y, z accelerations (not used)
            0, 0  # yaw, yaw_rate (not used)
        )
        time.sleep(1.0/cmd_send_rate)

# send_pos_vel(0.6,0.2)
# send_vel(0.2,0.0,0.0,2,10)
send_pos(0.6)
print("Movement command sent. Monitoring wheel distance for 10 seconds...")
print_wheel_distance(5)  # Monitor wheel distance for 10 seconds

# Disarm the rover
if not disarm():
    exit(1)

