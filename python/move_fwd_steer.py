# move forward and/or steer
from pymavlink import mavutil
import time

# Connect to the autopilot via UDP (PicoW)
master = mavutil.mavlink_connection('udp:192.168.4.1:14550', source_system=255, input=False)

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

def drive_forward(distance, speed=0.1):
    # Time = Distance / Speed
    duration = distance / speed
    print(f"Driving forward {distance}m...")
    send_vel(speed, 0, 0, duration, 10)
    # Give the EKF a moment to settle
    time.sleep(0.5)

def turn_left_90(direction ="left"):
    # We use Mask 1511.
    # The last parameter is yaw_rate in radians/sec.
    # 20 degrees/sec = ~0.35 rad/s
    turn_rate_deg = 30 #30
    turn_rate_rad = turn_rate_deg * 3.14 / 180.0
    yaw_rate_rad = turn_rate_rad if direction == "right" else -turn_rate_rad
    # Duration to get 90 degrees at 20 deg/s is 4.5 seconds
    duration = 90 / turn_rate_deg  + 0.15
    
    print("Turning left 90 degrees...")
    end_time = time.time() + duration
    while time.time() < end_time:
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            1511,       # Mask 1511 (Uses Vel X and Yaw Rate)
            0, 0, 0,    # Position
            0, 0, 0,    # Velocity (Set to 0 to pivot in place)
            0, 0, 0,    # Acceleration
            0, yaw_rate_rad # Negative is usually Left in ArduPilot
        )
        time.sleep(0.1)
    
    # Explicit Stop
    send_vel(0, 0, 0, 0.5, 10)
    time.sleep(0.5)



# Set GPS origin
print(f"Setting gps origin")
# set_gps_global_origin(-247743070, 1210455730, 100) # bg 51
# set_gps_global_origin(-353621474, 1491651746, 600) # desert
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
    mask = 1511
    while time.time() < end_time:
        master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            # mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            # 0b110111110111,  # type_mask: Use velocity (vx), ignore yaw
            # 3575,  # same as above in decimal, doesn't move rover
            # 1527,
            mask,
            # 0b0000111111000111,  # type_mask: Use velocity (vx)
            0, 0, 0,  # x, y, z positions (meters)
            vx, 0,0 ,  # x, y, z velocities (m/s)
            # vx, vy, vz,  # x, y, z velocities (m/s)
            0, 0, 0,  # x, y, z accelerations (not used)
            0, 0  # yaw, yaw_rate (not used)
        )
        time.sleep(1.0/cmd_send_rate)
    print("Stopping...")
    # master.mav.set_position_target_local_ned_send(
    #     0,  master.target_system, master.target_component,
    #     mavutil.mavlink.MAV_FRAME_BODY_NED,
    #     0b0000111111000111,  0, 0, 0,  vx/2, 0, 0, 0, 0, 0,  0, 0  
    # )
    # time.sleep(1.0/cmd_send_rate)
    # to test...
    for _ in range(3):
        master.mav.set_position_target_local_ned_send(
            0,  master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,  0, 0, 0,  vx/2, 0, 0, 0, 0, 0,  0, 0  
        )
        time.sleep(1.0/cmd_send_rate)
    for _ in range(3):
        master.mav.set_position_target_local_ned_send(
            0,  master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,  0, 0, 0,  0, 0, 0, 0, 0, 0,  0, 0  
        )
        time.sleep(1.0/cmd_send_rate)

# send_pos_vel(0.5,0.2)
# send_vel(0.2,0.0,0.0,5,10)
# send_pos(0.3)
# --- EXECUTE PATH
for i in range(2):
    drive_forward(0.5)
    turn_left_90()
print("Movement command sent. Monitoring wheel distance for 10 seconds...")
print_wheel_distance(5)  # Monitor wheel distance for 10 seconds

# Disarm the rover
# if not disarm():
#     exit(1)

