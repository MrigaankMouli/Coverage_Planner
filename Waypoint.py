from pymavlink import mavutil
import time
import chrono


drone = mavutil.mavlink_connection("udpin:127.0.0.1:14550")

drone.wait_heartbeat()
print("Heartbeat from system(system %u component %u)" % (drone.target_system, drone.target_component))


drone.mav.command_long_send(
    drone.target_system,
    drone.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4,
    0,0,0,0,0
)

msg = drone.recv_match(type='COMMAND_ACK',blocking=True)
print(msg)  

drone.mav.command_long_send(
    drone.target_system,
    drone.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,
    0, 
    0, 0, 0, 0, 0
)

msg = drone.recv_match(type='COMMAND_ACK',blocking=True)
print(msg)

drone.mav.command_long_send(
    drone.target_system,
    drone.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0,
    0,
    0,0,0,0,7
)

msg = drone.recv_match(type='COMMAND_ACK',blocking=True).result
print(f"Commmand Result:{msg}")

while True:
    msg = drone.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True)
    relative_alt = msg.relative_alt / 1000.0
    print(f"Current Relative Altitude: {relative_alt} meters")
    if relative_alt >=6.5:
        print("Target Altitude Reached")
        break

type_mask = 0b110111111000

drone.mav.set_position_target_local_ned_send(
    10,                           
    drone.target_system,         
    drone.target_component,      
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Frame type
    int(type_mask),              # Type mask 
    10, 0, -7,                    # x, y, z positions (in meters)
    0, 0, 0,                     # vx, vy, vz velocities (Ignored with the Typemask)
    0, 0, 0,                     # afx, afy, afz accelerations (Ignored with the Typemask)
    0, 0                          # yaw, yaw_rate (Ignored for now)
)

msg = drone.recv_match(type='COMMAND_ACK',blocking=True).result
print(f"Commmand Result:{msg}")

drone.mav.set_position_target_local_ned_send(
    10,                           
    drone.target_system,         
    drone.target_component,      
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Frame type
    int(type_mask),              # Type mask 
    0, 0, -7,                    # x, y, z positions (in meters)
    0, 0, 0,                     # vx, vy, vz velocities (Ignored with the Typemask)
    0, 0, 0,                     # afx, afy, afz accelerations (Ignored with the Typemask)
    0, 0                          # yaw, yaw_rate (Ignored for now)
)


# time.sleep(7) 

msg = drone.recv_match(type='COMMAND_ACK',blocking=True).result
print(f"Commmand Result:{msg}")

print("Sending the Land Command")

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0,
    0,
    0,0,0,0,0
)

time.sleep(7)

msg = master.recv_match(type='COMMAND_ACK',blocking=True)
print(msg)

while True:
    print(master.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True).relative_alt/1000)
    if (master.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True).relative_alt)/1000 <= 0.05:
        print("Landed Succesfully")
        break    

