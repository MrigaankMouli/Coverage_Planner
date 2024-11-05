from pymavlink import mavutil
import time

master = mavutil.mavlink_connection("udpin:127.0.0.1:14550")

master.wait_heartbeat()
print("Heartbeat from system(system %u component %u)" % (master.target_system, master.target_component))


master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4,
    0,0,0,0,0
)

msg = master.recv_match(type='COMMAND_ACK',blocking=True)
print(msg)  

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,
    0, 
    0, 0, 0, 0, 0
)

msg = master.recv_match(type='COMMAND_ACK',blocking=True)
print(msg)

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0,
    0,
    0,0,0,0,7
)

msg = master.recv_match(type='COMMAND_ACK',blocking=True).result
print(f"Commmand Result:{msg}")

while True:
    msg = master.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True)
    relative_alt = msg.relative_alt / 1000.0
    print(f"Current Relative Altitude: {relative_alt} meters")
    if relative_alt >=6.5:
        print("Target Altitude Reached")
        break

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

msg = master.recv_match(type='COMMAND_ACK',blocking=True)
print(msg)

while True:
    print(master.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True).relative_alt/1000)
    if (master.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True).relative_alt)/1000 <= 0.05:
        print("Landed Succesfully")
        break    


