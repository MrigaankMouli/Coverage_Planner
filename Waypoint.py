from pymavlink import mavutil
import time
import json

def send_local_position(controller, north, east, down=-10):
    controller.mav.set_position_target_local_ned_send(
        0,  
        controller.target_system, controller.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b110111111000,
        north, east, down,  
        0, 0, 0,
        0, 0, 0,  
        0, 0  
    )

def get_lat_lon(controller):
    msg = controller.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    latitude = msg.lat
    longitude = msg.lon 
    return latitude, longitude

controller = mavutil.mavlink_connection("udpin:127.0.0.1:14550")
controller.wait_heartbeat()
print("Connected to vehicle")

controller.mav.command_long_send(
    controller.target_system,
    controller.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4,
    0,0,0,0,0
)


controller.mav.command_long_send(
    controller.target_system,
    controller.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0)

time.sleep(1)

controller.mav.command_long_send(
    controller.target_system,
    controller.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 10
)

print("Takeoff command sent. Waiting for takeoff to complete.")
time.sleep(10)

square_points = [
    (0, 0),
    (20, 0),
    (20, 20),
    (0, 20),
    (0, 0)
]

coordinates = []

for north, east in square_points:
    send_local_position(controller, north, east, -10)
    print(f"Moving to waypoint: North {north}, East {east}, Down -10")

    while True:
        msg = controller.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        current_north = msg.x
        current_east = msg.y
        if abs(current_north - north) < 1.0 and abs(current_east - east) < 1.0:
            print("Waypoint reached")
            lat, lon = get_lat_lon(controller)
            print(f"Current Lat, Lon: {lat}, {lon}")
            coordinates.append({"lat": lat, "lon": lon})
            break
        time.sleep(1)

controller.mav.command_long_send(
    controller.target_system,
    controller.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0,
    0,
    0,0,0,0,0
)

print("Landing initiated.")

with open('coverage_boundary.json', 'w') as json_file:
    json.dump(coordinates, json_file)
