import json
from pymavlink import mavutil
import math
import time
from math import radians, sin, cos, sqrt, atan2, degrees, pi, asin

class MissionItem:
    def __init__(self, i, current, x, y, z):
        self.seq = i
        self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        self.current = current
        self.auto = 1
        self.param1 = 5.0
        self.param2 = 1.0
        self.param3 = 0.0
        self.param4 = 0.0
        self.x = int(x)
        self.y = int(y)
        self.z = int(z)
        self.mission_type = 0

def set_speed(controller, speed, speed_type=1):

    try:
        controller.mav.command_long_send(
            controller.target_system,
            controller.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,  
            speed_type,  
            speed,       
            -1,          
            0, 0, 0, 0   
        )

        msg = controller.recv_match(type='COMMAND_ACK', blocking=True)
        if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print(f"Speed set to {speed} m/s successfully.")
        else:
            print(f"Failed to set speed: {msg.result}")
    except Exception as e:
        print(f"Error setting speed: {e}")


def load_square_vertices_from_json(file_path):
    with open(file_path, 'r') as f:
        return json.load(f)

def wait_for_ack(controller, msg_type, timeout=5):
    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = controller.recv_match(type=msg_type, blocking=False)
        if msg:
            return msg
        time.sleep(0.1)
    print(f"Timeout waiting for {msg_type} acknowledgment")
    return None

def upload_mission(controller, home_pos, vertices):
    try:
        controller.mav.mission_clear_all_send(
            controller.target_system,
            controller.target_component
        )
        time.sleep(1)

        mission_items = []

        takeoff_item = MissionItem(0, current=1, x=home_pos[0], y=home_pos[1], z=10)
        takeoff_item.command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        takeoff_item.param1 = 0
        takeoff_item.param2 = 0
        takeoff_item.param3 = 0
        takeoff_item.param4 = 0
        mission_items.append(takeoff_item)

        for i, vertex in enumerate(vertices[1:], start=1):
            lat = vertex['lat']
            lon = vertex['lon']
            waypoint_item = MissionItem(i, current=0, x=lat, y=lon, z=10)
            mission_items.append(waypoint_item)

        land_item = MissionItem(len(vertices), current=0, x=home_pos[0], y=home_pos[1], z=0)
        land_item.command = mavutil.mavlink.MAV_CMD_NAV_LAND
        mission_items.append(land_item)

        controller.mav.mission_count_send(
            controller.target_system,
            controller.target_component,
            len(mission_items),
            0
        )
        time.sleep(1)

        for item in mission_items:
            controller.mav.mission_item_int_send(
                controller.target_system,
                controller.target_component,
                item.seq,
                item.frame,
                item.command,
                item.current,
                item.auto,
                item.param1,
                item.param2,
                item.param3,
                item.param4,
                item.x,
                item.y,
                item.z,
                item.mission_type
            )
            time.sleep(0.2)

        print("Mission uploaded successfully")
        return True

    except Exception as e:
        print(f"Mission upload error: {e}")
        return False

def main():
    controller = mavutil.mavlink_connection("udpin:127.0.0.1:14550")
    controller.wait_heartbeat()
    print("Connected to vehicle")

    controller.mav.command_long_send(
        controller.target_system,
        controller.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION,
        0, 0, 0, 0, 0, 0, 0
    )

    set_speed(controller,speed=0.3,speed_type=0)

    home_pos = None
    while home_pos is None:
        msg = controller.recv_match(type='HOME_POSITION', blocking=True)
        if msg:
            home_pos = (msg.latitude, msg.longitude, 0)
            print(f"Home position: {home_pos[0]/1e7 , home_pos[1]/1e7}")

    square_vertices = load_square_vertices_from_json('coverage_boundary.json')

    controller.mav.command_long_send(
        controller.target_system,
        controller.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4,
        0,0,0,0,0
    )

    msg = controller.recv_match(type = "COMMAND_ACK", blocking = True)
    print(msg)

    controller.mav.command_long_send(
        controller.target_system,
        controller.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0, 
        0, 0, 0, 0, 0
    )
    
    msg = controller.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("Arming Failed")
        return

    time.sleep(2)


    controller.mav.command_long_send(
        controller.target_system,
        controller.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0, 0, 0, 0, 10
    )

    msg = controller.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print(f"Takeoff command failed: {msg.result}")
        return
    
    while True:
        msg = controller.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True)
        relative_alt = msg.relative_alt / 1000.0
        print(f"Current Relative Altitude: {relative_alt} meters")
        if relative_alt >=9.5:
            print("Target Altitude Reached")
            break

    if not upload_mission(controller, home_pos, square_vertices):
        print("Mission upload failed")
        return

    controller.mav.command_long_send(
        controller.target_system,
        controller.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(1)

    try:
        while True:
            msg = controller.recv_match(type='MISSION_CURRENT', blocking=False)
            if msg:
                print(f"Current mission item: {msg.seq}")
            time.sleep(1)
    except KeyboardInterrupt:
        print("Mission monitoring stopped")

    controller.mav.command_long_send(
        controller.target_system,
        controller.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,
        0, 0, 0, 0, 0, 0
    )
    print("Mission completed and vehicle disarmed.")

if __name__ == "__main__":
    main()
