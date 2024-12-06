import json
from pymavlink import mavutil
import time

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
    print(f"Setting speed to {speed} of type {speed_type}")
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
    if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print(f"Failed to set speed: {msg.result}")
    else:
        print(f"Speed set successfully.")

def load_home_position(controller):
    print("Loading home position...")
    controller.mav.command_long_send(
        controller.target_system,
        controller.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION,
        0, 0, 0, 0, 0, 0, 0
    )
    while True:
        msg = controller.recv_match(type='HOME_POSITION', blocking=True)
        if msg:
            print(f"Home position loaded: {msg.latitude}, {msg.longitude}")
            return (msg.latitude, msg.longitude, 0)

def load_lap_waypoints(file_path):
    print(f"Loading lap waypoints from {file_path}...")
    with open(file_path, 'r') as f:
        data = json.load(f)
        return [{"lat": wp["latitude"], "lon": wp["longitude"]} for wp in data.get("lap_waypoints", [])]

def upload_mission(controller, home_pos, vertices):
    print("Uploading mission...")
    controller.mav.mission_clear_all_send(controller.target_system, controller.target_component)
    time.sleep(1)
    mission_items = [MissionItem(0, current=1, x=home_pos[0], y=home_pos[1], z=6)]
    mission_items[0].command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    print("Takeoff waypoint added.")

    for i, vertex in enumerate(vertices[1:], start=1):
        mission_items.append(MissionItem(i, current=0, x=vertex['lat'], y=vertex['lon'], z=6))
        print(f"Waypoint {i} added: {vertex['lat']}, {vertex['lon']}")

    mission_items.append(MissionItem(len(vertices), current=0, x=home_pos[0], y=home_pos[1], z=0))
    mission_items[-1].command = mavutil.mavlink.MAV_CMD_NAV_LAND
    print("Landing waypoint added.")

    controller.mav.mission_count_send(controller.target_system, controller.target_component, len(mission_items), 0)
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
        print(f"Mission item {item.seq} uploaded.")

def arm_drone(controller):
    print("Arming drone...")
    controller.mav.command_long_send(
        controller.target_system,
        controller.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0, 0, 0, 0, 0
    )
    time.sleep(2)
    print("Drone armed.")

def takeoff_drone(controller, altitude):
    print(f"Taking off to {altitude} meters...")
    controller.mav.command_long_send(
        controller.target_system,
        controller.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0, 0, 0, 0, altitude
    )
    while True:
        msg = controller.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True)
        if msg.relative_alt / 1000.0 >= altitude - 0.5:
            break
    print(f"Reached {altitude} meters.")

def disarm_drone(controller):
    print("Disarming drone...")
    controller.mav.command_long_send(
        controller.target_system,
        controller.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,
        0,
        0, 0, 0, 0, 0
    )
    time.sleep(2)
    print("Drone disarmed.")

def start_mission(controller):
    print("Starting mission...")
    controller.mav.command_long_send(
        controller.target_system,
        controller.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0, 0, 0, 0, 0, 0, 0
    )

def main():
    print("Initializing connection...")
    controller = mavutil.mavlink_connection("udpin:127.0.0.1:14550")
    controller.wait_heartbeat()
    print("Connection established.")

    set_speed(controller, 0.5, 0)
    set_speed(controller, 0.5, 2)
    set_speed(controller, 0.5, 3)

    home_pos = load_home_position(controller)
    lap_waypoints = load_lap_waypoints("Test Mission(I'm scared af).json")
    arm_drone(controller)
    takeoff_drone(controller, 6)
    upload_mission(controller, home_pos, lap_waypoints)
    start_mission(controller)

    print("Mission complete. Landing...")
    disarm_drone(controller)

if __name__ == "__main__":
    main()
