from pymavlink import mavutil
import time

def arm(controller):
    print("Arming... ")
    try:
        controller.mav.command_long_send(
            controller.target_system,
            controller.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0, 0, 0, 0, 0, 0
        )
        print("Vehicle Armed Succesfully")
        return True
    except Exception as e:
        print(f"Failed to Arm. Error:{e}")
        return False


def disarm(controller):
    print("Disarming... ")
    try:
        controller.mav.command_long_send(
            controller.target_system,
            controller.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0, 0, 0, 0, 0, 0
        )
        print("Vehicle Disarmed Succesfully")
        return True
    except Exception as e:
        print(f"Failed to Disarm. Error:{e}")
        return False


def main():
    controller = mavutil.mavlink_connection("/dev/ttyACM0")
    controller.wait_heartbeat()
    print("Connected to vehicle")

    if not arm(controller):
        print("Arming Failed")
        return

    time.sleep(4)

    if not disarm(controller):
        print("Disarming Failed")
        return


if __name__ == "__main__":
    main()