from pymavlink import mavutil

#Constants
SERIAL_PORT_CUBEPILOT = "/dev/ttyACM0"

print(f"Attempting to Connect with CubePilot on {SERIAL_PORT_CUBEPILOT}")
CubePilot = mavutil.mavlink_connection(SERIAL_PORT_CUBEPILOT)
CubePilot.wait_heartbeat()
print(f"Heartbeat from system (system {CubePilot.target_system} component {CubePilot.target_component})")
print("Connection to CubePilot is Sucsessfull")