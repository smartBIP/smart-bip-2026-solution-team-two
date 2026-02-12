import airsim
import time
import numpy as np
import cv2
import math
import winsound
import threading

# ======================================================
# FESTIVAL MEDICAL RESPONSE DRONE – SAFE VERSION
# ======================================================

print("Connecting to AirSim...")

client = airsim.MultirotorClient()
client.confirmConnection()

drone_name = "FestivalDrone"

# ------------------------------------------------------
# STORE HOME POSITION
# ------------------------------------------------------
home_state = client.getMultirotorState(vehicle_name=drone_name)
home_position = home_state.kinematics_estimated.position

HOME_X = home_position.x_val
HOME_Y = home_position.y_val
HOME_Z = home_position.z_val

CRUISE_ALTITUDE = -50   # 50 meters high
SAFE_DISTANCE = 30      # Detect obstacles early
STEP_SIZE = 6          # Move in small chunks
SPEED = 8               # Slow safe speed

print(f"Home position recorded: X={HOME_X}, Y={HOME_Y}")

# ------------------------------------------------------
# ARM
# ------------------------------------------------------
client.enableApiControl(True, drone_name)
client.armDisarm(True, drone_name)

# ------------------------------------------------------
# SMART STEP-BASED NAVIGATION
# ------------------------------------------------------

def play_speaker_message():
    try:
        winsound.PlaySound("emergency_message.wav", winsound.SND_FILENAME)
    except:
        print("Audio file not found.")

def smart_navigate(target_x, target_y, target_z):

    print("Starting smart navigation...")

    while True:

        state = client.getMultirotorState(vehicle_name=drone_name)
        current_pos = state.kinematics_estimated.position

        dx = target_x - current_pos.x_val
        dy = target_y - current_pos.y_val
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 5:
            print("Reached destination.")
            break

        # Calculate direction
        dir_x = dx / distance
        dir_y = dy / distance

        # -----------------------------
        # ROTATE DRONE TOWARD TARGET
        # -----------------------------
        yaw = math.degrees(math.atan2(dy, dx))
        client.rotateToYawAsync(yaw, vehicle_name=drone_name).join()

        next_x = current_pos.x_val + dir_x * STEP_SIZE
        next_y = current_pos.y_val + dir_y * STEP_SIZE

        # Read sensor AFTER rotation
        distance_data = client.getDistanceSensorData(
            distance_sensor_name="DistanceFront",
            vehicle_name=drone_name
        )

        print("Front distance:", distance_data.distance)

        if distance_data.distance != float('inf') and distance_data.distance < SAFE_DISTANCE:

            print("⚠ Obstacle detected! Trying lateral reroute...")

            client.hoverAsync(vehicle_name=drone_name).join()

            # Try LEFT (90° offset)
            left_yaw = yaw + 90
            client.rotateToYawAsync(left_yaw, vehicle_name=drone_name).join()

            client.moveByVelocityAsync(
                math.cos(math.radians(left_yaw)) * 5,
                math.sin(math.radians(left_yaw)) * 5,
                0,
                2,
                vehicle_name=drone_name
            ).join()

            time.sleep(1)

            continue

        # Move forward
        client.moveToPositionAsync(
            next_x,
            next_y,
            target_z,
            SPEED,
            vehicle_name=drone_name
        ).join()

    print("Navigation complete.")

# ------------------------------------------------------
# INCIDENT INPUT
# ------------------------------------------------------
print("\n==== INCIDENT DISPATCH SYSTEM ====")
incident_x = float(input("Enter Incident X coordinate: "))
incident_y = float(input("Enter Incident Y coordinate: "))

# ------------------------------------------------------
# TAKEOFF
# ------------------------------------------------------
print("Taking off...")
client.takeoffAsync(vehicle_name=drone_name).join()

print("Ascending to 50 meters...")
client.moveToZAsync(CRUISE_ALTITUDE, 5, vehicle_name=drone_name).join()

# ------------------------------------------------------
# FLY TO INCIDENT
# ------------------------------------------------------
smart_navigate(incident_x, incident_y, CRUISE_ALTITUDE)

client.hoverAsync(vehicle_name=drone_name).join()
print("Drone arrived at incident location.")

# ------------------------------------------------------
# ARRIVED AT INCIDENT (DESCEND FOR CLOSE INSPECTION)
# ------------------------------------------------------
INCIDENT_ALTITUDE = -10  # 20 meters above ground
SURVEILLANCE_DURATION = 25  # seconds, longer than audio

print("\nArrived above incident site at 50m.")
print(f"Descending to {abs(INCIDENT_ALTITUDE)} meters for close surveillance...")

client.moveToZAsync(INCIDENT_ALTITUDE, 3, vehicle_name=drone_name).join()
client.hoverAsync(vehicle_name=drone_name).join()
time.sleep(2)  # stabilize

# Play emergency audio in separate thread
print("Playing emergency speaker message...")
audio_thread = threading.Thread(target=play_speaker_message)
audio_thread.start()

# Lock current position
state = client.getMultirotorState(vehicle_name=drone_name)
incident_pos = state.kinematics_estimated.position

print("Hover locked at:")
print(f"X: {incident_pos.x_val:.2f}")
print(f"Y: {incident_pos.y_val:.2f}")
print(f"Z: {incident_pos.z_val:.2f}")

print(f"\nStarting {SURVEILLANCE_DURATION}-second surveillance capture...")

start_time = time.time()
frame_count = 0

while time.time() - start_time < SURVEILLANCE_DURATION:

    # Maintain position lock at INCIDENT_ALTITUDE
    client.moveToPositionAsync(
        incident_pos.x_val,
        incident_pos.y_val,
        INCIDENT_ALTITUDE,
        2,
        vehicle_name=drone_name
    )

    # Capture frame
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    ], vehicle_name=drone_name)

    img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
    img_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)

    filename = f"surveillance_frame_{frame_count+1}.png"
    cv2.imwrite(filename, img_rgb)

    print(f"Captured {filename}")
    frame_count += 1

    time.sleep(1)

print("Surveillance complete.")
# Ensure audio finishes before proceeding
audio_thread.join()


# ------------------------------------------------------
# RETURN TO BASE
# ------------------------------------------------------
print("Returning to base...")
smart_navigate(HOME_X, HOME_Y, CRUISE_ALTITUDE)

print("Descending...")
client.moveToZAsync(-5, 3, vehicle_name=drone_name).join()

print("Landing...")
client.landAsync(vehicle_name=drone_name).join()

# ------------------------------------------------------
# DISARM
# ------------------------------------------------------
client.armDisarm(False, drone_name)
client.enableApiControl(False, drone_name)

print("Mission completed successfully.")
