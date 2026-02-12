import airsim
import time
import math
import os
import winsound
import threading
import numpy as np
import cv2

class Report:
    def __init__(self, client, drone_name, title, severity, location, x, y, time, description, image=None, audio=None, isFatal=False):
        # Drone connection and report data
        self.client = client
        self.drone_name = drone_name
        self.title = title
        self.severity = severity
        self.location = location
        self.x = x
        self.y = y
        self.time = time
        self.description = description
        self.image = image
        self.audio = audio
        self.isFatal = isFatal

        # Setup constants
        self.CRUISE_ALTITUDE = -50  # 50 meters high
        self.SAFE_DISTANCE = 20     # Detect obstacles early
        self.STEP_SIZE = 6          # Move in small chunks
        self.SPEED = 8              # Slow safe speed
        self.INCIDENT_ALTITUDE = -10  # 20 meters above ground
        self.SURVEILLANCE_DURATION = 20  # seconds

    def __str__(self):
        return f"Report: {self.title}\nSeverity: {self.severity}\nLocation: {self.location}\nTime: {self.time}\nDescription: {self.description}\nFatal: {'Yes' if self.isFatal else 'No'}"

    def play_speaker_message(self):
        try:
            winsound.PlaySound("emergency_message.wav", winsound.SND_FILENAME)
        except:
            print("Audio file not found.")

    def navigate(self, target_x, target_y, target_z):
        print("Starting smart navigation...")

        while True:
            state = self.client.getMultirotorState(vehicle_name=self.drone_name)
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
            self.client.rotateToYawAsync(yaw, vehicle_name=self.drone_name).join()

            next_x = current_pos.x_val + dir_x * self.STEP_SIZE
            next_y = current_pos.y_val + dir_y * self.STEP_SIZE

            # Read sensor AFTER rotation
            distance_data = self.client.getDistanceSensorData(
                distance_sensor_name="DistanceFront",
                vehicle_name=self.drone_name
            )

            print("Front distance:", distance_data.distance)

            if distance_data.distance != float('inf') and distance_data.distance < self.SAFE_DISTANCE:
                print("Obstacle detected! Trying lateral reroute...")

                self.client.hoverAsync(vehicle_name=self.drone_name).join()

                # Try LEFT (90° offset)
                left_yaw = yaw + 90
                self.client.rotateToYawAsync(left_yaw, vehicle_name=self.drone_name).join()

                self.client.moveByVelocityAsync(
                    math.cos(math.radians(left_yaw)) * 5,
                    math.sin(math.radians(left_yaw)) * 5,
                    0,
                    2,
                    vehicle_name=self.drone_name
                ).join()

                time.sleep(1)
                continue

            # Move forward
            self.client.moveToPositionAsync(
                next_x,
                next_y,
                target_z,
                self.SPEED,
                vehicle_name=self.drone_name
            ).join()

        print("Navigation complete.")

    def capture_surveillance(self, duration, angles=8, radius=5):
    
        import math
        safe_title = self.title.replace(' ', '_')
        folder_path = os.path.join("surveillance_images", safe_title)
        os.makedirs(folder_path, exist_ok=True)

        print(f"\nStarting {duration}-second multi-angle surveillance for '{self.title}'...")
        print(f"Images will be saved to: {folder_path}")

        start_time = time.time()
        frame_count = 0

    # Get the drone's current position (incident center)
        state = self.client.getMultirotorState(vehicle_name=self.drone_name)
        center_pos = state.kinematics_estimated.position

    # Precompute positions around the incident (circle)
        positions = []
        for i in range(angles):
            angle_rad = 2 * math.pi * i / angles
            x = center_pos.x_val + radius * math.cos(angle_rad)
            y = center_pos.y_val + radius * math.sin(angle_rad)
            z = self.INCIDENT_ALTITUDE
            positions.append((x, y, z))

    # Loop through positions until duration ends
        while time.time() - start_time < duration:
            for pos in positions:
            # Move to position
                self.client.moveToPositionAsync(
                    pos[0], pos[1], pos[2], 2, vehicle_name=self.drone_name
                ).join()

            # Hover briefly for stable capture
                self.client.hoverAsync(vehicle_name=self.drone_name).join()
                time.sleep(0.5)

            # Capture frame
                responses = self.client.simGetImages(
                    [airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)],
                    vehicle_name=self.drone_name
                )
                img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
                frame = img1d.reshape(responses[0].height, responses[0].width, 3).copy()

            # Save frame
                filename = os.path.join(folder_path, f"frame_{frame_count+1}.png")
                cv2.imwrite(filename, frame)
                print(f"Captured {filename}")
                frame_count += 1

            # Stop if surveillance time exceeded
                if time.time() - start_time >= duration:
                    break

        print(f"Multi-angle surveillance complete for '{self.title}'.")

    def run(self):
        # Takeoff and ascend
        print("Taking off...")
        self.client.takeoffAsync(vehicle_name=self.drone_name).join()
        print("Ascending to 50 meters...")
        self.client.moveToZAsync(self.CRUISE_ALTITUDE, 5, vehicle_name=self.drone_name).join()

        # Navigate to the incident
        self.navigate(self.x, self.y, self.CRUISE_ALTITUDE)

        # Hover and play emergency message
        self.client.hoverAsync(vehicle_name=self.drone_name).join()
        print("Drone arrived at incident location.")
        print(f"Descending to {abs(self.INCIDENT_ALTITUDE)} meters for close surveillance...")
        self.client.moveToZAsync(self.INCIDENT_ALTITUDE, 3, vehicle_name=self.drone_name).join()
        self.client.hoverAsync(vehicle_name=self.drone_name).join()
        time.sleep(2)  # Stabilize

        # Play emergency audio
        print("Playing emergency speaker message...")
        audio_thread = threading.Thread(target=self.play_speaker_message)
        audio_thread.start()

        # Capture surveillance frames
        #self.capture_surveillance(self.SURVEILLANCE_DURATION)
        # Capture multi-angle surveillance frames
        self.capture_surveillance(self.SURVEILLANCE_DURATION, angles=8, radius=5)

        # Ensure audio finishes before proceeding
        audio_thread.join()

    def gohome(self):
        # Return to base
        print("Returning to base...")
        self.navigate(0, 0, self.CRUISE_ALTITUDE)
        print("Descending...")
        self.client.moveToZAsync(-5, 3, vehicle_name=self.drone_name).join()
        print("Landing...")
        self.client.landAsync(vehicle_name=self.drone_name).join()
        self.client.armDisarm(False, vehicle_name=self.drone_name)
        self.client.enableApiControl(False, vehicle_name=self.drone_name)


def getReports(client, drone_name):
    reports = [
        Report(
            client=client, drone_name=drone_name,
            title="Crash at Intersection",
            severity="High",
            location="Intersection of 5th and Main St.",
            x=50,
            y=50,
            time="2026-02-12 14:35",
            description="A minor car accident has occurred; the pedestrian’s condition is stable and not life-threatening.",
            image="injured_person.png",
            audio="accident_audio_001.wav",
            isFatal=False
        ),
        Report(
            client=client, drone_name=drone_name,
            title="Overturned Vehicle",
            severity="Medium",
            location="Near City Park, 3rd Ave.",
            x=25,
            y=10,
            time="2026-02-12 15:50",
            description="A major truck accident has occurred; the pedestrian is in critical condition. Guidance is provided to a bystander until paramedics arrive.",
            image="overturned_vehicle_image.png",
            audio="emergency_audio_002.wav",
            isFatal=True
        ),
        # Add more reports...
    ]
    return reports
