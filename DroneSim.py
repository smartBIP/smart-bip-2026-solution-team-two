import airsim
import numpy as np
import random
from report import Report, getReports

# ======================================================
# FESTIVAL MEDICAL RESPONSE DRONE – CLEAN VERSION
# ======================================================

# Setup AirSim Client and Initialize Drone
client = airsim.MultirotorClient()
client.confirmConnection()
drone_name = "FestivalDrone"
reports= getReports(client, drone_name)
for report in reports:
    client.enableApiControl(True, drone_name)
    client.armDisarm(True, drone_name)

    print("\n==== INCIDENT DISPATCH SYSTEM ====")

    
    drone_report = report
    drone_report.run()
    print(f"Created report: {drone_report}")
drone_report.gohome();

print("Mission completed successfully.")
