import airsim
import math
import time

# set grid dimensions and cell size
grid_size = 10
cell_size = 10
flight_height = 4.0 # meters

# connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()

# take off from starting position
start_pose = client.simGetVehiclePose()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()

# get current position and orientation
start_pose = client.simGetVehiclePose()
start_pos = start_pose.position
start_yaw = math.degrees(airsim.to_eularian_angles(start_pose.orientation)[2])

# calculate grid boundaries
min_x = start_pos.x_val - (grid_size * cell_size / 2)
max_x = start_pos.x_val + (grid_size * cell_size / 2)
min_y = start_pos.y_val - (grid_size * cell_size / 2)
max_y = start_pos.y_val + (grid_size * cell_size / 2)

# fly over each grid cell
for i in range(grid_size):
    for j in range(grid_size):
        # calculate target position
        target_x = min_x + (i + 0.5) * cell_size
        target_y = min_y + (j + 0.5) * cell_size

        # move drone to target position at a fixed height
        target_pose = airsim.Pose(airsim.Vector3r(target_x, target_y, -flight_height), airsim.to_quaternion(0, 0, math.radians(start_yaw)))
        client.moveToPositionAsync(target_pose.position.x_val, target_pose.position.y_val, target_pose.position.z_val, 5,
                                    drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom, yaw_mode=airsim.YawMode(False, 0)).join()

        # check for collisions and stop if necessary
        collision_info = client.simGetCollisionInfo()
        while collision_info.has_collided:
            print("Collision detected! Stopping...")
            client.hoverAsync().join()
            time.sleep(1)
            collision_info = client.simGetCollisionInfo()

        # take photo or do other operations
        client.simPause(True)
        response = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene)])
        # process image data here
        client.simPause(False)

# land at starting position
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)