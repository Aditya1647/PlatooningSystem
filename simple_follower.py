import carla
import time
import math
import logging
from threading import Thread

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def normalize_angle(angle):
    """
    Normalize an angle to the range [-pi, pi].
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    
    try:
        world = client.get_world()
    except Exception as e:
        logger.error(f"Failed to get CARLA world: {e}")
        return

    blueprint_library = world.get_blueprint_library()

    # Choose vehicle blueprints
    lead_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
    follower_bp = blueprint_library.filter('vehicle.tesla.model3')[0]

    # Set spawn points
    spawn_points = world.get_map().get_spawn_points()
    if len(spawn_points) < 2:
        logger.error("Not enough spawn points available.")
        return

    lead_transform = spawn_points[0]
    follower_transform = spawn_points[1]

    # Spawn the lead vehicle
    try:
        lead_vehicle = world.spawn_actor(lead_bp, lead_transform)
        lead_vehicle.set_autopilot(True)
        logger.info("Lead vehicle spawned and autopilot enabled.")
    except Exception as e:
        logger.error(f"Failed to spawn lead vehicle: {e}")
        return

    # Spawn the follower vehicle
    try:
        follower_vehicle = world.spawn_actor(follower_bp, follower_transform)
        logger.info("Follower vehicle spawned.")
    except Exception as e:
        logger.error(f"Failed to spawn follower vehicle: {e}")
        lead_vehicle.destroy()
        return

    # Set up the simulation parameters
    target_distance = 10.0  # Desired distance in meters
    kp = 0.5  # Proportional gain for speed control
    kd = 0.1  # Derivative gain for speed control (optional)
    max_throttle = 0.7
    max_brake = 1.0
    epsilon = 1e-3  # Small value to prevent division by zero

    # Initialize previous speed error for derivative control (if using PID)
    previous_speed_error = 0.0

    # Optional: Synchronize the follower's initial speed with the lead
    try:
        lead_velocity = lead_vehicle.get_velocity()
        lead_speed = math.sqrt(lead_velocity.x**2 + lead_velocity.y**2 + lead_velocity.z**2)
        control = carla.VehicleControl(throttle=0.5)  # Moderate throttle to reach desired speed
        follower_vehicle.apply_control(control)
        time.sleep(1)  # Allow some time to adjust speed
        follower_vehicle.apply_control(carla.VehicleControl(throttle=0.0))
        logger.info("Initialized follower vehicle speed.")
    except Exception as e:
        logger.warning(f"Failed to initialize follower speed: {e}")

    try:
        while True:
            # Get the state of the lead vehicle
            lead_transform = lead_vehicle.get_transform()
            lead_velocity = lead_vehicle.get_velocity()
            lead_speed = math.sqrt(lead_velocity.x**2 + lead_velocity.y**2 + lead_velocity.z**2)

            # Get the state of the follower vehicle
            follower_transform = follower_vehicle.get_transform()
            follower_velocity = follower_vehicle.get_velocity()
            follower_speed = math.sqrt(follower_velocity.x**2 + follower_velocity.y**2 + follower_velocity.z**2)

            # Calculate the distance between the vehicles
            distance = lead_transform.location.distance(follower_transform.location)

            # Longitudinal control: Adjust speed to maintain the target distance
            speed_error = (distance - target_distance)
            desired_speed = lead_speed + kp * speed_error

            # Optional: Derivative term for smoother control
            derivative = speed_error - previous_speed_error
            throttle = 0.0
            brake = 0.0

            if desired_speed > follower_speed:
                throttle = min(max_throttle, (desired_speed - follower_speed) / (desired_speed + epsilon))
            else:
                # Prevent division by zero
                speed_diff = follower_speed - desired_speed
                brake = min(max_brake, speed_diff / (follower_speed + epsilon))

            previous_speed_error = speed_error

            # Lateral control: Steer towards the lead vehicle's position
            target_vector = lead_transform.location - follower_transform.location
            target_yaw = math.atan2(target_vector.y, target_vector.x)
            current_yaw = math.radians(follower_transform.rotation.yaw)
            yaw_error = normalize_angle(target_yaw - current_yaw)

            # Proportional control for steering
            steer = max(-1.0, min(1.0, yaw_error * kp))  # Adjust kp as needed for steering sensitivity

            # Apply control to the follower vehicle
            control = carla.VehicleControl(throttle=throttle, brake=brake, steer=steer)
            follower_vehicle.apply_control(control)

            # Log the current state (optional)
            logger.debug(f"Distance: {distance:.2f}m | Desired Speed: {desired_speed:.2f}m/s | "
                         f"Follower Speed: {follower_speed:.2f}m/s | Throttle: {throttle:.2f} | "
                         f"Brake: {brake:.2f} | Steer: {steer:.2f}")

            # Wait for the next simulation step
            time.sleep(0.05)  # 20 Hz control loop

    except KeyboardInterrupt:
        logger.info("Simulation interrupted by user.")

    except Exception as e:
        logger.error(f"An error occurred during simulation: {e}")

    finally:
        # Clean up actors
        try:
            lead_vehicle.destroy()
            logger.info("Lead vehicle destroyed.")
        except Exception as e:
            logger.warning(f"Failed to destroy lead vehicle: {e}")

        try:
            follower_vehicle.destroy()
            logger.info("Follower vehicle destroyed.")
        except Exception as e:
            logger.warning(f"Failed to destroy follower vehicle: {e}")

def run_main():
    """
    Run the main function and handle unexpected exceptions.
    """
    try:
        main()
    except Exception as e:
        logger.error(f"Unexpected error: {e}")

if __name__ == '__main__':
    # Run main in a separate thread to allow graceful shutdown
    thread = Thread(target=run_main)
    thread.start()
    try:
        while thread.is_alive():
            thread.join(timeout=1.0)
    except KeyboardInterrupt:
        logger.info("Program interrupted by user. Exiting...")

