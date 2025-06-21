import carla
import time
import math
import logging
import numpy as np
import tflite_runtime.interpreter as tflite
from threading import Thread
import os

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

def load_tflite_model(model_path):
    """
    Load the TensorFlow Lite model.
    """
    if not os.path.isfile(model_path):
        logger.error(f"Model file not found at path: {model_path}")
        return None, None, None
    interpreter = tflite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    logger.info("TFLite model loaded successfully.")
    return interpreter, input_details, output_details

def predict_control(interpreter, input_details, output_details, inputs):
    """
    Perform inference using the TFLite model.
    """
    try:
        inputs = np.array(inputs, dtype=np.float32).reshape(input_details[0]['shape'])
        interpreter.set_tensor(input_details[0]['index'], inputs)
        interpreter.invoke()
        output_data = [interpreter.get_tensor(output_details[i]['index'])[0] for i in range(len(output_details))]
        throttle, brake, steer = output_data
        # Clamp outputs to valid ranges
        throttle = np.clip(throttle, 0.0, 1.0)
        brake = np.clip(brake, 0.0, 1.0)
        steer = np.clip(steer, -1.0, 1.0)
        return throttle, brake, steer
    except Exception as e:
        logger.error(f"Error during model prediction: {e}")
        return 0.0, 0.0, 0.0  # Safe fallback

def main():
    # Load the TFLite model
    model_path = "platooning_model.tflite"  # Path to your TFLite model
    interpreter, input_details, output_details = load_tflite_model(model_path)
    if interpreter is None:
        return  # Exit if model failed to load

    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)  # Increased timeout

    try:
        world = client.get_world()
    except Exception as e:
        logger.error(f"Failed to get CARLA world: {e}")
        return

    blueprint_library = world.get_blueprint_library()

    # Choose vehicle blueprints
    try:
        lead_bp = blueprint_library.find('vehicle.tesla.model3')
        follower_bp = blueprint_library.find('vehicle.tesla.model3')
    except Exception as e:
        logger.error(f"Vehicle blueprint not found: {e}")
        return

    # Set spawn points
    spawn_points = world.get_map().get_spawn_points()
    if len(spawn_points) < 2:
        logger.error("Not enough spawn points available.")
        return

    lead_transform = spawn_points[0]
    follower_transform = spawn_points[1]

    # Spawn the lead vehicle
    lead_vehicle = None
    try:
        lead_vehicle = world.spawn_actor(lead_bp, lead_transform)
        lead_vehicle.set_autopilot(True)
        logger.info("Lead vehicle spawned and autopilot enabled.")
    except Exception as e:
        logger.error(f"Failed to spawn lead vehicle: {e}")
        return

    # Spawn the follower vehicle
    follower_vehicle = None
    try:
        follower_vehicle = world.spawn_actor(follower_bp, follower_transform)
        logger.info("Follower vehicle spawned.")
    except Exception as e:
        logger.error(f"Failed to spawn follower vehicle: {e}")
        if lead_vehicle:
            lead_vehicle.destroy()
            logger.info("Lead vehicle destroyed due to follower spawn failure.")
        return

    # Set simulation to synchronous mode
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20 Hz
        world.apply_settings(settings)
        logger.info("Simulation set to synchronous mode.")
    except Exception as e:
        logger.error(f"Failed to set synchronous mode: {e}")
        # Continue without synchronous mode if desired

    # Set simulation parameters
    target_distance = 10.0  # Desired distance in meters
    previous_speed_error = 0.0  # Initialize previous speed error

    try:
        while True:
            # Advance simulation
            world.tick()

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

            # Prepare inputs for the model
            lead_yaw = math.radians(lead_transform.rotation.yaw)
            follower_yaw = math.radians(follower_transform.rotation.yaw)

            # Example update for previous_speed_error (you might need a better logic)
            speed_error = target_distance - distance
            # Optionally apply a filter or integral term
            previous_speed_error = speed_error

            inputs = [
                lead_speed,
                follower_speed,
                lead_yaw,
                follower_yaw,
                distance,
                target_distance,
                previous_speed_error,
            ]

            # Get the control outputs from the TFLite model
            throttle, brake, steer = predict_control(interpreter, input_details, output_details, inputs)

            # Apply control to the follower vehicle
            control = carla.VehicleControl(throttle=throttle, brake=brake, steer=steer)
            follower_vehicle.apply_control(control)

            # Log the current state
            logger.info(f"Distance: {distance:.2f}m | Throttle: {throttle:.2f} | Brake: {brake:.2f} | Steer: {steer:.2f}")

    except KeyboardInterrupt:
        logger.info("Simulation interrupted by user.")

    except Exception as e:
        logger.error(f"An error occurred during simulation: {e}")

    finally:
        # Restore original settings
        try:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
            logger.info("Simulation settings restored.")
        except Exception as e:
            logger.warning(f"Failed to restore simulation settings: {e}")

        # Clean up actors
        try:
            if lead_vehicle:
                lead_vehicle.destroy()
                logger.info("Lead vehicle destroyed.")
        except Exception as e:
            logger.warning(f"Failed to destroy lead vehicle: {e}")

        try:
            if follower_vehicle:
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
    run_main()
