import logging
import os
import time
import pandas as pd
import utilities

# Set up Logger
logger = logging.getLogger("Powertrain Optimization")
logger.setLevel(logging.WARNING)
ch = logging.StreamHandler()
ch.setFormatter(logging.Formatter('%(asctime)s %(name)s %(levelname)-8s %(message)s'))
logger.addHandler(ch)
fh = logging.FileHandler("Log.log")
fh.setFormatter(logging.Formatter('%(asctime)s %(name)s %(levelname)-8s %(message)s'))
logger.addHandler(fh)

class OperationalMode:
    def __init__(self, thrust_req, prop_dir, motor_dir):
        self.thrust_req = thrust_req
        self.prop_dir = prop_dir
        self.motor_data = self.read_motor_dir(motor_dir)

    def read_motor_dir(self, motor_dir):
        motor_data = {
            'ID': [],
            'kV': [],
            'i0': [],
            'R': []
        }

        for filename in os.listdir(motor_dir):
            if filename.endswith('.txt'):
                filepath = os.path.join(motor_dir, filename)
                id = os.path.splitext(filename)[0]

                try:
                    with open(filepath, 'r') as file:
                        lines = file.readlines()
                        if len(lines) == 3:
                            motor_data['name'].append(id)
                            motor_data['kV'].append(float(lines[0].strip()) / 60)  # rps/volt
                            motor_data['i0'].append(float(lines[1].strip()))        # Amps
                            motor_data['R'].append(float(lines[2].strip()))         # Ohms
                        else:
                            logger.error(f"File {filename} does not contain three lines.")
                            print(f"File {filename} does not contain three lines. See README for formatting details.")
                except Exception as e:
                    logger.error(f"Error reading {filename}: {e}")
                    print(f"Error reading {filename}: {e}. See README for formatting details.")

        motor_df = pd.DataFrame(motor_data)
        logger.info("Processed motor directory")

        return motor_df
    
class StaticThrust(OperationalMode):
    def __init__(self, thrust_req, prop_dir, motor_dir):
        super().__init__(thrust_req, prop_dir, motor_dir)
        self.prop_data = self.read_prop_dir(prop_dir)

        logger.info("Initialized StaticThrust object")

    def read_prop_dir(prop_dir):
        prop_data = {
            'ID': [],
            'diameter': [],
            'pitch': [],
            'RPM': [],
            'Ct': [],
            'Cp': []
        }

        for filename in os.listdir(prop_dir):
            if filename.endswith('.txt') and 'static' in filename:
                filepath = os.path.join(prop_dir, filename)
                parts = filename.split('_')

                # Extract make and model
                make = parts[0] 
                diameter_pitch = parts[1] 
                diameter, pitch = map(float, diameter_pitch.split('x'))
                model = parts[3].split('.')[0] 
                id = f"{make}-{model}" 

                try:
                    with open(filepath, 'r') as file:
                        lines = file.readlines()
                        if len(lines) >= 2:
                            for line in lines[1:]:  # Skip header line
                                if line.strip():  # Skip empty lines
                                    rpm, ct, cp = map(float, line.strip().split())
                                    prop_data['ID'].append(id)
                                    prop_data['diameter'].append(diameter)
                                    prop_data['pitch'].append(pitch)
                                    prop_data['RPM'].append(rpm)
                                    prop_data['Ct'].append(ct)
                                    prop_data['Cp'].append(cp)
                        else:
                            logger.error(f"File {filename} does not contain enough lines.")
                            print(f"File {filename} does not contain enough lines. See README for formatting details.")
                except Exception as e:
                    logger.error(f"Error reading {filename}: {e}")
                    print(f"Error reading {filename}: {e}. See README for formatting details.")

                prop_df = pd.DataFrame(prop_data)
                logger.info("Processed propeller directory")

                return prop_df

class StableFlight(OperationalMode):
    def __init__(self, thrust_req, prop_dir, motor_dir, target_velocity):
        super().__init__(thrust_req, prop_dir, motor_dir)
        self.target_velocity = target_velocity
        self.prop_data = self.read_prop_dir(prop_dir)

        logger.info("Initialized StableFlight object")


def get_user_inputs():
    try:
        # Get the operation mode
        while True:
            operation_mode = input("Enter operation mode (static thrust or stable flight): ").strip().lower()
            if operation_mode in ["static thrust", "stable flight"]:
                print("See the README to ensure your data files are formatted properly.")
                break
            else:
                logger.error("Invalid operation mode input.")
                print("Invalid input. Please enter 'static thrust' or 'stable flight'.")
        
        # Get the number of propellers
        while True:
            try:
                num_props = int(input("Enter number of propellers: "))
                if num_props > 0:
                    break
                else:
                    logger.error("Number of propellers not entered as a positive integer.")
                    print("Number of propellers must be a positive integer.")
            except ValueError:
                logger.error("Number of propellers not entered as a positive integer.")
                print("Invalid input. Please enter a positive integer.")
        
        # Get the expected vehicle weight
        while True:
            try:
                weight = float(input("Enter expected vehicle weight (kg): "))
                if weight > 0:
                    break
                else:
                    logger.error("Vehicle weight not entered as a positive number.")
                    print("Vehicle weight must be a positive number.")
            except ValueError:
                logger.error("Vehicle weight not entered as a positive number.")
                print("Invalid input. Please enter a positive number.")
        
        # Get the thrust factor
        while True:
            try:
                thrust_factor = float(input("Enter thrust factor: "))
                if thrust_factor > 0:
                    break
                else:
                    logger.error("Thrust factor not entered as a positive number.")
                    print("Thrust factor must be a positive number.")
            except ValueError:
                logger.error("Thrust factor not entered as a positive number.")
                print("Invalid input. Please enter a positive number.")

        target_velocity = 0        
        if operation_mode == 'stable flight':
            while True:
                try:
                    target_velocity = float(input("Enter target velocity in m/s: "))
                    if target_velocity > 0:
                        break
                    else:
                        logger.error("Target velocity not entered as a positive number.")
                        print("Target velocity must be a positive number.")
                except ValueError:
                    logger.error("Target velocity not entered as a positive number.")
                    print("Invalid input. Plese enter a positive number.")
        else:
            print("Since the selected mode is 'static thrust', target velocity is unnecessary.")
        
        # Get the motor directory
        while True:
            motor_dir = input("Enter motor directory: ").strip()
            if os.path.isdir(motor_dir):
                break
            else:
                logger.error("Invalid motor directory entered.")
                print("Invalid motor directory.")

        # Get the propeller directory
        while True:
            prop_dir = input("Enter propeller directory: ").strip()
            if os.path.isdir(prop_dir):
                break
            else:
                logger.error("Invalid propellor directory entered.")
                print("Invalid propeller directory.")

        # Log the inputs
        logger.info(f"Operation mode: {operation_mode}")
        logger.info(f"Number of propellers: {num_props}")
        logger.info(f"Expected vehicle weight: {weight} kg")
        logger.info(f"Thrust factor: {thrust_factor}")
        logger.info(f"Target velocity: {target_velocity} m/s")
        logger.info(f"Motor motor_dir: {motor_dir}")
        logger.info(f"Propeller motor_dir: {prop_dir}")
        
        return operation_mode, num_props, weight, thrust_factor, target_velocity, motor_dir, prop_dir
    
    except Exception as e:
        logger.error(f"Error while getting user inputs: {e}")
        raise

def get_thrust_req(num_props, weight, thrust_factor):
    """
    Calculate the thrust required from each propeller.

    :param num_props: Number of propellers
    :param weight: Total weight of the vehicle (kg)
    :param thrust_factor: Thrust factor (e.g., 1.2 for 20% extra thrust for stability)
    :return: Thrust required per propeller (N)
    """
    total_thrust = weight * thrust_factor * 9.81
    thrust_per_prop = total_thrust / num_props

    logger.info(f"Thrust required per propeller: {thrust_per_prop} N")
    return thrust_per_prop

def read_motor_dir(motor_dir):
    motor_data = {
        'ID': [],
        'kV': [],
        'i0': [],
        'R': []
    }

    for filename in os.listdir(motor_dir):
        if filename.endswith('.txt'):
            filepath = os.path.join(motor_dir, filename)
            id = os.path.splitext(filename)[0]
            
            try:
                with open(filepath, 'r') as file:
                    lines = file.readlines()
                    if len(lines) == 3:
                        motor_data['name'].append(id)
                        motor_data['kV'].append(float(lines[0].strip()) / 60)  # rps/volt
                        motor_data['i0'].append(float(lines[1].strip()))        # Amps
                        motor_data['R'].append(float(lines[2].strip()))         # Ohms
                    else:
                        print(f"File {filename} does not contain three lines. See README for formatting details.")
            except Exception as e:
                print(f"Error reading {filename}: {e}. See README for formatting details.")
    
    motor_df = pd.DataFrame(motor_data)
    logger.info("Processed motor directory")

    return motor_df

def main():
    print("\nWelcome to the powertrain optimizer for a propeller-based vehicle!")
    operation_mode, num_props, weight, thrust_factor, target_velocity, motor_dir, prop_dir = get_user_inputs()
    thrust_req = get_thrust_req(num_props, weight, thrust_factor)


if __name__ == 'main':
    start = time.time()
    main()
    end = time.time()
    total = end - start
    print(f"Time elapsed: {total} s")
    print("Done.")