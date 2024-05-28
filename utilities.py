import numpy as np
import logging
import os

# Set up Logger
def setup_logger():
    logger = logging.getLogger("Powertrain Optimization")
    logger.setLevel(logging.WARNING)
    ch = logging.StreamHandler()
    ch.setFormatter(logging.Formatter('%(asctime)s %(name)s %(levelname)-8s %(message)s'))
    logger.addHandler(ch)
    fh = logging.FileHandler("Log.log")
    fh.setFormatter(logging.Formatter('%(asctime)s %(name)s %(levelname)-8s %(message)s'))
    logger.addHandler(fh)
    return logger

def get_user_inputs(logger):
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

def get_thrust_req(num_props, weight, thrust_factor, logger):
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

# Motor functions
def eta_m(self, i0, V, R, i):
        """
        :param i0: no load current in amps
        :param V: potential difference in volts
        :param R: resistance in ohms
        :param i: current in amps
        :return: dimensionless efficiency between 0 and 1
        """
        return ((i - i0) * (V - i * R)) / (V * i)

def P_shaft(i0, V, R, n, kV):
    """
    :param i0: no load current in amps
    :param V: potential difference in volts
    :param R: resistance in ohms
    :param n: rotations per second
    :param kV: rotations per second per volt
    :return: shaft power in watts
    """
    return ((V - n / kV) / R - i0) * n / kV

def current(V, n, kV, R):
    """
    :param V: potential difference in volts
    :param n: rotations per second
    :param kV: rotations per second per volt
    :param R: resistance in ohms
    :return: current in amps
    """
    return (V - n / kV) / R

def Q_m(i, i0, kV):
    """
    :param i: current in amps
    :param i0:  no load current in amps
    :param kV: rotations per second per volt
    :return: shaft torque in N m
    """
    return (i - i0) / kV

# Propeller functions
def J(V, n, D):
    """
    find J as a function of (V)elocity, propeller rotatio(n)s per second, and propeller (D)iameter

    :param V: airspeed in m/s
    :param n: propeller rps
    :param D: propeller diameter in m
    :return:
    """
    return V / (n * D)

def omega(V, J, D, handle_array=False):
    """
    :param J: advance ratio
    :param V: velocity in m/s
    :param D: propeller diameter in m
    :return: rotations per second
    """
    if handle_array:
        result = []
        for i in J:
            result.append(omega(V, i, D))
        return np.asarray(result)

    return V / (J * D)