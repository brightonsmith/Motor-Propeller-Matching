import numpy as np
import logging
import os

# Set up Logger
def setup_logger():
    if os.path.exists('Log.log'):
        os.remove('Log.log')

    logger = logging.getLogger("Powertrain Optimization")
    logger.setLevel(logging.DEBUG)  # Set logger level to DEBUG to capture all levels of log messages

    ch = logging.StreamHandler()
    ch.setLevel(logging.WARNING)  # Console handler set to WARNING level
    ch.setFormatter(logging.Formatter('%(asctime)s %(name)s %(levelname)-8s %(message)s'))
    logger.addHandler(ch)

    fh = logging.FileHandler("Log.log")
    fh.setLevel(logging.INFO)  # File handler set to INFO level
    fh.setFormatter(logging.Formatter('%(asctime)s %(name)s %(levelname)-8s %(message)s'))
    logger.addHandler(fh)
    
    return logger

def get_user_inputs(logger):
    try:
        # Get the flight mode
        while True:
            flight_mode = input("Enter flight mode (static thrust or stable flight): ").strip().lower()
            if flight_mode in ["static thrust", "stable flight"]:
                print("See the README to ensure your data files are formatted properly.")
                break
            else:
                logger.error("Invalid operation mode input.")
                print("Invalid input. Please enter 'static thrust' or 'stable flight'.")

        # Get data type
        while True:
            data_type = input("Enter data type (UIUC or TYTO): ").strip().upper()
            if data_type in ["UIUC", "TYTO"]:
                print("See the README to ensure your data files are formatted properly.")
                break
            else:
                logger.error("Invalid data type input.")
                print("Invalid input. Please enter 'UIUC' or 'TYTO'.")
        
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
                thrust_factor = float(input("Enter desired thrust-to-weight ratio: "))
                if thrust_factor > 0:
                    break
                else:
                    logger.error("Thrust-to-weight ratio not entered as a positive number.")
                    print("Thrust-to-weight ratio must be a positive number.")
            except ValueError:
                logger.error("Thrust-to-weight ratio not entered as a positive number.")
                print("Invalid input. Please enter a positive number.")

        target_velocity = 0        
        if flight_mode == 'stable flight':
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
        logger.info(f"Flight mode: {flight_mode}")
        logger.info(f"Data type: {data_type}")
        logger.info(f"Number of propellers: {num_props}")
        logger.info(f"Expected vehicle weight: {weight} kg")
        logger.info(f"Thrust factor: {thrust_factor}")
        logger.info(f"Target velocity: {target_velocity} m/s")
        logger.info(f"Motor motor_dir: {motor_dir}")
        logger.info(f"Propeller motor_dir: {prop_dir}")
        
        return flight_mode, data_type, num_props, weight, thrust_factor, target_velocity, motor_dir, prop_dir
    
    except Exception as e:
        logger.error(f"Error while getting user inputs: {e}")
        raise

def get_thrust_req(num_props, weight, thrust_factor):
    """
    Calculate the required thrust per propeller.

    :param num_props: Number of propellers.
    :param weight: Expected vehicle weight in kg.
    :param thrust_factor: Factor to ensure enough thrust beyond just lifting the weight.
    :return: Required thrust per propeller in Newtons.
    """
    g = 9.81  # Acceleration due to gravity in m/s^2
    total_thrust = weight * g * thrust_factor
    thrust_per_prop = total_thrust / num_props
    return thrust_per_prop


def get_thrust(diameter, RPM, Ct):
    """
    Calculate the thrust produced by the propeller.

    :param diameter: Diameter of the propeller in meters
    :param RPM: Revolutions per minute of the propeller
    :param Ct: Thrust coefficient
    :return: Thrust in Newtons
    """
    rho = 1.225 # Air density in kg/m^3
    omega = (RPM / 60) * 2 * np.pi # Angular rate
    R = diameter / 2 # Radius
    A = np.pi * R ** 2 # Propeller swept area
    return 1/2 * rho * (omega * R) ** 2 * A * Ct

def get_prop_torque(diameter, RPM, Cp):
    """
    Calculate the torque required by the propeller.

    :param diameter: Diameter of the propeller in meters
    :param RPM: Revolutions per minute of the propeller
    :param Cp: Power coefficient
    :return: Torque in Newton-meters
    """
    rho = 1.225  # Air density in kg/m^3
    omega = (RPM / 60) * 2 * np.pi # Angular rate
    R = diameter / 2
    A = np.pi * R ** 2 # Propeller swept area
    return 1/2 * rho * (omega * R) ** 2 * A * R * Cp

def get_motor_torque(V, kV, i0, R, RPM):
    """
    Calculate the torque produced by the motor.

    :param V: Voltage applied to the motor
    :param kV: Motor constant in rpm/volt
    :param i0: No load current in amps
    :param R: Resistance in ohms
    :param RPM: Revolutions per minute of the motor
    :return: Torque in Newton-meters
    """
    omega = (RPM / 60) * 2 * np.pi # Angular rate
    kV_mod = (kV / 60) * 2 * np.pi # rad/s/Volt
    i = (V - omega / kV_mod) / R
    return (i - i0) / kV_mod

def get_voltage_req(torque, kV, i0, R, RPM):
    """
    Calculate the voltage required to produce the specified torque.

    :param torque_req: Required torque in Newton-meters
    :param kV: Motor constant in rpm/volt
    :param i0: No load current in amps
    :param R: Resistance in ohms
    :param RPM_req: Required revolutions per minute
    :return: Voltage required in volts
    """
    omega = (RPM / 60) * 2 * np.pi # Angular rate
    kV_mod = (kV / 60) * 2 * np.pi # rad/s/Volt
    return (torque * kV_mod + i0) * R + omega / kV_mod

def get_current_req(V, RPM, kV, R):
    """
    :param V: Potential difference in volts
    :param RPM: Revolutions per minute of the motor
    :param kV: Motor constant in rpm/volt
    :param R: Resistance in ohms
    :return: Current in amps
    """
    return (V - RPM / kV) / R

def get_eta_m(i0, V, R, i):
    """
    Calculate motor efficiency.

    :param i0: No load current in amps
    :param V: Potential difference in volts
    :param R: Resistance in ohms
    :param i: Current in amps
    :return: Dimensionless efficiency between 0 and 1
    """
    return ((i - i0) * (V - i * R)) / (V * i)

def get_P_shaft_req(T_req, diameter):
    """
    Calculate required power to hover (from momentum theory)

    :param T_req: Thrust required in Newtons
    :param diameter: Diameter of prop in meters
    :return: Required shaft power in Watts
    """
    rho = 1.225  # Air density in kg/m^3
    R = diameter / 2
    A = np.pi * R ** 2 # Propeller swept area
    return np.sqrt(T_req**3 / (2 * rho * A))

def get_P_shaft(i0, V, R, i):
    """
    Calculate shaft power.

    :param i0: No load current in amps
    :param V: Potential difference in volts
    :param R: Resistance in ohms
    :param i: Current in amps
    :return: Shaft power in Watts
    """
    return (i - i0) * (V - i*R)

def get_P_elec(V, i):
    """
    Calculate electric power required.
    
    :param V: Potential difference in volts
    :param i: Current in amps
    :return: electric power required in Watts
    """
    return V * i

def get_header(T_req, P_shaft_req):
    header = (
                "==========================================================\n"
                "Drone Motor-Propeller Optimization Results\n"
                "==========================================================\n"
                "This file contains the top 10 motor-propeller pairings \n"
                "optimized for static thrust.\n"
                "\n"
                "Each pairing is evaluated based on motor efficiency (eta_m)\n"
                "under the specified thrust requirements.\n"
                f"Thrust requirement: {T_req:.2f} N\n"
                f"Shaft power requirement: {P_shaft_req:.2f} W\n"
                "\n"
                "Fields:\n"
                "----------------------------------------------------------\n"
                "Rank                 : Ranking (1 - 10)\n"
                "Signature            : Unique identifier for the prop-motor pair\n"
                "Prop ID              : Unique identifier for the propeller\n"
                "Motor ID             : Unique identifier for the motor\n"
                "Applied motor voltage: Voltage applied to the motor (V)\n"
                "Applied motor current: Current applied to the motor (A)\n"
                "Desired Torque       : Torque required for equilibrium (N-m)\n"
                "Desired RPM          : Required RPM to achieve desired thrust (RPM)\n"
                "Motor efficiency     : Motor efficiency (dimensionless)\n"
                "Shaft power          : Power supplied by the motor shaft (W)\n"
                "Electric power       : Power supplied to the motor (W)\n"
                "----------------------------------------------------------\n"
                "\n"
                "Results:\n"
                "----------------------------------------------------------\n"
            )
    return header