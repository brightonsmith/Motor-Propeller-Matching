import matplotlib.pyplot as plt
import numpy as np
import os
import logging

'''
This is for level flight calculations, not for static thrust.
'''

# TODO: Motor and propeller file formatting/directory navigation/naming

# Logging Setup #
logger = logging.getLogger("Motor Propeller Matcher")
logger.setLevel(logging.WARNING)
ch = logging.StreamHandler()
ch.setFormatter(logging.Formatter('%(asctime)s %(name)s %(levelname)-8s %(message)s'))
logger.addHandler(ch)
fh = logging.FileHandler("Log.log")
fh.setFormatter(logging.Formatter('%(asctime)s %(name)s %(levelname)-8s %(message)s'))
logger.addHandler(fh)
# End Logging Setup #

def modified_regula_falsi(f, bounds, target, tol, max_iterations=100):
    """
    Solve for an x value at a given y value using the Regula Falsi method.

    INPUTS
    ------
    f : function
        Function to be evaluated.
    bounds: list or tuple
        List or tuple with two elements that bound the Regula Falsi method.
    target: float
        The f(x) value at which you are trying to find the x value.
    tol: float
        The tolerance between the estimated f(x) value and the actual f(x) value.
    max_iterations: int, optional
        Maximum number of iterations to prevent infinite loops (default is 100).

    OUTPUTS
    -------
    x : float
        The x value at which the estimated f(x) value is close enough to the target as specified by the tolerance.
    """
    a, b = bounds
    for _ in range(max_iterations):
        fa, fb = f(a), f(b)
        x = b - (fb - target) * (b - a) / (fb - fa)
        fx = f(x)
        
        if abs(fx - target) <= tol:
            return x
        
        if np.sign(fx - target) == np.sign(fa - target):
            a = x
        else:
            b = x
    
    logger.error("Regula Falsi method did not converge after %d iterations.", max_iterations)
    raise RuntimeError("Regula Falsi method did not converge after {} iterations.".format(max_iterations))

def interpolate(x, x1, x2, y1, y2):
    return (x - x1) * ((y2 - y1) / (x2 - x1)) + y1

def read_propeller_data(filename):
    propeller = {
        'name': "",
        'diameter': 0,
        'pitch': 0,
        'J': [],
        'Ct': [],
        'Cp': [],
        'eta': []
    }

    # Extract propeller name, diameter, and pitch from filename
    base_filename = filename[:filename.rindex('.')]
    name, diameter_pitch = base_filename.rsplit(' ', 1)
    diameter_str, pitch_str = diameter_pitch.split('x')
    propeller['name'] = name
    propeller['diameter'] = float(diameter_str) * 2.54 / 100  # Convert inches to meters
    propeller['pitch'] = float(pitch_str) * 2.54 / 100  # Convert inches to meters

    # Read and parse propeller data
    with open(filename, 'r') as file:
        next(file)  # Skip header line
        for line in file:
            data = line.split()
            if data:
                propeller['J'].append(float(data[0]))
                propeller['Ct'].append(float(data[1]))
                propeller['Cp'].append(float(data[2]))
                propeller['eta'].append(float(data[3]))

    return propeller

# Motor functions
def read_motor_data(filename):
    motor = {
        'name': "",
        'kV': 0,
        'i0': 0,
        'R': 0
    }

    # Extract motor name from filename
    motor['name'] = filename[:filename.rindex('.')]

    # Read motor data from file
    with open(filename, 'r') as file:
        lines = file.readlines()

        # Ensure at least three lines are present
        if len(lines) < 3:
            raise ValueError("Insufficient data in motor file.")

        # Extract and parse motor data
        motor['kV'] = float(lines[0].strip()) / 60  # Convert rpm/volt to rps/volt
        motor['i0'] = float(lines[1].strip())      # Amps
        motor['R'] = float(lines[2].strip())       # Ohms

    return motor

def eta_m(i0, V, R, i):
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


# Propeller Functions
def J(V, n, D):
    """
    find J as a function of (V)elocity, propeller rotatio(n)s per second, and propeller (D)iameter

    :param V: airspeed in m/s
    :param n: propeller rps
    :param D: propeller diameter in m
    :return:
    """
    return V / (n * D)


def pull_from_data(J, J_data, prop_data, handle_array=False):
    """
    Interpolate propeller data given an advance ratio, and the corresponding lists of J values and propeller data
    values. The list of J_data serves as x values and the list of prop_data serves as f(x) values. The way this function
    goes beyond simple interpolation given two points is by only needing the J at which the user needs data at. It then
    automatically finds the points that need to be used in the interpolation, and handles machine precision errors at
    the edge cases. It will not throw an exception if the J value is outside the domain of the propeller data, only
    print a debug statement indicating so and returning a value of zero for that specific point.

    :param J: The advance ratio at which you are interpolating from the propeller lists
    :param J_data: The propeller list of advance ratios that correspond to the Ct, Cp, or eta values
    :param prop_data: The Ct, Cp, or eta values that correspond to the advance ratios
    :param handle_array:
    :return:
    """
    if handle_array:
        result = []
        for i in J:
            result.append(pull_from_data(i, J_data, prop_data))
        return np.asarray(result)

    if abs(J - J_data[0]) < 0.001:
        return prop_data[0]
    elif abs(J - J_data[-1]) < 0.001:
        return prop_data[-1]

    if J < J_data[0]:
        print("J:         {}\n"
              "J_data[0]: {}".format(J, J_data[0]))
        logger.warning("The advance ratio was below the range of advance ratios tested on the propeller.")
        return 0
    elif J > J_data[-1]:
        print("J:          {}\n"
              "J_data[-1]: {}".format(J, J_data[-1]))
        logger.warning("The advance ratio was above the range of advance ratios tested on the propeller.")
        return 0

    i = 0
    while J >= J_data[i]:
        i += 1
    J1 = J_data[i - 1]
    J2 = J_data[i]
    prop1 = prop_data[i - 1]
    prop2 = prop_data[i]

    return interpolate(J, J1, J2, prop1, prop2)


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

rpm_to_rps = 1 / 60

print('\nWelcome to the motor-propeller matcher!\n')

print("Enter the performance targets.")
target_speed = float(input("Enter your target speed in m/s: "))  # m/s
T_req = float(input("Enter how many newtons of thrust per propeller you estimate is needed "
                    "to achieve level flight at target speed: "))  # N

propeller_directory = input("\nEnter the name of the directory containing your prepared propeller files.\n> ")
os.chdir("{}//{}".format(os.getcwd(), propeller_directory)) # TODO: format
propellers = []
for file in os.listdir(os.getcwd()):
    if file[-4:] == ".txt" or file[-4:] == ".dat":
        propellers.append(read_propeller_data(file))
os.chdir("{}//..".format(os.getcwd()))


motor_directory = input("\nEnter the name of the directory containing your prepared motor files.\n> ")
os.chdir("{}//{}".format(os.getcwd(), motor_directory))
motors = []
for file in os.listdir(os.getcwd()):
    if file[-4:] == ".txt" or file[-4:] == ".dat":
        motors.append(read_motor_data(file))
os.chdir("{}//..".format(os.getcwd()))

try:
    logger.info("Results directory created.")
    os.mkdir("Results")
except FileExistsError:
    logger.info("Results directory already existed.")
os.chdir("{}//Results".format(os.getcwd()))

top_10 = []

for motor in motors:
    print("\nTesting each propeller with {}...".format(motor['name']))
    logger.info("\nTesting each propeller with {}...".format(motor['name']))
    for propeller in propellers:
        def T(n, rho=1.225, handle_array=False): # TODO: Optimize
            """
            :param n: rotations per second
            :param rho: mass density of air, default 1.225 kg/m^3
            :return: thrust in N
            """
            return (1 / 2) * rho * (n * (propeller['diameter'] / 2))**2 * np.pi * (propeller['diameter'] / 2)**2\
                   * pull_from_data(J(target_speed, n, propeller['diameter']), propeller['J'], propeller['Ct'], handle_array)

        # Find required RPM by matching prop thrust to thrust required
        min_n = target_speed / (propeller['J'][-1] * propeller['diameter'])
        max_n = target_speed / (propeller['J'][0] * propeller['diameter'])
        required_RPM = modified_regula_falsi(T, (min_n, max_n), T_req, 0.001) * 60
        # print("Required RPM: {}".format(required_RPM))
        n = np.linspace(min_n, max_n)

        fig = plt.figure("Motor Propeller Matcher", figsize=(13.03408/2, 8.5))

        # Plot the thrust of the propeller at target speed as a function of n
        p1 = fig.add_subplot(4, 1, 2)
        p1.set(title="Thrust for {} at {:.2f} m/s".format(propeller['name'], target_speed),
               xlabel='RPM',
               ylabel='Thrust (N)',
               xlim=(0, required_RPM * 3),
               ylim=(0, T(required_RPM / 60) * 6))

        p1.plot(n * 60, T(n, handle_array=True),
                label='prop thrust at target speed')
        p1.plot(np.linspace(0, max_n * 60 * 2), T_req * np.ones(50), linestyle='dashed',  # / 4 is there
                label='thrust required for level flight')
        p1.plot((required_RPM, required_RPM), p1.get_ylim(), color='red', label='Required RPM')

        p1.legend(loc='upper right')

        # Plot the efficiency curve of the propeller
        p4 = fig.add_subplot(4, 1, 1)
        p4.set(title='Efficiency for {} at {:.2f} m/s'.format(propeller['name'], target_speed),  # figure out how to abstract (or not)
               xlabel='RPM',
               ylabel='$ \eta_p $',
               ylim=(0, 1),
               xlim=(0, required_RPM * 3))

        p4.scatter(omega(target_speed, propeller['J'], propeller['diameter'], True) * 60, propeller['eta'],
                   label='Efficiency for {}'.format(propeller['name']))
        p4.plot((required_RPM, required_RPM), p4.get_ylim(), color='red', label='Required RPM')
        propeller_efficiency = pull_from_data(J(target_speed, required_RPM / 60, propeller['diameter']),
                                              propeller['J'], propeller['eta'])
        p4.text(0, 0, "Efficiency: {:.2f}".format(propeller_efficiency))

        p4.legend(loc='upper right')


        # Calculate the required torque for level flight
        def Q(n, rho=1.225, handle_array=False):
            """
            :param n: rotations per second
            :param rho: mass density of air, default 1.225 kg/m^3
            :return: torque in N m
            """
            return (1 / 2) * rho * (n * (propeller['diameter'] / 2))**2 * np.pi * (propeller['diameter'] / 2)**3\
                   * pull_from_data(J(target_speed, n, propeller['diameter']), propeller['J'], propeller['Cp'],
                                    handle_array)

        required_torque = Q(required_RPM / 60)

        # Calculate the required voltage for level flight
        def voltage(n, Q, kV, i0, R):
            return (kV * Q + i0) * R + n / kV

        required_voltage = voltage(required_RPM / 60, required_torque, motor['kV'], motor['i0'], motor['R'])

        # Plot the torque of the propeller at target speed as a function of voltage and n
        p3 = fig.add_subplot(4, 1, 3)
        p3.set(title='Torque for ' + motor['name'] + ' and ' + propeller['name'],
               xlabel='RPM',
               ylabel='Torque (Nm)',
               xlim=(0, required_RPM * 3),
               ylim=(0, Q_m(current(required_voltage, 0, motor['kV'], motor['R']), motor['i0'], motor['kV']) * 1.1))

        n_mot = np.linspace(0, motor['kV'] * required_voltage, 500, False)

        p3.plot(n_mot * 60, Q_m(current(required_voltage, n_mot, motor['kV'], motor['R']), motor['i0'], motor['kV']),
                label='motor at {:.1f} volts'.format(required_voltage))
        p3.plot(n * 60, Q(n, handle_array=True),
                label='prop torque curve at target speed')
        p3.plot((required_RPM, required_RPM), p3.get_ylim(), color='red', label='Required RPM')

        p3.legend(loc='upper right')


        # Plot the efficiency curves of the motor as a function of n and voltage
        p2 = fig.add_subplot(4, 1, 4)
        p2.set(title='Efficiency Curve for ' + motor['name'],
               xlabel='RPM',
               ylabel='$ \eta_m $',
               ylim=(0, 1),
               xlim=(0, required_RPM * 3))

        p2.plot(n_mot * 60,
                eta_m(motor['i0'], required_voltage, motor['R'], current(required_voltage, n_mot, motor['kV'], motor['R'])),
                label='{:.1f} volts'.format(required_voltage))
        p2.plot((required_RPM, required_RPM), p2.get_ylim(), color='red', label='Required RPM')
        motor_efficiency = eta_m(motor['i0'], required_voltage, motor['R'], current(required_voltage,
                                                                           required_RPM / 60, motor['kV'], motor['R']))
        p2.text(0, 0, "Efficiency: {:.2f}".format(motor_efficiency))

        p2.legend(loc='upper right')

        print("    {} total efficiency: {:.2f}".format(propeller['name'],
              propeller_efficiency * motor_efficiency))
        logger.info("    {} total efficiency: {:.2f}".format(propeller['name'],
                    propeller_efficiency * motor_efficiency))
        print("    {} Advance Ratio at required RPM: {:.2f}".format(propeller['name'],
              J(target_speed, required_RPM * rpm_to_rps, propeller["diameter"])))
        logger.info("    {} Advance Ratio at required RPM: {:.2f}".format(propeller['name'],
                    J(target_speed, required_RPM * rpm_to_rps, propeller["diameter"])))

        fig.subplots_adjust(top=0.969, bottom=0.061, left=0.150, right=0.850, hspace=0.485)
        plt.savefig("{} with {}.jpg".format(motor['name'], propeller['name']))
        fig.delaxes(p1)
        fig.delaxes(p2)
        fig.delaxes(p3)
        fig.delaxes(p4)

        simulation_results = {
            'name': "{} with {}".format(motor['name'], propeller['name']),
            'efficiency': propeller_efficiency * motor_efficiency,
            'advance ratio': J(target_speed, required_RPM * rpm_to_rps, propeller["diameter"])
        }

        if not top_10:
            top_10.append(simulation_results)
        else:
            inserted = False

            for i in range(10):
                if i < len(top_10) and simulation_results['efficiency'] > top_10[i]['efficiency']:
                    top_10.insert(i, simulation_results)
                    inserted = True
                    break

            if not inserted and len(top_10) < 10:
                top_10.append(simulation_results)


        while len(top_10) > 10:
            top_10.pop(-1)

with open("Results.txt", 'w') as results_file:
    rank = 1
    for result in top_10:
        results_file.write("{}) {} Efficiency: {:.2f} Advance ratio at required RPM: {:.2f}\n"
                          .format(rank, result['name'], round(result['efficiency'], 2), round(result['advance ratio'], 2)))
        rank += 1

os.chdir("{}//..".format(os.getcwd()))
print("Results saved to './Results.txt'")
print("Done")