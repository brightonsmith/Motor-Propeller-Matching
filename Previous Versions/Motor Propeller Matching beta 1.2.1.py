import matplotlib.pyplot as plt
import numpy as np
import os

version = 'beta 1.2.1'

'''
This is for level flight calculations, not for static thrust.
'''

'''
Stuff I wanna put in:
- Choose from a list of motors or use manual entry
- Make it to where you just enter the filename of a motor and the filename of a propeller.
- Pull directly from UIUC-style eta, Ct, Cp data  [DONE]
  * and figure out how to easily pull the proper Ct and Cp given the advance ratio  [DONE]
  * After essential functions finished, find out how to pull photos  [DONE]
- Automate the process by manipulating the equations  [DONE]
- Make a plot of combined efficiency
'''

'''
-Notes-
The naming convention isn't consistent, nor is the order for the UIUC data, so watch out
Turns out you're just gonna have to make a custom data file each time. This might not be
such a big deal now that it might be best anyways
'''


def modified_regula_falsi(f, bounds, target, tol):
    iterations = 0
    a = bounds[0]
    b = bounds[1]
    x = b + ((target - f(b)) * (b - a)) / (f(b) - f(a))
    while abs(f(x) - target) > tol:
        iterations += 1
        if iterations > 100:
            raise Exception("Regula-Falsi method did not converge after 100 iterations.")
        if np.sign(f(x) - target) == np.sign(f(a) - target):
            a = x
        else:
            b = x
        x = b + ((target - f(b)) * (b - a)) / (f(b) - f(a))
    return x


def interpolate(x, x1, x2, y1, y2):
    return (x - x1) * ((y2 - y1) / (x2 - x1)) + y1


def read_propeller_data(filename):
    propeller = {
        'name': input("Enter propeller name: "),
        'diameter': float(input("Enter propeller diameter in inches: ")) * 2.54 / 100,  # Convert inches to meters
        'pitch': float(input("Enter propeller pitch in inches: ")) * 2.54 / 100,  # Convert inches to meters
        'J': [],
        'Ct': [],
        'Cp': [],
        'eta': []
    }
    with open(filename, 'r') as file:
        file.readline()
        for line in file:
            splitted = line.split()
            propeller['J'].append(float(splitted[0]))
            propeller['Ct'].append(float(splitted[1]))
            propeller['Cp'].append(float(splitted[2]))
            propeller['eta'].append(float(splitted[3]))

    return propeller


# Motor functions

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


def i(V, n, kV, R):
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
    return V / (n * D)


def pull_from_data(J, J_data, prop_data, handle_array=False):
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
        print("J:          {}\n"
              "J_data[0]: {}".format(J, J_data[0]))
        print("The advance ratio was below the range of advance ratios tested on the propeller.")
        return 0
    elif J > J_data[-1]:
        print("J:          {}\n"
              "J_data[-1]: {}".format(J, J_data[-1]))
        print("The advance ratio was above the range of advance ratios tested on the propeller.")
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


# BEGIN PROPER INPUT
print('\nWelcome to the motor-propeller matcher version {}!\n'
      'I hope that you find this tool useful for matching a motor and propeller.\n'.format(version))

print("To properly enter propeller data, you must choose a file with the following example format:\n"
      "J       CT       CP       eta\n"
      "0.165   0.0993   0.0539   0.304\n"
      "0.214   0.0947   0.0543   0.374\n"
      "0.255   0.0916   0.0548   0.427\n"
      "etc.\n")

propeller_filename = input("Enter the full filename, including the extension: ")

propeller = read_propeller_data(propeller_filename)

print('Now, enter the motor data.')

motor = {
    'name': input("Enter the motor name: "),
    'kV': float(input("Enter the kV rating in RPM per volt: ")) / 60,  # convert to rps / volt
    'i0': float(input("Enter the no load current in amps: ")),  # A
    'R': float(input("Enter the wind resistance in ohms: ")),  # Ohms
}


target_speed = float(input("Enter your target speed in m/s: ")) # m/s
T_req = float(input("Enter how many newtons of thrust per propeller you need "
                    "to achieve level flight at target speed: "))  # N
# END PROPER INPUT


def T(n, rho=1.225, handle_array=False):  # This isn't good but I don't know how else to do it | Well maybe it's not so bad
    """
    :param n: rotations per second
    :param rho: mass density of air, default 1.225 kg/m^3
    :return: thrust in N
    """
    return (1 / 2) * rho * (n * (propeller['diameter'] / 2))**2 * np.pi * (propeller['diameter'] / 2)**2\
           * pull_from_data(J(target_speed, n, propeller['diameter']), propeller['J'], propeller['Ct'], handle_array)


# find required RPM by matching prop thrust to thrust required
min_n = target_speed / (propeller['J'][-1] * propeller['diameter'])
max_n = target_speed / (propeller['J'][0] * propeller['diameter'])
required_RPM = modified_regula_falsi(T, (min_n, max_n), T_req, 0.001) * 60
# print("Required RPM: {}".format(required_RPM))
n = np.linspace(min_n, max_n)


fig = plt.figure("Motor Propeller Matcher version {}".format(version))  # And so it begins


# Plot the thrust of the propeller at target speed as a function of n ooh also plot thrust line
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
       ylim=(0, Q_m(i(required_voltage, 0, motor['kV'], motor['R']), motor['i0'], motor['kV']) * 1.1))

n_mot = np.linspace(0, motor['kV'] * required_voltage, 500, False)

p3.plot(n_mot * 60, Q_m(i(required_voltage, n_mot, motor['kV'], motor['R']), motor['i0'], motor['kV']),
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
        eta_m(motor['i0'], required_voltage, motor['R'], i(required_voltage, n_mot, motor['kV'], motor['R'])),
        label='{:.1f} volts'.format(required_voltage))
p2.plot((required_RPM, required_RPM), p2.get_ylim(), color='red', label='Required RPM')
motor_efficiency = eta_m(motor['i0'], required_voltage, motor['R'], i(required_voltage,
                                                                   required_RPM / 60, motor['kV'], motor['R']))
p2.text(0, 0, "Efficiency: {:.2f}".format(motor_efficiency))

p2.legend(loc='upper right')

print("Total efficiency: {:.2f}".format(propeller_efficiency * motor_efficiency))

fig.subplots_adjust(top=0.969, bottom=0.061, left=0.150, right=0.850, hspace=0.485)
plt.show()
