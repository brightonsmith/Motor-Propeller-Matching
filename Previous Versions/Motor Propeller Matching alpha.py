import matplotlib.pyplot as plt
import numpy as np

'''
This is for level flight calculations
'''

'''
Stuff I wanna put in:
- Choose from a list of motors or use manual entry
- Pull directly from UIUC-style eta, Ct, Cp data
  * and figure out how to easily pull the proper Ct and Cp given the advance ratio
- Automate the process by manipulating the equations
OR by instructing the user to mouse over the peak and record the number
- Make a plot of combined efficiency
'''

# n and kv should be used in rps, not rpm

KDE2550 =\
{
    'name': 'KDE2550',
    'kV': 42.5, # rps / volt
    'i0': 1.2,  # A
    'R': 0.044  # Ohms
}

Default =\
{
    'name': 'User Defined Motor',
    'kV': None,  # rps / volt
    'i0': None,  # A
    'R': None    # Ohms
}

motor = KDE2550  # Change this to a user-inputted choice (prolly not an if tree pls no) ooh make it read from a file!

# but for now...
propeller =\
{
    'name': 'DA4002 5" x 3.75"',
    'D': 5,  # in
    'pitch': 3.75,  # in
}

def eta_m(i0, V, R, i):
    """
    :param i0: no load current in amps
    :param V: potential difference in volts
    :param R: resistance in ohms
    :param i: current in amps
    :return: dimensionless efficiency between 0 and 1
    """
    return ((i - i0) * (V - i * R)) / (V * i)

# def eta_m(i0, V, R, n, kV):
#     return (1 - (i0 * R / (V - (n / kV)))) * n / (V * kV)


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


def T(n, v, R, Ct, rho=1.225):
    """
    :param n: rotations per second
    :param v: speed in meters per second
    :param R: propeller radius in meters
    :param Ct: coefficient of torque
    :param rho: mass density of air, default 1.225 kg/m^3
    :return: thrust in N
    """
    return (1 / 2) * rho * (n * R)**2 * np.pi * R**2 * Ct


def Q(n, v, R, Cp, rho=1.225):
    """
    :param n: rotations per second
    :param v: speed in meters per second
    :param R: propeller radius in meters
    :param Cp: coefficient of power
    :param rho: mass density of air, default 1.225 kg/m^3
    :return: torque in N m
    """
    return (1 / 2) * rho * (n * R)**2 * np.pi * R**3 * Cp


def omega(J, V, D):
    """
    :param J: advance ratio
    :param V: velocity in m/s
    :param D: propeller diameter in m
    :return: rotations per second
    """
    return V / (J * D)


# Plotting motor efficiency curves
n = np.linspace(0, 973, 100)
# voltages = [14.4, 18, 21.6]
voltages = [12.8, 15.4, 18]

# Power curve just for fun
# fig1 = plt.figure()
# p1 = fig1.add_subplot(1, 1, 1)
# p1.set(title='Power Curve for ' + motor['name'],
#        xlabel='RPM',
#        ylabel='$ P_{shaft} $ (W)',
#        ylim=(0, 3000))
#
#
# for voltage in voltages:
#     p1.plot(n * 60, P_shaft(motor['i0'], voltage, motor['R'], n, motor['kV']),
#             label='{} volts'.format(voltage))
#
# p1.legend()
# plt.tight_layout()
# plt.show()

J = np.asarray([0.194068, 0.236641, 0.288717, 0.330812, 0.378295, 0.424751, 0.472195, 0.522899, 0.574100, 0.616845,
                0.663208, 0.711810, 0.766780, 0.807060, 0.852601, 0.895451])
eta = np.asarray([0.259146, 0.311714, 0.375928, 0.426983, 0.478756, 0.521892, 0.561076, 0.597388, 0.622210, 0.632000,
                  0.629766, 0.614650, 0.557214, 0.462250, 0.221012, -0.393539])

target_speed = 50 / 3600 * 5280 * 12 * 2.54 / 100  # Started with 50 mph and converted to m/s
T_req = 2.3 * np.ones(n.shape) # N
prop_diameter = propeller['D'] * 2.54 / 100
# temporary while I figure out something better
# NO!!! YOU CAN'T DO THAT
Ct = 0.0439454  # both at J = 0.675
Cp = 0.0472174

fig = plt.figure()


# Plot the efficiency curve of the propeller
p4 = fig.add_subplot(4, 1, 1)
p4.set(title='Efficiency for ' + propeller['name'] + ' at 50 mph',  # figure out how to abstract (or not)
       xlabel='RPM',
       ylabel='$ \eta_p $',
       ylim=(0, 1),
       xlim=(0, n.max() * 60))

p4.scatter(omega(J, target_speed, prop_diameter) * 60, eta)


# Plot the thrust of the propeller at target speed as a function of n ooh also plot thrust line
p1 = fig.add_subplot(4, 1, 2)
p1.set(title='Thrust for ' + propeller['name'] + ' at 50 mph',  # figure out how to abstract (or not)
       xlabel='RPM',
       ylabel='$ T_p $ (N)',
       xlim=(0, n.max() * 60))

p1.plot(n * 60, T(n, target_speed, prop_diameter / 2, Ct),
        label='prop thrust at target speed')
p1.plot(n * 60, T_req / 4, linestyle='dashed',
        label='thrust required for level flight')

p1.legend()


# Plot the torque of the propeller at target speed as a function of voltage and n
p3 = fig.add_subplot(4, 1, 3)
p3.set(title='Torque for ' + motor['name'] + ' and ' + propeller['name'],
       xlabel='RPM',
       ylabel='$ Q_m $ (Nm)',
       ylim=(0, Q_m(i(voltages[-1], n.min(), motor['kV'], motor['R']), motor['i0'], motor['kV']) + 1),
       xlim=(0, n.max() * 60))

for voltage in voltages:
    p3.plot(n * 60, Q_m(i(voltage, n, motor['kV'], motor['R']), motor['i0'], motor['kV']),
            label='motor at {} volts'.format(voltage))
p3.plot(n * 60, Q(n, target_speed, prop_diameter / 2, Cp),
        label='prop torque curve at target speed')

p3.legend()


# Plot the efficiency curves of the motor as a function of n and voltage
p2 = fig.add_subplot(4, 1, 4)
p2.set(title='Efficiency Curve for ' + motor['name'],
       xlabel='RPM',
       ylabel='$ \eta_m $',
       ylim=(0, 1),
       xlim=(0, n.max() * 60))

# n0 = np.linspace(0, 611.944, 1000)  # zero efficiency n at target speed - 1 (to prevent wacky lines and zero division)
# n1 = np.linspace(0, 764.999, 1000)
# n2 = np.linspace(0, 917.999, 1000)

# p2.plot(n0, eta_m(motor['i0'], voltages[0], motor['R'], n0, motor['kV']),  # These work with the commented out functn
#             label='{} volts'.format(voltages[0]))
# p2.plot(n1, eta_m(motor['i0'], voltages[1], motor['R'], n1, motor['kV']),
#             label='{} volts'.format(voltages[1]))
# p2.plot(n2, eta_m(motor['i0'], voltages[2], motor['R'], n2, motor['kV']),
#             label='{} volts'.format(voltages[2]))

# These are a less sophisticated version of what follows
# p2.plot(n0 * 60, eta_m(motor['i0'], voltages[0], motor['R'], i(voltages[0], n0, motor['kV'], motor['R'])),
#             label='{} volts'.format(voltages[0]))
# p2.plot(n1 * 60, eta_m(motor['i0'], voltages[1], motor['R'], i(voltages[1], n1, motor['kV'], motor['R'])),
#             label='{} volts'.format(voltages[1]))
# p2.plot(n2 * 60, eta_m(motor['i0'], voltages[2], motor['R'], i(voltages[2], n2, motor['kV'], motor['R'])),
#             label='{} volts'.format(voltages[2]))

for voltage in voltages:
    current_n = np.linspace(0, motor['kV'] * voltage, 500, False)
    p2.plot(current_n * 60,
            eta_m(motor['i0'], voltage, motor['R'], i(voltage, current_n, motor['kV'], motor['R'])),
            label='{} volts'.format(voltage))


p2.legend()
fig.subplots_adjust(top=0.969, bottom=0.061, left=0.150, right=0.850, hspace=0.485)
plt.show()
