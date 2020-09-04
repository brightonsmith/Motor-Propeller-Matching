import matplotlib.pyplot as plt
import numpy as np

# n and kv should be used in rps, not rpm

KDE2550 =\
{
    'name': 'KDE2550',
    'kV': 153000,  # rps / volt
    'i0': 1.2,  # A
    'R': 0.044  # Ohms
}

motor = KDE2550


def eta_m(i0, V, R, i):
    return ((i - i0) * (V - i * R)) / (V * i)

# def eta_m(i0, V, R, n, kV):
#     return (1 - (i0 * R / (V - (n / kV)))) * n / (V * kV)


def P_shaft(i0, V, R, n, kV):
    return ((V - n / kV) / R - i0) * n / kV


def i(V, n, kV, R):
    return (V - n / kV) / R


def Q_m(i, i0, kV):
    return (i - i0) / kV


# Plotting motor efficiency curves
n = np.linspace(0, 3500000, 100)
voltages = [14.4, 18, 21.6]

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
#     p1.plot(n / 60, P_shaft(motor['i0'], voltage, motor['R'], n, motor['kV']),
#             label='{} volts'.format(voltage))
#
# p1.legend()
# plt.tight_layout()
# plt.show()


fig3 = plt.figure()
p3 = fig3.add_subplot(1, 1, 1)
p3.set(title='Torque for ' + motor['name'],
       xlabel='RPM',
       ylabel='$ Q_m $ (Nm)',
       ylim=(0, 0.004))

for voltage in voltages:
    p3.plot(n / 60, Q_m(i(voltage, n, motor['kV'], motor['R']), motor['i0'], motor['kV']),
            label='{} volts'.format(voltage))

p3.legend()
plt.tight_layout()
plt.show()


fig2 = plt.figure()
p2 = fig2.add_subplot(1, 1, 1)
p2.set(title='Efficiency Curve for ' + motor['name'],
       xlabel='RPM',
       ylabel='$ \eta_m $',
       ylim=(0, 1))

n0 = np.linspace(0, 2202999, 1000)  # zero efficiency n at target speed - 1 (to prevent wacky lines and zero division)
n1 = np.linspace(0, 2753999, 1000)
n2 = np.linspace(0, 3304799, 1000)

# p2.plot(n0, eta_m(motor['i0'], voltages[0], motor['R'], n0, motor['kV']),  # These work with the commented out functn
#             label='{} volts'.format(voltages[0]))
# p2.plot(n1, eta_m(motor['i0'], voltages[1], motor['R'], n1, motor['kV']),
#             label='{} volts'.format(voltages[1]))
# p2.plot(n2, eta_m(motor['i0'], voltages[2], motor['R'], n2, motor['kV']),
#             label='{} volts'.format(voltages[2]))

p2.plot(n0 / 60, eta_m(motor['i0'], voltages[0], motor['R'], i(voltages[0], n0, motor['kV'], motor['R'])),
            label='{} volts'.format(voltages[0]))
p2.plot(n1 / 60, eta_m(motor['i0'], voltages[1], motor['R'], i(voltages[1], n1, motor['kV'], motor['R'])),
            label='{} volts'.format(voltages[1]))
p2.plot(n2 / 60, eta_m(motor['i0'], voltages[2], motor['R'], i(voltages[2], n2, motor['kV'], motor['R'])),
            label='{} volts'.format(voltages[2]))


p2.legend()
plt.tight_layout()
plt.show()


T = 2.3  # N
