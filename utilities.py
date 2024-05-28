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