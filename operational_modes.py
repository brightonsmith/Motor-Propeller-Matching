import os
import pandas as pd
import numpy as np
from scipy.interpolate import CubicSpline
import utilities

class OperationalMode:
    def __init__(self, thrust_req, prop_dir, motor_dir, logger):
        self.logger = logger
        self.thrust_req = thrust_req
        self.prop_dir = prop_dir
        self.motor_data = self.read_motor_dir(motor_dir)
        self.RPM_req_dic = {}
        self.voltage_req_dic = {}
        self.eta_m_dic = {}

    def read_motor_dir(self, motor_dir):
        motor_data = {
            'ID': [],
            'kV': [],
            'i0': [],
            'R': [],
            'V_tol': [],
        }

        for filename in os.listdir(motor_dir):
            if filename.endswith('.txt'):
                filepath = os.path.join(motor_dir, filename)
                id = os.path.splitext(filename)[0]

                try:
                    with open(filepath, 'r') as file:
                        lines = file.readlines()
                        if len(lines) == 4:
                            motor_data['ID'].append(id)
                            motor_data['kV'].append(float(lines[0].strip()) / 60)   # rps/volt
                            motor_data['i0'].append(float(lines[1].strip()))        # Amps
                            motor_data['R'].append(float(lines[2].strip()))         # Ohms
                            motor_data['V_tol'].append(float(lines[3].strip()))     # Volts
                        else:
                            self.logger.error(f"File {filename} does not contain three lines.")
                            print(f"File {filename} does not contain four lines. See README for formatting details.")
                except Exception as e:
                    self.logger.error(f"Error reading {filename}: {e}")
                    print(f"Error reading {filename}: {e}. See README for formatting details.")

        motor_df = pd.DataFrame(motor_data)
        self.logger.info("Processed motor directory")

        return motor_df
    
    def get_thrust(self, diameter, RPM, Ct):
        # TODO: Docstring
        rho = 1.225 # Air density in kg/m^3
        A = np.pi * (diameter / 2) ** 2 # Propeller swept area
        return Ct * rho * A * (RPM / 60) ** 2 * (diameter / 2) ** 2 # RPM / 60 is omega
    
    def get_prop_torque(self, diameter, RPM, Cp):
        # TODO: Docstring
        rho = 1.225  # Air density in kg/m^3
        return Cp * rho * ((diameter / 2) ** 5) * (RPM / 60) ** 2 # RPM / 60 is omega

    def get_motor_torque(self, V, kV, i0, R, RPM):
        # TODO: Docstring
        i = (V - (RPM / 60) / kV) / R
        return (i - i0) / kV
    
    def get_voltage_req(self, torque_req, kV, i0, R, RPM_req):
        # TODO: Docstring
        return (torque_req * kV + i0) * R + (RPM_req / 60) / kV
    
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
    
class StaticThrust(OperationalMode):
    def __init__(self, thrust_req, prop_dir, motor_dir, logger):
        super().__init__(thrust_req, prop_dir, motor_dir, logger)
        self.prop_data = self.read_prop_dir(prop_dir)

        self.logger.info("Initialized StaticThrust object")

    def read_prop_dir(self, prop_dir):
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
                            self.logger.error(f"File {filename} does not contain enough lines.")
                            print(f"File {filename} does not contain enough lines. See README for formatting details.")
                except Exception as e:
                    self.logger.error(f"Error reading {filename}: {e}")
                    print(f"Error reading {filename}: {e}. See README for formatting details.")

                prop_df = pd.DataFrame(prop_data)
                self.logger.info("Processed propeller directory")

                return prop_df
            
    def get_RPM_req(self):
        for prop_id in self.prop_data['ID'].unique():
            prop_subset = self.prop_data[self.prop_data['ID'] == prop_id]
            thrust_vals = []
            RPM_vals = []

            for _, prop in prop_subset.iterrows():
                thrust = self.get_thrust(prop['diameter'], prop['RPM'], prop['Ct'])
                thrust_vals.append(thrust)
                RPM_vals.append(prop['RPM'])

            sorted_indices = np.argsort(thrust_vals)
            thrust_vals = np.array(thrust_vals)[sorted_indices]
            RPM_vals = np.array(RPM_vals)[sorted_indices]

            # Find the two thrust values between which the desired thrust is found
            if thrust_vals[0] <= self.thrust_req <= thrust_vals[-1]:
                spline = CubicSpline(thrust_vals, RPM_vals)
                RPM_req = spline(self.thrust_req)
                self.RPM_req_dic[prop_id] = RPM_req
                self.logger.info(f"Required RPM for prop {prop_id}: {RPM_req}")
            else:
                self.logger.warning(f"Desired thrust is out of range for prop {prop_id}.")

    def get_voltage_req(self):
        for prop_id, RPM_req in self.RPM_req_dic.items():
            prop_subset = self.prop_data[self.prop_data['ID'] == prop_id]

            prop_torque_vals = []
            RPM_vals = []

            for _, prop in prop_subset.iterrows():
                prop_torque = self.get_prop_torque(prop['diameter'], prop['RPM'], prop['Cp'])
                prop_torque_vals.append(prop_torque)
                RPM_vals.append(prop['RPM'])

            spline = CubicSpline(RPM_vals, prop_torque_vals)
            torque_req = spline(RPM_req)

            self.voltage_req_dic[prop_id] = {}

            for _, motor in self.motor_data.iterrows():
                motor_id = motor['ID']
                kV = motor['kV']
                i0 = motor['i0']
                R = motor['R']

                voltage_req = self.get_voltage_req(torque_req, kV, i0, R, RPM_req)

                if voltage_req <= motor['V_tol']:
                    self.voltage_req_dic[prop_id][motor_id] = voltage_req
                else:
                    self.logger.warning(f"Required voltage is beyond the voltage tolerance for prop {prop_id} with motor {motor_id}.")
                
                self.logger.info(f"Required voltage for {prop_id} with motor {motor_id}: {voltage_req}")

            # TODO: Torque required curves for plotting

    def get_eta_m_dic(self):
        for prop_id, motor_voltage_dict in self.voltage_req_dic.items():
            self.eta_m_dic[prop_id] = {}

            for motor_id, voltage_req in motor_voltage_dict.items():
                motor_row = self.motor_data[self.motor_data['ID'] == motor_id].iloc[0]
                kV = motor_row['kV']
                i0 = motor_row['i0']
                R = motor_row['R']

                RPM_req = self.RPM_req_dic[prop_id]
                current = (voltage_req - RPM_req / kV) / R

                eta_m = self.get_eta_m(i0, voltage_req, R, current)

                if 0 <= eta_m <= 1:
                    self.eta_m_dic[prop_id][motor_id] = eta_m
                else:
                    self.logger.warning(f"Eta_m for prop {prop_id} with motor {motor_id} is out of range.")

                self.logger.info(f"Motor efficiency for {prop_id} with motor {motor_id}: {eta_m}")
    

class StableFlight(OperationalMode):
    def __init__(self, thrust_req, prop_dir, motor_dir, target_velocity, logger):
        super().__init__(thrust_req, prop_dir, motor_dir, logger)
        self.target_velocity = target_velocity
        self.prop_data = self.read_prop_dir(prop_dir)

        self.logger.info("Initialized StableFlight object")
