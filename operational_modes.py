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
                            self.logger.error(f"File {filename} does not contain three lines.")
                            print(f"File {filename} does not contain three lines. See README for formatting details.")
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

            for _, row in prop_subset.iterrows():
                thrust = self.get_thrust(row['diameter'], row['RPM'], row['Ct'])
                thrust_vals.append(thrust)
                RPM_vals.append(row['RPM'])

            sorted_indices = np.argsort(thrust_vals)
            thrust_vals = np.array(thrust_vals)[sorted_indices]
            RPM_vals = np.array(RPM_vals)[sorted_indices]

            # Find the two thrust values between which the desired thrust is found
            if thrust_vals[0] <= self.thrust_req <= thrust_vals[-1]:
                spline = CubicSpline(thrust_vals, RPM_vals)
                RPM_req = spline(self.thrust_req)
                self.RPM_req_dic[prop_id] = RPM_req
                self.logger.info(f"Required RPM for {prop_id}: {RPM_req}")
            else:
                self.logger.warning(f"Desired thrust is out of range for propeller {prop_id}.")

    

class StableFlight(OperationalMode):
    def __init__(self, thrust_req, prop_dir, motor_dir, target_velocity, logger):
        super().__init__(thrust_req, prop_dir, motor_dir, logger)
        self.target_velocity = target_velocity
        self.prop_data = self.read_prop_dir(prop_dir)

        self.logger.info("Initialized StableFlight object")
