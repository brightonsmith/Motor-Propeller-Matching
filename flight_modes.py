import os
import numpy as np
import hashlib
import pandas as pd
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import utilities

class FlightMode:
    def __init__(self, T_req, prop_filepath, motor_filepath, logger):
        self.logger = logger
        self.T_req = T_req
        self.P_shaft_req = None
        self.signature = self.generate_signature(prop_filepath, motor_filepath)
        self.motor_id = os.path.splitext(os.path.basename(motor_filepath))[0]
        self.prop_id, self.diameter, self.pitch = self.get_prop_specs(prop_filepath)
        self.RPM_req = None
        self.V_req = None
        self.i_req = None
        self.Q_req = None
        self.eta_m = None
        self.P_shaft = None
        self.P_elec = None
        self.motor_data = self.read_motor_file(motor_filepath)

        self.logger.info(f"Initiated FlightMode prop-motor obj with signature: {self.signature}")

    def generate_signature(self, prop_filepath, motor_filepath):
        data_string = f"{self.T_req}_{prop_filepath}_{motor_filepath}"
        return hashlib.md5(data_string.encode()).hexdigest()[:8]  # Generate a short unique signature

    def get_prop_specs(self, prop_filepath):
        parts = os.path.basename(prop_filepath).split('_')
        make = parts[0] 
        diameter_pitch = parts[1]
        model = parts[3].split('.')[0] 
        prop_id = f"{make}-{model}" 

        try:
            diameter_str, pitch_str = diameter_pitch.split('x')

            # Convert to float
            diameter = float(diameter_str)
            pitch = float(pitch_str)

            # If diameter is 3 digits, divide by 10
            if len(diameter_str) == 3:
                diameter /= 10

            # Convert to meters
            diameter /= 39.37  # 1 inch = 39.37 meters
            pitch /= 39.37  # 1 inch = 39.37 meters
        except ValueError as e:
            # self.logger.error(f"Error processing file {prop_filepath} with signature {self.signature}: {e}")
            # print(f"Error processing file {prop_filepath} with signature {self.signature}: {e}")
            return prop_id, None, None

        return prop_id, diameter, pitch

    def read_motor_file(self, motor_filepath):
        motor_data = {
            'kV': 0,
            'i0': 0,
            'R': 0,
            'i_tol': 0
        }

        try:
            with open(motor_filepath, 'r') as file:
                lines = file.readlines()
                if len(lines) == 4:
                    motor_data['kV'] = float(lines[0].strip())        # rpm/volt
                    motor_data['i0'] = float(lines[1].strip())        # Amps
                    motor_data['R'] = float(lines[2].strip())         # Ohms
                    motor_data['i_tol'] = float(lines[3].strip())     # Amps
                else:
                    self.logger.error(f"File {motor_filepath} does not contain four lines.")
                    print(f"File {motor_filepath} does not contain four lines. See README for formatting details.")
        except Exception as e:
            self.logger.error(f"Error reading {motor_filepath}: {e}")
            print(f"Error reading {motor_filepath}: {e}. See README for formatting details.")

        self.logger.info(f"Processed motor file {motor_filepath}")

        return motor_data
    
    def get_signature(self):
        return self.signature
    
    def get_P_shaft_req(self):
        return self.P_shaft_req
    
class StaticThrust(FlightMode):
    def __init__(self, T_req, prop_filepath, motor_filepath, logger):
        super().__init__(T_req, prop_filepath, motor_filepath, logger)

    def read_prop_file(self):
        raise NotImplementedError("Subclasses must implement this method.")
    
    def get_RPM_req(self):
        raise NotImplementedError("Subclasses must implement this method.")

    def get_voltage_req(self):
        raise NotImplementedError("Subclasses must implement this method.")

    def get_eta_m(self):
        """
        Calculate the motor efficiency for the motor-propeller pairing.
        """
        
        if self.i_req is None:
            self.logger.warning(f"Current requirement not met for prop-motor {self.signature}. Cannot calculate motor efficiency.")
            return

        i0 = self.motor_data['i0']
        R = self.motor_data['R']

        eta_m = utilities.get_eta_m(i0, self.V_req, R, self.i_req)
        self.P_shaft_req = utilities.get_P_shaft_req(self.T_req, self.diameter)
        self.P_shaft = utilities.get_P_shaft(i0, self.V_req, R, self.i_req)
        self.P_elec = utilities.get_P_elec(self.V_req, self.i_req)

        if 0 <= eta_m <= 1:
            self.eta_m = eta_m
            self.logger.info(f"Motor efficiency for prop-motor {self.signature}: {eta_m}")
        else:
            self.eta_m = None
            self.logger.warning(f"Calculated eta_m for prop-motor {self.signature} is out of range.")

        if self.P_shaft_req > self.P_shaft:
            self.eta_m = None
            self.logger.warning(f"Prop-motor {self.signature} does not generate enough shaft power for hover.")

    def obj_results_to_string(self, rank):
        """
        Make results into a string.
        """
        if self.eta_m is not None:
            result_string = (
                f"Rank: {rank}\n"
                f"Signature: {self.signature}\n"
                f"Prop ID: {self.prop_id}\n"
                f"Motor ID: {self.motor_id}\n"
                f"Applied motor voltage: {self.V_req:.2f}\n"
                f"Applied motor current: {self.i_req:.2f}\n"
                f"Desired RPM: {self.RPM_req:.2f}\n"
                f"Desired torque: {self.Q_req:.2f}\n"
                f"Motor efficiency: {self.eta_m:.2f}\n"
                f"Shaft power: {self.P_shaft:.2f}\n"
                f"Electric Power: {self.P_elec:.2f}\n"
                "----------------------------------------------------------\n"
            )

            self.logger.info(f"Results for prop-motor {self.signature} generated successfully.")
            return result_string
        else:
            self.logger.warning(f"Motor efficiency (eta_m) is None for prop-motor {self.signature}. No results to generate.")
            return ""
        
    def get_arrays(self):
        raise NotImplementedError("Subclasses must implement this method.")
    
    def plot_obj(self):
        """
        Plot the thrust vs RPM, torque vs RPM, and motor efficiency vs RPM.
        """
        if self.RPM_req is None or self.V_req is None or self.eta_m is None:
            self.logger.warning(f"Cannot plot data because some required values are missing for prop {self.prop_id} with motor {self.motor_id}.")
            return

        # Prepare data for plotting
        RPM_vals, thrust_vals, prop_torque_vals = self.get_arrays()
        motor_torque_vals = np.array([utilities.get_motor_torque(self.V_req, self.motor_data['kV'], self.motor_data['i0'], self.motor_data['R'], RPM) for RPM in self.prop_data['RPM']])
        eta_m_vals = np.array([utilities.get_eta_m(self.motor_data['i0'], self.V_req, self.motor_data['R'], utilities.get_current_req(self.V_req, RPM, self.motor_data['kV'], self.motor_data['R'])) for RPM in self.prop_data['RPM']])

        # Spline interpolation
        thrust_spline = CubicSpline(RPM_vals, thrust_vals)
        prop_torque_spline = CubicSpline(RPM_vals, prop_torque_vals)
        motor_torque_spline = CubicSpline(RPM_vals, motor_torque_vals)
        eta_m_spline = CubicSpline(RPM_vals, eta_m_vals)

        # RPM range for plotting
        RPM_range = np.linspace(min(RPM_vals), max(RPM_vals), 500)
        
        # Create subplots
        fig, axs = plt.subplots(3, 1, figsize=(10, 15))

        # Global title
        fig.suptitle(f"Performance Analysis for Prop-Motor {self.signature} at T = {self.T_req:.2f} N", fontsize=18, fontweight='bold')

        # Thrust vs RPM
        axs[0].plot(RPM_range, thrust_spline(RPM_range), label='Thrust Curve')
        axs[0].axvline(self.RPM_req, color='r', linestyle='--', label='Required RPM')
        axs[0].axhline(self.T_req, color='g', linestyle='--', label='Desired Thrust')
        axs[0].annotate(
            fr'$RPM_{{req}}$: {self.RPM_req:.2f}', 
            xy=(self.RPM_req, max(thrust_vals)), 
            xytext=(self.RPM_req + 0.1 * self.RPM_req, max(thrust_vals) + 0.1 * max(thrust_vals)),
            arrowprops=dict(facecolor='black', arrowstyle="->", shrinkA=0, shrinkB=0), fontsize=16
        )
        axs[0].set_title('Thrust vs RPM', fontsize=16)
        axs[0].set_xlabel('RPM', fontsize=14)
        axs[0].set_ylabel('Thrust (N)', fontsize=14)
        axs[0].legend(fontsize=14)
        axs[0].grid(False)

        # Torque vs RPM
        axs[1].plot(RPM_range, prop_torque_spline(RPM_range), label='Prop Torque Curve')
        axs[1].plot(RPM_range, motor_torque_spline(RPM_range), label=f'Motor Torque at {self.V_req:.2f} V')
        axs[1].axvline(self.RPM_req, color='r', linestyle='--', label='Required RPM')
        axs[1].annotate(
            fr'$Torque_{{req}}$: {self.Q_req:.2f}', 
            xy=(self.RPM_req, self.Q_req), 
            xytext=(self.RPM_req + 0.1 * self.RPM_req, self.Q_req + 1),
            arrowprops=dict(facecolor='black', arrowstyle="->", shrinkA=0, shrinkB=0), fontsize=16
        )
        axs[1].set_title('Torque vs RPM', fontsize=16)
        axs[1].set_xlabel('RPM', fontsize=14)
        axs[1].set_ylabel('Torque (N-m)', fontsize=14)
        axs[1].set_ylim(bottom = 0)
        axs[1].legend(fontsize=14)
        axs[1].grid(False)

        # Motor efficiency (eta_m) vs RPM
        axs[2].plot(RPM_range, eta_m_spline(RPM_range), label='Motor Efficiency Curve')
        axs[2].axvline(self.RPM_req, color='r', linestyle='--', label='Required RPM')
        axs[2].annotate(fr'$\eta_m$: {self.eta_m:.2f}', xy=(self.RPM_req, self.eta_m), xytext=(self.RPM_req + 0.1 * self.RPM_req, self.eta_m + 0.1),
                        arrowprops=dict(facecolor='black',  arrowstyle="->", shrinkA=0, shrinkB=0), fontsize=16)
        axs[2].set_title(r'Motor Efficiency ($\eta_m$) vs RPM', fontsize=16)
        axs[2].set_xlabel('RPM', fontsize=14)
        axs[2].set_ylabel(r'$\eta_m$', fontsize=14)
        axs[2].legend(fontsize=14)
        axs[2].grid(False)

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        plt.savefig("figures/" + self.signature)

class StaticThrustUIUC(StaticThrust):
    def __init__(self, T_req, prop_filepath, motor_filepath, logger):
        super().__init__(T_req, prop_filepath, motor_filepath, logger)
        self.prop_data = self.read_prop_file(prop_filepath)

    def read_prop_file(self, prop_filepath):
        prop_data = {
            'RPM': [],
            'Ct': [],
            'Cp': []
        }

        try:
            with open(prop_filepath, 'r') as file:
                lines = file.readlines()
                if len(lines) >= 2:
                    for line in lines[1:]:  # Skip header line
                        if line.strip():  # Skip empty lines
                            RPM, Ct, Cp = map(float, line.strip().split())
                            prop_data['RPM'].append(RPM)
                            prop_data['Ct'].append(Ct)
                            prop_data['Cp'].append(Cp)
                else:
                    self.logger.error(f"File {prop_filepath} does not contain enough lines.")
                    print(f"File {prop_filepath} does not contain enough lines. See README for formatting details.")
        except Exception as e:
            self.logger.error(f"Error reading {prop_filepath}: {e}")
            print(f"Error reading {prop_filepath}: {e}. See README for formatting details.")

        self.logger.info(f"Processed propeller file {prop_filepath}")

        return prop_data
    
    def get_RPM_req(self):
        """
        Calculate the required RPM for the propeller to achieve the desired thrust.
        """

        if self.diameter is None:
            self.logger.warning(f"Diameter not formatter properly for prop-motor {self.signature}. Cannot calculate RPM required.")
            return
        thrust_vals = []
        RPM_vals = []

        for i in range(len(self.prop_data['RPM'])):
            RPM = self.prop_data['RPM'][i]
            Ct = self.prop_data['Ct'][i]
            thrust = utilities.get_thrust(self.diameter, RPM, Ct)
            thrust_vals.append(thrust)
            RPM_vals.append(RPM)

        sorted_indices = np.argsort(thrust_vals)
        thrust_vals = np.array(thrust_vals)[sorted_indices]
        RPM_vals = np.array(RPM_vals)[sorted_indices]
        
        # Find the two thrust values between which the desired thrust is found
        if thrust_vals[0] <= self.T_req <= thrust_vals[-1]:
            spline = CubicSpline(thrust_vals, RPM_vals)
            RPM_req = spline(self.T_req)
            self.RPM_req = RPM_req
            self.logger.info(f"Required RPM: {RPM_req}")
        else:
            self.logger.warning(f"Desired thrust {self.T_req} N is out of range for the propeller data.")

    def get_voltage_req(self):
        """
        Calculate the required voltage for the motor-propeller pairing to achieve the desired thrust.
        """
        if self.RPM_req is None:
            self.logger.warning(f"RPM requirement not met for prop-motor {self.signature}. Cannot calculate voltage required.")
            return
        
        prop_torque_vals = []
        RPM_vals = []

        for i in range(len(self.prop_data['RPM'])):
            RPM = self.prop_data['RPM'][i]
            Cp = self.prop_data['Cp'][i]
            prop_torque = utilities.get_prop_torque(self.diameter, RPM, Cp)
            prop_torque_vals.append(prop_torque)
            RPM_vals.append(RPM)

        # Calculate the required torque for the desired RPM
        spline = CubicSpline(RPM_vals, prop_torque_vals)
        self.Q_req = spline(self.RPM_req)

        # Extract motor data
        kV = self.motor_data['kV']
        i0 = self.motor_data['i0']
        R = self.motor_data['R']
        i_tol = self.motor_data['i_tol']

        # Calculate required voltage
        self.V_req = utilities.get_voltage_req(self.Q_req, kV, i0, R, self.RPM_req)

        # Calculate required current
        i_req = utilities.get_current_req(self.V_req, self.RPM_req, kV, R)
        
        if i_req <= i_tol:
            self.i_req = i_req
            self.logger.info(f"Required current for prop-motor {self.signature}: {i_req}")
        else:
            self.i_req = None
            self.logger.warning(f"Required voltage is beyond the current tolerance for prop-motor {self.signature}.") 

    def get_arrays(self):
        RPM_vals = np.array(self.prop_data['RPM'])
        thrust_vals = np.array([utilities.get_thrust(self.diameter, RPM, Ct) for RPM, Ct in zip(self.prop_data['RPM'], self.prop_data['Ct'])])
        prop_torque_vals = np.array([utilities.get_prop_torque(self.diameter, RPM, Cp) for RPM, Cp in zip(self.prop_data['RPM'], self.prop_data['Cp'])])

        return RPM_vals, thrust_vals, prop_torque_vals
 
class StaticThrustTYTO(StaticThrust):
    def __init__(self, T_req, prop_filepath, motor_filepath, logger):
        super().__init__(T_req, prop_filepath, motor_filepath, logger)
        self.prop_data = self.read_prop_file(prop_filepath)

    def read_prop_file(self, prop_filepath):
        df = pd.read_csv(prop_filepath)

        prop_data = {
            'RPM': df['Rotation speed (rpm)'].tolist(),
            'T': (df['Thrust (kgf)'] * 9.807).tolist(),
            'Q': df['Torque (N⋅m)'].tolist()
        }

        self.logger.info(f"Processed propeller file {prop_filepath}")

        return prop_data
    
    def get_RPM_req(self):
        """
        Calculate the required RPM for the propeller to achieve the desired thrust.
        """

        if self.diameter is None:
            self.logger.warning(f"Diameter not formatter properly for prop-motor {self.signature}. Cannot calculate RPM required.")
            return
        thrust_vals = []
        RPM_vals = []

        for i in range(len(self.prop_data['RPM'])):
            RPM = self.prop_data['RPM'][i]
            thrust = self.prop_data['T'][i]
            thrust_vals.append(thrust)
            RPM_vals.append(RPM)

        sorted_indices = np.argsort(thrust_vals)
        thrust_vals = np.array(thrust_vals)[sorted_indices]
        RPM_vals = np.array(RPM_vals)[sorted_indices]
        
        # Find the two thrust values between which the desired thrust is found
        if thrust_vals[0] <= self.T_req <= thrust_vals[-1]:
            spline = CubicSpline(thrust_vals, RPM_vals)
            RPM_req = spline(self.T_req)
            self.RPM_req = RPM_req
            self.logger.info(f"Required RPM: {RPM_req}")
        else:
            self.logger.warning(f"Desired thrust {self.T_req} N is out of range for the propeller data.")

    def get_voltage_req(self):
        """
        Calculate the required voltage for the motor-propeller pairing to achieve the desired thrust.
        """
        if self.RPM_req is None:
            self.logger.warning(f"RPM requirement not met for prop-motor {self.signature}. Cannot calculate voltage required.")
            return
        
        prop_torque_vals = []
        RPM_vals = []

        for i in range(len(self.prop_data['RPM'])):
            RPM = self.prop_data['RPM'][i]
            prop_torque = self.prop_data['Q'][i]
            prop_torque_vals.append(prop_torque)
            RPM_vals.append(RPM)

        # Calculate the required torque for the desired RPM
        spline = CubicSpline(RPM_vals, prop_torque_vals)
        self.Q_req = spline(self.RPM_req)

        # Extract motor data
        kV = self.motor_data['kV']
        i0 = self.motor_data['i0']
        R = self.motor_data['R']
        i_tol = self.motor_data['i_tol']

        # Calculate required voltage
        self.V_req = utilities.get_voltage_req(self.Q_req, kV, i0, R, self.RPM_req)

        # Calculate required current
        i_req = utilities.get_current_req(self.V_req, self.RPM_req, kV, R)
        
        if i_req <= i_tol:
            self.i_req = i_req
            self.logger.info(f"Required current for prop-motor {self.signature}: {i_req}")
        else:
            self.i_req = None
            self.logger.warning(f"Required voltage is beyond the current tolerance for prop-motor {self.signature}.")

    def get_arrays(self):
        RPM_vals = np.array(self.prop_data['RPM'])
        thrust_vals = np.array(self.prop_data['T'])
        prop_torque_vals = np.array(self.prop_data['Q'])

        return RPM_vals, thrust_vals, prop_torque_vals


class StableFlight(FlightMode):
    def __init__(self, T_req, prop_filepath, motor_filepath, target_velocity, logger):
        super().__init__(T_req, prop_filepath, motor_filepath, logger)
        self.target_velocity = target_velocity
        self.prop_data = self.read_prop_filename(prop_filepath)

        self.logger.info("Initialized StableFlight prop-motor obj")
