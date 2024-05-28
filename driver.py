import logging
import os
import time
import pandas as pd
import utilities
import operational_modes

def main():
    logger = utilities.setup_logger()
    print("\nWelcome to the powertrain optimizer for a propeller-based vehicle!")
    operation_mode, num_props, weight, thrust_factor, target_velocity, motor_dir, prop_dir = utilities.get_user_inputs()
    thrust_req = utilities.get_thrust_req(num_props, weight, thrust_factor)

if __name__ == 'main':
    start = time.time()
    main()
    end = time.time()
    total = end - start
    print(f"Time elapsed: {total} s")
    print("Done.")