import time
import os
import utilities
import flight_modes

def main():
    logger = utilities.setup_logger()
    print("\nWelcome to the powertrain optimizer for a propeller-based vehicle!")
    is_continuing = True

    while is_continuing:
        flight_mode, data_type, num_props, weight, thrust_factor, target_velocity, motor_dir, prop_dir = utilities.get_user_inputs(logger)
        T_req = utilities.get_thrust_req(num_props, weight, thrust_factor)

        if flight_mode == 'stable flight':
            # Handle stable flight mode (TODO)
            logger.warning("Stable flight mode is not yet implemented.")
            continue

        prop_files = []
        motor_files = []

        # Walk through motor_dir to find all text files
        for root, _, files in os.walk(motor_dir):
            for file in files:
                if file.endswith('.txt'):
                    motor_files.append(os.path.join(root, file))

        if data_type == 'UIUC':
            # Walk through prop_dir to find all text files
            for root, _, files in os.walk(prop_dir):
                for file in files:
                    if 'static' in file and file.endswith('.txt'):
                        prop_files.append(os.path.join(root, file))
        else:
            # Walk through prop_dir to find all text files
            for root, _, files in os.walk(prop_dir):
                for file in files:
                    if 'static' in file and file.endswith('.csv'):
                        prop_files.append(os.path.join(root, file))

        top_10 = []

        for prop_filepath in prop_files:
            for motor_filepath in motor_files:
                if data_type == 'UIUC':
                    static_thrust = flight_modes.StaticThrustUIUC(T_req, prop_filepath, motor_filepath, logger)
                else:
                    static_thrust = flight_modes.StaticThrustTYTO(T_req, prop_filepath, motor_filepath, logger)

                # Step 1: Thrust matching
                static_thrust.get_RPM_req()

                # Step 2: Torque matching
                static_thrust.get_voltage_req()

                # Step 3: Motor efficiency
                static_thrust.get_eta_m()

                if static_thrust.eta_m is not None:
                    # Add the result to the top_10 list
                    top_10.append(static_thrust)
                    top_10 = sorted(top_10, key=lambda x: x.eta_m, reverse=True)[:10]  # Keep only top 10

                logger.info(f"Processed motor {motor_filepath} with propeller {prop_filepath}.")

        if len(top_10) == 0:
            logger.warning("Found no good matches.")
            print("Found no good matches.")
            is_continuing = input("Do you want to run another analysis? (y/n): ").strip().lower() == 'y'

            if is_continuing:
                continue
            else:
                break
        
        if not os.path.exists("results"):
            os.makedirs("results")

        results_filepath = f"results/{num_props}_{weight}_{thrust_factor}_{top_10[0].get_signature()}.txt"

        with open(results_filepath, 'w') as file: 
            header = utilities.get_header(T_req, top_10[0].get_P_shaft_req())
            file.write(header)

            for i, result in enumerate(top_10, start=1):
                result_string = result.obj_results_to_string(rank=i)
                file.write(result_string)

        logger.info(f"Results file {results_filepath} created successfully.")
        print(f"Saved results to {results_filepath}")

        if not os.path.exists("figures"):
            os.makedirs("figures")

        while True:
            try:
                plot_count = int(input(f"Found {len(top_10)} good matches. How many plots would you like to make? (0 - 10): ").strip())
                if 0 <= plot_count <= 10:
                    break
                else:
                    print("Please enter a number between 0 and 10.")
            except ValueError:
                print("Invalid input. Please enter a valid integer.")

        # Plot the desired number of figures
        if len(top_10) >= plot_count:
            for i in range(plot_count):
                top_10[i].plot_obj()
        else:
            for i in range(len(top_10)):
                top_10[i].plot_obj()
        
        logger.info(f"Plots saved to 'figures' directory.")

        is_continuing = input("Do you want to run another analysis? (y/n): ").strip().lower() == 'y'
        
    logger.info("Done")


if __name__ == '__main__':
    start = time.time()
    main()
    end = time.time()
    total = end - start
    print(f"Time elapsed: {total} s")
    print("Done.")