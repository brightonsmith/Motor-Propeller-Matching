import time
import utilities
import flight_modes

def main():
    logger = utilities.setup_logger()
    print("\nWelcome to the powertrain optimizer for a propeller-based vehicle!")
    is_continuing = True

    while is_continuing:
        flight_mode, num_props, weight, thrust_factor, target_velocity, motor_dir, prop_dir = utilities.get_user_inputs()
        thrust_req = utilities.get_thrust_req(num_props, weight, thrust_factor)

        if flight_mode == 'stable flight':
            # Placeholder for stable flight implementation
            print("Stable flight mode is not yet implemented. Exiting.")
            break  # Exiting the loop as stable flight is not handled yet
        else:
            static_thrust = flight_modes.StaticThrust(thrust_req, prop_dir, motor_dir, logger)

            # Step 1: Thrust matching
            static_thrust.build_RPM_req_dic()

            # Step 2: Torque matching
            static_thrust.build_voltage_req_dic()

            # Step 3: Motor efficiency
            static_thrust.build_eta_m_dic()

            results_filename = input(f"Enter results filename for {static_thrust.signature}: ")
            static_thrust.write_results_file(results_filename)

            logger.info(f"Discarding object {static_thrust.signature}")

            # TODO: update using virtual thrust

        while True:
            continue_response = input("Do you want to run another analysis? (y/n): ").strip().lower()
            if continue_response in ['y', 'n']:
                is_continuing = (continue_response == 'y')
                break
            else:
                logger.error("Invalid continuation response. Must enter 'y' or 'n'.")
                print("Invalid input. Please enter 'y' or 'n'.")
    
    logger.info("Done")


if __name__ == 'main':
    start = time.time()
    main()
    end = time.time()
    total = end - start
    print(f"Time elapsed: {total} s")
    print("Done.")