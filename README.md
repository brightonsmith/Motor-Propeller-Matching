# Motor-Propeller-Matching
Author: Brighton Smith

## Description
A Python script for evaluating efficiency alignment of a motor/propeller combination. This program is a physical implementation of the MIT paper called "DC Motor / Propeller Matching", available [here](https://web.mit.edu/drela/Public/web/qprop/motorprop.pdf). The program pulls from two propeller databases:
1. UIUC Propeller Data Site: a webpage including wind tunnel measurements for propellers used on small UAVs and model aircraft. The website is available [here](https://m-selig.ae.illinois.edu/props/propDB.html), but all of the data files are available in the REPO under `propeller_data_UIUC`.
2. Tyto Robotics Database: a community-driven database of motors and propellers commonly used in UAV propulsion systems, available [here](https://database.tytorobotics.com/). Some example files are included in the REPO under `propeller_data_TYTO`.

In terms of motor data, the program only relies on basic motor properties that can be found on any motor-specification page. 

## Operative Modes
This program has two modes, `static thrust` and `stable flight`. The difference between the two is that `static thrust` is designed for UAV's in a strict hover configuration, while `stable flight` defines a lateral target velocity to be met by the vehicle. **Note that `stable flight` mode has not yet been implemented, but will be added in a future update.**

## Propeller Data Formatting
If in `static thrust` mode, the propeller file names should be of the form `[make]_[diameter]x[pitch]_static_[info]` with a `.txt` extension for UIUC data and a `.csv` extension for TYTO data. If in `stable flight` mode, the propeller file names should be of the form `[make]_[diameter]x[pitch]_[model]_[info]` with the same corresponding extensions. Note that `[info]` can be any relevant info that the user may want to store (i.e. model, motor used during propeller trials, etc.). Spaces can be used between the underscores.

### UIUC Formatting
UIUC `static thrust` files should look like this:
```
RPM     CT      CP
data    data    data
data    data    data
data    data    data
data    data    data
```
while `stable flight` files should look like this:
```
J       CT       CP       eta
data    data     data     data
data    data     data     data
data    data     data     data
data    data     data     data
```
## TYTO Formatting
TYTO has the same formatting for both `static thrust` and `stable flight`, and usually includes many extraneous data columns. For the purpose of the program, TYTO data must include the following columns:
```
Rotation speed (rpm)    Thrust (kgf)    Torque (N⋅m)
data                    data            data
data                    data            data
data                    data            data
data                    data            data
```

## Motor Data Formatting
As mentioned before, the program only needs basic motor properties to operate, specifically the Kv ratio, zero-load current, internal resistance, and maximum operating current. Therefore, a viable motor file input takes the form
```
[Kv (RPM/V)]
[i_0 (A)]
[R (Ω)]
[i_max (A)]
```
Several examples have been provided in `motor_data`. 

## Usage
To run this program, run the following commands in a shell environment:
```sh
# Install the required packages
pip install -r requirements.txt

# Run the Python script
python driver.py
```
The program will walk you through a series of user-friendly commands. Just make sure the propeller and motor data is contained within the current directory, and that it is formatted correctly. If good motor-propeller matches are found, the file will be saved to ```results/[number of props]_[total weight]_[thrust-to-weight ratio]_[signature of best match].txt```, where the signature is a hash code unique to the best motor-propeller match. Since resulting figures are saved to ```figures/[signature].png```, the naming convention of the results text file makes it easy to find the corresponding figure of the most favorable match.

## Additional Updates for Future Versions
- Implement `stable flight` mode
- Generate more plots, especially power plots
- Enforce the power required value more rigorously