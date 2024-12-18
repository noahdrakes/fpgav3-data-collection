import argparse
import numpy as np
import json
import csv
import os

# osaUnitToSI Factor
deg2rad = np.pi / 180
mm2m = 0.001
enc_unit = [deg2rad, mm2m]

def unitConvert(configuration, fileName):
    with open(configuration, 'r') as file:
        robot_json = json.load(file)
    robot_data = np.genfromtxt(fileName, delimiter=",", dtype=None, names=True,encoding='utf-8')

    Robots = robot_json['Robots']
    Actuators = Robots[0]['Actuators']
    Curr_B2C, Curr_C2B, Curr_Nm2C, Enc_B2P, Pot_B2V, Pot_V2P = {}, {}, {}, {}, {}, {}
    Time, Torq_FB, Torq_CMD, Pos_FB, Vel_FB = {}, {}, {}, {}, {}

    # Extract column names
    column_names = robot_data.dtype.names

    # Find the channels 'MOTOR_CURRENT_i'
    motor_columns = [name for name in column_names if name.startswith('MOTOR_CURRENT_')]
    motorCurrentChannels = max(int(name.split('_')[-1]) for name in motor_columns)

    if (bool(Actuators[0]['Pot']['LookupTable']) == (motorCurrentChannels == 8)):
        raise ValueError(
            "Invalid configuration: Either 'LookupTable' must be None if robot is 'Si', "
            "or 'Lookuptable' must not be None if robot is 'classic'."
        )

    # Equation reference:
    # https://github.com/jhu-saw/sawRobotIO1394/blob/417e6bd88b15dfcc6f9647bc0d677ef097c001cf/core/components/code/mtsRobot1394.cpp#L859C1-L988C2
    Time['TIMESTAMP'] = robot_data['TIMESTAMP']
    for i in range(len(Actuators)):
        Curr_B2C[i] = Actuators[i]['Drive']['BitsToCurrent']
        Curr_C2B[i] = Actuators[i]['Drive']['CurrentToBits']
        Curr_Nm2C[i] = Actuators[i]['Drive']['EffortToCurrent']
        Enc_B2P[i] = Actuators[i]['Encoder']['BitsToPosition']
        Pot_B2V[i] = Actuators[i]['Pot']['BitsToVoltage']
        Pot_V2P[i] = Actuators[i]['Pot']['SensorToPosition']
        Torq_FB[f'TORQUE_FEEDBACK_{i+1}'] = ((robot_data[f'MOTOR_CURRENT_{i+1}'] * Curr_B2C[i]['Scale']) + Curr_B2C[i]['Offset']) / Curr_Nm2C[i][
            'Scale']
        Torq_CMD[f'TORQUE_COMMAND_{i+1}'] = ((robot_data[f'MOTOR_STATUS_{i+1}'] - Curr_C2B[i]['Offset']) / Curr_C2B[i]['Scale']) / Curr_Nm2C[i]['Scale']
        Pos_FB[f'POSITION_FEEDBACK_{i+1}'] = ((robot_data[f'ENCODER_POS_{i+1}'] - 0x800000) * Enc_B2P[i]['Scale']) \
                                             * enc_unit[bool(Actuators[i]['JointType'] == "PRISMATIC")]
        Vel_FB[f'VELOCITY_FEEDBACK_{i+1}'] = ((robot_data[f'ENCODER_VEL_{i+1}']) * Enc_B2P[i]['Scale']) \
                                             * enc_unit[bool(Actuators[i]['JointType'] == "PRISMATIC")]

    return {
        "Time": Time,
        "Torq_FB": Torq_FB,
        "Torq_CMD": Torq_CMD,
        "Pos_FB": Pos_FB,
        "Vel_FB": Vel_FB
    }

def saveConfigFile(fileName, dataConverted):
    fileNameConverted = f"{os.path.splitext(fileName)[0]}_unitConvert.csv"

    # Combine all the columns into one table
    columns = {}
    for key, sub_dict in dataConverted.items():
        for sub_key, values in sub_dict.items():
            columns[sub_key] = values

    # Ensure all columns have the same number of rows (fill with None if needed)
    max_len = max(len(values) for values in columns.values())
    for key in columns:
        if len(columns[key]) < max_len:
            columns[key].extend([None] * (max_len - len(columns[key])))

    # Write to CSV
    with open(fileNameConverted, mode='w', newline='') as file:
        writer = csv.writer(file)
        # Write the header
        writer.writerow(columns.keys())
        # Write the rows
        for row in zip(*columns.values()):
            writer.writerow(row)

    print(f"Data written to {fileNameConverted}")

def main():
    # Create the argument parser
    parser = argparse.ArgumentParser(description="Parse arguments for mode and file path.")

    parser.add_argument(
        "-c",
        "--configuration",
        type=str,
        help="path for the robot configuration file",
        default=None
    )
    parser.add_argument(
        "-f",
        "--file-name",
        type=str,
        help="path for the collected data csv file",
        default=None
    )

    # Parse the arguments
    args = parser.parse_args()

    dataConverted = {}
    dataConverted = unitConvert(args.configuration, args.file_name)

    saveConfigFile(args.file_name, dataConverted)

if __name__ == "__main__":
    main()
