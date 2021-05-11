#!/usr/bin/env python3

import json
import matplotlib.pyplot as plt
import pandas as pd
import sys

# Load data
filename = sys.argv[1]

# Make DataFrame to facillitate plotting
if filename.endswith(".json"):
    with open(filename) as json_file:
        raw_data = json.load(json_file)
    unit = raw_data["units"]

    # Get Unit
    if unit == "Meters":
        abbreviation = "m"
    elif unit == "Feet":
        abbreviation = "ft"
    elif unit == "Inches":
        abbreviation = "in"
    elif unit == "Degrees":
        abbreviation = "deg"
    elif unit == "Rotations":
        abbreviation = "rot"
    elif unit == "Radians":
        abbreviation = "rad"
    else:
        raise ValueError("Invalid Unit")

    # Make Columns
    columns = ["Timestamp (s)", "Test"]
    if "Drive" in raw_data["test"]:
        columns.extend([
            "Left Volts (V)",
            "Right Volts (V)",
            f"Left Position ({abbreviation})",
            f"Right Position ({abbreviation})",
            f"Left Velocity ({abbreviation}/s)",
            f"Right Velocity ({abbreviation}/s)",
            "Gyro Position (deg)",
            "Gyro Rate (deg/s)",
        ])
        unit_columns = columns[4:8]
    else:
        columns.extend([
            "Volts (V)", f"Position ({abbreviation})",
            f"Velocity ({abbreviation}/s)"
        ])
        unit_columns = columns[3:]

    prepared_data = pd.DataFrame(columns=columns)
    for test in (test for test in raw_data.keys() if "-" in test):
        formatted_entry = [[pt[0]] + [test] + pt[1:] for pt in raw_data[test]]
        prepared_data = prepared_data.append(
            pd.DataFrame(formatted_entry, columns=columns))

    units_per_rot = raw_data["unitsPerRotation"]

    for column in unit_columns:
        prepared_data[column] *= units_per_rot
else:
    prepared_data = pd.read_csv(filename)

# First 2 columns are Timestamp and Test
for column in prepared_data.columns[2:]:
    # Configure Plot Labels
    plt.figure()
    plt.xlabel("Timestamp (s)")
    plt.ylabel(column)

    # Configure title without units
    print(column)
    end = column.find("(")
    plt.title(f"{column[:end].strip()} vs Time")

    # Plot data for each test
    for test in pd.unique(prepared_data["Test"]):
        test_data = prepared_data[prepared_data["Test"] == test]
        plt.plot(test_data["Timestamp (s)"], test_data[column], label=test)
    plt.legend()

plt.show()
