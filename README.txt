
# Van der Pol Oscillator Simulation

This project simulates the Van der Pol oscillator with a trigger mechanism and visualizes the results. The implementation includes a main script to set up and run the simulation, a plotting function to generate results, and several auxiliary functions used internally. The auxiliary functions should not be modified to ensure the results work

---

## Table of Contents

- [Project Structure](#project-structure)
- [Dependencies](#dependencies)
- [Usage](#usage)
- [Function Descriptions](#function-descriptions)
- [Notes](#notes)

---

## Project Structure

```
project-folder/
    |-- runVanDerPolTrigger.m       # Main script to set up and run the simulation
    |-- plotSimulationResults.m     # Generates plots based on simulation results
    |-- 
```

---

## Dependencies

Make sure you have the following software installed:

- MATLAB (R2021a or later recommended)

This code requires Simulink tool boxes.

---

## Usage

### Step 1: Initialize and Run Simulation

Run the main script `runVanDerPolTrigger.m` to initialize and execute the simulation. Be sure your abs tol and rel tol match those described in the paper. Larger tolerances will have larger errors

```matlab
runVanDerPolTrigger
```

This script sets up all necessary parameters and calls the auxiliary functions to perform the simulation.

### Step 2: Plot the Results

After the simulation completes, run the `plotSimulationResults.m` script to generate and display the results.

```matlab
plotSimulationResults
```

The plots will display key results as shown in the referenced paper.

---

## Function Descriptions

### 1. `runVanDerPolTrigger.m`

This is the main script that:

- Initializes simulation parameters (e.g., time step, oscillator parameters).
- Calls auxiliary functions to compute the oscillator's behavior over time.

### 2. `plotSimulationResults.m`

This script generates plots based on the simulation results. The plots provide visual insights into the oscillator’s dynamics.

### 3. Auxiliary Functions (in `auxiliaryFunctions/`)

These functions handle specific calculations and helper tasks required by the main simulation. They are not intended to be edited.

---

## Notes

- The auxiliary functions are pre-configured and should not be modified to ensure the accuracy and stability of the simulation.
- Make sure to run `runVanDerPolTrigger.m` before executing `plotSimulationResults.m` to ensure the results are correctly generated.
- If you encounter issues, check the parameter setup in `runVanDerPolTrigger.m`.

---

## Contact

For questions or issues related to the code, please contact the project owner or team lead.
