# Dynamic Traffic Signal Control using Reinforcement Learning

This project explores the application of various Reinforcement Learning (RL) algorithms to optimize traffic signal control in simulated urban environments. It was developed as part of an IITM MTP (Master's Thesis Project).

## Scenarios/Environments

This repository provides three simulated traffic scenarios:

*   **Single Junction:** A single intersection where traffic flow is managed. This scenario is ideal for initial algorithm testing and parameter tuning.
*   **2 Junctn Corridor:** A corridor with two consecutive junctions, allowing for the study of coordinated signal control.
*   **2x2 Grid Network:** A grid of four intersections, representing a more complex urban environment.

## Algorithms Implemented

The following RL algorithms and baseline controllers are implemented:

*   **Monte Carlo Control (MCC):** An RL method that learns from complete episodes of interaction with the environment. It's suitable for episodic tasks and estimates state-action values based on the mean return.
*   **Q-Learning:** A value-based off-policy temporal difference (TD) learning algorithm. It learns the optimal action-selection policy by iteratively updating Q-values for state-action pairs.
*   **Q-Learning with Function Approximation (QLFA):** An extension of Q-learning that uses a function approximator (e.g., a linear model) to estimate Q-values. This allows Q-learning to be applied to problems with large or continuous state spaces.
*   **Fixed-Time Controller:** A traditional baseline controller that operates traffic signals with pre-set, fixed cycle lengths and phase timings. It does not adapt to real-time traffic conditions.

## Software Dependencies

*   **SUMO (Simulation of Urban MObility):** A microscopic, continuous-space, and discrete-time traffic flow simulation platform. Latest stable version recommended.
*   **Python 3:** Python 3.6+ is recommended.
*   **Python Libraries:**
    *   `numpy`: For numerical computations and handling large arrays.
    *   `matplotlib`: For generating plots and visualizing results.
    *   `traci`: SUMO TraCI API for Python. This library is usually included with the SUMO installation and provides the interface to control and interact with the simulation.

## Repository Structure

The repository is organized as follows:

*   `Single Junction/`: Contains all code, data, and result files related to the single junction scenario.
*   `2 Junctn Corridor/`: Contains all code, data, and result files for the two-junction corridor scenario.
*   `2x2Gridnw/`: Contains all code, data, and result files for the 2x2 grid network scenario.

Within each scenario directory, you will typically find:

*   Python scripts for running simulations and implementing the different algorithms (e.g., `runner.py` which might be the main execution script, `mcc.py`, `qlearning.py`, `qFA.py`).
*   `data/` subdirectory: This holds SUMO-specific configuration files:
    *   `.sumocfg`: The SUMO configuration file that defines the simulation parameters, network, routes, and additional files.
    *   `.net.xml`: The road network definition file.
    *   `.rou.xml`: The vehicle routes and traffic demand definition file.
*   `Results/` subdirectory: This directory is used to store plots, graphs, and any other output data generated from running the simulations.
*   `sumoPath.py`: (Present in some directories) A utility script likely used to configure the system's Python path to include the SUMO `tools` directory, which is necessary for `traci` to function correctly.

### Running Simulations

1.  **Prerequisites:**
    *   Ensure SUMO is installed and the `SUMO_HOME` environment variable is set. The `tools` directory within your SUMO installation (e.g., `SUMO_HOME/tools`) should be accessible by your Python environment, as it contains the `traci` library. Some scripts in this repository use `sumoPath.py` to help locate SUMO, but a correct `SUMO_HOME` setup is generally recommended.
    *   Install Python 3 and the required libraries: `numpy` and `matplotlib`. You can typically install them using pip:
        ```bash
        pip install numpy matplotlib
        ```

2.  **Execution:**
    *   Navigate to the desired scenario directory:
        *   `cd "Single Junction"`
        *   `cd "2 Junctn Corridor"`
        *   `cd "2x2Gridnw"`
    *   The main scripts for running simulations are usually named after the algorithm they implement (e.g., `mcc.py`, `qlearning.py`, `qFA.py`). The `Single Junction` scenario also has a `runner.py` which executes the MCC algorithm.
    *   To run a specific simulation, execute its corresponding Python script. For example:
        *   In `Single Junction/`:
            *   `python runner.py` (for MCC vs Fixed)
            *   `python qlearning.py` (for Tabular Q-learning vs Fixed)
            *   `python qFA.py` (for QLFA/QFS vs Fixed)
        *   In `2 Junctn Corridor/`:
            *   `python mcc.py`
            *   `python qlearning.py`
            *   `python qFA.py`
        *   In `2x2Gridnw/`:
            *   `python qFA.py` (This scenario primarily features QLFA/QFS)
    *   Most simulation scripts support a `--gui` option to run SUMO with its graphical interface:
        ```bash
        python <script_name.py> --gui
        ```
    *   The `genRoutes.py` script found in each scenario directory is used to generate the traffic route files (e.g., `sq.rou.xml`, `2sig.rou.xml`, `2x2.rou.xml`) and is not run as a primary simulation entry point. These route files are then used by the simulation scripts.

## Understanding Results

Simulations typically evaluate the performance of different traffic signal control algorithms. The results are often saved as plots (e.g., `.png`, `.eps` files) in the `Results/` subdirectory of each scenario. Common performance metrics include:

*   **Average Waiting Time:** The average time vehicles spend waiting at intersections.
*   **Total Number of Vehicles (Throughput):** The total count of vehicles that successfully pass through the network during the simulation period.
*   **Queue Lengths:** Average or maximum queue lengths at intersections.
*   **Algorithm Comparison:** Plots often compare the performance of the RL-based controllers against the fixed-time controller or against each other.

## How to Contribute

Contributions to this project are welcome. Please follow these standard guidelines:

1.  **Fork the repository** on GitHub.
2.  **Create a new branch** for your feature or bug fix:
    ```bash
    git checkout -b feature/YourAmazingFeature
    ```
3.  **Make your changes** and commit them with clear, descriptive messages:
    ```bash
    git commit -m 'Add some amazing feature'
    ```
4.  **Push your changes** to your forked repository:
    ```bash
    git push origin feature/YourAmazingFeature
    ```
5.  **Open a Pull Request** on the original repository to propose your changes.

## License

License information to be added. For now, please assume standard academic research use.
