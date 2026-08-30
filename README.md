# Teleoperation Simulation Environment for a Curiosity-Inspired Rover

A simulation environment for teleoperating a rover inspired by NASA's **Curiosity** (Mars Science Laboratory), built with **CoppeliaSim** and a **Python** API. The system enables remote control of the rover's locomotion and robotic manipulator via a joystick (Xbox One controller), with data logging and analysis of the dynamic behavior across different simulated scenarios.

Undergraduate Thesis (TCC) — Mechatronics Engineering, Federal University of São João del-Rei (UFSJ).

**Author:** Gustavo Spoti Costa — gustavo-s-c@aluno.edu.com.br
**Advisor:** Prof. Filipe Augusto Santos Rocha — filipe@ufsj.edu.br

---

## 📋 About the project

Exploration of extreme environments — planetary surfaces, high-risk industrial areas, underground regions — drives the development of remote-control architectures that are safe, stable, and efficient. This project develops and validates a simulation environment for teleoperating a virtual model of the Curiosity rover, serving as an experimental platform for studies in teleoperation, mobile robotics, and digital twins applied to space exploration.

The rover's 3D model was obtained from NASA's official repository and configured in CoppeliaSim with joints and physical properties that approximately reproduce its mechanical structure and **rocker-bogie** locomotion system.

### General objective

To develop and validate a simulation environment integrated with a Python API for remote control of a Curiosity-inspired rover using a joystick.

### Specific objectives

- Evaluate the feasibility of the developed control architecture;
- Analyze the dynamic behavior of the system;
- Investigate the response of the robotic manipulator and the rocker-bogie suspension under different simulated conditions.

---

## 🛠️ Tools and technologies

| Tool | Function |
|---|---|
| **CoppeliaSim** | Physics simulation environment — rover modeling and execution of dynamic interactions |
| **Python** | Development of the communication API between the controller and the simulator |
| **Pygame** | Reading input from the Xbox One controller |
| **Matplotlib** | Chart generation and analysis of collected data |
| **Visual Studio Code** | Development and debugging environment |

---

## 🏗️ System architecture

The control flow follows this path:

```
Joystick (Xbox One) → Python API → CoppeliaSim → Rover + Robotic Arm
```

Joystick signals are captured via Pygame, processed by the API (smoothing, acceleration limiting, adaptive control), and converted into velocity/position commands sent to CoppeliaSim in continuous communication (*streaming*) mode.

The API is organized into functional blocks:

1. **Initialization** — remote connection to CoppeliaSim (`simxStart`) and retrieval of joint handles;
2. **Signal processing** — deadzone, logistic smoothing curve, and acceleration limiting;
3. **Command selection** — switching between locomotion mode and manipulator mode;
4. **Communication with CoppeliaSim** — continuous transmission of control commands;
5. **Logging and post-processing** — data collection (positions, timing, input signals) and chart generation.

### Rover model

The rover is composed of the following subsystems, organized hierarchically from the main chassis:

- **Rocker-bogie suspension** — *rocker*, *bogie*, and *upper link* elements, configured as dynamic joints in spring mode to reproduce passive adaptation to the terrain;
- **Steering system** — steering supports on the front and rear wheels (position control);
- **Wheels** — traction joints with velocity control;
- **Robotic manipulator** — 4 degrees of freedom (`joint_mani_p1`, `p2`, `p3`, `tool`), with dynamic position control.

### Operation modes (Xbox One controller)

**Locomotion mode:**
- Left stick (vertical axis) → wheel linear velocity
- Right stick (horizontal axis) → steering

**Manipulator mode:**
- Left stick (horizontal) → base rotation (P1)
- Left stick (vertical) → arm elevation (P2)
- Right stick (vertical) → elbow joint (P3)
- RT/LT triggers → end-effector rotation (Tool)

Top center button → toggles between the two modes.

### Signal processing

- **Deadzone** of 0.1 on the analog axes;
- **Logistic smoothing**: `f(x) = 2 / (1 + e^(-kx)) - 1`, with `k = 0.5`;
- **Artificial acceleration/deceleration limiting**, to simulate inertia;
- **Adaptive steering control**, reducing angular steering velocity at higher speeds;
- **Automatic steering return** to the center position, via a simplified PD controller (`Kp = 25.0`, `Kd = 8.0`).

---

## 📊 Results

- **Stable and continuous operation**, with no lockups or communication failures between the API and the simulator;
- Scene update rate constant at **20 FPS**, with a real-time factor of **0.890** (89% of wall-clock speed);
- Average communication latency of **6 ms** (local execution);
- Coherent behavior of the rocker-bogie suspension and the robotic manipulator on flat and irregular terrain;
- **Comparison between physics engines** (Bullet 2.78, Bullet 2.83, ODE, and Newton): the choice of engine primarily affects locomotion (sliding, suspension deformation, wheel locking), with no perceptible effect on the manipulator. **ODE** and **Newton** delivered the best performance, with ODE showing a slight edge on irregular terrain.

Comparative video of the rover's performance across the evaluated physics engines: [youtu.be/LFJZ8wvNDwA](https://youtu.be/LFJZ8wvNDwA?si=kjcsSZdWWTaLPe5N)

---

## ⚠️ Limitations

- Lack of precise physical data for the original rover (mass, friction, materials), requiring approximations;
- Control based on simplified techniques (smoothing, acceleration limiting, PD control only for steering return) — no full PID controllers;
- Communication executed locally, reducing the latency effects present in real space teleoperation scenarios;
- Manipulator without a functional gripping mechanism (advanced physical interaction with objects not implemented);
- The system represents a **functional approximation** of the Curiosity rover, not a complete digital twin.

---

## 🚀 Future work

- Implementation of full PID controllers;
- More precise physical models (mass distribution, friction, materials);
- Simulation of communication delays (realistic latency for space missions);
- Autonomous navigation algorithms and AI-based techniques for operational assistance;
- Integration of the architecture with physical robotic platforms.

---

## ▶️ How to run

### Prerequisites

- [CoppeliaSim](https://www.coppeliarobotics.com/) installed
- Python 3.9+
- An Xbox One controller (or compatible gamepad) connected to your machine

### Steps

1. **Clone this repository**
```bash
   git clone https://github.com/Guts-Robotics/rover-teleoperation-coppeliasim.git
   cd rover-teleoperation-coppeliasim
```

2. **Install the Python libraries**

   All required libraries are listed in [`requirements.txt`](requirements.txt). Install them all at once with:
```bash
   pip install -r requirements.txt
```

   If you prefer to install them individually:
```bash
   pip install pygame
   pip install matplotlib
   pip install numpy
```

   > **Note:** `pygame` is responsible for reading the Xbox controller inputs (analog sticks, triggers, and buttons). Make sure your controller is connected **before** running the script, otherwise `pygame` won't detect it.

3. **Add the CoppeliaSim Remote API to your Python project**

   This project communicates with CoppeliaSim through its **Legacy Remote API**, which is not distributed via `pip` and must be added manually:

   1. Locate your CoppeliaSim installation folder. By default:
      - Windows: `C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\`
      - Linux: `~/CoppeliaSim/`
      - macOS: `/Applications/coppeliaSim.app/`

   2. Navigate to:
```
      programming/legacyRemoteApi/remoteApiBindings/
```

   3. Copy the following files into this project's [`src/`](src/) folder:
      - `python/python/sim.py`
      - `python/python/simConst.py`
      - The compiled library for your operating system, found in `lib/lib/<your-platform>/`:
        - Windows: `remoteApi.dll`
        - Linux: `remoteApi.so`
        - macOS: `remoteApi.dylib`

   4. No `pip install` is required for these files — they just need to be in the same folder as the control script (or included in your `PYTHONPATH`). Once copied, you can import the API normally in Python:
```python
      import sim
```

4. **Open the scene in CoppeliaSim**

   Launch CoppeliaSim and open the rover scene file provided in this repository: [`scenes/rover_scene.ttt`](scenes/rover_scene.ttt).

5. **Enable the Remote API server on the scene**

   Make sure the scene has the continuous Remote API server service enabled (in CoppeliaSim: `Add > Remote API server service`, typically on port `19999`).

6. **Start the simulation**

   Press the play button in CoppeliaSim to start the physics simulation.

7. **Run the control API**
```bash
   python src/control_api.py
```

8. **Control the rover**

   - Use the **left stick** to move the rover forward/backward, and the **right stick** to steer.
   - Press the **top center button** to switch between locomotion and manipulator mode.

   See the [Operation modes](#operation-modes-xbox-one-controller) section above for the full control mapping.

---

## 📚 Main references

- NASA — [Curiosity Rover 3D Model](https://science.nasa.gov/resource/curiosity-rover-3d-model/)
- NASA — [Mars Science Laboratory (MSL) Curiosity](https://science.nasa.gov/mission/msl-curiosity/)
- Rohmer, E., Singh, S. P. N., Freese, M. — *V-REP: A Versatile and Scalable Robot Simulation Framework*, IROS 2013.

The full reference list is available in the complete thesis (TCC) document.

---

## 📄 Full document

The complete TCC report — including detailed methodology, algorithms, joint configuration tables, and full results — is available here: [TCC_Gustavo_Spoti_Costa.pdf](docs/TCC_Gustavo_Spoti_Costa.pdf).

---

## 📝 License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.
