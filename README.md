# Benefits of Varying Navigation Strategies in Teams of Robots

[![Publication DOI](https://img.shields.io/badge/Publication-Springer-blue)](https://link.springer.com/chapter/10.1007/978-3-031-71533-4_5)

This repository contains the source code, simulation environments, and analysis scripts for the paper:

> **Seyed Amirhosein Mohaddesi** (2024). *Benefits of Varying Navigation Strategies in Teams of Robots*. In: Advances in Robotics Research. Springer.  
> [Read the publication](https://link.springer.com/chapter/10.1007/978-3-031-71533-4_5)

The work is inspired by recent human navigation studies and investigates how **population variability** in navigation strategies affects multi-robot team performance.

<p align="center">
  <a href="https://www.youtube.com/watch?v=vQg6a5GYKL8" target="_blank">
    <img src="https://img.youtube.com/vi/vQg6a5GYKL8/maxresdefault.jpg" 
         alt="Demo Video" width="720">
  </a>
</p>

# Benefits of Varying Navigation Strategies in Teams of Robots

---

## 📖 Summary

We evaluate robot teams using:
1. **Route (RT)** – Follow predefined paths.
2. **Survey (SW)** – Take shortest paths while avoiding obstacles.
3. **Mixed strategies** – e.g., `0.4RT 0.6SW`, `0.6RT 0.4SW`.
4. **Human-inspired switching** – `0.9RT 0.1SW` (90% Route, 10% Survey).

**Key results:**
- SW is fastest in mission time.
- RT covers more environment.
- Mixed strategies offer a trade-off between coverage and speed.
- Variability in strategies benefits both robotic and biological systems.

---

## 📂 Repository Structure

```
Benefits-of-varying-navigation-sterategies-in-robots--main/
├── PR2Maze/
│   ├── controllers/              # Webots controller C++ source code
│   │   ├── SpikeWave/             # SpikeWave pathfinding implementation
│   │   ├── GoalAssignment*.cpp    # Strategy-specific goal assignment
│   │   ├── AssignedGoals*.txt     # Pre-assigned goal sets
│   │   ├── RobotList.txt          # List of robot IDs
│   │   ├── Sources.txt / Goals.txt# Start and goal positions
│   │   └── map_generator/         # Map generation scripts
│   ├── worlds/                    # Webots .wbt simulation worlds
│   └── results/                   # Simulation output logs
├── mapLoader.m / spikeWave.m      # MATLAB scripts for data analysis
├── topoWave.m                     # Topological wave exploration script
└── README.md
```

---

## 🛠 Requirements

- [Webots](https://cyberbotics.com/) R2021a or later  
- C++17 compiler (GCC/Clang)  
- MATLAB (for result analysis scripts)  
- Linux or macOS (tested on Ubuntu 20.04)

---

## 🚀 Setup & Running

1. **Clone repository**
   ```bash
   git clone https://github.com/YOUR_USERNAME/Benefits-of-varying-navigation-strategies.git
   cd Benefits-of-varying-navigation-strategies
   ```

2. **Open Webots**
   - Open the `.wbt` world in `PR2Maze/worlds/`.

3. **Build controllers**
   - In Webots, set the controller for each robot (e.g., `SpikeWave`) and build.

4. **Run simulation**
   - Press Play to start and observe team navigation under different strategies.

---

## 📊 Example Results

| Strategy        | Time Taken | Coverage |
|----------------|------------|----------|
| Route          | Medium     | High     |
| Survey         | Fastest    | Lower    |
| 0.4RT 0.6SW    | Balanced   | Balanced |
| 0.9RT 0.1SW    | Near-human | Balanced |

---

## 📜 Citation

If you use this code, please cite:

```
@inproceedings{mohaddesi2024navigation,
  title={Benefits of Varying Navigation Strategies in Teams of Robots},
  author={Mohaddesi, Seyed Amirhosein},
  booktitle={Advances in Robotics Research},
  year={2024},
  publisher={Springer}
}
```

---

## 📄 License

This project is licensed under the MIT License – see [LICENSE](LICENSE) for details.
