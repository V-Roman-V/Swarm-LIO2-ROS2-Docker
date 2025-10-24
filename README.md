# Swarm-LIO2-ROS1-Docker

> This branch contains the **ROS 1 (Noetic)** Dockerized version of Swarm-LIO2.  
> The ROS 2 port is developed in a [`main`](https://github.com/V-Roman-V/Swarm-LIO2-ROS2-Docker/tree/main) branch.

<div align="center">
    <h2>Swarm-LIO2-Docker: Docker Port of Swarm-LIO2</h2>
    <strong>Open-source community fork of HKU-MARS Swarm-LIO2</strong>
    <br><br>
    <a href="https://github.com/hku-mars/Swarm-LIO2" target="_blank">
        Original Swarm-LIO2 Repository (ROS 1)
    </a>
    <br><br>
    <img src="./image/mars_logo.svg" alt="HKU MaRS Lab" width="180"/>
</div>

---

### Introduction

**Swarm-LIO2-Docker** is an **open-source community-driven port** of [**Swarm-LIO2**](https://github.com/hku-mars/Swarm-LIO2), rewritten to support **Docker-based deployment**.

Swarm-LIO2 is a *decentralized, efficient LiDAR-inertial odometry and mapping framework* designed for UAV swarms,  
developed by the **HKU MARS Lab** and published in *IEEE Transactions on Robotics 2025*.

---

### Project Status

| Component | Status |
|------------|:------:|
| Repository setup | ✅ |
| Dockerfile | ✅ |
| Simulation / example data testing | In progress |
| Documentation | In progress |

---

### Quick Start (ROS 1)

1. **Build Docker image**

   ```bash
   cd docker
   docker compose build
   ```

2. **Start container**

   ```bash
   docker compose up
   ```

3. **TODO**

---

### Tech Stack

- **ROS 1 Noetic** (`ros:noetic-ros-base`)
- **Ubuntu 20.04** (Focal Fossa)
- **GTSAM 4.2** (built from [GTSAM repo](https://github.com/borglab/gtsam.git))
- **Livox SDK** (built from [Livox-SDK repo](https://github.com/Livox-SDK/Livox-SDK))
- **PCL** (via `libpcl-dev` + `ros-noetic-pcl-ros`)
- **Eigen3** (`libeigen3-dev`)
- **Boost**, **TBB**, and **METIS** for graph optimization
- **MAVROS** + **MAVROS Extras**
- **TF**, **CV Bridge**, **Image Transport**, **Visualization Msgs**
- **Python 3.8**, `matplotlib`, `pip`
- **Docker + docker-compose**

---

### Testing

For testing, [S3E dataset](https://pengyu-team.github.io/S3E) was used.

The tests were performed on the **S3E_Square_1** and **TODO** sequences.

### Results

*TODO*

---

### Roadmap

#### Phase 1: Setup & Porting
- [x] Fork original repository  
- [x] Define ROS 1 and Docker roadmap  
- [x] Create Dockerfiles for build/runtime  
- [x] Add multi-stage build support (builder → runtime image)  

#### Phase 2: Testing
- [ ] Test with simulated data
- [ ] Publish example bags and setup guide  

#### Phase 3: Documentation
- [ ] Add installation & usage instructions 

---

### Reference

Original paper:  
> [**Swarm-LIO2: Decentralized, Efficient LiDAR-Inertial Odometry for UAV Swarms**](https://arxiv.org/abs/2409.17798)  
> *F. Zhu, Y. Ren, L. Yin, F. Kong, Q. Liu, R. Xue, W. Liu, Y. Cai, G. Lu, H. Li, F. Zhang*  
> *IEEE Transactions on Robotics, 2025.*

Videos:  
- [YouTube Demo](https://youtu.be/Q7cJ9iRhlrY)  
- [Bilibili Demo](https://www.bilibili.com/video/BV1vTsMeqEQm)

---

### License and Attribution

Based on [Swarm-LIO2](https://github.com/hku-mars/Swarm-LIO2) © HKU MARS Lab.  
Docker port maintained by **Roman Voronov**.  
This fork follows the same license as the original project.

---

### Contributing

Contributions are very welcome!  
If you’re interested in:
- testing in real UAV environments,
- or improving documentation —

please open an **issue** or **pull request**.

---
