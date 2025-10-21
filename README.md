# Swarm-LIO2-ROS2-Docker

<div align="center">
    <h2>Swarm-LIO2-ROS2-Docker: ROS 2 and Docker Port of Swarm-LIO2</h2>
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

**Swarm-LIO2-ROS2-Docker** is an **open-source community-driven port** of [**Swarm-LIO2**](https://github.com/hku-mars/Swarm-LIO2),  
rewritten to support **ROS 2** and **Docker-based deployment**.

The goal is to make Swarm-LIO2 easier to integrate into modern robotics stacks using **ROS 2 Humble** distribution,  
and to simplify reproducible builds through **containerization**.

Swarm-LIO2 is a *decentralized, efficient LiDAR-inertial odometry and mapping framework* designed for UAV swarms,  
developed by the **HKU MARS Lab** and published in *IEEE Transactions on Robotics 2025*.

---

### Project Status

| Component | Status |
|------------|:------:|
| Repository setup | ✅ |
| ROS 2 package structure | 🚧 In progress |
| Core node porting (lio_sam, swarm_lio) | ⏳ Planned |
| ROS 2 launch files | ⏳ Planned |
| Dockerfile (base + runtime images) | 🚧 In progress |
| Simulation / example data testing | ⏳ Planned |

---

### Planned Tech Stack

- **ROS 2 Humble**
- **Ubuntu 22.04**
- **GTSAM** (latest stable)
- **PCL ≥ 1.12**
- **Eigen ≥ 3.4**
- **Livox ROS Driver 2 (ROS 2 branch)**
- **Docker + docker-compose**

---

### Repository Structure (Planned)

```plaintext
Swarm-LIO2-ROS2-Docker/
├── docker/
│ ├── Dockerfile.base
│ ├── Dockerfile.dev
│ └── docker-compose.yaml
├── src/
│ ├── swarm_lio2/
│ ├── udp_bridge/
│ └── common/
├── launch/
│ ├── single_uav.launch.xml
│ └── multi_uav.launch.xml
├── config/
│ └── params.yaml
└── README.md
```
---

### TODO Roadmap

#### Phase 1: Setup & Porting
- [x] Fork original repository  
- [x] Define ROS 2 and Docker roadmap  
- [ ] Create ROS 2 package structure
- [ ] Port core Swarm-LIO2 nodes (C++)  
- [ ] Replace `ros::Publisher/Subscriber` with `rclcpp` equivalents  
- [ ] Update message definitions to ROS 2 `.msg` types  

#### Phase 2: Build System & Docker
- [ ] Create Dockerfiles for build/runtime  
- [ ] Add multi-stage build support (builder → runtime image)  

#### Phase 3: Launch & Testing
- [ ] Write ROS 2 launch XML files  
- [ ] Test with Livox ROS 2 drivers and real/simulated data  
- [ ] Publish example bags and setup guide  

#### Phase 4: Documentation
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
ROS 2 and Docker port maintained by **Roman Voronov**.  
This fork follows the same license as the original project.

---

### Contributing

Contributions are very welcome!  
If you’re interested in:
- helping with ROS 2 migration,
- testing in real UAV environments,
- or improving documentation —

please open an **issue** or **pull request**.

---
