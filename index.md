---
layout: default
---

<div class="hero">
  <div class="hero__text">
    <h1>Arno Laurie</h1>
    <p class="hero__subtitle">SLAM & State Estimation</p>
    <p class="hero__sub2">Robotics MSc<span class="sep">·</span>EPFL<span class="sep">·</span>ERC 2025 Winner</p>
    <div class="hero__badges">
      <a href="mailto:arno.laurie@epfl.ch" class="hero__badge">
        <svg viewBox="0 0 24 24"><path d="M20 4H4c-1.1 0-2 .9-2 2v12c0 1.1.9 2 2 2h16c1.1 0 2-.9 2-2V6c0-1.1-.9-2-2-2zm0 4l-8 5-8-5V6l8 5 8-5v2z"/></svg>
        Email
      </a>
      <a href="https://ch.linkedin.com/in/arno-laurie-816a73229" class="hero__badge">
        <svg viewBox="0 0 24 24"><path d="M19 3a2 2 0 012 2v14a2 2 0 01-2 2H5a2 2 0 01-2-2V5a2 2 0 012-2h14m-.5 15.5v-5.3a3.26 3.26 0 00-3.26-3.26c-.85 0-1.84.52-2.32 1.3v-1.11h-2.79v8.37h2.79v-4.93c0-.77.62-1.4 1.39-1.4a1.4 1.4 0 011.4 1.4v4.93h2.79M6.88 8.56a1.68 1.68 0 001.68-1.68c0-.93-.75-1.69-1.68-1.69a1.69 1.69 0 00-1.69 1.69c0 .93.76 1.68 1.69 1.68m1.39 9.94v-8.37H5.5v8.37h2.77z"/></svg>
        LinkedIn
      </a>
      <a href="https://github.com/Netfluxx" class="hero__badge">
        <svg viewBox="0 0 24 24"><path d="M12 2A10 10 0 002 12c0 4.42 2.87 8.17 6.84 9.5.5.08.66-.23.66-.5v-1.69c-2.77.6-3.36-1.34-3.36-1.34-.46-1.16-1.11-1.47-1.11-1.47-.91-.62.07-.6.07-.6 1 .07 1.53 1.03 1.53 1.03.87 1.52 2.34 1.07 2.91.83.09-.65.35-1.09.63-1.34-2.22-.25-4.55-1.11-4.55-4.92 0-1.11.38-2 1.03-2.71-.1-.25-.45-1.29.1-2.64 0 0 .84-.27 2.75 1.02.79-.22 1.65-.33 2.5-.33.85 0 1.71.11 2.5.33 1.91-1.29 2.75-1.02 2.75-1.02.55 1.35.2 2.39.1 2.64.65.71 1.03 1.6 1.03 2.71 0 3.82-2.34 4.66-4.57 4.91.36.31.69.92.69 1.85V21c0 .27.16.59.67.5C19.14 20.16 22 16.42 22 12A10 10 0 0012 2z"/></svg>
        GitHub
      </a>
    </div>
  </div>
  <div class="hero__image">
    <img src="picture_of_me_xplore.jpg" alt="Arno Laurie">
  </div>
</div>

---

<div id="about" class="section reveal" markdown="1">

## About Me

<p class="about-text">
I deployed the full autonomous navigation stack for EPFL Xplore's Mars rover solo — a LiDAR-inertial SLAM pipeline, 9-axis IMU integration, wheel odometry, and a custom EKF, all running reliably in GPS-denied outdoor terrain. The rover won the <strong>European Rover Challenge 2025</strong>.
</p>

<p class="about-text">
I am now building a custom 3D SLAM system from scratch in C++ using <strong>GTSAM factor-graph optimization</strong> — LiDAR-inertial fusion, place recognition, and loop closure. I have also tuned production-grade systems (LIO-SAM, FAST-LIO2, GLIM) on real hardware, implemented EKFs and UKFs across multiple platforms, and built a custom 6-layer AHRS PCB with full firmware running the <strong>VqF</strong> attitude filter.
</p>

<p class="about-text">
My focus: <strong>robust localization and state estimation for robots operating in GPS-denied, unstructured environments</strong>.
</p>

</div>

---

<div id="projects" class="section reveal" markdown="1">

## Projects

<span class="section-label">Featured — SLAM & State Estimation</span>

<div class="project-card project-card--featured reveal" markdown="1">

<span class="project-tag">&#9733; ERC 2025 Winner</span>

### LiDAR-Inertial SLAM — Mars Rover Navigation Stack
*EPFL Xplore | Team Leader Autonomous Navigation | Sep 2024 — Aug 2025*

<img src="ERC_win.jpg" alt="ERC 2025 Winning Rover">

Deployed the **full autonomous navigation stack solo** for a 4-wheeled Mars rover competing in the European Rover Challenge 2025 — GPS-denied outdoor terrain, no fallback.

**SLAM & Localization:**
- LiDAR-inertial SLAM pipeline (Ouster 3D LiDAR + 9-axis IMU)
- Custom Extended Kalman Filter fusing wheel odometry, IMU, and LiDAR-inertial odometry
- Global pose corrections via triangulation and trilateration with convex optimization (CVXPY/ECOS)
- Sub-15 cm accuracy in GPS-denied outdoor environments
- Production SLAM systems tuned on hardware: **LIO-SAM**, **FAST-LIO2**, **GLIM**

**Planning & Control:**
- Nav2 stack with Hybrid A\* global planner
- Pure Pursuit path tracking with double-Ackermann kinematics
- Dynamic obstacle avoidance

<img src="lidar_slam.jpg" alt="LiDAR SLAM Output">

**Technologies:**
`C++` `Python` `ROS2` `OpenCV` `Docker` `Gazebo` `Arduino` `maxon EPOS`

[View Detailed Showcase →](#project-rover-winner)

</div>

<div class="project-card project-card--featured reveal" markdown="1">

<span class="project-tag">&#9733; Ongoing</span>

### Custom 3D SLAM System — GTSAM Factor Graphs
*Personal Project | Fall 2025 — Ongoing*

Building a complete LiDAR SLAM system from scratch in C++ using **GTSAM factor-graph optimization** as the back-end. No black-box libraries — every component designed and implemented from the ground up.

**Architecture:**
- **Front-end:** LiDAR scan-to-map matching for incremental odometry; IMU preintegration for inter-frame constraint generation
- **Back-end:** GTSAM incremental smoothing (iSAM2) for real-time factor graph optimization
- **Place Recognition:** descriptor-based loop closure detection
- **Back-end trigger:** pose-graph optimization on detected loop closures

**Benchmarked against production systems** (LIO-SAM, FAST-LIO2, GLIM) on the same hardware and datasets.

**Technologies:**
`C++` `GTSAM` `ROS2` `PCL` `Eigen` `LiDAR`

</div>

<div class="project-card project-card--featured reveal" markdown="1">

<span class="project-tag">&#9733; Hardware + Firmware</span>

### Custom 6-Layer AHRS PCB & Drone Visual-Inertial Odometry
*Personal Project | Fall 2025 — Ongoing*

**AHRS Hardware:**
Designed and built a custom **6-layer PCB** in KiCad implementing a full AHRS (Attitude and Heading Reference System). Firmware runs the **VqF (Versatile Quaternion-based Filter)** algorithm — a state-of-the-art sensor fusion filter for robust attitude estimation from 9-axis IMU data (accelerometer + gyroscope + magnetometer).

**Visual-Inertial Odometry:**
The AHRS PCB serves as the IMU unit for an **FPV drone VIO** pipeline — camera + IMU tight coupling for GPS-denied state estimation. Implements EKF-based fusion of visual feature tracks and inertial measurements.

**Technologies:**
`KiCad` `C` `STM32` `VqF` `OpenCV` `EKF/UKF` `ROS2`

</div>

<span class="section-label" style="margin-top: 2rem; display: block;">Other Projects</span>

<div class="project-card reveal" markdown="1">

### EPFL Xplore — Current Rover (Software Systems Engineer)
*Sep 2025 — Ongoing*

Leading end-to-end software architecture for the 2025/26 competition rover: autonomous navigation (ROS2/Nav2), robotic arm control (MoveIt), real-time wireless communication, and sensor fusion (IMU, LiDAR, cameras). Coordinating hardware-software integration across a multidisciplinary team on NVIDIA Jetson platforms.

**Technologies:**
`C++` `Python` `ROS2` `MoveIt` `Docker` `maxon EPOS`

</div>


<div class="project-card reveal" markdown="1">

### STM32 RTOS Autonomous Mobile Robot
*Academic Project | Spring 2025*

Real-time autonomous navigation on the e-puck 2 platform using ChibiOS RTOS. Extended Kalman Filter for localization, real-time obstacle detection and mapping, efficient RTOS task scheduling, sensor fusion with IMU and proximity sensors.

**Technologies:**
`STM32` `ChibiOS` `C` `EKF` `Embedded Systems`

</div>

<div class="project-card reveal" markdown="1">

### Thymio Autonomous Mobile Robot
*Academic Project | Fall 2025*

Autonomous navigation with EKF localization and ArUco tag triangulation & trilateration using **convex optimization** (CVXPY/ECOS). Global path planning and real-time obstacle detection.

**Technologies:**
`Python` `EKF` `OpenCV` `CVXPY`

</div>

<div class="project-card reveal" markdown="1">

### Solar Tracking Solar Oven
*Personal Project | Fall 2025 — Ongoing*

2-DoF sun-tracking system with custom DC-DC Buck converter PCB, ESP32 on FreeRTOS, lux sensor array, PID motor control, and mechanical design in Fusion 360.

**Technologies:**
`ESP32` `FreeRTOS` `KiCad` `Fusion 360` `PID`

</div>

<div class="project-card reveal" markdown="1">

### Direction of Arrival — LibreSDR
*Personal Project | Fall 2025*

MUSIC and Root-MUSIC algorithm implementations for direction-of-arrival estimation on the LibreSDR platform.

<img src="libresdr.jpg" alt="LibreSDR DoA" style="max-width: 360px;">

**Technologies:**
`Python` `SDR` `Signal Processing`

</div>

</div>

---

<div id="skills" class="section reveal" markdown="1">

## Technical Skills

<div class="skills-section">

<div class="skills-grid">
<div>

### SLAM & Localization

<div class="skill-item">
  <div class="skill-header"><span class="skill-name">LiDAR-Inertial SLAM</span><span class="skill-level">Advanced</span></div>
  <div class="skill-bar"><div class="skill-progress" style="width: 90%;"></div></div>
</div>

<div class="skill-item">
  <div class="skill-header"><span class="skill-name">GTSAM / Factor Graphs</span><span class="skill-level">Advanced</span></div>
  <div class="skill-bar"><div class="skill-progress" style="width: 85%;"></div></div>
</div>

<div class="skill-item">
  <div class="skill-header"><span class="skill-name">LIO-SAM · FAST-LIO2 · GLIM</span><span class="skill-level">Intermediate</span></div>
  <div class="skill-bar"><div class="skill-progress" style="width: 72%;"></div></div>
</div>

<div class="skill-item">
  <div class="skill-header"><span class="skill-name">Visual-Inertial Odometry</span><span class="skill-level">Intermediate</span></div>
  <div class="skill-bar"><div class="skill-progress" style="width: 65%;"></div></div>
</div>

### State Estimation

<div class="skill-item">
  <div class="skill-header"><span class="skill-name">Extended Kalman Filter</span><span class="skill-level">Advanced</span></div>
  <div class="skill-bar"><div class="skill-progress" style="width: 92%;"></div></div>
</div>

<div class="skill-item">
  <div class="skill-header"><span class="skill-name">Unscented Kalman Filter</span><span class="skill-level">Advanced</span></div>
  <div class="skill-bar"><div class="skill-progress" style="width: 78%;"></div></div>
</div>

<div class="skill-item">
  <div class="skill-header"><span class="skill-name">IMU / LiDAR / Camera Fusion</span><span class="skill-level">Advanced</span></div>
  <div class="skill-bar"><div class="skill-progress" style="width: 90%;"></div></div>
</div>

<div class="skill-item">
  <div class="skill-header"><span class="skill-name">AHRS / Attitude Estimation</span><span class="skill-level">Advanced</span></div>
  <div class="skill-bar"><div class="skill-progress" style="width: 85%;"></div></div>
</div>

</div>
<div>

### Programming

<div class="skill-item">
  <div class="skill-header"><span class="skill-name">C++</span><span class="skill-level">Advanced</span></div>
  <div class="skill-bar"><div class="skill-progress" style="width: 90%;"></div></div>
</div>

<div class="skill-item">
  <div class="skill-header"><span class="skill-name">Python</span><span class="skill-level">Advanced</span></div>
  <div class="skill-bar"><div class="skill-progress" style="width: 88%;"></div></div>
</div>

<div class="skill-item">
  <div class="skill-header"><span class="skill-name">MATLAB / Simulink</span><span class="skill-level">Intermediate</span></div>
  <div class="skill-bar"><div class="skill-progress" style="width: 68%;"></div></div>
</div>

### Robotics Frameworks

<div class="skill-item">
  <div class="skill-header"><span class="skill-name">ROS2</span><span class="skill-level">Advanced</span></div>
  <div class="skill-bar"><div class="skill-progress" style="width: 88%;"></div></div>
</div>

<div class="skill-item">
  <div class="skill-header"><span class="skill-name">Nav2</span><span class="skill-level">Intermediate</span></div>
  <div class="skill-bar"><div class="skill-progress" style="width: 75%;"></div></div>
</div>

<div class="skill-item">
  <div class="skill-header"><span class="skill-name">OpenCV</span><span class="skill-level">Intermediate</span></div>
  <div class="skill-bar"><div class="skill-progress" style="width: 75%;"></div></div>
</div>

<div class="skill-item">
  <div class="skill-header"><span class="skill-name">MoveIt</span><span class="skill-level">Intermediate</span></div>
  <div class="skill-bar"><div class="skill-progress" style="width: 62%;"></div></div>
</div>

### Embedded & Hardware

<div class="skill-item">
  <div class="skill-header"><span class="skill-name">Arduino / NVIDIA Jetson</span><span class="skill-level">Advanced</span></div>
  <div class="skill-bar"><div class="skill-progress" style="width: 85%;"></div></div>
</div>

<div class="skill-item">
  <div class="skill-header"><span class="skill-name">STM32 / ESP32</span><span class="skill-level">Intermediate</span></div>
  <div class="skill-bar"><div class="skill-progress" style="width: 68%;"></div></div>
</div>

<div class="skill-item">
  <div class="skill-header"><span class="skill-name">KiCad — PCB Design</span><span class="skill-level">Intermediate</span></div>
  <div class="skill-bar"><div class="skill-progress" style="width: 70%;"></div></div>
</div>

<div class="skill-item">
  <div class="skill-header"><span class="skill-name">FreeRTOS / ChibiOS</span><span class="skill-level">Intermediate</span></div>
  <div class="skill-bar"><div class="skill-progress" style="width: 70%;"></div></div>
</div>

</div>
</div>

</div>

</div>

---

<div id="experience" class="section reveal" markdown="1">

## Experience

<div class="exp-item reveal" markdown="1">

### EPFL Xplore — Software Systems Engineer
<div class="exp-meta">Sep 2025 — Ongoing <span class="sep">·</span> <span class="loc">Lausanne, Switzerland</span></div>

Leading end-to-end software architecture of the 2025/26 competition rover: autonomous navigation (ROS2/Nav2), robotic arm control (MoveIt), and real-time wireless communication. Coordinating perception, planning, and control integration across a multidisciplinary team on Jetson platforms. Containerized deployment with Docker; sensor fusion across IMU, LiDAR, and cameras.

`C++` `Python` `ROS2` `Docker` `Jetson` `maxon EPOS`

</div>

<div class="exp-item reveal" markdown="1">

### EPFL Xplore — Team Leader, Autonomous Navigation
<div class="exp-meta">Sep 2024 — Aug 2025 <span class="sep">·</span> <span class="loc">Lausanne, Switzerland</span></div>

Led the development of the full navigation subsystem for the ERC 2025 rover. Solo deployment of the complete SLAM and navigation stack — LiDAR-inertial odometry, 9-axis IMU integration, wheel odometry, and custom EKF, running in GPS-denied outdoor terrain. **Won 1st place at the European Rover Challenge 2025.**

`C++` `Python` `ROS2` `OpenCV` `Docker` `Gazebo` `Arduino`

</div>

<div class="exp-item reveal" markdown="1">

### EPFL Xplore — Software Engineer
<div class="exp-meta">Sep 2023 — Sep 2024 <span class="sep">·</span> <span class="loc">Lausanne, Switzerland</span></div>

Designed and implemented ROS2-based manual and autonomous navigation for an outdoor rover. 2D LiDAR SLAM integration, low-level PID motor controller on Arduino with custom wheel odometry. Collaborated with mechanical and electrical engineers on system integration.

`C++` `Python` `ROS2` `Arduino`

</div>

<div class="exp-item reveal" markdown="1">

### ETML — Machining Intern
<div class="exp-meta">August 2024 <span class="sep">·</span> <span class="loc">Lausanne, Switzerland</span></div>

Hands-on manual metal machining: turning, milling, drilling, sawing, tapping, and brazing.

</div>

</div>

---

<div id="education" class="section reveal" markdown="1">

## Education

<div class="education-item" markdown="1">

### EPFL — Master of Science in Robotics
*2025 — Ongoing*

**Relevant Coursework:**
- Manipulation & Computer Vision
- Autonomous Navigation
- Sensor Fusion and State Estimation
- Machine Learning & Convex Optimization
- Multivariable and Non-Linear Control, Model Predictive Control

</div>

<div class="education-item" markdown="1">

### EPFL — Bachelor of Science in Microengineering
*GPA: **5.38 / 6** | 2022 — 2025*

**Relevant Coursework:**
- Electronics I & II, OOP, Digital System Design
- AVR Microcontrollers & Embedded Systems
- Control Systems, Signals and Systems
- Real and Complex Analysis, Linear Algebra
- Introduction to PCB Design & Manufacturing

</div>

<div class="education-item" markdown="1">

### École Européenne Luxembourg II
*Baccalauréat Scientifique | **95.02 / 100** | 2022*
- Secretary of the BAC Committee · Yearbook Committee

</div>

</div>

---

<div class="section reveal" markdown="1">

## Awards

<div class="award-item">
  <div class="award-icon">🏆</div>
  <div>
    <h3>European Rover Challenge 2025 — 1st Place</h3>
    <p>Led the autonomous navigation subsystem (SLAM, localization, path planning) that earned EPFL Xplore first place in the international Mars rover competition.</p>
  </div>
</div>

<div class="award-item">
  <div class="award-icon">🥈</div>
  <div>
    <h3>Luxembourg Informatics Olympiad — Semi-Finalist</h3>
    <p>Applied optimization and path-finding algorithms in competitive programming.</p>
  </div>
</div>

</div>

---

<div class="section reveal" markdown="1">

## Languages

<div class="lang-grid">
  <div class="lang-item">🇫🇷 <strong>French</strong><br><span style="color: var(--text-muted); font-size: 0.85rem;">Native</span></div>
  <div class="lang-item">🇬🇧 <strong>English</strong><br><span style="color: var(--text-muted); font-size: 0.85rem;">Fluent — TOEFL 112/120</span></div>
  <div class="lang-item">🇩🇪 <strong>German</strong><br><span style="color: var(--text-muted); font-size: 0.85rem;">Basic</span></div>
  <div class="lang-item">🇳🇱 <strong>Dutch</strong><br><span style="color: var(--text-muted); font-size: 0.85rem;">Basic</span></div>
</div>

</div>

---

<div class="section reveal" markdown="1">

## Interests

<div class="hobby-grid">
  <span class="hobby-item">🧗 Rock Climbing</span>
  <span class="hobby-item">🚁 FPV Drones</span>
  <span class="hobby-item">🏂 Snowboarding</span>
  <span class="hobby-item">🎸 Electric Guitar</span>
  <span class="hobby-item">⛵ Sailing</span>
</div>

</div>

---

<div class="section reveal" markdown="1">

## Detailed Project Showcases

<div class="showcase" id="project-rover-winner" markdown="1">

### ERC 2025 — LiDAR-Inertial SLAM Navigation Stack

#### Problem
Design and deploy a complete autonomous navigation system for a Mars rover operating in GPS-denied, rough outdoor terrain — with no fallback localization source.

<img src="nav2_irl.jpg" alt="EPFL Xplore Rover Navigation in the Field">

#### Localization & State Estimation

**Sensor suite:** Ouster 3D LiDAR + 9-axis IMU (accelerometer, gyroscope, magnetometer) + wheel encoders

- **LiDAR-inertial odometry** as the primary odometry source — high-frequency, drift-bounded
- **Custom EKF** fusing wheel odometry, IMU, and LiDAR-inertial poses; adaptive noise covariance tuning for outdoor terrain
- **Double-Ackermann kinematics** model for accurate motion prediction during tight turns
- **Triangulation + trilateration** for absolute pose correction using visual landmarks — solved as a Second-Order Cone Program (SOCP) with CVXPY + ECOS

<img src="lidar_slam.jpg" alt="LiDAR SLAM Occupancy Map">

**Production SLAM systems tuned on hardware:** LIO-SAM, FAST-LIO2, GLIM — used as references and for benchmarking the custom EKF stack.

#### Perception & Planning

- 3D Ouster LiDAR for obstacle detection and costmap generation
- Camera-based landmark detection (OpenCV)
- Nav2 Hybrid A\* global planner + Pure Pursuit path tracking
- Dynamic obstacle avoidance with local planner

<img src="nav_waypoints.jpg" alt="Waypoint Navigation and Obstacle Detection">

#### Results

| Metric | Result |
|--------|--------|
| Competition | European Rover Challenge 2025 — **1st Place** |
| Localization accuracy | Sub 15 cm in GPS-denied outdoor terrain |
| Deployment | Solo, full stack on NVIDIA Jetson in Docker |

<img src="convex_landmark.jpg" alt="Triangulation+Trilateration — SOCP Formulation" style="max-width: 700px;">

*Landmark-based global pose correction: solved as a Second-Order Cone Program (SOCP) using CVXPY and the ECOS solver.*

</div>


</div>

---

<div id="contact" class="section contact-section reveal" markdown="1">

## Contact

Open to internship opportunities, research collaborations, and robotics projects — particularly in SLAM, state estimation, and autonomous systems.

📧 **Email:** [arno.laurie@epfl.ch](mailto:arno.laurie@epfl.ch)

🔗 **LinkedIn:** [linkedin.com/in/arno-laurie](https://ch.linkedin.com/in/arno-laurie-816a73229)

💻 **GitHub:** [github.com/Netfluxx](https://github.com/Netfluxx)

📍 **Location:** Lausanne, Switzerland

</div>

---

*Last updated: June 2026*
