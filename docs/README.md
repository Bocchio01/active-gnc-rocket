# Big-picture architecture (how the pieces fit)

1. **Requirements & constraints** — target mass, max altitude, center of gravity, burn time, desired manoeuvres (bank, yaw, pitch), recovery method, legal limits.
2. **Sensing** — IMU (accelerometer + gyro), magnetometer, barometer/altimeter, GPS (if needed), optionally cameras/optical flow for low-altitude guidance.
3. **Estimation** — sensor fusion / state estimator (attitude, angular rates, position/velocity if GPS). EKF / complementary filter.
4. **Guidance** — trajectory planner / pitch program / apogee/targeting logic.
5. **Control** — attitude controller (inner loop) + autopilot (outer loop/guidance tracking). PID/LQR/Backstepping/MPC options.
6. **Actuation** — thrust vectoring (gimbaled nozzle or moving motor mount), canards or aerodynamic surfaces, or multiple vectored cold-gas thrusters for test rigs.
7. **Flight computer / real-time stack** — flight controller (Pixhawk/STM32) running autopilot firmware or your custom C++ RT code.
8. **Comm/telemetry** — telemetry radio + ground station (QGroundControl / custom MATLAB GUI).
9. **Safety & recovery** — hardware kill switch, e-match + redundant recovery deployment, geofence/altitude limiters.
10. **Verification & testing** — simulation (MATLAB/Simulink + Simscape), Software-in-the-loop (SITL), Hardware-in-the-loop (HIL), bench tests, tilt table, vibration tests, full static thrust tests, captive-carry flights.

---

# Hardware & software recommendations (practical, proven)

* **Flight controller / autopilot**: Pixhawk family (PX4/Ardupilot) is widely used and documented; good I/O for IMU, GPS, servo outputs, telemetry. Using it lets you leverage PX4 logging/telemetry and QGroundControl. ([docs.px4.io][2])
* **IMU options**: high-quality MEMS IMUs (e.g., ADIS16488, or the IMU on Pixhawk for prototyping). If you need high-dynamics, pick an IMU with high gyro saturation range and high sample rate.
* **Microcontroller option**: STM32 (C/C++) or use the Pixhawk FMU and write PX4 modules or custom firmware (C++). Pixhawk supports MAVLink which is useful for SITL/HIL. ([docs.px4.io][2])
* **Actuators**: digital high-torque servos for nozzle gimbal or control surfaces. For prototyping, gimbal via two servos is simplest. For fast dynamics use brushless gimbal motors with ESC + controller.
* **Propulsion for tests**: start with small certified model rocket motors (A/B/C class) or a cold-gas thruster rig to test control logic without explosive classification issues.
* **Ground tools**: Thrust stand, tilt/table rig, vibration table, test harness for captive-carry/drop tests.
* **Software**: MATLAB + Simulink for modeling and algorithm prototyping. Use Simulink + Embedded Coder to generate C code or translate algorithms to C++ for embedded. Use PX4 tools / SITL for integration if you adopt Pixhawk. ([discuss.px4.io][3])

---

# Algorithms & control stack (concrete)

* **State Estimation**: IMU-driven attitude filter (Mahony / Madgwick / multiplicative EKF). For full state (position/vel) use an EKF (e.g., multisensor EKF fusing IMU+baro+GPS).
* **Guidance**:

  * Simple: pitch-program + PID for attitude tracking (suitable for many small rockets).
  * Advanced: closed-loop guidance (Proportional Navigation for agile steering) or Model Predictive Control (MPC) for constrained maneuvers.
* **Control**:

  * Inner loop: rate damping PID or LQR on angular rates.
  * Outer loop: attitude quaternion PID or quaternion feedback with body-axis pointing objective.
  * Control allocation: map requested torques to servo/gimbal actuators; consider saturation and rate limits.
* **Fault handling**: watchdogs, sensor sanity checks, fallback modes (stability-only) and safe abort logic.
* **Sampling & real-time**: inner loop at ≥ 200–1000 Hz for attitude loop depending on dynamics; outer loop at lower rates.

---

# Project phases, tasks & deliverables

I'll break it into **iterative phases** — each phase ends with specific deliverables and tests.

### Phase 0 — Requirements, safety, legal (1–2 weeks)

* Write clear specs: mass, motor class, max altitude, target maneuvers, payload.
* Check local laws, coordinate with local model rocketry club / aeroclub, apply for permissions (if needed). ([europerocketry.com][1])
* Deliverable: Requirements doc + legal checklist + risk assessment.

### Phase 1 — Simulation & algorithms (2–6 weeks)

* Build dynamics model in MATLAB/Simulink (6-DOF rocket model, thrust profile, aerodynamic coefficients).
* Implement sensor models (IMU noise, biases, baro, GPS) and simulate different scenarios (wind, off-nominal motor).
* Implement and tune state estimator (EKF or complementary + gyro bias estimation).
* Implement guidance + controller in Simulink; run Monte-Carlo sims.
* Deliverable: Simulink model, plots, controller parameters, test report.

### Phase 2 — Ground hardware integration (4–8 weeks)

* Select flight computer (Pixhawk/STM32) and IMU; set up telemetry and logging.
* Implement drivers and middleware (HAL): IMU driver, baro, GPS, servo outputs. If using Pixhawk + PX4, you can write a PX4 module in C++. Otherwise implement a small RT loop on STM32 (FreeRTOS recommended).
* Port estimator & controller from MATLAB to C/C++ (Embedded Coder or manual port).
* Bench-test actuators, calibrate sensors, verify PWM outputs.
* Deliverable: bench-tested embedded software with telemetry and logging.

### Phase 3 — HIL / captive tests (2–6 weeks)

* Hardware-in-the-loop: run autopilot connected to Simulink dynamics in real-time (Simulink external mode / MAVLink) to validate control/latency.
* Tilt table / captive-carry test to verify attitude control and estimation under thrust excitation.
* Thrust-stand tests of motor + actuator response.
* Deliverable: HIL test logs, tuned controller gains.

### Phase 4 — Flight tests (small increments)

* **Low-power tethered / short hops** (with certified small motors). Monitor telemetry, iterate control.
* **Free-flight low altitude** with recovery and safety spotters.
* Gradually increase complexity only after validation.
* Deliverable: flight logs, post-flight analysis, updated models.

---

# Concrete repo structure (suggested)

```
/rocket-gcn
├─ /docs
│  └─ requirements.md, safety.md, test_plan.md
├─ /sim
│  ├─ 6dof_model.m / Simulink model (.slx)
│  └─ datasets/
├─ /matlab
│  └─ estimator/, guidance/, controller/   % prototypes & scripts
├─ /firmware
│  ├─ /stm32             % MCU code (FreeRTOS)
│  └─ /px4_module        % optional PX4 module (C++)
├─ /hw
│  └─ schematics/, bom.csv
├─ /tools
│  └─ hw_test_scripts/   % thrust stand, tilt tests
├─ /log
└─ README.md
```

---

# Example development flow for a single feature (attitude estimator)

1. Prototype Mahony/Madgwick + simple gyro bias estimator in MATLAB. Validate offline on prerecorded IMU logs (from bench).
2. Move to Simulink as a block that accepts raw IMU + mag + baro and outputs quaternion + ang rates.
3. Unit-test in Simulink (SIL): compare outputs vs ground truth model.
4. Generate C code with Embedded Coder **or** port the algorithm by hand to C++ (pay attention to fixed-point vs floating point, stack usage).
5. Run HIL: connect C code on flight computer to the Simulink rocket model.
6. Flight test on tether.

---

# Real-time / embedded considerations

* Use CPU profiling to ensure estimator + controller run within deadlines.
* Prioritize deterministic behavior: use fixed time step RT scheduler (e.g. FreeRTOS or PX4's work queue).
* Carefully manage sensor buses (SPI/I2C) and vibration isolation; filter structural vibrations (notch filters).

---

# Testing rigs & procedures (must do these before flight)

* **Thrust stand** — measure thrust curve, confirm motor/time profile.
* **Tilt table** — test attitude response to commanded pitch/roll.
* **Vibration/shock** — to catch sensor disconnects.
* **HIL** — run SITL with full telemetry & log.
* **Back-to-back sensor tests** — compare multiple IMUs if possible.

---

# Safety & fail-safe rules (must implement)

* Hardware kill switch (manual), and software kill (telemetry + watchdog).
* Redundant sensor sanity checks (if gyro floats to ±inf or bias spikes -> safe mode).
* Recovery deployment redundancy (separate power/battery for ejection charges).
* Minimum number of observers/spotters and fire-extinguishers on launch site.

---

# Timeline (rough, depends on availability)

* Week 0–2: requirements, legal checks, initial Simulink model.
* Week 2–6: estimator + controller design in MATLAB/Simulink.
* Week 6–12: hardware selection, bench integration, porting to embedded.
* Week 12–16: HIL, tilt/captive testing.
* Week 16+: incremental flight tests.

---

# Useful references & community resources

* German/regulatory resources and community discussions about model rocketry laws in Germany. Important to read before any motor-based flight: ([europerocketry.com][1])
* Pixhawk / PX4 hardware docs — good practical option for flight computer and telemetry: ([docs.px4.io][2])
* Community reports of using Pixhawk for rockets (useful for SITL/HIL approaches): ([discuss.px4.io][3])
* NAR / general model rocketry safety laws (good background if you're unfamiliar): ([National Association of Rocketry][4])

---

# Immediate first steps (what I recommend you do next — you can start now)

1. **Write a 1-page requirements doc** (mass, motor class you plan to use, target altitude, payload, recovery, launch site constraints). I can provide a template.
2. **Check local regs** for your municipality/region in Germany and contact a local rocketry club or Verband for launch permissions. (Critical.) ([europerocketry.com][1])
3. **Build a simple Simulink 6-DOF model** (I can produce a starter Simulink/Matlab script that models a slender rocket with a thrust curve and simple aerodynamics). Use MATLAB for estimator/controller prototyping.
4. **Choose flight controller**: buy a Pixhawk (if you want quick integration) or pick an STM32 dev board for full control. I can draft the driver + module layout for either. ([docs.px4.io][2])

---

If you want, I can immediately:

* produce a **requirements template** and **risk checklist**,
* scaffold a **Simulink 6-DOF starter model** (MATLAB code + Simulink block list you can run), or
* create a **firmware module skeleton** (C++ PX4 module or STM32 FreeRTOS project) that implements a basic attitude loop and MAVLink telemetry.

Which of the above do you want me to generate *right now*? (I'll produce the chosen artifact in this reply.)

[1]: https://www.europerocketry.com/index.php?id=regulations_germany&utm_source=chatgpt.com "Regulations Germany - Rocketnews"
[2]: https://docs.px4.io/main/en/flight_controller/pixhawk4?utm_source=chatgpt.com "Holybro Pixhawk 4 | PX4 Guide (main)"
[3]: https://discuss.px4.io/t/using-pixhawk-as-rocket-flight-computer/29748?utm_source=chatgpt.com "Using PixHawk as Rocket Flight Computer - PX4 Autopilot"
[4]: https://www.nar.org/LawsandRegulations?utm_source=chatgpt.com "Laws and Regulations"
