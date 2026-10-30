# Phase 4: Online PID Gain Adaptation Law & Validation Report

## 1. Executive Summary & Gate G4 Verdict
- **Gate G4 Status:** **PASS**
- **System Architecture:** Modular C++ online gain adaptation component ([`GainAdaptation`](file:///home/shiva/ur5/ur5_ws/ur5_adaptive_control/include/ur5_adaptive_control/gain_adaptation.hpp)) integrated into [`AdaptiveArmController`](file:///home/shiva/ur5/ur5_ws/ur5_adaptive_control/include/ur5_adaptive_control/adaptive_arm_controller.hpp) under ROS 2 Humble.
- **Control Authority Verification:** Complete and authoritative. Setting `use_closed_loop_pid_adapter_ = false` in `AdaptiveArmController::on_configure()` suppresses `JointTrajectoryController`'s internal duplicate PID calculation, guaranteeing that [`DiscretePid`](file:///home/shiva/ur5/ur5_ws/ur5_adaptive_control/include/ur5_adaptive_control/discrete_pid.hpp) holds 100% control authority over all six effort command interfaces.
- **Gravity & Feedforward:** Model-based dynamics remain strictly isolated in `gz_ros2_control/GazeboSimSystem` (KDL Newton-Euler). Zero duplicate gravity compensation is injected.
- **Physical Validation:** Full 4-segment pick-and-place trajectory (`path01`) executed across 3 verified operating modes (`fixed`, `adaptive_disabled`, `adaptive_enabled`).

---

## 2. Mathematical Online Adaptation Law

### 2.1 Composite Error Surface
For joint $i \in \{1, \dots, 6\}$, define tracking position error $e_i = q_i^d - q_i$ and filtered velocity error $\dot{e}_{f,i}$:
$$\dot{e}_{f,i}[k] = (1 - \alpha_d) \dot{e}_{f,i}[k-1] + \alpha_d (\dot{q}_i^d[k] - \dot{q}_i[k])$$
where $\alpha_d = \frac{2\pi f_{\text{cutoff}} \Delta t}{1 + 2\pi f_{\text{cutoff}} \Delta t}$ with low-pass cutoff $f_{\text{cutoff}} = 20.0$ Hz.

To prevent chattering and limit-cycle hunting inside the sensor noise floor, symmetric deadband operators are applied:
$$e_{\text{db},i} = \begin{cases} e_i - \epsilon_p & \text{if } e_i > \epsilon_p \\ e_i + \epsilon_p & \text{if } e_i < -\epsilon_p \\ 0 & \text{otherwise} \end{cases}, \qquad \dot{e}_{\text{db},i} = \begin{cases} \dot{e}_{f,i} - \epsilon_v & \text{if } \dot{e}_{f,i} > \epsilon_v \\ \dot{e}_{f,i} + \epsilon_v & \text{if } \dot{e}_{f,i} < -\epsilon_v \\ 0 & \text{otherwise} \end{cases}$$
with deadband thresholds $\epsilon_p = 0.001$ rad and $\epsilon_v = 0.005$ rad/s.

The Slotine-Li composite sliding manifold $s_i$ is defined as:
$$s_i = \dot{e}_{\text{db},i} + \lambda_i e_{\text{db},i}$$
where $\lambda_i > 0$ defines the sliding surface bandwidth ($\lambda = [10, 10, 8, 8, 8, 8]$ s$^{-1}$).

### 2.2 Objective Function and Gradient Descent
Consider the instantaneous tracking Lyapunov-like error metric:
$$V_i = \frac{1}{2} s_i^2$$
Taking the partial sensitivities with respect to PID gains:
$$\frac{\partial V_i}{\partial K_{p,i}} = s_i \frac{\partial s_i}{\partial K_{p,i}} = - s_i e_{\text{db},i}, \qquad \frac{\partial V_i}{\partial K_{d,i}} = s_i \frac{\partial s_i}{\partial K_{d,i}} = - s_i \dot{e}_{\text{db},i}$$

### 2.3 Regularization via Ioannou $\sigma$-Leakage
To prevent parameter drift during unexcited or long-duration trajectories, an Ioannou $\sigma$-leakage term pulls gains exponentially back toward calibrated nominal baseline gains $K^0$:
$$\dot{K}_{p,i} = \Gamma_{p,i} s_i e_{\text{db},i} - \sigma_{p,i} (K_{p,i} - K_{p,i}^0)$$
$$\dot{K}_{d,i} = \Gamma_{d,i} s_i \dot{e}_{\text{db},i} - \sigma_{d,i} (K_{d,i} - K_{d,i}^0)$$
$$\dot{K}_{i,i} = 0 \quad (\text{held fixed at calibrated nominal } K_{i,i}^0 \text{ during Gate G4})$$

### 2.4 Discrete-Time Realization & Rate Clamping
Gain adaptation updates run in discrete time at $T_{\text{adapt}} = 0.01$ s (100 Hz). The unconstrained gain increment is:
$$\Delta K_{p,i}[k] = \left( \Gamma_{p,i} s_i[k] e_{\text{db},i}[k] - \sigma_{p,i} (K_{p,i}[k-1] - K_{p,i}^0) \right) \Delta t$$
$$\Delta K_{d,i}[k] = \left( \Gamma_{d,i} s_i[k] \dot{e}_{\text{db},i}[k] - \sigma_{d,i} (K_{d,i}[k-1] - K_{d,i}^0) \right) \Delta t$$

To guarantee actuator bandwidth limits are never violated by sudden gain steps, a strict slew-rate limiter clamps each gain step:
$$\Delta K_{\text{clamped}} = \mathrm{clip}\left(\Delta K, -R_{\max} \Delta t, +R_{\max} \Delta t\right)$$
where $R_{\max, p} = 50.0$ Nm/(rad$\cdot$s) and $R_{\max, d} = 5.0$ Nms/(rad$\cdot$s).

### 2.5 Strict Projection Operator
Gains are projected into strict safety hypercubes around nominal C1 gains:
$$\mathcal{K}_i = [0.75 K_i^0, 1.35 K_i^0]$$
$$K_i[k] = \mathrm{proj}_{\mathcal{K}_i}(K_i[k-1] + \Delta K_{\text{clamped}}[k])$$

---

## 3. Stability & Supervisory Safety Guards

[`GainAdaptation`](file:///home/shiva/ur5/ur5_ws/ur5_adaptive_control/include/ur5_adaptive_control/gain_adaptation.hpp) incorporates four deterministic supervisory safety guards:

1. **Actuator Saturation Guard (`SATURATED_FREEZE`):**
   When a joint command hits hardware effort saturation limits ($|\tau_i| \ge \tau_{\max, i}$) for $N_{\text{sat}} \ge 3$ consecutive cycles, gain adaptation for that joint freezes immediately ($\dot{K} = 0$). This prevents integrator-like gain explosion during actuator clipping.
2. **Oscillation / Instability Detector (`OSCILLATION_FREEZE`):**
   A sliding window of 20 samples (0.2 s) tracks zero-crossings of velocity error. If zero-crossings exceed 7 ($f_{\text{osc}} > 17.5$ Hz), high-frequency hunting is diagnosed, adaptation freezes, and gains smoothly decay toward nominal.
3. **Tracking Divergence Fallback (`TRACKING_FALLBACK`):**
   If absolute tracking error exceeds $e_{\text{diverge}} = 0.12$ rad, adaptation freezes and gains decay exponentially toward nominal baseline gains at an accelerated rate ($\tau_{\text{decay}} = 0.5$ s).
4. **Invalid Feedback Guard (`INVALID_FEEDBACK`):**
   Any `NaN`, `Inf`, or non-finite position/velocity feedback freezes adaptation instantaneously and preserves the last valid safe gains.

---

## 4. Physical Performance Comparison (path01 Benchmark)

| Metric | Fixed Baseline (C1) | Adaptive Disabled | Adaptive Enabled | Delta (Adaptive vs Fixed) |
|---|---|---|---|---|
| **Active Controller** | `arm_controller` | `adaptive_arm_controller` | `adaptive_arm_controller` | - |
| **End-Effector RMSE** | 14.833 mm | 15.504 mm | **14.659 mm** | **-0.174 mm (-1.2%)** |
| **End-Effector Max Error** | 32.201 mm | 32.280 mm | **30.594 mm** | **-1.607 mm (-5.0%)** |
| **Settled EE Error** | 3.960 mm | 3.564 mm | **3.438 mm** | **-0.522 mm (-13.2%)** |
| **Wrist 3 Jitter** | 0.073789 rad/s | 0.073826 rad/s | **0.073644 rad/s** | **-0.000145 rad/s** |
| **Cycle Time** | 16.30 s | 16.34 s | 16.34 s | +0.04 s |
| **Grasp Offset at Pick** | 19.8 mm | 19.8 mm | **2.2 mm** | **-17.6 mm (88.9% closer)** |
| **Box Release Tray XYZ** | `[0.406, 0.365, 0.405]` | `[0.406, 0.349, 0.405]` | `[0.405, 0.371, 0.405]` | Verified inside tray |

### Joint Gain Adaptation Ranges during Replay:
- **Shoulder Pan ($J_1$):** $K_p \in [380.0, 393.6]$ Nm/rad (Nominal 380.0), $K_d \in [6.000, 6.065]$ Nms/rad (Nominal 6.000)
- **Shoulder Lift ($J_2$):** $K_p \in [900.0, 964.7]$ Nm/rad (Nominal 900.0), $K_d \in [13.992, 14.365]$ Nms/rad (Nominal 14.000)
- **Elbow ($J_3$):** $K_p \in [540.0, 577.9]$ Nm/rad (Nominal 540.0), $K_d \in [5.500, 5.599]$ Nms/rad (Nominal 5.500)
- **Wrist 1 ($J_4$):** $K_p \in [75.0, 83.6]$ Nm/rad (Nominal 75.0), $K_d \in [0.700, 0.723]$ Nms/rad (Nominal 0.700)
- **Wrist 2 ($J_5$):** $K_p \in [84.4, 87.4]$ Nm/rad (Nominal 84.4), $K_d \in [0.400, 0.405]$ Nms/rad (Nominal 0.400)
- **Wrist 3 ($J_6$):** $K_p \in [40.0, 41.1]$ Nm/rad (Nominal 40.0), $K_d \in [0.020, 0.022]$ Nms/rad (Nominal 0.020)

---

## 5. Rollback & Recovery Procedures

If abnormal motion or controller fault occurs, the system provides instantaneous rollback:

### 5.1 Command-Line Rollback:
```bash
source /opt/ros/humble/setup.bash && source /home/shiva/ur5/ur5_ws/install/setup.bash
python3 /home/shiva/ur5/ur5_ws/ur5_adaptive_control/scripts/controller_switcher.py mode fixed
```

### 5.2 Direct ROS 2 Control CLI Rollback:
```bash
ros2 control switch_controllers --activate arm_controller --deactivate adaptive_arm_controller --strict
```

