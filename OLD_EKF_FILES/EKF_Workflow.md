# Extended Kalman Filter (EKF): Step-by-Step Practical Workflow & Data Transformations

This document provides a clear, practical guide to how data flows through an Extended Kalman Filter (EKF) step-by-step. It breaks down the exact inputs, mathematical transformations, matrix dimensions, and outputs at every stage of execution.

---

## 1. High-Level Workflow Overview

The Extended Kalman Filter operates in a continuous **Predict-Update Loop** at every discrete time step $k$ (e.g., every 10 milliseconds in a real-time robot controller).

```
                      +-----------------------------------+
                      |      INITIALIZATION (Step k=0)    |
                      |  Initial State Estimate:  x_0     |
                      |  Initial Uncertainty:     P_0     |
                      +-----------------+-----------------+
                                        |
                                        v
+-----------------------------------------------------------------------------------+
| RECURSIVE ITERATION LOOP (Step k = 1, 2, 3, ...)                                  |
|                                                                                   |
|  INPUTS FOR STEP k:                                                               |
|    • Control Command u_k  (e.g., wheel speed, throttle)                           |
|    • Sensor Data z_k      (e.g., GPS, Radar reading)                              |
|                                                                                   |
|  +─────────────────────────────────────────────────────────────────────────────+  |
|  │ STEP 1: PREDICT (TIME UPDATE)                                               │  |
|  │  1. Run physics model:      x_{k|k-1} = f(x_{k-1|k-1}, u_k)                 │  |
|  │  2. Calculate Jacobian:     F_k = df/dx | x_{k-1|k-1}                        │  |
|  │  3. Grow uncertainty:       P_{k|k-1} = F_k P_{k-1|k-1} F_k^T + Q_k          │  |
|  +─────────────────────────────────────┬───────────────────────────────────────+  |
|                                        │                                          |
|                                        v                                          |
|  +─────────────────────────────────────┴───────────────────────────────────────+  |
|  │ STEP 2: UPDATE (MEASUREMENT CORRECTION)                                     │  |
|  │  1. Calculate Jacobian:     H_k = dh/dx | x_{k|k-1}                        │  |
|  │  2. Compute Innovation:     y_k = z_k - h(x_{k|k-1})                        │  |
|  │  3. Compute Sensor Noise:   S_k = H_k P_{k|k-1} H_k^T + R_k                  │  |
|  │  4. Compute Kalman Gain:    K_k = P_{k|k-1} H_k^T S_k^{-1}                  │  |
|  │  5. Correct State Estimate: x_{k|k} = x_{k|k-1} + K_k y_k                    │  |
|  │  6. Shrink Uncertainty:     P_{k|k} = (I - K_k H_k) P_{k|k-1}              │  |
|  +─────────────────────────────────────┬───────────────────────────────────────+  |
|                                        │                                          |
|  OUTPUT FOR STEP k:                    │                                          |
|    • Updated State Estimate x_{k|k}    │                                          |
|    • Updated Uncertainty P_{k|k}       │                                          |
|                                        v                                          |
|  (Pass x_{k|k} and P_{k|k} forward as input to Step k+1)                          |
+-----------------------------------------------------------------------------------+
```

---

## 2. Realistic Sample Scenario: 2D Autonomous Robot Tracking

To make the data structures and numbers concrete, we use a 2D mobile robot example throughout this guide.

### System Setup
* **State Vector $\mathbf{x}$ ($n=4$ variables):**
  $$\mathbf{x} = \begin{bmatrix} x \\ y \\ v \\ \theta \end{bmatrix} \begin{matrix} \text{Position along X axis (meters)} \\ \text{Position along Y axis (meters)} \\ \text{Forward speed (meters/sec)} \\ \text{Heading angle (radians)} \end{matrix}$$

* **Control Vector $\mathbf{u}$ ($m=2$ inputs):**
  $$\mathbf{u}_k = \begin{bmatrix} a_k \\ \omega_k \end{bmatrix} \begin{matrix} \text{Forward acceleration command } (m/s^2) \\ \text{Steering rate command } (rad/s) \end{matrix}$$

* **Measurement Vector $\mathbf{z}$ ($p=2$ sensor readings):**
  $$\mathbf{z}_k = \begin{bmatrix} r_k \\ \phi_k \end{bmatrix} \begin{matrix} \text{Radar distance measurement (meters)} \\ \text{Radar bearing angle measurement (radians)} \end{matrix}$$

---

## 3. Step-by-Step Data Flow & Transformations

---

### Step 0: Filter Initialization

Before receiving any motion or sensor data, the filter must be initialized with a starting point and starting confidence.

#### 1. Sample Inputs
* **Initial State Estimate $\hat{\mathbf{x}}_{0|0}$ ($4 \times 1$ column vector):**
  $$\hat{\mathbf{x}}_{0|0} = \begin{bmatrix} 0.0 \\ 0.0 \\ 0.0 \\ 0.0 \end{bmatrix}$$
  *(We assume the robot starts at origin $(0,0)$ at rest facing East).*

* **Initial Covariance Matrix $\mathbf{P}_{0|0}$ ($4 \times 4$ matrix):**
  $$\mathbf{P}_{0|0} = \begin{bmatrix}
  0.1 & 0 & 0 & 0 \\
  0 & 0.1 & 0 & 0 \\
  0 & 0 & 0.01 & 0 \\
  0 & 0 & 0 & 0.01
  \end{bmatrix}$$
  *(Diagonal elements represent our initial variance/uncertainty. Small values mean high initial confidence).*

* **Process Noise Covariance $\mathbf{Q}_k$ ($4 \times 4$ matrix):**
  Represents physics model unmodeled noise (e.g., wheel slip):
  $$\mathbf{Q}_k = \begin{bmatrix}
  0.01 & 0 & 0 & 0 \\
  0 & 0.01 & 0 & 0 \\
  0 & 0 & 0.05 & 0 \\
  0 & 0 & 0 & 0.02
  \end{bmatrix}$$

* **Measurement Noise Covariance $\mathbf{R}_k$ ($2 \times 2$ matrix):**
  Represents radar sensor noise variances (distance variance $\sigma_r^2$, angle variance $\sigma_\phi^2$):
  $$\mathbf{R}_k = \begin{bmatrix}
  0.09 & 0 \\
  0 & 0.0025
  \end{bmatrix}$$

---

### Step 1: Predict Phase (Time Update)

**Goal:** Use physics equations to predict where the robot moved over time interval $\Delta t = 0.1 \text{ seconds}$.

#### 1. Inputs for Predict Step
* Previous updated state $\hat{\mathbf{x}}_{k-1|k-1} = \begin{bmatrix} 0.0 \\ 0.0 \\ 0.0 \\ 0.0 \end{bmatrix}$
* Previous updated covariance $\mathbf{P}_{k-1|k-1}$
* Control input $\mathbf{u}_k = \begin{bmatrix} a_k \\ \omega_k \end{bmatrix} = \begin{bmatrix} 1.0 \\ 0.1 \end{bmatrix}$ *(accelerate at $1 m/s^2$, turn at $0.1 rad/s$)*

#### 2. Transformations & Operations

##### A. Non-Linear Physics Projection
Push previous state through non-linear motion function $f(\mathbf{x}_{k-1}, \mathbf{u}_k)$:

$$f(\mathbf{x}_{k-1}, \mathbf{u}_k) = \begin{bmatrix}
x_{k-1} + v_{k-1} \cos(\theta_{k-1}) \Delta t \\
y_{k-1} + v_{k-1} \sin(\theta_{k-1}) \Delta t \\
v_{k-1} + a_k \Delta t \\
\theta_{k-1} + \omega_k \Delta t
\end{bmatrix}$$

Plug in numbers ($\Delta t = 0.1$):

$$\hat{\mathbf{x}}_{k|k-1} = \begin{bmatrix}
0.0 + (0.0)\cos(0.0)(0.1) \\
0.0 + (0.0)\sin(0.0)(0.1) \\
0.0 + (1.0)(0.1) \\
0.0 + (0.1)(0.1)
\end{bmatrix} = \begin{bmatrix} 0.0 \\ 0.0 \\ 0.1 \\ 0.01 \end{bmatrix}$$

##### B. Compute Process Jacobian Matrix $\mathbf{F}_k$ ($4 \times 4$)
Take partial derivatives of motion function $f$ w.r.t state vector $\mathbf{x} = \begin{bmatrix} x & y & v & \theta \end{bmatrix}^T$:

$$\mathbf{F}_k = \frac{\partial f}{\partial \mathbf{x}} = \begin{bmatrix}
1 & 0 & \cos(\theta)\Delta t & -v\sin(\theta)\Delta t \\
0 & 1 & \sin(\theta)\Delta t & v\cos(\theta)\Delta t \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}_{\hat{\mathbf{x}}_{k-1|k-1}}$$

Evaluating at $\hat{\mathbf{x}}_{k-1|k-1}$ ($\theta = 0, v = 0$):

$$\mathbf{F}_k = \begin{bmatrix}
1 & 0 & 0.1 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}$$

##### C. Uncertainty Propagation
Calculate prior covariance $\mathbf{P}_{k|k-1}$ using matrix multiplication:

$$\mathbf{P}_{k|k-1} = \mathbf{F}_k \mathbf{P}_{k-1|k-1} \mathbf{F}_k^T + \mathbf{Q}_k$$

#### 3. Outputs of Predict Step
* **Predicted State Vector $\hat{\mathbf{x}}_{k|k-1}$ ($4 \times 1$):** $\begin{bmatrix} 0.0 & 0.0 & 0.1 & 0.01 \end{bmatrix}^T$
* **Predicted Covariance Matrix $\mathbf{P}_{k|k-1}$ ($4 \times 4$):** Increased uncertainty due to process noise $\mathbf{Q}_k$.

---

### Step 2: Update Phase (Measurement Correction)

**Goal:** A sensor measurement arrives. Compare sensor reading $\mathbf{z}_k$ with predicted measurement $h(\hat{\mathbf{x}}_{k|k-1})$ to correct state estimate and shrink uncertainty.

#### 1. Inputs for Update Step
* Predicted state vector $\hat{\mathbf{x}}_{k|k-1} = \begin{bmatrix} 0.0 & 0.0 & 0.1 & 0.01 \end{bmatrix}^T$
* Predicted covariance matrix $\mathbf{P}_{k|k-1}$
* Actual sensor measurement reading from radar $\mathbf{z}_k = \begin{bmatrix} r_{raw} \\ \phi_{raw} \end{bmatrix} = \begin{bmatrix} 0.05 \\ 0.02 \end{bmatrix}$

#### 2. Transformations & Operations

##### A. Non-Linear Measurement Prediction
Predict what sensor *should* see based on our state estimate $\hat{\mathbf{x}}_{k|k-1}$ using observation model $h(\mathbf{x})$:

$$h(\hat{\mathbf{x}}_{k|k-1}) = \begin{bmatrix} \sqrt{x^2 + y^2} \\ \arctan(y/x) \end{bmatrix} = \begin{bmatrix} \sqrt{0^2 + 0^2} \\ \arctan(0/0) \end{bmatrix} = \begin{bmatrix} 0.0 \\ 0.0 \end{bmatrix}$$

##### B. Compute Innovation Residual Vector $\mathbf{y}_k$ ($2 \times 1$)
Subtract predicted measurement from real sensor reading:

$$\mathbf{y}_k = \mathbf{z}_k - h(\hat{\mathbf{x}}_{k|k-1}) = \begin{bmatrix} 0.05 \\ 0.02 \end{bmatrix} - \begin{bmatrix} 0.0 \\ 0.0 \end{bmatrix} = \begin{bmatrix} 0.05 \\ 0.02 \end{bmatrix}$$

*(The positive innovation means the sensor saw the robot slightly further ahead than physics predicted).*

##### C. Compute Measurement Jacobian Matrix $\mathbf{H}_k$ ($2 \times 4$)
Take partial derivatives of $h(\mathbf{x})$ w.r.t state vector $\mathbf{x}$:

$$\mathbf{H}_k = \frac{\partial h}{\partial \mathbf{x}} = \begin{bmatrix}
\frac{x}{\sqrt{x^2+y^2}} & \frac{y}{\sqrt{x^2+y^2}} & 0 & 0 \\[6pt]
\frac{-y}{x^2+y^2} & \frac{x}{x^2+y^2} & 0 & 0
\end{bmatrix}_{\hat{\mathbf{x}}_{k|k-1}}$$

##### D. Compute Innovation Covariance Matrix $\mathbf{S}_k$ ($2 \times 2$)
Calculate total measurement space uncertainty:

$$\mathbf{S}_k = \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^T + \mathbf{R}_k$$

##### E. Compute Optimal Kalman Gain Matrix $\mathbf{K}_k$ ($4 \times 2$)
Determine how much to trust the sensor vs. physics prediction:

$$\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{S}_k^{-1}$$

* If sensor noise $\mathbf{R}_k$ is very small $\implies \mathbf{K}_k$ is large $\implies$ Trust sensor.
* If process noise $\mathbf{Q}_k$ is very small $\implies \mathbf{K}_k$ is small $\implies$ Trust physics prediction.

##### F. Correct State Estimate $\hat{\mathbf{x}}_{k|k}$ ($4 \times 1$)
Multiply Kalman gain by innovation residual and add to prediction:

$$\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \mathbf{y}_k$$

##### G. Update & Shrink Covariance Matrix $\mathbf{P}_{k|k}$ ($4 \times 4$)
Reduce estimation uncertainty because new information arrived:

$$\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_{k|k-1}$$

#### 3. Outputs of Update Step
* **Final Corrected State Vector $\hat{\mathbf{x}}_{k|k}$ ($4 \times 1$):** Optimal estimate of position, speed, and heading.
* **Final Corrected Covariance Matrix $\mathbf{P}_{k|k}$ ($4 \times 4$):** Updated smaller uncertainty matrix.

---

## 4. Input vs. Output Summary Table Across Filter Stages

| Filter Stage | Input Data | Input Matrix Dimensions | Applied Operation / Function | Output Data | Output Matrix Dimensions |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Initialization** | User assumptions | $\mathbf{x}_0 \in \mathbb{R}^{4 \times 1}$, $\mathbf{P}_0 \in \mathbb{R}^{4 \times 4}$ | Set initial state & covariance | $\hat{\mathbf{x}}_{0\|0}$, $\mathbf{P}_{0\|0}$ | $4 \times 1$, $4 \times 4$ |
| **Predict State** | $\hat{\mathbf{x}}_{k-1\|k-1}$, $\mathbf{u}_k$ | $4 \times 1$, $2 \times 1$ | Non-linear physics $f(\mathbf{x}, \mathbf{u})$ | $\hat{\mathbf{x}}_{k\|k-1}$ | $4 \times 1$ |
| **Predict Covariance**| $\mathbf{P}_{k-1\|k-1}$, $\mathbf{Q}_k$, $\mathbf{F}_k$ | $4 \times 4$, $4 \times 4$, $4 \times 4$ | Linearized propagation $\mathbf{F}\mathbf{P}\mathbf{F}^T + \mathbf{Q}$ | $\mathbf{P}_{k\|k-1}$ | $4 \times 4$ |
| **Predict Sensor** | $\hat{\mathbf{x}}_{k\|k-1}$ | $4 \times 1$ | Non-linear observation $h(\mathbf{x})$ | $h(\hat{\mathbf{x}}_{k\|k-1})$ | $2 \times 1$ |
| **Compute Innovation**| $\mathbf{z}_k$, $h(\hat{\mathbf{x}}_{k\|k-1})$ | $2 \times 1$, $2 \times 1$ | Subtraction $\mathbf{z}_k - h(\hat{\mathbf{x}})$ | $\mathbf{y}_k$ | $2 \times 1$ |
| **Kalman Gain** | $\mathbf{P}_{k\|k-1}$, $\mathbf{H}_k$, $\mathbf{R}_k$ | $4 \times 4$, $2 \times 4$, $2 \times 2$ | Matrix inversion $\mathbf{P}\mathbf{H}^T(\mathbf{H}\mathbf{P}\mathbf{H}^T+\mathbf{R})^{-1}$ | $\mathbf{K}_k$ | $4 \times 2$ |
| **State Correction** | $\hat{\mathbf{x}}_{k\|k-1}$, $\mathbf{K}_k$, $\mathbf{y}_k$ | $4 \times 1$, $4 \times 2$, $2 \times 1$ | Addition $\hat{\mathbf{x}} + \mathbf{K}\mathbf{y}$ | $\hat{\mathbf{x}}_{k\|k}$ | $4 \times 1$ |
| **Covariance Update** | $\mathbf{P}_{k\|k-1}$, $\mathbf{K}_k$, $\mathbf{H}_k$ | $4 \times 4$, $4 \times 2$, $2 \times 4$ | Matrix subtraction $(\mathbf{I}-\mathbf{K}\mathbf{H})\mathbf{P}$ | $\mathbf{P}_{k\|k}$ | $4 \times 4$ |

---

## 5. Practical Engineering Considerations

1. **Handling Missing Sensor Data:** If a sensor measurement $\mathbf{z}_k$ fails to arrive (e.g., GPS signal lost in a tunnel), skip Step 2 (Update) and simply set $\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1}$ and $\mathbf{P}_{k|k} = \mathbf{P}_{k|k-1}$. The filter will continue predicting based on physics alone (though uncertainty $\mathbf{P}$ will grow at each step).
2. **Asynchronous Multi-Sensor Fusion:** If you have multiple sensors operating at different rates (e.g., IMU at 100 Hz, GPS at 5 Hz), run the Predict step at 100 Hz and trigger the Update step only when a GPS measurement arrives.
3. **Joseph Form Numerical Safeguard:** For real-world code implementation, update covariance using the Joseph Form rather than $(\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}$ to prevent floating-point rounding errors from destroying matrix symmetry:
   $$\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_{k|k-1} (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k)^T + \mathbf{K}_k \mathbf{R}_k \mathbf{K}_k^T$$
