# Extended Kalman Filter (EKF): A First-Principles Mathematical Proof & Guide

> **Target Audience:** Learners with a background in single/multivariable calculus and basic matrix algebra, who want a complete, step-by-step mathematical proof of the Extended Kalman Filter without needing a prior degree in statistics.

---

## Table of Contents
1. [Introduction & Visual Intuition](#1-introduction--visual-intuition)
2. [Glossary of Statistics & Unfamiliar Math Notation](#2-glossary-of-statistics--unfamiliar-math-notation)
3. [Dynamic System Definition](#3-dynamic-system-definition)
4. [Mathematical Tools & Matrix Calculus Foundations](#4-mathematical-tools--matrix-calculus-foundations)
5. [Step 1: Derivation of the Predict Step (Time Update)](#5-step-1-derivation-of-the-predict-step-time-update)
6. [Step 2: Linearization of Measurement Equation](#6-step-2-linearization-of-measurement-equation)
7. [Step 3: Derivation of the Update Step (Measurement Correction)](#7-step-3-derivation-of-the-update-step-measurement-correction)
8. [Step 4: Derivation of the Optimal Kalman Gain (Trace Minimization)](#8-step-4-derivation-of-the-optimal-kalman-gain-trace-minimization)
9. [Summary Table of All EKF Equations](#9-summary-table-of-all-ekf-equations)
10. [Concrete Worked Example: 2D Radar Tracking](#10-concrete-worked-example-2d-radar-tracking)

---

## 1. Introduction & Visual Intuition

### 1.1 Why Do We Need Filtering?
In the real world, sensors are noisy. GPS receivers jump by several meters, accelerometers drift, and radar readings contain random electrical noise. At the same time, our physical models of vehicle motion (physics equations like $d = vt$) are incomplete because of unmodeled forces like wind gusts, friction, or bumpy roads.

State estimation is the process of combining **imperfect physics predictions** with **noisy sensor measurements** to find the single best guess of where an object actually is.

### 1.2 The Linear Kalman Filter vs. The Extended Kalman Filter
If a system is strictly **linear** (meaning all physics equations look like straight lines: $y = ax + b$), a standard Kalman Filter works perfectly. In a linear system, if our uncertainty is shaped like a smooth bell curve (a **Gaussian distribution**), passing it through a linear equation keeps it shaped like a smooth bell curve.

$$
\text{Gaussian Bell Curve} \xrightarrow{\text{Linear Function } y = Ax + b} \text{Transformed Gaussian Bell Curve}
$$

However, real-world systems are almost always **non-linear**:
* Radar measures distance using square roots: $r = \sqrt{x^2 + y^2}$
* Orientation angles use trigonometric functions: $\theta = \arctan(y/x)$
* Car steering involves sines and cosines: $\dot{x} = v \cos(\theta)$

When you push a smooth Gaussian bell curve through a curved, non-linear function, the curve distorts, bends, and gets skewed. It is no longer a simple Gaussian curve.

```
Gaussian Input Probability
        /\
       /  \
      /    \
-----/------\-----
        |
        v
 [ Non-Linear Function y = sin(x) or sqrt(x) ]
        |
        v
Distorted Non-Gaussian Output
      _/\
    _/   \_/\_   <-- Skewed, multi-peaked, impossible to describe with simple formulas!
```

Tracking a non-Gaussian distribution analytically requires calculating complex integrals at every microsecond, which is impossible for onboard computers. 

### 1.3 The Extended Kalman Filter Solution: Local Linearization
The Extended Kalman Filter (EKF) uses calculus to solve this problem. Instead of trying to track the distorted curved function, the EKF computes the **tangent line** (or **tangent plane** in 3D) at the exact point where we currently think the robot is.

By replacing the curved function with its local tangent line (using a **first-order Taylor series expansion** and a matrix of partial derivatives called a **Jacobian**), the EKF turns a non-linear system into a locally linear system! This allows us to keep using simple Gaussian bell curves and matrix algebra.

---

## 2. Glossary of Statistics & Unfamiliar Math Notation

To ensure you can follow every step of the proof without needing a statistics degree, here is a complete explanation of every statistical symbol and concept used in this document.

---

### 2.1 The Expected Value Operator $\mathbb{E}[X]$

#### What is it?
In statistics, the **Expected Value** $\mathbb{E}[X]$ of a random variable $X$ is simply its **weighted average** or **mean** value ($\mu$). If you measure a noisy distance 1,000 times, the expected value is the average of all those 1,000 readings.

#### Discrete Definition
If $X$ can take values $x_i$ with probabilities $P(x_i)$:
$$
\mathbb{E}[X] = \sum_{i} x_i P(x_i)
$$

#### Continuous Definition (Calculus connection)
If $X$ has a continuous probability density function $p(x)$:
$$
\mathbb{E}[X] = \int_{-\infty}^{\infty} x \cdot p(x) \, dx
$$

#### Key Rules of Expectation You Need for the Proof
1. **Expectation of a Constant $c$:** A fixed constant doesn't vary, so its average is itself:
   $$
   \mathbb{E}[c] = c
   $$
2. **Linearity of Expectation:** Expectation can be distributed across addition and scalar multiplication just like an integral:
   $$
   \mathbb{E}[a X + b Y] = a \mathbb{E}[X] + b \mathbb{E}[Y]
   $$
   *(Where $a, b$ are fixed numbers or deterministic matrices, and $X, Y$ are random variables or random vectors).*
3. **Expectation of Zero-Mean Noise $w$:** We assume sensor noise bounces randomly above and below zero. Therefore, its average is zero:
   $$
   \mathbb{E}[w] = 0
   $$

---

### 2.2 Variance $\sigma^2$ and Covariance Matrix $\mathbf{P}$

#### Variance (Single Variable)
Variance measures the **spread** or **uncertainty** of a single variable around its mean $\mu = \mathbb{E}[X]$:
$$
\text{Var}(X) = \sigma^2 = \mathbb{E}\left[ (X - \mathbb{E}[X])^2 \right]
$$
* If variance is small (e.g., $0.01$), the sensor is very precise.
* If variance is large (e.g., $500.0$), the sensor is extremely noisy.

#### Covariance (Two Variables)
Covariance measures how two variables vary *together*:
$$
\text{Cov}(X, Y) = \mathbb{E}\left[ (X - \mathbb{E}[X])(Y - \mathbb{E}[Y]) \right]
$$
* Positive covariance: When $X$ goes up, $Y$ tends to go up (e.g., car speed and engine noise).
* Zero covariance: $X$ and $Y$ are completely independent (e.g., outdoor temperature and your fuel level).

#### Covariance Matrix for Vectors
When dealing with a vector of states $\mathbf{x} = \begin{bmatrix} x_1 \\ x_2 \end{bmatrix}$ (like position $x$ and velocity $v$), we bundle all variances and covariances into a square matrix called the **Covariance Matrix** $\mathbf{P}$:

$$
\mathbf{P} = \text{Cov}(\mathbf{x}) = \mathbb{E}\left[ (\mathbf{x} - \mathbb{E}[\mathbf{x}])(\mathbf{x} - \mathbb{E}[\mathbf{x}])^T \right]
$$

Let's expand this matrix for a 2D state vector $\mathbf{x} = \begin{bmatrix} x \\ y \end{bmatrix}$ with mean $\hat{\mathbf{x}} = \begin{bmatrix} \hat{x} \\ \hat{y} \end{bmatrix}$:

$$
\mathbf{x} - \hat{\mathbf{x}} = \begin{bmatrix} x - \hat{x} \\ y - \hat{y} \end{bmatrix}
$$

Multiply the vector by its transpose:

$$
(\mathbf{x} - \hat{\mathbf{x}})(\mathbf{x} - \hat{\mathbf{x}})^T = \begin{bmatrix} x - \hat{x} \\ y - \hat{y} \end{bmatrix} \begin{bmatrix} x - \hat{x} & y - \hat{y} \end{bmatrix} = \begin{bmatrix} (x - \hat{x})^2 & (x - \hat{x})(y - \hat{y}) \\ (y - \hat{y})(x - \hat{x}) & (y - \hat{y})^2 \end{bmatrix}
$$

Apply expectation $\mathbb{E}[\cdot]$ to every element inside the matrix:

$$
\mathbf{P} = \begin{bmatrix} 
\mathbb{E}[(x - \hat{x})^2] & \mathbb{E}[(x - \hat{x})(y - \hat{y})] \\ 
\mathbb{E}[(y - \hat{y})(x - \hat{x})] & \mathbb{E}[(y - \hat{y})^2] 
\end{bmatrix} = \begin{bmatrix} \text{Var}(x) & \text{Cov}(x,y) \\ \text{Cov}(y,x) & \text{Var}(y) \end{bmatrix}
$$

> **Crucial Insight:** 
> * The **diagonal elements** of $\mathbf{P}$ represent the variance (uncertainty) of each individual state variable.
> * The **off-diagonal elements** represent how errors in one variable correlate with errors in another.
> * Covariance matrices are always **symmetric** ($\mathbf{P} = \mathbf{P}^T$).

---

### 2.3 Vector Transpose $\mathbf{x}^T$ and Matrix Multiplication
* A column vector $\mathbf{a} = \begin{bmatrix} a_1 \\ a_2 \end{bmatrix}$ transposed becomes a row vector $\mathbf{a}^T = \begin{bmatrix} a_1 & a_2 \end{bmatrix}$.
* **Outer Product:** Vector $\mathbf{a} \in \mathbb{R}^{n \times 1}$ multiplied by $\mathbf{a}^T \in \mathbb{R}^{1 \times n}$ results in an $n \times n$ matrix:
  $$
  \mathbf{a} \mathbf{a}^T = \begin{bmatrix} a_1 \\ a_2 \end{bmatrix} \begin{bmatrix} a_1 & a_2 \end{bmatrix} = \begin{bmatrix} a_1^2 & a_1 a_2 \\ a_2 a_1 & a_2^2 \end{bmatrix}
  $$
* **Transpose Product Rule:** For any two matrices $\mathbf{A}$ and $\mathbf{B}$:
  $$
  (\mathbf{A} \mathbf{B})^T = \mathbf{B}^T \mathbf{A}^T
  $$

---

### 2.4 Trace of a Matrix $\text{tr}(\mathbf{A})$
The **Trace** of a square matrix $\mathbf{A}$, denoted $\text{tr}(\mathbf{A})$, is simply the **sum of the numbers along its main diagonal**:

$$
\text{tr}\left( \begin{bmatrix} a_{11} & a_{12} \\ a_{21} & a_{22} \end{bmatrix} \right) = a_{11} + a_{22}
$$

#### Why do we use Trace in EKF?
Since the diagonal elements of the error covariance matrix $\mathbf{P}$ represent the variance of each state variable ($\text{Var}(x_1) + \text{Var}(x_2) + \dots + \text{Var}(x_n)$), **taking the trace of $\mathbf{P}$ gives the total scalar estimation error variance of the entire system!** To make our state estimate as accurate as possible, we use calculus to find the Kalman gain matrix $\mathbf{K}$ that **minimizes $\text{tr}(\mathbf{P})$**.

#### Essential Trace Identities Used in Derivatives
1. **Cyclic Property:**
   $$
   \text{tr}(\mathbf{A}\mathbf{B}\mathbf{C}) = \text{tr}(\mathbf{B}\mathbf{C}\mathbf{A}) = \text{tr}(\mathbf{C}\mathbf{A}\mathbf{B})
   $$
2. **Transpose Property:**
   $$
   \text{tr}(\mathbf{A}) = \text{tr}(\mathbf{A}^T)
   $$
3. **Trace Matrix Calculus Derivative Rules:**
   * $\frac{\partial \text{tr}(\mathbf{X} \mathbf{A})}{\partial \mathbf{X}} = \mathbf{A}^T$
   * $\frac{\partial \text{tr}(\mathbf{X} \mathbf{B} \mathbf{X}^T)}{\partial \mathbf{X}} = 2 \mathbf{X} \mathbf{B}$ *(when $\mathbf{B}$ is a symmetric matrix)*

---

### 2.5 Subscript Notation: $k$, $k-1$, $k|k-1$, and $k|k$

The subscripts track time and conditional information:

| Notation | Meaning | Plain English Translation |
| :--- | :--- | :--- |
| $k$ | Current time step $t_k$ | "Right now" |
| $k-1$ | Previous time step $t_{k-1}$ | "One step ago" |
| $\hat{\mathbf{x}}_{k\|k-1}$ | Prior state estimate | "Our guess of the state at step $k$, **before** looking at sensor measurement $\mathbf{z}_k$" (also written as $\hat{\mathbf{x}}_k^-$) |
| $\hat{\mathbf{x}}_{k\|k}$ | Posterior state estimate | "Our updated guess of the state at step $k$, **after** incorporating sensor measurement $\mathbf{z}_k$" (also written as $\hat{\mathbf{x}}_k$) |
| $\mathbf{P}_{k\|k-1}$ | Prior covariance | "Our estimation uncertainty at step $k$, **before** the sensor measurement" |
| $\mathbf{P}_{k\|k}$ | Posterior covariance | "Our estimation uncertainty at step $k$, **after** the sensor measurement" |

---

## 3. Dynamic System Definition

We model physical systems in discrete time steps $k = 1, 2, 3, \dots$ using two fundamental equations:

```
[ Step k-1 State x_{k-1} ] ---> ( Non-linear Motion Model f(x) + Noise w_k ) ---> [ Step k True State x_k ]
                                                                                         |
                                                                                         v
                                                                             ( Non-linear Measurement h(x) + Noise v_k )
                                                                                         |
                                                                                         v
                                                                             [ Sensor Data z_k ]
```

### 3.1 State Transition Equation (Physics Process Model)
$$
\mathbf{x}_k = f(\mathbf{x}_{k-1}, \mathbf{u}_k) + \mathbf{w}_k
$$

Where:
* $\mathbf{x}_k \in \mathbb{R}^n$ is the $n$-dimensional **true state vector** at time step $k$ (e.g., position, velocity, angle).
* $\mathbf{u}_k \in \mathbb{R}^m$ is the $m$-dimensional **control input vector** (e.g., steering command, throttle input).
* $f(\mathbf{x}_{k-1}, \mathbf{u}_k)$ is a non-linear vector-valued function describing system dynamics from physics.
* $\mathbf{w}_k \in \mathbb{R}^n$ is the **process noise vector** representing unmodeled disturbances (e.g., wind, bumps, friction).

---

### 3.2 Measurement Equation (Sensor Observation Model)
$$
\mathbf{z}_k = h(\mathbf{x}_k) + \mathbf{v}_k
$$

Where:
* $\mathbf{z}_k \in \mathbb{R}^p$ is the $p$-dimensional **measurement vector** received from physical sensors.
* $h(\mathbf{x}_k)$ is a non-linear function mapping the internal state $\mathbf{x}_k$ to sensor readings.
* $\mathbf{v}_k \in \mathbb{R}^p$ is the **measurement noise vector** representing electrical or optical sensor noise.

---

### 3.3 Noise Assumptions & Statistics
We make standard assumptions about noise vector properties:

1. **Zero Mean Process Noise:**
   $$
   \mathbb{E}[\mathbf{w}_k] = \mathbf{0}
   $$
2. **Process Noise Covariance Matrix $\mathbf{Q}_k \in \mathbb{R}^{n \times n}$:**
   $$
   \mathbb{E}[\mathbf{w}_k \mathbf{w}_j^T] = \begin{cases} \mathbf{Q}_k & \text{if } k = j \\ \mathbf{0} & \text{if } k \neq j \end{cases}
   $$
3. **Zero Mean Measurement Noise:**
   $$
   \mathbb{E}[\mathbf{v}_k] = \mathbf{0}
   $$
4. **Measurement Noise Covariance Matrix $\mathbf{R}_k \in \mathbb{R}^{p \times p}$:**
   $$
   \mathbb{E}[\mathbf{v}_k \mathbf{v}_j^T] = \begin{cases} \mathbf{R}_k & \text{if } k = j \\ \mathbf{0} & \text{if } k \neq j \end{cases}
   $$
5. **Noise Independence:** Process noise and measurement noise are completely independent:
   $$
   \mathbb{E}[\mathbf{w}_k \mathbf{v}_j^T] = \mathbf{0} \quad \text{for all } k, j
   $$

---

## 4. Mathematical Tools & Matrix Calculus Foundations

---

### 4.1 Single-Variable Taylor Series (Linearization Intuition)

#### What is it in Plain English?
A Taylor Series is a mathematical tool that lets you approximate any complicated curved function $g(y)$ (like $\sin(y)$ or $\sqrt{y}$) near a specific point $y_0$ using a simple straight tangent line.

#### The General Formula
For any smooth function $g(y)$ near an anchor point $y_0$:

$$
g(y) = \underbrace{g(y_0)}_{\text{Height at anchor}} + \underbrace{\left.\frac{dg}{dy}\right|_{y_0} (y - y_0)}_{\text{Linear Slope Adjustment}} + \underbrace{\frac{1}{2!} \left.\frac{d^2g}{dy^2}\right|_{y_0} (y - y_0)^2 + \dots}_{\text{Curvature \& Higher-Order Terms}}
$$

#### Breaking Down Each Term:
* **$y_0$ (Anchor Point):** Your current starting location (e.g., your estimated position).
* **$y$ (Target Point):** A nearby location close to $y_0$.
* **$(y - y_0)$ (Step Size):** The small distance between where you are ($y_0$) and where you want to evaluate ($y$).
* **$\left.\frac{dg}{dy}\right|_{y_0}$ (Slope):** The derivative (rate of change) of the function measured right at the anchor point $y_0$.

#### The EKF First-Order Approximation (Dropping Higher-Order Terms)
When $y$ is very close to $y_0$, the step size $(y - y_0)$ is a tiny decimal (e.g., $0.01$). Squaring a tiny decimal makes it negligible ($(0.01)^2 = 0.0001$). 

Therefore, the EKF ignores the squared and higher-order terms, turning the complicated curve into a **straight line equation**:

$$
g(y) \approx g(y_0) + \left.\frac{dg}{dy}\right|_{y_0} (y - y_0)
$$

*(Notice this is simply the familiar point-slope line equation: $y = y_0 + \text{slope} \cdot \Delta x$).*


---

### 4.2 Multivariate Taylor Series & The Jacobian Matrix
When $g(\mathbf{y})$ takes a **vector input** $\mathbf{y} = \begin{bmatrix} y_1 \\ y_2 \\ \vdots \\ y_n \end{bmatrix}$ and outputs a **vector** $g(\mathbf{y}) = \begin{bmatrix} g_1(\mathbf{y}) \\ g_2(\mathbf{y}) \\ \vdots \\ g_q(\mathbf{y}) \end{bmatrix}$, the single derivative $\frac{dg}{dy}$ generalizes into a matrix of all possible partial derivatives called the **Jacobian Matrix** $\mathbf{J}_g$.

#### First-Order Multivariate Taylor Expansion:
$$
g(\mathbf{y}) \approx g(\mathbf{y}_0) + \mathbf{J}_g(\mathbf{y}_0) (\mathbf{y} - \mathbf{y}_0)
$$

Where the Jacobian matrix $\mathbf{J}_g(\mathbf{y}_0) \in \mathbb{R}^{q \times n}$ is defined as:

$$
\mathbf{J}_g(\mathbf{y}_0) = \begin{bmatrix}
\frac{\partial g_1}{\partial y_1} & \frac{\partial g_1}{\partial y_2} & \cdots & \frac{\partial g_1}{\partial y_n} \\
\frac{\partial g_2}{\partial y_1} & \frac{\partial g_2}{\partial y_2} & \cdots & \frac{\partial g_2}{\partial y_n} \\
\vdots & \vdots & \ddots & \vdots \\
\frac{\partial g_q}{\partial y_1} & \frac{\partial g_q}{\partial y_2} & \cdots & \frac{\partial g_q}{\partial y_n}
\end{bmatrix}_{\mathbf{y} = \mathbf{y}_0}
$$

---

### 4.3 EKF Jacobians Defined: $\mathbf{F}_k$ and $\mathbf{H}_k$

In the Extended Kalman Filter, we define two specific Jacobians at every time step:

#### 1. Process Model Jacobian Matrix $\mathbf{F}_k \in \mathbb{R}^{n \times n}$:
Evaluated at the previous posterior estimate $\hat{\mathbf{x}}_{k-1|k-1}$:
$$
\mathbf{F}_k = \left. \frac{\partial f(\mathbf{x}, \mathbf{u}_k)}{\partial \mathbf{x}} \right|_{\hat{\mathbf{x}}_{k-1|k-1}}
$$

#### 2. Measurement Model Jacobian Matrix $\mathbf{H}_k \in \mathbb{R}^{p \times n}$:
Evaluated at the predicted prior estimate $\hat{\mathbf{x}}_{k|k-1}$:
$$
\mathbf{H}_k = \left. \frac{\partial h(\mathbf{x})}{\partial \mathbf{x}} \right|_{\hat{\mathbf{x}}_{k|k-1}}
$$

---

## 5. Step 1: Derivation of the Predict Step (Time Update)

The goal of the Predict step is to compute our prior state prediction $\hat{\mathbf{x}}_{k|k-1}$ and prior covariance matrix $\mathbf{P}_{k|k-1}$ before any sensor measurements arrive.

---

### 5.1 Derivation of State Prediction Equation

By definition, the prior state prediction $\hat{\mathbf{x}}_{k|k-1}$ is the expected value of true state $\mathbf{x}_k$:

$$
\hat{\mathbf{x}}_{k|k-1} = \mathbb{E}[\mathbf{x}_k]
$$

Substitute true state equation $\mathbf{x}_k = f(\mathbf{x}_{k-1}, \mathbf{u}_k) + \mathbf{w}_k$:

$$
\hat{\mathbf{x}}_{k|k-1} = \mathbb{E}\left[ f(\mathbf{x}_{k-1}, \mathbf{u}_k) + \mathbf{w}_k \right]
$$

Apply linearity of expectation ($\mathbb{E}[A + B] = \mathbb{E}[A] + \mathbb{E}[B]$):

$$
\hat{\mathbf{x}}_{k|k-1} = \mathbb{E}\left[ f(\mathbf{x}_{k-1}, \mathbf{u}_k) \right] + \mathbb{E}[\mathbf{w}_k]
$$

Since process noise has zero mean ($\mathbb{E}[\mathbf{w}_k] = \mathbf{0}$):

$$
\hat{\mathbf{x}}_{k|k-1} = \mathbb{E}\left[ f(\mathbf{x}_{k-1}, \mathbf{u}_k) \right]
$$

Now, expand $f(\mathbf{x}_{k-1}, \mathbf{u}_k)$ using Taylor series around operating point $\hat{\mathbf{x}}_{k-1|k-1}$:

$$
f(\mathbf{x}_{k-1}, \mathbf{u}_k) \approx f(\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{u}_k) + \mathbf{F}_k (\mathbf{x}_{k-1} - \hat{\mathbf{x}}_{k-1|k-1})
$$

Substitute this approximation back into the expectation:

$$
\hat{\mathbf{x}}_{k|k-1} \approx \mathbb{E}\left[ f(\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{u}_k) + \mathbf{F}_k (\mathbf{x}_{k-1} - \hat{\mathbf{x}}_{k-1|k-1}) \right]
$$

Distribute expectation $\mathbb{E}[\cdot]$:

$$
\hat{\mathbf{x}}_{k|k-1} \approx \mathbb{E}\left[ f(\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{u}_k) \right] + \mathbf{F}_k \mathbb{E}\left[ \mathbf{x}_{k-1} - \hat{\mathbf{x}}_{k-1|k-1} \right]
$$

Look at the term $\mathbb{E}[\mathbf{x}_{k-1} - \hat{\mathbf{x}}_{k-1|k-1}]$. Because $\hat{\mathbf{x}}_{k-1|k-1}$ is defined as the mean of $\mathbf{x}_{k-1}$, the average difference $\mathbb{E}[\mathbf{x}_{k-1} - \hat{\mathbf{x}}_{k-1|k-1}] = \mathbf{0}$.

Thus, the second term drops out, giving the **State Prediction Equation**:

$$
\hat{\mathbf{x}}_{k|k-1} = f(\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{u}_k)
$$

---

### 5.2 Derivation of Prior Error Vector Equation

To see how state uncertainty grows, define the **prior estimation error vector**:

$$
\tilde{\mathbf{x}}_{k|k-1} = \mathbf{x}_k - \hat{\mathbf{x}}_{k|k-1}
$$

Substitute true state $\mathbf{x}_k = f(\mathbf{x}_{k-1}, \mathbf{u}_k) + \mathbf{w}_k$ and predicted state $\hat{\mathbf{x}}_{k|k-1} = f(\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{u}_k)$:

$$
\tilde{\mathbf{x}}_{k|k-1} = \left[ f(\mathbf{x}_{k-1}, \mathbf{u}_k) + \mathbf{w}_k \right] - f(\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{u}_k)
$$

Substitute Taylor series approximation $f(\mathbf{x}_{k-1}, \mathbf{u}_k) \approx f(\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{u}_k) + \mathbf{F}_k (\mathbf{x}_{k-1} - \hat{\mathbf{x}}_{k-1|k-1})$:

$$
\tilde{\mathbf{x}}_{k|k-1} \approx f(\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{u}_k) + \mathbf{F}_k (\mathbf{x}_{k-1} - \hat{\mathbf{x}}_{k-1|k-1}) + \mathbf{w}_k - f(\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{u}_k)
$$

Cancel matching terms $f(\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{u}_k)$:

$$
\tilde{\mathbf{x}}_{k|k-1} \approx \mathbf{F}_k (\mathbf{x}_{k-1} - \hat{\mathbf{x}}_{k-1|k-1}) + \mathbf{w}_k
$$

Since previous posterior error is $\tilde{\mathbf{x}}_{k-1|k-1} = \mathbf{x}_{k-1} - \hat{\mathbf{x}}_{k-1|k-1}$:

$$
\tilde{\mathbf{x}}_{k|k-1} \approx \mathbf{F}_k \tilde{\mathbf{x}}_{k-1|k-1} + \mathbf{w}_k
$$

---

### 5.3 Derivation of Prior Covariance Matrix Equation

By definition of covariance, prior covariance matrix $\mathbf{P}_{k|k-1}$ is:

$$
\mathbf{P}_{k|k-1} = \mathbb{E}\left[ \tilde{\mathbf{x}}_{k|k-1} \tilde{\mathbf{x}}_{k|k-1}^T \right]
$$

Substitute error vector $\tilde{\mathbf{x}}_{k|k-1} = \mathbf{F}_k \tilde{\mathbf{x}}_{k-1|k-1} + \mathbf{w}_k$:

$$
\mathbf{P}_{k|k-1} = \mathbb{E}\left[ (\mathbf{F}_k \tilde{\mathbf{x}}_{k-1|k-1} + \mathbf{w}_k) (\mathbf{F}_k \tilde{\mathbf{x}}_{k-1|k-1} + \mathbf{w}_k)^T \right]
$$

Apply transpose rule $(\mathbf{A} + \mathbf{B})^T = \mathbf{A}^T + \mathbf{B}^T$ and $(\mathbf{A}\mathbf{B})^T = \mathbf{B}^T\mathbf{A}^T$:

$$
(\mathbf{F}_k \tilde{\mathbf{x}}_{k-1|k-1} + \mathbf{w}_k)^T = \tilde{\mathbf{x}}_{k-1|k-1}^T \mathbf{F}_k^T + \mathbf{w}_k^T
$$

Multiply term-by-term inside brackets using FOIL rule:

$$
\mathbf{P}_{k|k-1} = \mathbb{E}\left[ (\mathbf{F}_k \tilde{\mathbf{x}}_{k-1|k-1})(\tilde{\mathbf{x}}_{k-1|k-1}^T \mathbf{F}_k^T) + (\mathbf{F}_k \tilde{\mathbf{x}}_{k-1|k-1})\mathbf{w}_k^T + \mathbf{w}_k(\tilde{\mathbf{x}}_{k-1|k-1}^T \mathbf{F}_k^T) + \mathbf{w}_k \mathbf{w}_k^T \right]
$$

Distribute expectation operator $\mathbb{E}[\cdot]$ across all 4 terms:

$$
\mathbf{P}_{k|k-1} = \mathbb{E}\left[ \mathbf{F}_k \tilde{\mathbf{x}}_{k-1|k-1} \tilde{\mathbf{x}}_{k-1|k-1}^T \mathbf{F}_k^T \right] + \mathbb{E}\left[ \mathbf{F}_k \tilde{\mathbf{x}}_{k-1|k-1} \mathbf{w}_k^T \right] + \mathbb{E}\left[ \mathbf{w}_k \tilde{\mathbf{x}}_{k-1|k-1}^T \mathbf{F}_k^T \right] + \mathbb{E}\left[ \mathbf{w}_k \mathbf{w}_k^T \right]
$$

Evaluate each term individually:

1. **Term 1:** Factor out constant matrix $\mathbf{F}_k$:
   $$
   \mathbb{E}\left[ \mathbf{F}_k \tilde{\mathbf{x}}_{k-1|k-1} \tilde{\mathbf{x}}_{k-1|k-1}^T \mathbf{F}_k^T \right] = \mathbf{F}_k \mathbb{E}\left[ \tilde{\mathbf{x}}_{k-1|k-1} \tilde{\mathbf{x}}_{k-1|k-1}^T \right] \mathbf{F}_k^T = \mathbf{F}_k \mathbf{P}_{k-1|k-1} \mathbf{F}_k^T
   $$
2. **Terms 2 & 3:** Process noise $\mathbf{w}_k$ is independent of previous state error $\tilde{\mathbf{x}}_{k-1|k-1}$, so cross-terms are zero:
   $$
   \mathbb{E}\left[ \mathbf{F}_k \tilde{\mathbf{x}}_{k-1|k-1} \mathbf{w}_k^T \right] = \mathbf{F}_k \mathbb{E}[\tilde{\mathbf{x}}_{k-1|k-1}] \mathbb{E}[\mathbf{w}_k^T] = \mathbf{0}
   $$
   $$
   \mathbb{E}\left[ \mathbf{w}_k \tilde{\mathbf{x}}_{k-1|k-1}^T \mathbf{F}_k^T \right] = \mathbf{0}
   $$
3. **Term 4:** By definition of process noise covariance $\mathbf{Q}_k$:
   $$
   \mathbb{E}\left[ \mathbf{w}_k \mathbf{w}_k^T \right] = \mathbf{Q}_k
   $$

Summing all non-zero terms yields the **Prior Covariance Propagation Equation**:

$$
\mathbf{P}_{k|k-1} = \mathbf{F}_k \mathbf{P}_{k-1|k-1} \mathbf{F}_k^T + \mathbf{Q}_k
$$

---

## 6. Step 2: Linearization of Measurement Equation

Before correcting our prediction using a sensor reading $\mathbf{z}_k$, we linearize measurement function $h(\mathbf{x}_k)$ around predicted state $\hat{\mathbf{x}}_{k|k-1}$.

---

### 6.1 Taylor Expansion of Measurement Model
Expand non-linear observation model $h(\mathbf{x}_k)$ around $\hat{\mathbf{x}}_{k|k-1}$:

$$
h(\mathbf{x}_k) \approx h(\hat{\mathbf{x}}_{k|k-1}) + \left. \frac{\partial h}{\partial \mathbf{x}} \right|_{\hat{\mathbf{x}}_{k|k-1}} (\mathbf{x}_k - \hat{\mathbf{x}}_{k|k-1})
$$

Substitute measurement Jacobian $\mathbf{H}_k = \left. \frac{\partial h}{\partial \mathbf{x}} \right|_{\hat{\mathbf{x}}_{k|k-1}}$:

$$
h(\mathbf{x}_k) \approx h(\hat{\mathbf{x}}_{k|k-1}) + \mathbf{H}_k (\mathbf{x}_k - \hat{\mathbf{x}}_{k|k-1})
$$

Substitute this back into sensor equation $\mathbf{z}_k = h(\mathbf{x}_k) + \mathbf{v}_k$:

$$
\mathbf{z}_k \approx h(\hat{\mathbf{x}}_{k|k-1}) + \mathbf{H}_k (\mathbf{x}_k - \hat{\mathbf{x}}_{k|k-1}) + \mathbf{v}_k
$$

Since prior estimation error is $\tilde{\mathbf{x}}_{k|k-1} = \mathbf{x}_k - \hat{\mathbf{x}}_{k|k-1}$:

$$
\mathbf{z}_k \approx h(\hat{\mathbf{x}}_{k|k-1}) + \mathbf{H}_k \tilde{\mathbf{x}}_{k|k-1} + \mathbf{v}_k
$$

---

### 6.2 Derivation of Innovation Residual Vector $\mathbf{y}_k$
The **Measurement Innovation (or Residual)** $\mathbf{y}_k$ measures the difference between actual observed sensor reading $\mathbf{z}_k$ and predicted measurement $h(\hat{\mathbf{x}}_{k|k-1})$:

$$
\mathbf{y}_k = \mathbf{z}_k - h(\hat{\mathbf{x}}_{k|k-1})
$$

Substitute linearized expression for $\mathbf{z}_k$:

$$
\mathbf{y}_k \approx \left[ h(\hat{\mathbf{x}}_{k|k-1}) + \mathbf{H}_k \tilde{\mathbf{x}}_{k|k-1} + \mathbf{v}_k \right] - h(\hat{\mathbf{x}}_{k|k-1})
$$

Cancel $h(\hat{\mathbf{x}}_{k|k-1})$ terms:

$$
\mathbf{y}_k \approx \mathbf{H}_k \tilde{\mathbf{x}}_{k|k-1} + \mathbf{v}_k
$$

---

### 6.3 Derivation of Innovation Covariance Matrix $\mathbf{S}_k$
Innovation covariance $\mathbf{S}_k$ quantifies total measurement uncertainty:

$$
\mathbf{S}_k = \mathbb{E}\left[ \mathbf{y}_k \mathbf{y}_k^T \right]
$$

Substitute $\mathbf{y}_k = \mathbf{H}_k \tilde{\mathbf{x}}_{k|k-1} + \mathbf{v}_k$:

$$
\mathbf{S}_k = \mathbb{E}\left[ (\mathbf{H}_k \tilde{\mathbf{x}}_{k|k-1} + \mathbf{v}_k) (\mathbf{H}_k \tilde{\mathbf{x}}_{k|k-1} + \mathbf{v}_k)^T \right]
$$

Expand transpose and multiply out terms:

$$
\mathbf{S}_k = \mathbb{E}\left[ \mathbf{H}_k \tilde{\mathbf{x}}_{k|k-1} \tilde{\mathbf{x}}_{k|k-1}^T \mathbf{H}_k^T + \mathbf{H}_k \tilde{\mathbf{x}}_{k|k-1} \mathbf{v}_k^T + \mathbf{v}_k \tilde{\mathbf{x}}_{k|k-1}^T \mathbf{H}_k^T + \mathbf{v}_k \mathbf{v}_k^T \right]
$$

Distribute expectation $\mathbb{E}[\cdot]$:
1. $\mathbb{E}[\mathbf{H}_k \tilde{\mathbf{x}}_{k|k-1} \tilde{\mathbf{x}}_{k|k-1}^T \mathbf{H}_k^T] = \mathbf{H}_k \mathbb{E}[\tilde{\mathbf{x}}_{k|k-1} \tilde{\mathbf{x}}_{k|k-1}^T] \mathbf{H}_k^T = \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^T$
2. Cross-terms evaluate to zero because sensor noise $\mathbf{v}_k$ is independent of state error $\tilde{\mathbf{x}}_{k|k-1}$.
3. $\mathbb{E}[\mathbf{v}_k \mathbf{v}_k^T] = \mathbf{R}_k$

Summing non-zero terms yields the **Innovation Covariance Equation**:

$$
\mathbf{S}_k = \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^T + \mathbf{R}_k
$$

---

## 7. Step 3: Derivation of the Update Step (Measurement Correction)

---

### 7.1 Linear Correction Structure
We define posterior state estimate $\hat{\mathbf{x}}_{k|k}$ as prior state estimate $\hat{\mathbf{x}}_{k|k-1}$ plus a correction weighted by **Kalman Gain Matrix $\mathbf{K}_k$**:

$$
\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \mathbf{y}_k
$$

Substitute innovation $\mathbf{y}_k = \mathbf{z}_k - h(\hat{\mathbf{x}}_{k|k-1})$:

$$
\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \left( \mathbf{z}_k - h(\hat{\mathbf{x}}_{k|k-1}) \right)
$$

---

### 7.2 Derivation of Posterior Error Vector Equation
Define posterior estimation error vector $\tilde{\mathbf{x}}_{k|k}$:

$$
\tilde{\mathbf{x}}_{k|k} = \mathbf{x}_k - \hat{\mathbf{x}}_{k|k}
$$

Substitute correction equation $\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \left( \mathbf{z}_k - h(\hat{\mathbf{x}}_{k|k-1}) \right)$:

$$
\tilde{\mathbf{x}}_{k|k} = \mathbf{x}_k - \left[ \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \left( \mathbf{z}_k - h(\hat{\mathbf{x}}_{k|k-1}) \right) \right]
$$

Group prior error term $\tilde{\mathbf{x}}_{k|k-1} = \mathbf{x}_k - \hat{\mathbf{x}}_{k|k-1}$:

$$
\tilde{\mathbf{x}}_{k|k} = \tilde{\mathbf{x}}_{k|k-1} - \mathbf{K}_k \left( \mathbf{z}_k - h(\hat{\mathbf{x}}_{k|k-1}) \right)
$$

Substitute linearized measurement error $(\mathbf{z}_k - h(\hat{\mathbf{x}}_{k|k-1})) \approx \mathbf{H}_k \tilde{\mathbf{x}}_{k|k-1} + \mathbf{v}_k$:

$$
\tilde{\mathbf{x}}_{k|k} = \tilde{\mathbf{x}}_{k|k-1} - \mathbf{K}_k (\mathbf{H}_k \tilde{\mathbf{x}}_{k|k-1} + \mathbf{v}_k)
$$

Distribute gain matrix $\mathbf{K}_k$:

$$
\tilde{\mathbf{x}}_{k|k} = \tilde{\mathbf{x}}_{k|k-1} - \mathbf{K}_k \mathbf{H}_k \tilde{\mathbf{x}}_{k|k-1} - \mathbf{K}_k \mathbf{v}_k
$$

Factor out prior error vector $\tilde{\mathbf{x}}_{k|k-1}$:

$$
\tilde{\mathbf{x}}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \tilde{\mathbf{x}}_{k|k-1} - \mathbf{K}_k \mathbf{v}_k
$$

*(Where $\mathbf{I}$ is the $n \times n$ identity matrix).*

---

### 7.3 Derivation of Posterior Covariance Matrix (Joseph Form)
By definition of covariance, posterior error covariance matrix $\mathbf{P}_{k|k}$ is:

$$
\mathbf{P}_{k|k} = \mathbb{E}\left[ \tilde{\mathbf{x}}_{k|k} \tilde{\mathbf{x}}_{k|k}^T \right]
$$

Substitute posterior error expression $\tilde{\mathbf{x}}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \tilde{\mathbf{x}}_{k|k-1} - \mathbf{K}_k \mathbf{v}_k$:

$$
\mathbf{P}_{k|k} = \mathbb{E}\left[ \left( (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \tilde{\mathbf{x}}_{k|k-1} - \mathbf{K}_k \mathbf{v}_k \right) \left( (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \tilde{\mathbf{x}}_{k|k-1} - \mathbf{K}_k \mathbf{v}_k \right)^T \right]
$$

Expand transpose on right factor:

$$
\left( (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \tilde{\mathbf{x}}_{k|k-1} - \mathbf{K}_k \mathbf{v}_k \right)^T = \tilde{\mathbf{x}}_{k|k-1}^T (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k)^T - \mathbf{v}_k^T \mathbf{K}_k^T
$$

Multiply all four terms inside brackets:

$$
\begin{aligned}
\mathbf{P}_{k|k} = \mathbb{E}\Big[ & (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \tilde{\mathbf{x}}_{k|k-1} \tilde{\mathbf{x}}_{k|k-1}^T (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k)^T \\
& - (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \tilde{\mathbf{x}}_{k|k-1} \mathbf{v}_k^T \mathbf{K}_k^T \\
& - \mathbf{K}_k \mathbf{v}_k \tilde{\mathbf{x}}_{k|k-1}^T (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k)^T \\
& + \mathbf{K}_k \mathbf{v}_k \mathbf{v}_k^T \mathbf{K}_k^T \Big]
\end{aligned}
$$

Distribute expectation operator $\mathbb{E}[\cdot]$:

1. **Term 1:** Factor out non-random matrices:
   $$
   (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbb{E}\left[ \tilde{\mathbf{x}}_{k|k-1} \tilde{\mathbf{x}}_{k|k-1}^T \right] (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k)^T = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_{k|k-1} (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k)^T
   $$
2. **Terms 2 & 3:** Cross-terms evaluate to zero because sensor noise $\mathbf{v}_k$ is independent of prior state error $\tilde{\mathbf{x}}_{k|k-1}$.
3. **Term 4:**
   $$
   \mathbf{K}_k \mathbb{E}\left[ \mathbf{v}_k \mathbf{v}_k^T \right] \mathbf{K}_k^T = \mathbf{K}_k \mathbf{R}_k \mathbf{K}_k^T
   $$

Summing all non-zero terms gives the **Joseph Form Covariance Update Equation**:

$$
\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_{k|k-1} (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k)^T + \mathbf{K}_k \mathbf{R}_k \mathbf{K}_k^T
$$

---

## 8. Step 4: Derivation of the Optimal Kalman Gain (Trace Minimization)

Now we answer the central question: **What choice of gain matrix $\mathbf{K}_k$ gives the smallest possible estimation error variance?**

---

### 8.1 Defining Cost Function $J(\mathbf{K}_k)$
We define cost function $J(\mathbf{K}_k)$ as total error variance, given by the trace of posterior covariance matrix $\mathbf{P}_{k|k}$:

$$
J(\mathbf{K}_k) = \text{tr}(\mathbf{P}_{k|k})
$$

Expand Joseph form expression for $\mathbf{P}_{k|k}$ inside the trace:

$$
\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_{k|k-1} (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k)^T + \mathbf{K}_k \mathbf{R}_k \mathbf{K}_k^T
$$

Expand factor $(\mathbf{I} - \mathbf{K}_k \mathbf{H}_k)^T = (\mathbf{I} - \mathbf{H}_k^T \mathbf{K}_k^T)$:

$$
(\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_{k|k-1} (\mathbf{I} - \mathbf{H}_k^T \mathbf{K}_k^T) = (\mathbf{P}_{k|k-1} - \mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1}) (\mathbf{I} - \mathbf{H}_k^T \mathbf{K}_k^T)
$$

Multiply out term-by-term:

$$
= \mathbf{P}_{k|k-1} - \mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{K}_k^T - \mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1} + \mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{K}_k^T
$$

Add noise term $\mathbf{K}_k \mathbf{R}_k \mathbf{K}_k^T$:

$$
\mathbf{P}_{k|k} = \mathbf{P}_{k|k-1} - \mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{K}_k^T - \mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1} + \mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{K}_k^T + \mathbf{K}_k \mathbf{R}_k \mathbf{K}_k^T
$$

Group the last two terms by factoring out $\mathbf{K}_k$ on left and $\mathbf{K}_k^T$ on right:

$$
\mathbf{P}_{k|k} = \mathbf{P}_{k|k-1} - \mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{K}_k^T - \mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1} + \mathbf{K}_k \left( \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^T + \mathbf{R}_k \right) \mathbf{K}_k^T
$$

Recall innovation covariance definition $\mathbf{S}_k = \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^T + \mathbf{R}_k$:

$$
\mathbf{P}_{k|k} = \mathbf{P}_{k|k-1} - \mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{K}_k^T - \mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1} + \mathbf{K}_k \mathbf{S}_k \mathbf{K}_k^T
$$

---

### 8.2 Taking Derivative of Trace with Respect to $\mathbf{K}_k$
Now apply trace operator $\text{tr}(\cdot)$ to all terms:

$$
\text{tr}(\mathbf{P}_{k|k}) = \text{tr}(\mathbf{P}_{k|k-1}) - \text{tr}(\mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{K}_k^T) - \text{tr}(\mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1}) + \text{tr}(\mathbf{K}_k \mathbf{S}_k \mathbf{K}_k^T)
$$

Using trace transpose identity $\text{tr}(\mathbf{A}) = \text{tr}(\mathbf{A}^T)$:

$$
\text{tr}(\mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{K}_k^T) = \text{tr}\left( (\mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{K}_k^T)^T \right) = \text{tr}(\mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1}^T)
$$

Since covariance matrix $\mathbf{P}_{k|k-1}$ is symmetric ($\mathbf{P}_{k|k-1}^T = \mathbf{P}_{k|k-1}$):

$$
\text{tr}(\mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{K}_k^T) = \text{tr}(\mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1})
$$

Combine matching trace terms:

$$
\text{tr}(\mathbf{P}_{k|k}) = \text{tr}(\mathbf{P}_{k|k-1}) - 2 \text{tr}(\mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1}) + \text{tr}(\mathbf{K}_k \mathbf{S}_k \mathbf{K}_k^T)
$$

Differentiate with respect to matrix $\mathbf{K}_k$ using standard matrix calculus rules:
1. $\frac{\partial \text{tr}(\mathbf{P}_{k|k-1})}{\partial \mathbf{K}_k} = \mathbf{0}$
2. $\frac{\partial \text{tr}(\mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1})}{\partial \mathbf{K}_k} = (\mathbf{H}_k \mathbf{P}_{k|k-1})^T = \mathbf{P}_{k|k-1} \mathbf{H}_k^T$
3. $\frac{\partial \text{tr}(\mathbf{K}_k \mathbf{S}_k \mathbf{K}_k^T)}{\partial \mathbf{K}_k} = 2 \mathbf{K}_k \mathbf{S}_k$ *(since $\mathbf{S}_k$ is symmetric)*

Combining derivatives yields:

$$
\frac{\partial \text{tr}(\mathbf{P}_{k|k})}{\partial \mathbf{K}_k} = -2 \mathbf{P}_{k|k-1} \mathbf{H}_k^T + 2 \mathbf{K}_k \mathbf{S}_k
$$

---

### 8.3 Solving for Optimal Kalman Gain Matrix $\mathbf{K}_k$
To find the minimum cost value, set derivative equal to zero matrix $\mathbf{0}$:

$$
-2 \mathbf{P}_{k|k-1} \mathbf{H}_k^T + 2 \mathbf{K}_k \mathbf{S}_k = \mathbf{0}
$$

Divide by 2:

$$
-\mathbf{P}_{k|k-1} \mathbf{H}_k^T + \mathbf{K}_k \mathbf{S}_k = \mathbf{0}
$$

Rearrange terms:

$$
\mathbf{K}_k \mathbf{S}_k = \mathbf{P}_{k|k-1} \mathbf{H}_k^T
$$

Post-multiply both sides by inverse matrix $\mathbf{S}_k^{-1}$:

$$
\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{S}_k^{-1}
$$

Substitute innovation covariance formula $\mathbf{S}_k = \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^T + \mathbf{R}_k$:

$$
\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}_k^T \left( \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^T + \mathbf{R}_k \right)^{-1}
$$

This is the **Optimal Extended Kalman Gain Matrix**.

---

### 8.4 Simplifying Posterior Covariance Equation
Now substitute identity $\mathbf{K}_k \mathbf{S}_k = \mathbf{P}_{k|k-1} \mathbf{H}_k^T$ back into expanded Joseph form equation:

$$
\mathbf{P}_{k|k} = \mathbf{P}_{k|k-1} - \mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{K}_k^T - \mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1} + \mathbf{K}_k \mathbf{S}_k \mathbf{K}_k^T
$$

Replace term $\mathbf{K}_k \mathbf{S}_k \mathbf{K}_k^T$ with $(\mathbf{K}_k \mathbf{S}_k) \mathbf{K}_k^T = (\mathbf{P}_{k|k-1} \mathbf{H}_k^T) \mathbf{K}_k^T$:

$$
\mathbf{P}_{k|k} = \mathbf{P}_{k|k-1} - \mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{K}_k^T - \mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1} + \mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{K}_k^T
$$

Matching terms $-\mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{K}_k^T$ and $+\mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{K}_k^T$ cancel out completely:

$$
\mathbf{P}_{k|k} = \mathbf{P}_{k|k-1} - \mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1}
$$

Factor out $\mathbf{P}_{k|k-1}$ to the right:

$$
\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_{k|k-1}
$$

This is the **Simplified Posterior Covariance Update Equation**.

---

## 9. Summary Table of All EKF Equations

```
┌────────────────────────────────────────────────────────────────────────┐
│                        Predict Step (Time Update)                      │
├────────────────────────────────────────────────────────────────────────┤
│ 1. Prior State Estimate:                                               │
│    x_{k|k-1} = f(x_{k-1|k-1}, u_k)                                     │
│                                                                        │
│ 2. Process Jacobian Matrix:                                            │
│    F_k = df/dx | x_{k-1|k-1}                                           │
│                                                                        │
│ 3. Prior Covariance Matrix:                                            │
│    P_{k|k-1} = F_k P_{k-1|k-1} F_k^T + Q_k                            │
└──────────────────────────────────┬─────────────────────────────────────┘
                                   │
                                   v
┌────────────────────────────────────────────────────────────────────────┐
│                       Update Step (Measurement Correction)             │
├────────────────────────────────────────────────────────────────────────┤
│ 4. Measurement Jacobian Matrix:                                        │
│    H_k = dh/dx | x_{k|k-1}                                             │
│                                                                        │
│ 5. Innovation Residual:                                                │
│    y_k = z_k - h(x_{k|k-1})                                            │
│                                                                        │
│ 6. Innovation Covariance Matrix:                                       │
│    S_k = H_k P_{k|k-1} H_k^T + R_k                                     │
│                                                                        │
│ 7. Optimal Kalman Gain Matrix:                                         │
│    K_k = P_{k|k-1} H_k^T S_k^{-1}                                      │
│                                                                        │
│ 8. Posterior State Estimate:                                           │
│    x_{k|k} = x_{k|k-1} + K_k y_k                                       │
│                                                                        │
│ 9. Posterior Covariance Matrix:                                        │
│    P_{k|k} = (I - K_k H_k) P_{k|k-1}                                  │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 10. Concrete Worked Example: 2D Radar Tracking

To solidify understanding, we apply the derivation to a 2D vehicle tracking problem with non-linear range and bearing sensor measurements.

---

### 10.1 System Definition
* **State Vector $\mathbf{x}$:** 2D cartesian position $(x, y)$ and velocity $(\dot{x}, \dot{y})$:
  $$
  \mathbf{x} = \begin{bmatrix} x \\ y \\ \dot{x} \\ \dot{y} \end{bmatrix} \in \mathbb{R}^4
  $$

* **Linear Physics Motion Model (Constant Velocity):**
  $$
  f(\mathbf{x}_{k-1}) = \begin{bmatrix}
  x_{k-1} + \dot{x}_{k-1} \Delta t \\
  y_{k-1} + \dot{y}_{k-1} \Delta t \\
  \dot{x}_{k-1} \\
  \dot{y}_{k-1}
  \end{bmatrix}
  $$

  Since motion model $f(\mathbf{x})$ is already linear in this example, its Jacobian $\mathbf{F}_k$ is constant:
  $$
  \mathbf{F}_k = \begin{bmatrix}
  1 & 0 & \Delta t & 0 \\
  0 & 1 & 0 & \Delta t \\
  0 & 0 & 1 & 0 \\
  0 & 0 & 0 & 1
  \end{bmatrix}
  $$

---

### 10.2 Non-Linear Sensor Measurement Model
A radar station located at origin $(0,0)$ measures Range $r$ (distance) and Bearing $\phi$ (angle):

$$
\mathbf{z} = h(\mathbf{x}) = \begin{bmatrix} r \\ \phi \end{bmatrix} = \begin{bmatrix} \sqrt{x^2 + y^2} \\ \arctan\left(\frac{y}{x}\right) \end{bmatrix}
$$

This function $h(\mathbf{x}): \mathbb{R}^4 \to \mathbb{R}^2$ is non-linear because of the square root and arctan operations.

---

### 10.3 Step-by-Step Calculation of Measurement Jacobian $\mathbf{H}_k$
We compute partial derivatives of $h(\mathbf{x}) = \begin{bmatrix} h_1 \\ h_2 \end{bmatrix}$ with respect to $\mathbf{x} = \begin{bmatrix} x & y & \dot{x} & \dot{y} \end{bmatrix}$:

#### 1. First Row: Range Derivatives ($h_1 = \sqrt{x^2 + y^2}$)
Use power chain rule $\frac{d}{dx}(u(x)^{1/2}) = \frac{1}{2} u(x)^{-1/2} u'(x)$:
* Partial derivative w.r.t $x$:
  $$
  \frac{\partial h_1}{\partial x} = \frac{1}{2\sqrt{x^2 + y^2}} \cdot 2x = \frac{x}{\sqrt{x^2 + y^2}}
  $$
* Partial derivative w.r.t $y$:
  $$
  \frac{\partial h_1}{\partial y} = \frac{1}{2\sqrt{x^2 + y^2}} \cdot 2y = \frac{y}{\sqrt{x^2 + y^2}}
  $$
* Range does not depend on velocity components:
  $$
  \frac{\partial h_1}{\partial \dot{x}} = 0, \quad \frac{\partial h_1}{\partial \dot{y}} = 0
  $$

#### 2. Second Row: Bearing Angle Derivatives ($h_2 = \arctan(y/x)$)
Recall single-variable calculus rule $\frac{d}{du} \arctan(u) = \frac{1}{1 + u^2}$:
* Partial derivative w.r.t $x$ (using quotient rule on $u = y/x$ where $\frac{\partial u}{\partial x} = -y/x^2$):
  $$
  \frac{\partial h_2}{\partial x} = \frac{1}{1 + (y/x)^2} \cdot \left( -\frac{y}{x^2} \right) = \frac{1}{\frac{x^2 + y^2}{x^2}} \cdot \left( -\frac{y}{x^2} \right) = \frac{-y}{x^2 + y^2}
  $$
* Partial derivative w.r.t $y$ (where $\frac{\partial u}{\partial y} = 1/x$):
  $$
  \frac{\partial h_2}{\partial y} = \frac{1}{1 + (y/x)^2} \cdot \left( \frac{1}{x} \right) = \frac{x^2}{x^2 + y^2} \cdot \frac{1}{x} = \frac{x}{x^2 + y^2}
  $$
* Bearing angle does not depend on velocity components:
  $$
  \frac{\partial h_2}{\partial \dot{x}} = 0, \quad \frac{\partial h_2}{\partial \dot{y}} = 0
  $$

Putting it all together yields the **Measurement Jacobian Matrix**:

$$
\mathbf{H}_k = \begin{bmatrix}
\frac{x}{\sqrt{x^2 + y^2}} & \frac{y}{\sqrt{x^2 + y^2}} & 0 & 0 \\[8pt]
\frac{-y}{x^2 + y^2} & \frac{x}{x^2 + y^2} & 0 & 0
\end{bmatrix}_{\mathbf{x} = \hat{\mathbf{x}}_{k|k-1}}
$$

At each filter iteration, we plug our current numerical estimates $(\hat{x}_{k|k-1}, \hat{y}_{k|k-1})$ into this matrix to evaluate local linear slopes for the update step.
