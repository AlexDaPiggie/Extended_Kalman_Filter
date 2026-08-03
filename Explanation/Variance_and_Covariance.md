# Concept: Variance and Covariance

## Advanced Prerequisites
- **Matrices and Vectors**: To hold multiple variables.
- **Expected Value**: The statistical average outcome.

## Overview
**Variance** measures how spread out a single set of numbers is from its average. **Covariance** measures how *two* variables move together. If they both go up at the same time, the covariance is positive. If one goes up while the other goes down, it is negative.

## The Three-Stage Math Progression

### 1. Algebra Format (Basic Statistics)
If you have a set of data points $X$, and their average is $\mu$:
Variance Equation: $Var(X) = \text{Average of } (X - \mu)^2$
Covariance of $X$ and $Y$: $Cov(X, Y) = \text{Average of } ( (X - \mu_X) \cdot (Y - \mu_Y) )$

### 2. Calculus Format (Continuous Random Variables)
If the data isn't separate points, but a continuous curve (probability density function $f(x)$), we use integrals instead of averages.
Equation: $Var(x) = \int_{-\infty}^{\infty} (x - \mu)^2 f(x) dx$

### 3. Linear Algebra Format (The Covariance Matrix)
When tracking a vector of many variables $X = [x_1, x_2, \dots]$, we organize all their variances and covariances into a single square matrix called $\Sigma$ (or $P$ in the Kalman Filter).
Equation: $\Sigma = \begin{bmatrix} Var(x_1) & Cov(x_1, x_2) \\ Cov(x_2, x_1) & Var(x_2) \end{bmatrix}$

**Step-by-Step Matrix Property Proof:**
Why does the diagonal hold the variances?
- Step 1: Look at the top-left cell. This is the covariance of $x_1$ with $x_1$.
- Step 2: $Cov(x_1, x_1) = \text{Average of } ( (x_1 - \mu_1) \cdot (x_1 - \mu_1) )$
- Step 3: This simplifies to $\text{Average of } (x_1 - \mu_1)^2$.
- Step 4: By definition, that is exactly the $Var(x_1)$!
