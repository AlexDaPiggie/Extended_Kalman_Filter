# Concept: Taylor Series Expansion

## Advanced Prerequisites
- **Derivatives**: Knowing how to find the slope of a curve.

## Overview
The **Taylor Series** is a mathematical tool that lets us approximate a complicated, curvy function by adding up simpler polynomial pieces (like lines and parabolas) based on the function's derivatives at a specific point.

## The Three-Stage Math Progression

### 1. Algebra Format (Linear Approximation)
If you know a function's value at point $a$ and its slope $m$, you can estimate a nearby point $x$ using a straight line.
Equation: $y \approx y_{start} + m \cdot (x - a)$

### 2. Calculus Format (The Full Taylor Series)
To make the approximation perfectly wrap around the curve, we don't just use the first slope (1st derivative). We add the "curvature" (2nd derivative), and so on infinitely.
Equation: $f(x) = f(a) + f'(a)(x-a) + \frac{f''(a)}{2}(x-a)^2 + \frac{f'''(a)}{6}(x-a)^3 + \dots$

**Step-by-step derivation of the 1st Order (Linear) Taylor Approximation:**
- Step 1: Start with the full infinite series.
- Step 2: Assume $x$ is extremely close to $a$.
- Step 3: Because $(x-a)$ is tiny (e.g., 0.01), squaring it makes it incredibly tiny (0.0001).
- Step 4: We can safely throw away the squared terms and everything after it for a "good enough" approximation.
- Result: $f(x) \approx f(a) + f'(a)(x-a)$. This is the exact trick the EKF uses to straighten curves!

### 3. Linear Algebra Format (Multivariable Taylor Series)
If our function has many input variables (a vector $X$) and outputs a vector, we replace the 1st derivative with a matrix of slopes (the **Jacobian Matrix**, $J$).
Equation: $F(X) \approx F(A) + J \cdot (X - A)$
