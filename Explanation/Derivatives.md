# Concept: Derivatives and Partial Derivatives

## Advanced Prerequisites
- **Limits**: The idea of getting infinitely close to a point.

## Overview
A **derivative** measures the exact slope or rate of change of a curve at a specific point. A **partial derivative** is used when a curve exists in multiple dimensions (like a 3D hill) — it measures the slope in one specific direction (like pointing North) while ignoring the others.

## The Three-Stage Math Progression

### 1. Algebra Format (Slope of a Line)
The slope ($m$) of a straight line connecting two points $(x_1, y_1)$ and $(x_2, y_2)$ is "rise over run".
Equation: $m = \frac{y_2 - y_1}{x_2 - x_1}$

### 2. Calculus Format (The Derivative)
For a curve, the slope changes constantly. We find the slope at a single point $x$ by picking a second point infinitely close to it (distance $h$).
Equation: $f'(x) = \lim_{h \to 0} \frac{f(x+h) - f(x)}{h}$

**Partial Derivative:** If we have a 3D surface $z = f(x, y)$, the partial derivative with respect to $x$ (written as $\frac{\partial f}{\partial x}$) treats $y$ like a constant number.
Example derivation for $f(x, y) = x^2 \cdot y$:
- Step 1: Treat $y$ as a constant multiplier (like the number 5).
- Step 2: Take the standard derivative of $x^2$, which is $2x$.
- Step 3: Multiply them together. $\frac{\partial f}{\partial x} = 2x \cdot y$.

### 3. Linear Algebra Format (The Gradient Vector)
In higher math, we bundle all the partial derivatives of a function into a single vector called the **Gradient** ($\nabla f$).
Equation: $\nabla f = \begin{bmatrix} \frac{\partial f}{\partial x} \\ \frac{\partial f}{\partial y} \end{bmatrix}$
This vector always points in the direction of the steepest uphill slope.
