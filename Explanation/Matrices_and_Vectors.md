# Concept: Matrices and Vectors

## Advanced Prerequisites
- None (Foundational level extending basic high school algebra)

## Overview
A **vector** is simply a list of numbers. A **matrix** is a grid of numbers. We use them to keep track of multiple variables at the same time and perform math on all of them at once.

## The Three-Stage Math Progression

### 1. Algebra Format (1D Math)
In basic algebra, we work with single numbers (scalars). 
Equation: $y = a \cdot x$
Example: $y = 3 \cdot 5 = 15$

### 2. Calculus Format (Continuous Change)
While vectors aren't inherently calculus, calculus applies to vectors moving through space (like velocity vectors).
If a position vector is $r(t) = [x(t), y(t)]$, its velocity is the derivative: $v(t) = [x'(t), y'(t)]$.

### 3. Linear Algebra Format (Multi-Dimensional Math)
In linear algebra, $x$ and $y$ become vectors (columns of numbers), and $A$ becomes a matrix (a grid).
Equation: $Y = A \cdot X$

**Step-by-Step Matrix Multiplication:**
Let $A = \begin{bmatrix} 1 & 2 \\ 3 & 4 \end{bmatrix}$ and $X = \begin{bmatrix} 5 \\ 6 \end{bmatrix}$.
- Step 1: Multiply the first row of $A$ by the column $X$.
  - $(1 \cdot 5) + (2 \cdot 6) = 5 + 12 = 17$
- Step 2: Multiply the second row of $A$ by the column $X$.
  - $(3 \cdot 5) + (4 \cdot 6) = 15 + 24 = 39$
- Final Vector $Y = \begin{bmatrix} 17 \\ 39 \end{bmatrix}$.
