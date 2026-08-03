# The Extended Kalman Filter

Jul 24, 2022

—

in Maths, Tutorial

This is the third part of the series dedicated to one of the most popular sensor de-noising technique: Kalman filters. This article will explain how to model non-linear processes to improve the filter performance, something known as the Extended Kalman Filter.

You can read all the tutorials in this online course here:

* Part 1. A Gentle Introduction to the Kalman Filter

* Part 2. The Mathematics of the Kalman Filter: The Kalman Gain

* Part 3. Modelling Kalman Filters: Liner Models

* Part 4: The Extended Kalman Filter: Non-Linear Models

* Part 5. Implementing the Kalman Filter 🚧

## Introduction

At the end of the previous article, we derived the equations for a Kalman filter able to work with linear models. In a nutshell, this means that we could use such a filter for any signal or quantity which changes over time in a linear fashion. If the assumption that both the measurement and process noises follow a normal distribution, Kalman filters are proven to be optimal.

Let&#8217;s recall the current structure of the Kalman filter:

And all of the equations that have been derived so far:

Initialisation

Prediction stepHow we think the system should evolve, solely based on its model.

Correction stepThe most likely estimation of the system state, integrating the sensor data.

Iteration

You can get a feeling for how the system behaves using the interactive chart below, which gives you the ability to control the amount of noise in the process () and in the measurement ():

Scale

-

Noise

-

-

-->

0.001

0.08

The rest of this article will focus on dismantling one of the strongest limitation of the current derivation: linear models.

We are now ready to fix this introducing the so-called Extended Kalman Filter.

## Non-Linear Models

The &#8220;magic&#8221; of the Kalman lies in a simple idea: both the sensor measurements () and the best estimate so far () follow a normal distribution, and the joint probability of two normal distributions remains a normal distribution.

This means that in the next time frame the process can be repeated, since the new best estimated is once again a normal distribution.

However, updating the model alters its probability distribution. Allowing any function can break the assumption of its normal distribution. And if that fails, the guarantee of optimality fails as well.

The reason why this does not happen when using a linear model is that a linear combination of two normal distribution is a normal distribution itself. Under those assumptions, the equation for the state prediction respects the constraints that ultimately yields a good result:

 (1)    

What this means is that the &#8220;vanilla&#8221; implementation of the Kalman filter is guaranteed to be optimal only for processes which evolution can be modelled as a line. This is a very strong constraint, as many real-life processes tend to be non-linear.

In reality, what would really make a difference is the ability to use any generic function  as our model:

 (2)    

While this is not always possible using Kalman filters, there is a variant that can handle non-linear functions, as long as they are differentiable: Extended Kalman filters. Intuitively speaking, a function is differentiable if it can be drawn as a continuous, smooth line.

What an EFK does is finding a linear approximation of the function around its current estimate. So, in a way, even EKFs are still relying on a linear model. The interactive chart below show a sinusoid function; while non-linear, it can be approximated at any given point with a tangent line.

30

Such approximation can be very accurate if we stay around the tangent point. Translated to a signal, this means that approximating differentiable functions with a tangent line to their current state estimate is a good solution for short time intervals.

### Model Linearisation

To do so, the first step is to find a way to &#8220;linearise&#8221; a model around the current estimate. This means replace the non-linear model  with its tangent line at .

One way way to approach this is to recall the equation of a line in its point-slope form:

 (3)    

Such an equation defines a line which passes through the point  and has slope . In our case, the point we want is  and the slope is given by .

In order for this to work, it is necessary for the function that models the evolution of the system to be differentiable. This means that its first derivative can be calculated; a property that not all functions have. However, most well-behaved functions are differentiable in the majority of their domain.

❓ Is there a connection between the slope and the first derivative?

The slope of a line () is defined as the ratio between its rise and the run. Given any two distinct points on a line,  and , in fact:

 (4)    

We can think of the first derivative of a function about a point as an extension of the concept of slope, applicable to functions in general. The equation is pretty much the same, with the difference that the interval between the two points must be arbitrarily small. This is why the formal equation for the derivative uses a limit to make such interval as small as possible:

 (5)    

You can easily see that when the first derivative is calculated on the equation of a line, it converges to its slope. This is ultimately why we can use the value of the first derivative as the slope of the tangent line.

❓ Is there a connection with the Taylor Series?

The equation of a line presented in the point-slope form might look suspiciously familiar to the ones of you who studied Calculus. That is in fact the expansion of the Taylor series!

The Taylor series is a discrete sum of infinite terms, which can be used to approximate certain functions to an arbitrary degree of precision. Each new term in the series results in a better approximation, so the more terms, the more precisely we can model a function. The Taylor series looks like this:

 (6)    

where  is the -th derivative of the function  evaluated at .

Expanding the first two terms of the Taylor series yields the equation of tangent line. This provides the best linear approximation of a function around a point.

Thanks to this linearisation trick, we can now approximate the function  with its tangent line evaluated at :

 (7)    

It should be noticed that both  and  are actual numbers, and they correspond to the value of the model at , and the slope of its tangent line at that point. So, regardless of the complexity of , this approximation is a linear function.

We can now rearrange the terms in (7) to better reveal its linear nature, in a form that we have already encountered:

 (8)    

We can now replace  and  from the original equations for the prediction and correction steps:

 (9)    

and:

 (10)    

It is to be noted that in many derivations of the Extended Kalman Filter, you may find  still including the original non-linear function . If it is well approximated by its linear counterpart, this will not really be a problem, and any error will be amortised as a higher process noise (i.e.: less certainty on the evolution of the system).

This derivation is really needed in the calculation of , which only depends on  (the slope or tangent), and not on the entire function.

When the function is highly non-linear, even EFK can have issues adapting to its temperamental behaviour. In that case, another variant called the Unscented Kalman filter (UKF) finds ample application.

## Numerical Estimation

The Extended Kalman Filters relies on the strong assumption that we can model the evolution of the system as a differentiable function. While a system might be evolving in such a way, it does not mean we are immediately able to derive the necessary equations.

This is why Extended Kalman Filters often rely on numerical estimation of the first derivative, rather than using its actual mathematical formulation. This makes such filters able to better handle rapid changes in behaviours, at the cost of a less precise measure overall.

The only thing needed in this case is to simply use the previous two best estimates ( and ) to calculate an approximation of the first derivative as seen in (5).

The interactive chart below shows the evolution of two Extended Kalman Filters. The one on top uses the actual first derivative, while the one on the bottom approximates it numerically.

Scale

-

Noise

-

-

-->

0.02

0.08

## Further Extensions&#8230;

### Feedback Loop

The current derivation is for a Kalman filter that is &#8220;passive&#8221;, in the sense that it does not interact with the system it measures. This is often not the case: in the example of a thermostat, for instance, the observations might be used to determine whether or not to turn heating on or off.

More advanced versions of the Kalman filter also include a control factor (), which can be used to update the evolution of the model based on the action that the filter is taking. Mathematically, this is done by updating the equation that controls the process evolution:

 (11)    

In the equation above,  is known as the control-input model, and modulates the contribution of the control factor () on the process evolution.

For instance, if the Kalman filter detects that temperature is too low, it could trigger a thermostat to turn the heating on. In that case, we could set  to indicate that, with  set as a coefficient that indicates how fast the temperature is expected to raise once the heating is on.

This allows for a more complex, yet accurate prediction of the system. As seen before, the control-input model is expected to be linear, or at least differentiable if we are using an Extended Kalman Filter.

### Observation Model

For the entire duration of this series we have simply assumed that the sensor would return readings in the same scale of the original process. This is not necessarily the case, especially for electronics sensors which might register a temperature as a current drop across a resistor, rather than degrees.

Traditionally, the &#8220;complete&#8221; formulation of the Kalman filter includes a factor , which is known as the observation model. In a nutshell, it allows to remap values sampled from the sensor in the same scale and unit of the process property under examination:

 (12)    

### Extending Into Multiple Dimensions

Right now we have presented a derivation of the Kalman filter that works on scalar quantities, meaning that only works on a single numbers. In reality, however, there might be properties that we want to estimate that are multi-dimensional.  In one of its more general formulations, Kalman filters are actually presented in matrix form. Under this new framework, the positions (), the measurements () and the filter estimates () becomes vectors which can have many elements.

The position of a building, for instance, is likely going to include at least two independent variables: latitude and longitude. One could easily use two separate Kalman filter for both properties, but that is very wasteful because it completely ignores how the two coordinates are connected.

State Prediction

Let&#8217;s see a concrete example, imaging a multi-dimensional filter which measures two quantities at the same time, such as latitude and longitude:

To avoid confusion, the matrix (or vector) version of a variable is indicated in bold italic (in accordance with the ISO standard).

During the past few articles we have seen how the process evolves over time (1) as a linear combination of the previous state (or as its function, in the case of the Extended Kalman Filter):

 (13)    

This can be rethought in terms of matrices as:

 (14)    

The two expressions seem pretty much the same, but they are fundamentally different. Under this new framework,  is a square matrix, and  a vector (2&#215;2 and 2&#215;1, respectively). This simple change allows the components of the new state () to be combined together.

In the case of a static object which is not expected to move,  is going to be the identity matrix (), and  the zero vector:

 (15)    

❓ How does matrix multiplication work?

Recalling how the the matrix multiplication work, you can see that multiplying a 2&#215;2 matrix by a 2&#215;1 vector yields a 2&#215;1 vector:

 (16)    

One important fact to remember is that matrix multiplication is not commutative: this means that A x is not the same as x A. In fact, inverting the order of the terms would result in a 2&#215;2 matrix, not a 2&#215;1 vector:

 (17)    

It should also be noticed that the matrix multiplication is nothing more than a specific way to linearly combine the components of a vector. This means that, as long as the individual component are normally distributed, they will remain normally distributed after the multiplication. In fact, the property of being normally distributed is preserved by linear combination.

Extended Kalman Filter

Equation (15) expresses the state prediction step in its matrix form. However we have seen how the Extended Kalman Filter supports not just linear combinations, but any differentiable function.

Things get a bit more complex when we have to calculate the first derivative of such function  in a way that is compatible with our matrix expressions. Loosely speaking, the extension of the derivative in multiple dimensions is known as a Jacobian. This is a matrix which elements are the partial derivative of a given function, calculated with respect to each dimension of the system:

 (18)    

For instance:

 (19)    

State Update

Due to the fact that matrix multiplication is non-commutative, we should be very careful in how terms are rearranged. For instance:

 (20)    

has to be expressed in this way to ensure that the matrix multiplication yields the correct result:

 (21)    

Kalman Gain

Even the expression for Kalman gain requires some attention. In fact, scalar division needs to be replace with its matrix counterpart: multiplication by the inverse.

 (22)    

Another tricky aspect is that other scalar properties, such as the variance , are replaced by they multi-dimensional analogue: covariance matrices.

🎯 Multivariate Normal Distribution

So far we have explored the concept of normal distribution in one dimension. However, many phenomena are normally distributed, while also being multi-dimensional. For instance, the uncertain position of a building can be thought as a 2D vector, which latitude and longitude components are two separate normal distributions:

If a 1D normal distribution is visualised as a bell, its 2D equivalent looks like a cloud. If the cloud is somewhat &#8220;squished&#8221; in an oblique direction, it means that the two components of the distribution are somewhat correlated.

The variance of a multivariate distribution takes the form of a covariance matrix, which indicates the variance across all axis combinations:

 (23)    

## Conclusion

We have finally concluded the theoretical overview of the Kalman Filter, along with some of its many variants and evolutions.

Initialisation

Prediction step

Correction step

## What&#8217;s Next&#8230;

You can read all the tutorials in this online course here:

* Part 1. A Gentle Introduction to the Kalman Filter

* Part 2. The Mathematics of the Kalman Filter: The Kalman Gain

* Part 3. Modelling Kalman Filters: Liner Models

* Part 4: The Extended Kalman Filter: Non-Linear Models

* Part 5. Implementing the Kalman Filter 🚧

The next and final part of this series will focus on a simple, efficient and effective implementation of the Kalman filter in C#.

### Further Readings

* &#8220;Kalman Filter For Dummies&#8221; by Bilgin Esme

* &#8220;Kalman&#8221; by Greg Czerniak

* &#8220;Understanding the Basis of the Kalman Filter Via a Simple and Intuitive Derivation&#8221; by Ramsey Faragher

* &#8220;Extended Kalman Filter&#8221; by Lei Zhou

* &#8220;Kalman filter&#8221; by David Khudaverdyan

* &#8220;Kalman Filter Interview&#8221; by Harveen Singh

* &#8220;Kalman Filter Simulation&#8221; by Richard Teammco

* &#8220;Extended Kalman Filter: Why do we need an Extended Version?&#8221; by Harveen Singh Chadha

* &#8220;The Unscented Kalman Filter: Anything EKF can do I can do it better!&#8221; by Harveen Singh Chadha

* &#8220;A New Approach to Linear Filtering and Prediction Problems&#8221; by Rudolf E. Kálmán

arduino  filter  kalman  maths

## 💖 Support this blog

This website exists thanks to the support of patrons on Patreon. If you think my work has helped or inspired you, please consider joining!

👉

  Become a member

👈

## Comments

### 5 responses to &#8220;The Extended Kalman Filter&#8221;

* 

Sampritha Keshav

April 16, 2025

The next part(part 5) has been moved from which i cannot complete the course please update the link

Reply

* 

kalmanschmalman

July 22, 2024

I believe the scalar expression for the variance prediction in the Conclusions section is incorrect. The correct expression is derived in the Model Linearisation section.

Reply

* 

Thao

December 29, 2023

If d = 2
F (2&#215;1)
P (2&#215;2)
FT = (1&#215;2)
FxPxFT (2&#215;1)(2&#215;2)(1&#215;2)
it is not work! sir

Reply

* 

Kalman Filters: From Theory to Implementation &#8211; Alan Zucconi

March 29, 2023

[&#8230;] 4: The Extended Kalman Filter: Non-Linear [&#8230;]

Reply

* 

The Mathematics of the Kalman Filter &#8211; Alan Zucconi

August 5, 2022

[&#8230;] 4: The Extended Kalman Filter: Non-linear [&#8230;]

Reply

### Leave a Reply Cancel reply

Your email address will not be published. Required fields are marked *

Comment * 

Name * 

Email * 

Website 

">