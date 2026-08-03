Skip to content

                    August 3, 2026

		Uncategorized	

This is the second part of the tutorial on the extended (nonlinear) Kalman filter. In this tutorial, we explain how to implement the extended filter in Python. In the first tutorial part, we explained how to derive the extended Kalman filter. The YouTube videos accompanying this tutorial are given below. The GitHub page with the code used in this tutorial is given here.

The first part of the tutorial:

The second part of the tutorial

## Test Example and Discretization

To test the performance of the extended Kalman filter, we consider a pendulum system. This is a classical dynamical system and if we use the programming analogy, this example can be seen as a &#8220;Hello World&#8221; example of control engineering and control theory. In our previous tutorial given over here, we derived an equation of motion and a state space model of the pendulum system. The state-space model has the following form 

 (1)    

where  is the angle of the pendulum,  is the angular velocity,  is the gravitational acceleration constant, and  is the length of the pendulum. We assume that only the angle of the pendulum is directly measured. Consequently, the output equation is 

 (2)    

The model (1) is the continuous time form. Although we can develop the extended Kalman filter by directly using the continuous time form, the first part of this tutorial is based on the discrete-time system model. Consequently, in order to keep this second part of the tutorial consistent with the first one, we need to discretize the system (1).

The state-space model can be written in the general form

 (3)    

where 

 (4)    

We use the forward Euler discretization to approximate the first derivative

 (5)    

where  is a discrete-time instant and  is the discretization step size. From the last equation, we obtain 

 (6)    

By replacing the approximate equality sign with the strict equality, we obtain 

 (7)    

Next, by combining the expression for the function  that defined in (4) with the equation (7), we obtain 

 (8)    

The last equation can be written in the compact form

 (9)    

where 

 (10)    

Note that the function  does not depend on time. Consequently, we do not have the index  in its subscript (see the first part of this tutorial). By using the notation from the first part of this tutorial, we write the output equation (2) as follows 

 (11)    

where  is a scalar output equation

 (12)    

## Jacobian Matrix

Next, we need to compute the Jacobian matrices of the functions  and . Let us first partition the matrix 

 (13)    

where 

 (14)    

The Jacobian of the state vector function  is 

 (15)    

The Jacobian matrix of the output function  is 

 (16)    

On the basis of calculated Jacobians, we obtain the following matrices necessary for the extended Kalman filter implementation

 (17)    

## Python Code

The code presented in this tutorial are posted on the GitHub page. We wrote a Python class that implements the extended Kalman filter. First, for completeness, we present the complete class, and then we explain its functions and variables.

import numpy as np 

class ExtendedKalmanFilter(object):

    # x0 - initial guess of the state vector - this is the initial a posteriori estimate
    # P0 - initial guess of the covariance matrix of the state estimation error
    # Q  - covariance matrix of the process noise 
    # R  - covariance matrix of the measurement noise
    # dT - discretization period for the forward Euler method

    def __init__(self,x0,P0,Q,R,dT):

        # initialize vectors and matrices
        self.x0=x0
        self.P0=P0
        self.Q=Q
        self.R=R
        self.dT=dT

        # model parameters
        # gravitational constant
        self.g=9.81
        # length of the pendulum 
        self.l=1

        # this variable is used to track the current time step k of the estimator 
        # after every measurement arrives, this variables is incremented for +1 
        self.currentTimeStep=0

        # this list is used to store the a posteriori estimates hat{x}_k^{+} starting from the initial estimate 
        # note: the list starts from hat{x}_0^{+}=x0 - where x0 is an initial guess of the estimate provided by the user
        self.estimates_aposteriori=&#x5B;]
        self.estimates_aposteriori.append(x0)

        # this list is used to store the a apriori estimates hat{x}_k^{-} starting from hat{x}_1^{-}
        # note: hat{x}_0^{-} does not exist, that is, the list starts from the time index 1
        self.estimates_apriori=&#x5B;]

        # this list is used to store the a posteriori estimation error covariance matrices P_k^{+}
        # note: the list starts from P_0^{+}=P0, where P0 is the initial guess of the covariance provided by the user
        self.estimationErrorCovarianceMatricesAposteriori=&#x5B;]
        self.estimationErrorCovarianceMatricesAposteriori.append(P0)

        # this list is used to store the a priori estimation error covariance matrices P_k^{-}
        # note: the list starts from P_1^{-}, that is, it starts from the time index 1
        self.estimationErrorCovarianceMatricesApriori=&#x5B;]

        # this list is used to store the Kalman gain matrices K_k
        self.gainMatrices=&#x5B;]

        # this list is used to store prediction errors error_k=y_k-self.outputEquation(x_k^{-})
        self.errors=&#x5B;]

    # here is the continuous state-space model
    # inputs:
    #       x - state vector 
    #       t - time
    # NOTE THAT WE ARE NOT USING time since the dynamics is time invariant
    # output: 
    #       dxdt - the value of the state function (derivative of x)
    def stateSpaceContinuous(self,x,t):
        dxdt=np.array(&#x5B;&#x5B;x&#x5B;1,0]],&#x5B;-(self.g/self.l)*np.sin(x&#x5B;0,0])]])
        return dxdt

    # this function defines the discretized state-space model 
    # we use the forward Euler discretization 
    # input: 
    #       x_k   - current state x_{k}
    # output:
    #       x_kp1 - state propagated in time x_{k+1}
    def discreteTimeDynamics(self,x_k):
        # note over here that we are not using "self.currentTimeStep*self.DT" since the dynamics is time invariant
        # however, you might need to use this argument if your dynamics is time varying
        x_kp1=x_k+self.dT*self.stateSpaceContinuous(x_k,self.currentTimeStep*self.dT)
        return x_kp1

    # this function returns the Jacobian of the discrete-time state equation 
    # evaluated at x_k
    # That is, it returns the matrix A
    # input: 
    #       x_k - state 
    # output: 
    #       A - the Jacobian matrix of the state equation with respect to state
    def jacobianStateEquation(self,x_k):
        A=np.zeros(shape=(2,2))
        A&#x5B;0,0]=1
        A&#x5B;0,1]=self.dT
        A&#x5B;1,0]=-self.dT*(self.g/self.l)*np.cos(x_k&#x5B;0,0])
        A&#x5B;1,1]=1
        return A

    # this function returns the Jacobian of the output equation 
    # evaluated at x_k
    # That is, it returns the matrix C
    # Note that since in the case of the pendulum the output is a linear function 
    # and consequently, we actually do not use x_k
    # however, in the case of nonlinear output functions we need x_k
    # input: 
    #      x_k - state 
    # output: 
    #      C   - the Jacobian matrix of the output equation with respect to state
    def jacobianOutputEquation(self,x_k):
        C=np.zeros(shape=(1,2))
        C&#x5B;0,0]=1
        return C

    # this is the output equation
    # input: 
    #       x_k - state
    # output: 
    #       x_k&#x5B;0]- output value at the current state
    def outputEquation(self,x_k):
        return x_k&#x5B;0]

    # this function propagates x_{k-1}^{+} through the model to compute x_{k}^{-}
    # this function also propagates P_{k-1}^{+} through the time-covariance model to compute P_{k}^{-}
    # at the end, this function increments the time index self.currentTimeStep for +1

    def propagateDynamics(self):
        # propagate the a posteriori estimate to compute the a priori estimate
        xk_minus=self.discreteTimeDynamics(self.estimates_aposteriori&#x5B;self.currentTimeStep])
        # linearize the dynamics at the a posteriori estimate 
        Akm1=self.jacobianStateEquation(self.estimates_aposteriori&#x5B;self.currentTimeStep])
        # propagate the a posteriori covariance matrix in time to compute the a priori covariance
        Pk_minus=np.matmul(np.matmul(Akm1,self.estimationErrorCovarianceMatricesAposteriori&#x5B;self.currentTimeStep]),Akm1.T)+self.Q

        # memorize the computed values and increment the time step
        self.estimates_apriori.append(xk_minus)
        self.estimationErrorCovarianceMatricesApriori.append(Pk_minus)
        self.currentTimeStep=self.currentTimeStep+1

    # this function computes the a posteriori estimate by using the measurements
    # this function should be called after propagateDynamics() because the time step should be increased and states and covariances should be propagated         
    # input:
    #       currentMeasurement - measurement at the time step k
    def computeAposterioriEstimate(self,currentMeasurement):

        # linearize the output equation at the a priori estimate for the time step k
        Ck=self.jacobianOutputEquation(self.estimates_apriori&#x5B;self.currentTimeStep-1]) 

        # compute the Kalman gain matrix
        # keep in mind that the a priori indices start from k=1, that is why we index a priori quantities with "self.currentTimeStep-1"
        Smatrix= self.R+np.matmul(np.matmul(Ck,self.estimationErrorCovarianceMatricesApriori&#x5B;self.currentTimeStep-1]),Ck.T)
        # Kalman gain matrix
        Kk=np.matmul(self.estimationErrorCovarianceMatricesApriori&#x5B;self.currentTimeStep-1],np.matmul(Ck.T,np.linalg.inv(Smatrix)))

        # update the estimate
        # prediction error
        error_k=currentMeasurement-self.outputEquation(self.estimates_apriori&#x5B;self.currentTimeStep-1])
        # a posteriori estimate
        xk_plus=self.estimates_apriori&#x5B;self.currentTimeStep-1]+np.matmul(Kk,np.array(&#x5B;error_k]))

        # update the covariance matrix
        # a posteriori covariance matrix update 
        IminusKkC=np.eye(self.x0.shape&#x5B;0])-np.matmul(Kk,Ck)
        Pk_plus=np.matmul(IminusKkC,np.matmul(self.estimationErrorCovarianceMatricesApriori&#x5B;self.currentTimeStep-1],IminusKkC.T))+np.matmul(Kk,np.matmul(self.R,Kk.T))

        # update the lists that store the vectors and matrices
        # Kalman gain matrix
        self.gainMatrices.append(Kk)
        # errors
        self.errors.append(error_k)
        # a posteriori estimates
        self.estimates_aposteriori.append(xk_plus)
        # a posteriori covariance matrix
        self.estimationErrorCovarianceMatricesAposteriori.append(Pk_plus)

First, we define the __init__() function that is repeated over here for completness

def __init__(self,x0,P0,Q,R,dT):

        # initialize vectors and matrices
        self.x0=x0
        self.P0=P0
        self.Q=Q
        self.R=R
        self.dT=dT

        # model parameters
        # gravitational constant
        self.g=9.81
        # length of the pendulum 
        self.l=1

        # this variable is used to track the current time step k of the estimator 
        # after every measurement arrives, this variables is incremented for +1 
        self.currentTimeStep=0

        # this list is used to store the a posteriori estimates hat{x}_k^{+} starting from the initial estimate 
        # note: the list starts from hat{x}_0^{+}=x0 - where x0 is an initial guess of the estimate provided by the user
        self.estimates_aposteriori=&#x5B;]
        self.estimates_aposteriori.append(x0)

        # this list is used to store the a apriori estimates hat{x}_k^{-} starting from hat{x}_1^{-}
        # note: hat{x}_0^{-} does not exist, that is, the list starts from the time index 1
        self.estimates_apriori=&#x5B;]

        # this list is used to store the a posteriori estimation error covariance matrices P_k^{+}
        # note: the list starts from P_0^{+}=P0, where P0 is the initial guess of the covariance provided by the user
        self.estimationErrorCovarianceMatricesAposteriori=&#x5B;]
        self.estimationErrorCovarianceMatricesAposteriori.append(P0)

        # this list is used to store the a priori estimation error covariance matrices P_k^{-}
        # note: the list starts from P_1^{-}, that is, it starts from the time index 1
        self.estimationErrorCovarianceMatricesApriori=&#x5B;]

        # this list is used to store the Kalman gain matrices K_k
        self.gainMatrices=&#x5B;]

        # this list is used to store prediction errors error_k=y_k-self.outputEquation(x_k^{-})
        self.errors=&#x5B;]

The input variables are 

* &#8220;x0&#8221; is the initial guess of the a posteriori state  that is selected by the user. 

* &#8220;P0&#8221; is the initial guess of the a posteriori covariance matrix  that is selected by the user.

* &#8220;Q&#8221; is the covariance matrix of the process noise. 

* &#8220;R&#8221; is the covariance matrix of the measurement noise.

* &#8220;dT&#8221; is the discretization time constant  in (5).

On the code lines 22 and 24 we define the constants  and . Next, we explain the variables that are created and initialized by this function

* &#8220;self.currentTimeStep&#8221; is used to memorize the current time step .

* &#8220;self.estimates_aposteriori&#8221; is the list used to store the a posteriori estimates 

* &#8220;self.estimates_apriori&#8221; is the list used to store the a priori estimates 

* &#8220;self.estimationErrorCovarianceMatricesAposteriori&#8221; is the list used to store 

* &#8220;self.estimationErrorCovarianceMatricesApriori&#8221; is the list used to store 

* &#8220;self.gainMatrices&#8221; is the list used to store the gain matrices 

* &#8220;self.errors&#8221; is the list used to store the errors 
 (18)    

The right-hand side of the continuous time state equation (1) is defined by 

    def stateSpaceContinuous(self,x,t):
        dxdt=np.array(&#x5B;&#x5B;x&#x5B;1,0]],&#x5B;-(self.g/self.l)*np.sin(x&#x5B;0,0])]])
        return dxdt

The discrete-time dynamics (8) is shifted one step in time, and is defined by the following function

    def discreteTimeDynamics(self,x_k):
        # note over here that we are not using "self.currentTimeStep*self.DT" since the dynamics is time invariant
        # however, you might need to use this argument if your dynamics is time varying
        x_kp1=x_k+self.dT*self.stateSpaceContinuous(x_k,self.currentTimeStep*self.dT)
        return x_kp1

The Jacobian matrix (15) of the state equation is defined by 

    def jacobianStateEquation(self,x_k):
        A=np.zeros(shape=(2,2))
        A&#x5B;0,0]=1
        A&#x5B;0,1]=self.dT
        A&#x5B;1,0]=-self.dT*(self.g/self.l)*np.cos(x_k&#x5B;0,0])
        A&#x5B;1,1]=1
        return A

The Jacobian matrix (16) of the output equation is defined by 

    def jacobianOutputEquation(self,x_k):
        C=np.zeros(shape=(1,2))
        C&#x5B;0,0]=1
        return C

The output equation (2) is defined by 

    def outputEquation(self,x_k):
        return x_k&#x5B;0]

The first step of the Kalman filter (see the first part of the tutorial) is to propagate a posteriori estimate and covariance matrix through the dynamics:

 (19)    

This is achieved by the following function

    def propagateDynamics(self):
        # propagate the a posteriori estimate to compute the a priori estimate
        xk_minus=self.discreteTimeDynamics(self.estimates_aposteriori&#x5B;self.currentTimeStep])
        # linearize the dynamics at the a posteriori estimate 
        Akm1=self.jacobianStateEquation(self.estimates_aposteriori&#x5B;self.currentTimeStep])
        # propagate the a posteriori covariance matrix in time to compute the a priori covariance
        Pk_minus=np.matmul(np.matmul(Akm1,self.estimationErrorCovarianceMatricesAposteriori&#x5B;self.currentTimeStep]),Akm1.T)+self.Q

        # memorize the computed values and increment the time step
        self.estimates_apriori.append(xk_minus)
        self.estimationErrorCovarianceMatricesApriori.append(Pk_minus)
        self.currentTimeStep=self.currentTimeStep+1

Note that in this step we also update the appropriate lists and the current time step .

In the second step, we calculate 

 (20)    

 (21)    

 (22)    

These steps are implemented by the following function

 def computeAposterioriEstimate(self,currentMeasurement):

        # linearize the output equation at the a priori estimate for the time step k
        Ck=self.jacobianOutputEquation(self.estimates_apriori&#x5B;self.currentTimeStep-1]) 

        # compute the Kalman gain matrix
        # keep in mind that the a priori indices start from k=1, that is why we index a priori quantities with "self.currentTimeStep-1"
        Smatrix= self.R+np.matmul(np.matmul(Ck,self.estimationErrorCovarianceMatricesApriori&#x5B;self.currentTimeStep-1]),Ck.T)
        # Kalman gain matrix
        Kk=np.matmul(self.estimationErrorCovarianceMatricesApriori&#x5B;self.currentTimeStep-1],np.matmul(Ck.T,np.linalg.inv(Smatrix)))

        # update the estimate
        # prediction error
        error_k=currentMeasurement-self.outputEquation(self.estimates_apriori&#x5B;self.currentTimeStep-1])
        # a posteriori estimate
        xk_plus=self.estimates_apriori&#x5B;self.currentTimeStep-1]+np.matmul(Kk,np.array(&#x5B;error_k]))

        # update the covariance matrix
        # a posteriori covariance matrix update 
        IminusKkC=np.eye(self.x0.shape&#x5B;0])-np.matmul(Kk,Ck)
        Pk_plus=np.matmul(IminusKkC,np.matmul(self.estimationErrorCovarianceMatricesApriori&#x5B;self.currentTimeStep-1],IminusKkC.T))+np.matmul(Kk,np.matmul(self.R,Kk.T))

        # update the lists that store the vectors and matrices
        # Kalman gain matrix
        self.gainMatrices.append(Kk)
        # errors
        self.errors.append(error_k)
        # a posteriori estimates
        self.estimates_aposteriori.append(xk_plus)
        # a posteriori covariance matrix
        self.estimationErrorCovarianceMatricesAposteriori.append(Pk_plus)

This function also updates the appropriate lists. This class should be saved in the file called &#8220;ExtendedKalmanFilter.py&#8221;

Next, we present the driver code that explains how to use this class.  The following code lines import the necessary libraries, simulate the pendulum dynamics by using the odeint() function and the forward Euler method. This is done to quantify the model differences between very precise simulation of the dynamics and simulation by using the forward Euler method that is used to develop the dynamics. Note that we are using the simulations obtained by using the odeint() function as &#8220;true&#8221; measurements. In this tutorial, we explain how to simulate the dynamics by using the odeint() function. 

import numpy as np
import matplotlib.pyplot as plt

# this function is used to integrate the dynamics
from scipy.integrate import odeint
# in this class we implement the extended Kalman filter
from ExtendedKalmanFilter import ExtendedKalmanFilter

# discretization time step
# it is used for both integration of the pendulum differential equation and for
# forward Euler method discretization
deltaTime=0.01
# initial condition for generating the simulation data 
x0=np.array(&#x5B;np.pi/3,0.2])

# time steps for simulation
simulationSteps=400
# total simulation time 
totalSimulationTimeVector=np.arange(0,simulationSteps*deltaTime,deltaTime)

# this state-space model defines the continuous dynamics of the pendulum
# this function is passed as an argument of the odeint() function for integrating (solving) the dynamics 
def stateSpaceModel(x,t):
    g=9.81
    # 
    l=1
    dxdt=np.array(&#x5B;x&#x5B;1],-(g/l)*np.sin(x&#x5B;0])])
    return dxdt

# here we integrate the dynamics
# the output: "solutionOde" contains time series of the angle and angular velocity 
# these time series represent the time series of the true state that we want to estimate
solutionOde=odeint(stateSpaceModel,x0,totalSimulationTimeVector)

# uncomment this if you want to plot the simulation results 
#plt.plot(totalSimulationTimeVector, solutionOde&#x5B;:, 0], &#039;b&#039;, label=&#039;x1&#039;)
#plt.plot(totalSimulationTimeVector, solutionOde&#x5B;:, 1], &#039;g&#039;, label=&#039;x2&#039;)
#plt.legend(loc=&#039;best&#039;)
#plt.xlabel(&#039;time&#039;)
#plt.ylabel(&#039;x1(t), x2(t)&#039;)
#plt.grid()
#plt.savefig(&#039;simulation.png&#039;,dpi=600)
#plt.show()

# here we compare the forward Euler discretization with the odeint()
# this is important for evaluating the accuracy of the forward Euler method 

forwardEulerState=np.zeros(shape=(simulationSteps,2))
# set the initial states to match the initial state used in the odeint()
forwardEulerState&#x5B;0,0]=x0&#x5B;0]
forwardEulerState&#x5B;0,1]=x0&#x5B;1]

# propagate the forward Euler dynamics
for timeIndex in range(simulationSteps-1):
    forwardEulerState&#x5B;timeIndex+1,:]=forwardEulerState&#x5B;timeIndex,:]+deltaTime*stateSpaceModel(forwardEulerState&#x5B;timeIndex,:],timeIndex*deltaTime)

# plot the comparison results
plt.plot(totalSimulationTimeVector, solutionOde&#x5B;:, 0], &#039;r&#039;, linewidth=3, label=&#039;Angle - ODEINT&#039;)
plt.plot(totalSimulationTimeVector, forwardEulerState&#x5B;:, 0], &#039;b&#039;, linewidth=2, label=&#039;Angle- Forward Euler&#039;)
plt.legend(loc=&#039;best&#039;)
plt.xlabel(&#039;time &#x5B;s]&#039;)
plt.ylabel(&#039;Angle-x1(t)&#039;)
plt.grid()
plt.savefig(&#039;comparison.png&#039;,dpi=600)
plt.show()

This code produces the following graph

Figure 1: Comparison of the odeint() integration with the forward Euler discretization.

We can see that the forward Euler method sufficiently accurately discretizes the dynamics.

Next, we select the initial estimate and the initial covariance matrix, simulate the Kalman filter, and plot the results. This is done by using the following code

#create the Kalman filter object 
# this is an initial guess of the state estimate
x0guess=np.zeros(shape=(2,1))
x0guess&#x5B;0]=x0&#x5B;0]+4*np.random.randn()
x0guess&#x5B;1]=x0&#x5B;1]+4*np.random.randn()

# initial value of the covariance matrix
P0=10*np.eye(2,2)
# discretization stpe
dT=deltaTime
# process noise covariance matrix
# note that we do not have the process noise
Q=0.0001*np.eye(2,2)
# measurement noise covariance matrix
# note that we do not have measurement noise in this simulation 
# see driverCodeNoise.py for the performance when the measurement noise
# is affecting the outputs
R=np.array(&#x5B;&#x5B;0.0001]])

# create the extended Kalman filter object
KalmanFilterObject=ExtendedKalmanFilter(x0guess,P0,Q,R,dT)

# simulate the extended Kalman filter 
for j in range(simulationSteps-1):
    # TWO STEPS
    # (1) propagate a posteriori estimate and covariance matrix
    KalmanFilterObject.propagateDynamics()

    # (2) take into account the current measurement and 
    # compute the a posteriori estimate and covarance matrix
    # note that we use the exact solution of the differential 
    # equations as measurements
    # note also that we only measure the angle of the pendulum
    KalmanFilterObject.computeAposterioriEstimate(solutionOde&#x5B;j, 0])

# Estimates
#KalmanFilterObject.estimates_aposteriori
# Covariance matrices
#KalmanFilterObject.estimationErrorCovarianceMatricesAposteriori
# Kalman gain matrices
#KalmanFilterObject.gainMatrices.append(Kk)
# errors
#KalmanFilterObject.errors.append(error_k)

# extract the state estimates in order to plot the results
estimateAngle=&#x5B;]
estimateAngularVelocity=&#x5B;]
for j in np.arange(np.size(totalSimulationTimeVector)):
    estimateAngle.append(KalmanFilterObject.estimates_aposteriori&#x5B;j]&#x5B;0,0])
    estimateAngularVelocity.append(KalmanFilterObject.estimates_aposteriori&#x5B;j]&#x5B;1,0])

# create vectors corresponding to the true values in order to plot the results
trueAngle=solutionOde&#x5B;:,0]
trueAngularVelocity=solutionOde&#x5B;:,1]

# plot the results
steps=np.arange(np.size(totalSimulationTimeVector))
fig, ax = plt.subplots(2,1,figsize=(10,15))
ax&#x5B;0].plot(steps,trueAngle,color=&#039;red&#039;,linestyle=&#039;-&#039;,linewidth=6,label=&#039;True angle&#039;)
ax&#x5B;0].plot(steps,estimateAngle,color=&#039;blue&#039;,linestyle=&#039;-&#039;,linewidth=3,label=&#039;Estimate of angle&#039;)
ax&#x5B;0].set_xlabel("Discrete-time steps k",fontsize=14)
ax&#x5B;0].set_ylabel("Angle",fontsize=14)
ax&#x5B;0].tick_params(axis=&#039;both&#039;,labelsize=12)
ax&#x5B;0].grid()
ax&#x5B;0].legend(fontsize=14)

ax&#x5B;1].plot(steps,trueAngularVelocity,color=&#039;red&#039;,linestyle=&#039;-&#039;,linewidth=6,label=&#039;True angular velocity&#039;)
ax&#x5B;1].plot(steps,estimateAngularVelocity,color=&#039;blue&#039;,linestyle=&#039;-&#039;,linewidth=3,label=&#039;Angular velocity estimate&#039;)
ax&#x5B;1].set_xlabel("Discrete-time steps k",fontsize=14)
ax&#x5B;1].set_ylabel("Angular Velocity",fontsize=14)
ax&#x5B;1].tick_params(axis=&#039;both&#039;,labelsize=12)
ax&#x5B;1].grid()
ax&#x5B;1].legend(fontsize=14)
fig.savefig(&#039;estimationResults.png&#039;,dpi=600)

The results are shown below. 

 We can observe that the initial guess quickly converges to the true system states. The Kalman filter is created as follows 

KalmanFilterObject=ExtendedKalmanFilter(x0guess,P0,Q,R,dT)

The Kalman filter is simulated by using the following loop 

for j in range(simulationSteps-1):
    # TWO STEPS
    # (1) propagate a posteriori estimate and covariance matrix
    KalmanFilterObject.propagateDynamics()

    # (2) take into account the current measurement and 
    # compute the a posteriori estimate and covarance matrix
    # note that we use the exact solution of the differential 
    # equations as measurements
    # note also that we only measure the angle of the pendulum
    KalmanFilterObject.computeAposterioriEstimate(solutionOde&#x5B;j, 0])

First, we propagate the dynamics, and then we compute the a posteriori state estimates by using the measurements. 

        admin

            View all posts by admin &rarr;        

### You might also like

### 

                        Linear Quadratic Regulator (LQR) for Non-Zero Set-Points in Python by Using Control Systems Library                    

                August 1, 2023August 1, 2023

### 

                        What is the difference between function declaration and definition in C or C++?                    

                July 19, 2024April 9, 2026

### 

                        Review of the Methods of Statics                    

                August 21, 2021August 22, 2021

This website contains more than 250 free tutorials! Every tutorial is accompanied by a YouTube video. All the tutorials are completely free. The website has more than 50,000 visitors per month from all over the world! 

					Search for:

		Recent Posts

* 
					Download SPY Stocks Time Series and Compute the RSI in Python

* 
					Derive and Draw by Hand Bode plots of Second order Transfer Functions

* 
					Model and Simulate Gears and Gear Assemblies in FreeCAD

* 
					How to Model Gear Assemblies in FreeCAD

* 
					How to create gears in FreeCAD

		Recent Comments
* admin on Deep Q Networks (DQN) in Python From Scratch by Using OpenAI Gym and TensorFlow- Reinforcement Learning Tutorial
* Mike on Deep Q Networks (DQN) in Python From Scratch by Using OpenAI Gym and TensorFlow- Reinforcement Learning Tutorial
* admin on Easy Introduction to Observability and Open-loop Observers with MATLAB Implementation
* Cakan on Easy Introduction to Observability and Open-loop Observers with MATLAB Implementation
* admin on Simple and Easy-to-Understand Introduction to Recurrent Neural Networks for Time-Series Prediction in Keras and TensorFlowArchives

* July 2026

* February 2026

* January 2026

* December 2025

* August 2025

* July 2025

* June 2025

* March 2025

* February 2025

* January 2025

* December 2024

* November 2024

* October 2024

* September 2024

* August 2024

* July 2024

* June 2024

* May 2024

* April 2024

* March 2024

* February 2024

* January 2024

* December 2023

* November 2023

* October 2023

* September 2023

* August 2023

* July 2023

* June 2023

* May 2023

* April 2023

* March 2023

* February 2023

* January 2023

* December 2022

* November 2022

* October 2022

* September 2022

* August 2022

* July 2022

* June 2022

* May 2022

* April 2022

* March 2022

* February 2022

* January 2022

* December 2021

* November 2021

* October 2021

* September 2021

* August 2021

* June 2021

* April 2021

* March 2021

* February 2021

* January 2021

* December 2020

* November 2020

* October 2020

* September 2020

* July 2020

* June 2020

* May 2020

* March 2020

* January 2020

* December 2019

* November 2019

* September 2019

* August 2019

* July 2019

* June 2019

* May 2019

Categories

* 3D Modeling

* ABB Robotics

* ABB RobotStudio

* aerospace tutorial

* AI Agents

* Alibaba

* Anaconda

* AP Physics

* Arduino

* Automation

* Blender

* Bode plots

* C/C++

* CAD

* Calculus

* CapCut tutorial

* CapCut Tutuorial

* cmake

* ComfyUI

* Computer Science

* Computer Vision

* Conda

* Control Systems Lectures

* Control Theory

* Control/Estimation

* CUDA

* Dash

* data science

* DEepSeek

* DeepSeek Janus

* DeepSeek Janus-Pro

* DeepSeek-R1

* DeepSeek-V3

* Digital Signal Processing

* Docker

* drones

* Dynamics

* Electrical Engineering

* Electronics

* Engineering/FEM

* Finance and stocks

* fluid mechanics

* Flux

* FPGA

* FreeCAD

* FTX-Video

* Gazebo

* Gazebo Harmonic

* Gemma

* gnuplot

* Google

* Graph Theory

* gym-aloha

* Hugging Face

* Hunyuan3D

* Image to Video

* Industrial Robotics

* Inkscape

* Internet

* Kalman Filter

* Kazam

* Kinematics

* LangChain

* Latex

* lidar

* Linux

* Linux Ubuntu

* Llama

* Llama 3.1

* Llama 3.2

* Llama 3.3

* llama.cpp

* LLM

* LocalAI

* localization

* LTspice

* Machine Learning

* Machine Learning/Data Science

* Math/Optimization

* Mathematics

* MATLAB

* matplotlib

* mechanical engineering

* mechanics

* Mechatronics/Robotics

* Microsoft Office

* Mistral

* Mobile Robots

* Model Predictive Control

* MoveIt 2

* Multimodal Model

* n8n AI agents

* Node.js

* Nonlinear Systems

* Numerical Computing

* Numpy

* NVIDIA

* OCR

* Ollama

* olmOCR-7B

* Open WebUI

* OpenAI

* OpenAI Gym

* OpenCV

* OpenThinker

* Operational Amplifiers

* Optimal Control

* Optimization

* Pandas

* Particle Filters

* Phi LLM

* Physics

* PLC

* Programming

* Pygame

* Python

* PyTorch

* quadcopter

* qwen

* QwQ-32B

* RAG

* Raspberry Pi

* Reinforcement Learning

* ROS

* ROS/ROS2

* ROS2

* ROS2 Humble

* ROS2 Jazzy

* Scientific Computing

* Scientific plotting

* Serial Communication

* Signal Processing

* Simulink

* SLAM

* Sliding mode control

* smolagents

* Spyder

* Statistics

* STM32 microcontroller

* Streamlit

* strength of materials

* SymPy

* System Identification

* Teensy

* Text to Video

* Text-to-Image

* Text-To-Speech

* time series

* Tinkercad

* Transfer function

* Ubuntu

* Uncategorized

* Verilog

* Video Editing

* Visual language

* Vivado

* VLC Media Player

* VS Code

* WebUI

* Windows

* WSL

* YOLO

			Meta

* Log in

* Entries feed

* Comments feed

* WordPress.org

				Go to mobile version