![RCTVC](Image/Main_page.png)
***
## Direct by Ruherpan

## Intro

A Two wheel robot maintains stability by moving its wheels. The advantage of a Two wheel robot is its ability to perform a pivot turn, allowing it to turn with a zero turning radius. However, before analyzing Two wheel robot, we must first understand how to control an inverted pendulum system, which is inherently unstable, to achieve stability. The inverted
pendulum system is a classic example of an unstable system. It consists of a pendulum with its center of gravity above its central point, This means that without a balancing force, the system easily falls over. To achieve stability, control inputs must continuously adjust the system position to prevent the pendulum from falling due to gravity. This requires precise and responsive control mechanisms. After analyzing the inverted pendulum system, we can then analyze the Two wheel robot. By examining the Two wheel robot system, we can derive the equations for control inputs necessary to achieve stability. Once we define the control inputs for the Two wheel robot, we can use the Control Moment Gyro (CMG) to control the robot.

## Inverted pendulum with cart System modelling
![RCTVC](Image/Inverted_pendulum_with_cart.png)

**Figure 1:** Inverted pendulum on cart

### Inverted pendulum on cart Parameters

|Symbol|Description|Value|
|:---|:---|:---|
|g|Gravity (m/s^2)|9.81|
|M|Mass of the cart (kg)|2|
|m|Mass of the pendulum (kg)|0.5|
|L|Length of pendulum (m)|0.3|
|r|Radius of the pendulum (m)|0.05|
|I|Moment of inertia|$I = \frac{(mr^2)}{2}$|

**Table 1: Inverted pendulum on cart Parameters**

Analyzing the system by dividing it into stick and cart

**Newtons’ second law for the center of mass of a stick**

- $\sum F_{x,m}:  R_x = m \ddot{x}_m$

- $\sum F_{y,m}:  R_y - mg = m \ddot{y}_m$

**Newton's second law for cart movement in x direction**

- $\sum F_x:  M \ddot{x} + R_x = F$

**Torque equation for the center of mass of the stick**

- $\sum \tau: R_x \frac{L}{2} \cos\theta + R_y \frac{L}{2} \sin\theta = I \ddot{\theta}$

**Relationship between x and $x_m$**

- $x_m = x - \frac{L}{2} \sin \theta$

- $y_m = \frac{L}{2} \cos \theta$

The equation of motion that defines the dynamics of the 'Inverted pendulum on cart' can be defined as: 

- $I \ddot{\theta} = \frac{L}{2} m \ddot{x} \cos \theta + \frac{L}{2} mg \sin \theta - \left(\frac{L}{2}\right)^2 m \ddot{\theta}$

**Equation 1**

- $F = (M + m) \ddot{x} + \frac{L}{2} m \sin \theta \cdot \dot{\theta}^2 - \frac{L}{2} m \cos \theta \cdot \ddot{\theta}$

**Equation 2**

If we rearrange this equation in terms of $\ddot{x}$ and $\ddot{\theta}$ :

- $\ddot{x} = \frac{1}{(M + m)} \left[ F - \frac{L}{2} m \sin \theta \cdot \dot{\theta}^2 + \frac{L}{2} m \cos \theta \cdot \ddot{\theta} \right]$

- $\ddot{\theta} = \frac{1}{a} \left( \frac{L}{2} m \ddot{x} \cos \theta + \frac{L}{2} mg \sin \theta \right) \qquad \qquad  \therefore a = I + \left( \frac{L}{2} \right)^2 m$

$\ddot{x}$ and $\ddot{\theta}$ can be defined as:

- $\ddot{x} = \frac{1}{b} \left[ Fa - a \frac{L}{2} m \sin \theta \cdot \dot{\theta}^2 + \left( \frac{L }{2} m \right)^2 g \sin \theta \cos \theta \right] \qquad \qquad \therefore b = a(M + m) - \left( \frac{L}{2} m \right)^2 \cos^2 \theta$

**Equation 3**

- $\ddot{\theta} = \frac{1}{b} \left[ F \frac{L}{2} m \cos \theta - \left( \frac{L}{2} m \right)^2 \ sin \theta \cos \theta \cdot \dot{\theta}^2 + (M + m) \frac{L}{2} mg \sin \theta \right] $

**Equation 4**

## Two wheel robot system modelling System modelling

![RCTVC](Image/Robot.png)

**Figure 2:** Two wheel robot

### Two wheel robot Parameters

|Symbol|Description|Value|
|:---|:---|:---|
|g|Gravity (m/s^2)|9.81|
|M|Mass of the cart (kg)|2|
|m|Mass of the pendulum (kg)|0.5|
|L|Length of pendulum (m)|0.3|
|$r_w$|Radius of the wheel (m)|0.03|
|$r_b$|Radius of the robot body (m)|0.03|
|$I_b$|Moment of inertia of the robot body|$I = \frac{(m(r_b)^2)}{2}$|
|$C_{\alpha}$|viscosity coefficient of the wheel|0.00055|

**Table 2: Two wheel robot Parameters**

Using the same method we use to analyze the Inverted pendulum on a cart, we can derive the equation of a Two wheel robot. The equation of motion that defines the dynamics of the 'Two wheel robot' can be defined as follows($f_{\text{ext}}$ = external force):

**Relationship between x and $x_m$**

- $x_m = x + \frac{L}{2} \sin \theta$

- $y_m = \frac{L}{2} \cos \theta$

**Newton's second law for cart movement in x direction**
Since $x$ is the length of the arc, it has the relationship $x=ra$. we can get the following equation:

- $\left( m_w + m_b + \frac{I_w}{r^2} \right) \ddot{x} = m_j l \sin \theta \cdot \dot{\theta}^2 - m_j l \cos \theta \cdot \ddot{\theta} + \frac{\tau}{r} + f_w$

**Torque equation for the center of mass of the Robot**

- $I \ddot{\theta} = l_b m_b \ddot{x}_w \cos \theta + l_b m_b g \sin \theta - l\ _b^2m_b \cdot \ddot{\theta}$

**Equation 5**

- $F_w = (m_w + m_b) \ddot{x_w} + l_b m_b \sin \theta \cdot \dot{\theta}^2 - l_b m_b \cos \theta \cdot \ddot{\theta} +f_{\text{ext}}$

To convert the above equation into an equation for the torque acting on the wheel, both sides must be multiplied by $r$. Since $x$ is the length of the arc, it has the relationship $x=ra$. we can get the following equation:

- $\tau_w = (m_w + m_b) r^2 \ddot{\alpha} + r m_b l_b \sin \theta \cdot \dot{\theta}^2 - r m_b l_b \cos \theta \cdot \ddot{\theta} + f_{\text{ext}}$

**Equation 6**

Linearizing equations 5 and 6 leads to the following equations:

- $\left( I_b + m_b l_b^2 \right) \ddot{\theta} = r l_b m_b \ddot{\alpha} + l_b m_b g \cdot \theta$

**Equation 7**

- $\tau_w = \left( m_w + m_b \right) r^2 \ddot{\alpha} - r m_b l_b \ddot{\theta} + f_{\text{ext}}$

**Equation 8**

If $f_{\text{ext}}$ is the viscous torque that occurs when the wheel rotates, we get the following equation by 
using **Equation 8**:

- $u_{\alpha} = \left[ \left( m_w + m_b \right) r^2 + I_w \right] \ddot{\alpha} - r m_b l_b \ddot{\theta} + C_{\alpha} \dot{\alpha}$

**Equation 9**

## Two wheel robot with CMG system modelling

### Two wheel robot with CMG Parameters

## PID Controller

### PID (Proportional Integral Derivative) Control

A PID controller is basically known as feedback controller. PID controller can control continuously changing objects. PID controller continuously calculates the error value $e(t)$. It can be defined as the difference between desired point $x_d$ and a measured variable $x$. ($e(t) = x_d - x$) 

The PID controller follows the equation below:

- $u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) d\tau + K_d \frac{de(t )}{dt}$ 

**Equation 10**

![RCTVC](Image/PID.png)

**Figure 3:** PID 

As you can see in Figure 3, the PID system automatically apply accurate and responsive correction to a control funtion. To control a object using PID, u is the control input for controlling the object. For example, if you want to control the andgle, denoted as $\theta$, of an inverted pendulum, the error value $e(t)$ would be $e(t) = \theta_d - \theta$. By using the error value in **Equation 10**, you can get the control input u

### PID Tuning

|Control Type|$K_p$|$T_i$|$T_d$|
|:---:|:---:|:---:|:---:|
|P|$0.5K_u$|$\infty$|0|
|PI|$0.45K_u$|$\frac{1}{1.2} T_u$|0|
|PID|$0.6K_u$|$0.5T_u$|$0.125T_u$|


**Table 3:  Ziegler Nichols method**

We will use Ziegler Nichols method for PID tuning. The Ziegler Nichols method is a technique for obtaining gain value using an experimental optimization approach. I will show you how to perform PID tuning using the 'Inverted Pendulum on Cart' as described in **Equation 4**

1. Set ${K_i} , {K_d} = 0$, and find the value of constant amplitude through $K_p$ value. The cycle is $T_u = 1.7$ and $K_p = K_u = 30$ 

![RCTVC](Image/kp_tuning.jpg)

**Figure 4:** constant amplitude graph through ${K_p}$

2. If we organize **Equation 10** we can get $K_i = \frac{K_p}{T_i}, K_d = K_p T_d$ and this can be defined as: 

- $K_i = 1.2 \frac{K_u}{T_u}, K_d = 0.075 K_u T_u$

3. If we calculate the above equation, we can get $K_i$ and $K_d$

- $K_p = 30, K_i = 21, K_d = 3.825$

![RCTVC](Image/PID_tuning.jpg)

**Figure 5:** Inverted Pendulum on Cart PID control

## Inverted Pendulum on Cart Simulation

![RCTVC](Image/simulation/Inverted_Pendulum_on_Cart_u0.jpg) 

**Figure 6:** Inverted Pendulum on Cart u0

![RCTVC](Image/simulation/Inverted_Pendulum_on_Cart_PD.jpg)

**Figure 7:** Inverted Pendulum on Cart PD

![RCTVC](Image/simulation/Inverted_Pendulum_on_Cart_PID.jpg) 

**Figure 8:** Inverted Pendulum on Cart PID

## Two wheel robot

The dynamics of a Two wheel robot are not much different from an inverted pendulum, and its mechanism is based on an inverted pendulum.
To analize Two wheel robot system, we follow the same process as for an inverted pendulum

![RCTVC](Image/Two_wheel_robot.png)

**Relationship between x and $x_m$**

- $x_b = x - \frac{L}{b} \sin \theta$

- $y_b = \frac{L}{b} \cos \theta$

**Newton's second law for robot bodies**

- $ \left( m_w + m_b + \frac{I_w}{r^2} \right) \ddot{x} = m_j l \sin \theta \cdot \dot{\theta}^2 - m_j l \cos \theta \cdot \ddot{\theta} + \frac{\tau}{r} + f_w $
  
- $\sum F_{y, b} = R_y = m_b \left( g - l_b \cos \theta \cdot \dot{\theta}^2 - l_b \sin \theta \cdot \ddot{\theta} \right)$

**Torque equation for the center of mass of the robot body**

- $\sum \tau = R_x l_b \cos \theta + R_y l_b \sin \theta = I_b \ddot{\theta}$

If we linearize the above equation, we get the following equation:

- $I \ddot{\theta} = l_b m_b \ddot{x}_w + l_b m_b g \cdot \theta - l_b^2 m_b \cdot \ddot{\theta}$

At this time, $x$ is an arc of a circle, so it has the relationship $x = r \alpha$.

- $(I_b + m_b l_b^2) \cdot \ddot{\theta} = rl_b m_b \ddot{\alpha} + l_b m_b g \cdot \theta$

**Newton's second law for wheel movement in x direction**

![RCTVC](Image/wheel.jpg)

- $\sum F_x = m_w \ddot{x_w}_ + m_b \ddot{x_b} + f_{ext} = F$
  
  $\therefore f_{ext} = \text{external force}$

To convert the above equation into an equation for the torque acting on the wheel, multiply both sides by $r$.

- $\tau_w = (m_w + m_b) r \ddot{x_w} + r l_b m_b \sin \theta \cdot \dot{\theta}^2 - r l_b m_b \cos \theta \cdot \ddot{\theta} + f_{ext}$

At this time, $x$ is an arc of a circle, so it has the relationship $x = r \alpha$.

- $\tau_w = (m_w + m_b) r^2 \ddot{\alpha} + r m_b l_b \sin \theta \cdot \dot{\theta}^2 - r m_b l_b \cos \theta \cdot \ddot{\theta} + f_{ext}$
  
If we linearize the above equation, we get the following equation:

- $\tau_w = (m_w + m_b) r^2 \ddot{\alpha} - r m_b l_b \ddot{\theta} + f_{ext}$


**All torque u acting on the wheel $u_\alpha$**

- $u_\alpha = \left[ (m_w + m_b) r^2 + I_w \right] \ddot{\alpha} - r m_b l_b \ddot{\theta} + C_\alpha \dot{\alpha}$

  $\therefore f_{ext} = \mu \cdot \omega = C_\alpha \dot{\alpha}$

![RCTVC](Image/simulation/Two_wheel_robot_PD.jpg)

**Figure 9:** Two wheel robot PD 

![RCTVC](Image/simulation/Two_wheel_robot_PID.jpg)

**Figure 10:** Two wheel robot PID 
 

