---
layout: post
title: "Forward Differential Kinematics"
author: "Nick Rotella"
categories: journal
tags: [tutorial,python,qt,opengl,robotsim]
image: ARM.svg
---

In this series of tutorials, we'll go over the basics of manipulator robot kinematics.  So far, we've discussed how to parameterize coordinate frames in a kinematic chain via the Denavit-Hartenberg parameters and compute the pose of the endeffector as function of the joint variables and robot geometry via *forward direct* kinematics. Now, we'll present *forward differential* kinematics which describes how joint velocities combine through manipulator geometry to result in the endeffector velocity.

* Table of contents:
{:toc}


# Forward Differential Kinematics

In order to compute derivatives of the link poses - in geometric terms, link *twists* - from derivatives of the joint variables, we use **forward differential** kinematics. This mapping is described by a matrix
called the **Jacobian**.

There are two ways to arrive at such a mapping - the first is called the
*geometric* Jacobian and the second is called the *analytical*
Jacobian.  

The geometric Jacobian is derived in a manner similar to that of the
direct kinematics function in which one sums up the contributions of each
individual joint velocity to the total end-effector velocity; this mapping
is configuration-dependent.  

The analytical Jacobian results from differentiating the direct kinematics
function (when this function describes the pose with reference to a minimal
representation in operational space) with respect to the joint variables.

## Geometric Jacobian

For an n-DOF manipulator we have the direct kinematics equation

$$
T_{e}(\mathbf{q}) = 
\begin{pmatrix}
R_{e}(\mathbf{q}) & & \mathbf{p}_{e}(\mathbf{q})\\
 & & \\
\mathbf{0}^{T} & & 1
\end{pmatrix}
$$

It is desired to express the end-effector linear velocity
$$\mathbf{\dot{p}}_{e}$$ and the end-effector angular velocity
$$\mathbf{\omega}_{e}$$ in terms of the joint velocities $$\mathbf{\dot{q}}$$.  It will be
shown that these relations are both linear in the joint velocities and are given
by

$$
\begin{align*}
\mathbf{\dot{p}}_{e} &= J_{P}(\mathbf{q})\mathbf{\dot{q}} \\
\mathbf{\omega}_{e} &= J_{O}(\mathbf{q})\mathbf{\dot{q}}
\end{align*}
$$

where $$J_{P}\in R^{3\times n}$$ and $$J_{O}\in R^{3\times n}$$ are the Jacobian
matrices relating the contributions of the joint velocities to the
end-effector linear and angular velocities, respectively.  We can write this in
compact form as the *differential kinematics equation*

$$
\mathbf{v}_{e} = \begin{pmatrix}\mathbf{\dot{p}}_{e}\\
\mathbf{\omega}_{e}\end{pmatrix} = J(\mathbf{q})\mathbf{\dot{q}}
$$

where 

$$
J = \begin{pmatrix}J_{P}\\J_{O}\end{pmatrix}
$$

is the manipulator geometric Jacobian which is, in general, a function of the
manipulator configuration.  This matrix is derived as follows.

First, consider the time derivative of a rotation matrix $$R=R(t)$$.  Since such a
matrix is orthogonal,

$$
R(t)R(t)^{T} = I
$$

Differentiating this expression with respect to time yields

$$
R(t)\dot{R}^{T}(t) + \dot{R}(t)R^{T}(t) = 0
$$

Defining $$S(t) = \dot{R}(t)R(t)^{T}$$ we have

$$
S(t) + S^{T}(t) = 0
$$

which implies that the matrix $$S(t)$$ must be *skew-symmetric* since the sum
of it and its transpose equals the zero matrix.

Since $$R^{-1}(t) = R(t)$$ we can solve our expression for $$S(t)$$ to yield 

$$
\dot{R}(t) = S(t)R(t)
$$

which is the differential equation relating the rotation matrix to its
derivative via the skew-symmetric operator $$S$$.

Consider a constant vector $$\mathbf{p'}$$ in a rotating reference frame
described by $$R(t)$$ and its image $$\mathbf{p(t)} = R(t)\mathbf{p'}$$ in the fixed
frame in which $$R(t)$$ is defined.  Taking the derivative of $$\mathbf{p(t)}$$
yields

$$
\mathbf{\dot{p}}(t) = \dot{R}(t)\mathbf{p'}
$$

which, using the definition of the derivative of a rotation matrix, can be
written as

$$
\mathbf{\dot{p}}(t) = S(t)R(t)\mathbf{p'}
$$

From mechanics, however, we know that this is simply given by
$$\mathbf{\dot{p}}(t) = \omega(t)\times \mathbf{p}(t) = \omega(t)\times
R(t)\mathbf{p'}$$ where $$\mathbf{\omega}(t)=[\omega_{x}, \omega_{y},
\omega{z}]^{T}$$ is the angular velocity of the rotating frame with respect to the reference frame at time $$t$$.

This means that we must have 

$$
S(\mathbf{\omega}(t)) =
\begin{pmatrix}
0 & -\omega_{z} & \omega_{y} \\
\omega_{z} & 0 & -\omega_{x} \\
-\omega_{y} & \omega_{x} & 0
\end{pmatrix}
$$

We can thus write $$\dot{R} = S(\mathbf{\omega})R$$.  Further, for a rotation
matrix we have the property

$$
RS(\mathbf{\omega})R^{T} = S(R\mathbf{\omega})
$$

Now consider the coordinate transformation of a point $$P$$ from Frame 1 to Frame
0 given by

$$
\mathbf{p}^{0} = \mathbf{o}_{1}^{0} + R_{1}^{0}\mathbf{p}^{1}
$$

Differentiating this expression with respect to time yields

$$
\begin{align*}
\mathbf{\dot{p}}^{0} &= \mathbf{\dot{o}}_{1}^{0} + R_{1}^{0}\mathbf{\dot{p}}^{1}
+ \dot{R}_{1}^{0}\mathbf{p}^{1} \\
&= \mathbf{\dot{o}}_{1}^{0} + R_{1}^{0}\mathbf{\dot{p}}^{1}
+ S(\mathbf{\omega}_{1}^{0})R_{1}^{0}\mathbf{p}^{1} \\
&= \mathbf{\dot{o}}_{1}^{0} + R_{1}^{0}\mathbf{\dot{p}}^{1}
+ \mathbf{\omega}_{1}^{0}\times \mathbf{r}_{1}^{0} 
\end{align*}
$$

where $$\mathbf{r}_{1}^{0} = R_{1}^{0}\mathbf{p}^{1}$$ represents the point
$$P$$ after it has been rotated into Frame 0 but not translated, ie $$r_{1}^{0} =
p^{0} - o_{1}^{0}$$.
This is known as the *velocity composition rule*.  If the point $$P$$ is fixed in Frame 1 then this reduces to

$$
\mathbf{\dot{p}}^{0} = \mathbf{\dot{o}}_{1}^{0} + \mathbf{\omega}_{1}^{0}\times
\mathbf{r}_{1}^{0}
$$

Now consider deriving the relationships between the linear and angular
velocities of successive frames. Using the same DH convention for choosing link
frames, it can be shown that

$$
\mathbf{\dot{p}}_{i} = \mathbf{\dot{p}}_{i-1} + \mathbf{\dot{v}}_{i-1,i} +
\mathbf{\omega}_{i-1} \times \mathbf{r}_{i-1,i}
$$

gives the linear velocity of Link $$i$$ as a function of the translational
and rotational velocities of Link $$i-1$$.  Note that all vectors are expressed
with respect to a fixed Frame 0 and that $$\mathbf{v}_{i-1,i}$$ denotes the
velocity of the origin of Frame $$i$$ with respect to the origin of Frame $$i-1$$ as
expressed in terms of Frame 0. In addition,

$$
\mathbf{\omega}_{i} = \mathbf{\omega}_{i-1} + \mathbf{\omega}_{i-1,i}
$$

gives the angular velocity of Link $$i$$ as a function of the angular velocities
of Link $$i-1$$ and of Link $$i$$ with respect to Link $$i-1$$
($$\mathbf{\omega}_{i-1,i}$$).

Using these general results, we have that for a *prismatic joint*

$$
\mathbf{\omega}_{i} = \mathbf{\omega}_{i-1}
$$

since the orientation of Frame $$i$$ with respect to $$i-1$$ does not change when
Joint $$i$$ is moved and thus $$\mathbf{\omega}_{i-1,i}=0$$.  For the linear
velocity we have

$$
\mathbf{\dot{p}}_{i} = \mathbf{\dot{p}}_{i-1} +
\dot{d}_{i}\mathbf{z}_{i-1} + \mathbf{\omega}_{i} \times \mathbf{r}_{i-1,i}
$$

since this joint is articulated in the direction of axis $$\mathbf{z}_{i}$$.

For a *revolute joint* we have

$$
\mathbf{\omega}_{i} = \mathbf{\omega}_{i-1} + \dot{\theta}_{i}\mathbf{z}_{i-1}
$$

and

$$
\mathbf{\dot{p}}_{i} = \mathbf{\dot{p}}_{i-1} + \mathbf{\omega}_{i} \times \mathbf{r}_{i-1,i}
$$

## Jacobian Computation

Consider the expression $$\mathbf{\dot{p}}_{e}(\mathbf{q})$$
relating the end-effector position to the joint variables.  Differentiating this yields

$$
\mathbf{\dot{p}}_{e} = \sum_{i=1}^{n}\frac{\partial \mathbf{p}_{e}}{\partial
q_{i}}\dot{q_{i}} = \sum_{i=1}^{n}{J_{P}}_{i}\dot{q}_{i}
$$

due to the chain rule.  Thus, the linear velocity of the end-effector can be
obtained as the sum of $$n$$ terms, each of which represents the contribution of
a single joint to the end-effector linear velocity when all other joints are
still.
 
Thus, we have $${J_{P}}_{i} = \mathbf{z}_{i-1}$$ for a prismatic joint and
$${J_{P}}_{i} = \mathbf{z}_{i-1} \times (\mathbf{p}_{e} - \mathbf{p}_{i-1})$$ for
a revolute joint.

The angular velocity of the end-effector is given by

$$
\mathbf{\omega}_{e} = \mathbf{\omega}_{n} =
\sum_{i=1}^{n}\mathbf{\omega}_{i-1,i} = \sum_{i=1}^{n}{J_{O}}_{i}\dot{q}_{i}
$$

and thus for a prismatic joint $${J_{O}}_{i} = 0$$ and for a revolute joint
$${J_{O}}_{i} = \mathbf{z}_{i-1}$$.

In summary, the full Jacobian is formed from $$3\times 1$$ vectors $${J_{P}}_{i}$$
and $${J_{O}}_{i}$$ as

$$
J = 
\begin{pmatrix}
{J_{P}}_{1} & {J_{P}}_{2} & \cdots & {J_{P}}_{n} \\
 & & & \\
{J_{O}}_{1} & {J_{O}}_{2} & \cdots & {J_{O}}_{n}
\end{pmatrix}
$$

where we have

$$
\begin{pmatrix}{J_{P}}_{i}\\{J_{O}}_{i}\end{pmatrix} = 
\begin{cases}
\begin{pmatrix}\mathbf{z}_{i-1}\\ \mathbf{0}\end{pmatrix} &\text{for a
 prismatic joint}
\\
\begin{pmatrix}\mathbf{z}_{i-1} \times (\mathbf{p}_{e} - \mathbf{p}_{i-1})\\
\mathbf{z}_{i-1}\end{pmatrix} &\text{for a revolute joint}
\end{cases}
$$

The vectors on which the Jacobian depends are functions of the joint variables
and can be computed from the direct kinematics relations as follows.

* $$\mathbf{z}_{i-1}$$ is given by the third column of the rotation matrix
  $$R_{i-1}^{0}$$; if $$z_{0}=[0, 0, 1]^{T}$$ then $$\mathbf{z}_{i-1} =
  R_{1}^{0}(q_{1})\cdots R_{i-1}^{i-2}(q_{i-1})\mathbf{z}_{0}$$.
* $$\mathbf{p}_{e}$$ is given by the first three elements of the fourth
  coumn of the homogeneous transformation matrix $$T_{e}^{0} =
  A_{1}^{0}(q_{1})\cdots A_{n}^{n-1}(q_{n})$$.
* $$\mathbf{p}_{i-1}$$ is given by the first three elements of the fourth
  column of the homogeneous transformation matrix $$T_{i-1}^{0} =
  A_{1}^{0}(q_{1})\cdots A_{i-1}^{i-2}(q_{i-1})$$

Finally, note that the Jacobian has been developed here to describe the
end-effector velocities with respect to the base frame.  If it is desired to
represent the Jacobian with respect to a different Frame *u* then the
relation is

$$
J^{u} = \begin{pmatrix}R^{u}&0\\0&R^{u}\end{pmatrix}J
$$

# Wrapping up

In this tutorial, we introduced the basics for defining manipulator link frames via Denavit-Hartenberg parameters, as well as computing the incremental homogeneous transformations between successive links. We worked through the specifics for a three-link planar arm and a more complex anthropomorphic arm. Next, we'll implement classes to define general manipulator kinematics based on this theory.