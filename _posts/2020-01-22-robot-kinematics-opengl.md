---
layout: post
title: "Robot Kinematics in OpenGL"
author: "Nick Rotella"
categories: journal
tags: [tutorial,python,qt,opengl,robotsim]
image: opengl_grid.png
---


# Forward (Direct) Kinematics

A robotics manipulator consists of a series of rigid bodies or *links*
connected by means of *joints*.  Joints can be of two types:
*revolute* (rotational) and *prismatic* (translational).  The entire
structure of the manipulator is formed by the links and joints and is known as a
*kinematic chain*.  One end of the chain is constrained to a *base*
while the other end is usually connected to an *end-effector* for
manipulation of objects in space.

The goal of *direct kinematics* is to find a mapping - via a sequence of
homogeneous transformations - which descibes the pose of the end-effector with
respect to the base in terms of the joint variables.

Consider an open chain of $$n+1$$ links connected by $$n$$ joints as shown below.

![manipulator.svg](../assets/img/manipulator.svg "Manipulator with n revolute joints"){: .center-image}

Here, the first link - Link 0 - is the fixed base. Each joint, denoted by $$q_{i}$$, provides the structure with a
single *degree of freedom* (DOF). Notice that joint $$q_{i-1}$$ is always the first to affect link $$i$$; this difference in indexing is a common cause of confusion.

Attached to each link from 0 (base) to $$n$$ is the *link frame* denoted by $$L_{i}$$; the coordinate transformation from frame $$n$$ to frame $$0$$ is
then

$$
A_{n}^{0}(\mathbf{q}) = A_{1}^{0}(q_{1})A_{2}^{1}(q_{2})\cdots
A_{n}^{n-1}(q_{n})
$$

where each homogeneous transformation $$A_{i}^{i-1}$$ is incremental (defined relative to the
preceding link) and is thus a function of only one joint variable.

![manipulator_transforms.svg](../assets/img/manipulator_transforms.svg "Manipulator homogeneous transformations"){: .center-image}

The direct
kinematics function from the end-effector to the base also requires
transformations from link 0 to the base and from link $$n$$ to the end-effector;
these are typically constant transformations. The direct kinematics function is
thus written as

$$
A_{endeff}^{base}(\mathbf{q}) = A_{0}^{b}A_{n}^{0}(\mathbf{q})A_{endeff}^{n}
$$

where we introduce $$\mathbf{q}=[q_{0},q_{1},\ldots,q_{n-1}]$$ to denote the *vector* of all joint states.

## Defining link frames

In the previous section, we described loosely how direct kinematics allows us to compute the homogeneous transform of the endeffector - or any frame in between - relative to the base. We also
introduced *link frames* and made the choice to locate them each at the origin of the preceding joint. However, we didn't actually describe how to *define* the axes of each link frame - and as it turns out, there are a number of reasonable ways to go about this. Probably the most well-known is the Denavit-Hartenberg convention.

### Denavit-Hartenberg Convention

This convention describes how to define the link frames such
that computation of the direct kinematics function is accomplished in a general,
systematic fashion. It's a bit complicated to visualize in three dimensions, so if it's still confusing I suggest [checking out this video](https://www.youtube.com/watch?v=rA9tm0gTln8) which does a great job at visualizing the process of defining frames. That said, the basic rules are as follows.

#### Defining link frames

The frame corresponding to the $$i^{th}$$ link is located at
joint $$i-1$$ and is defined as follows:

 * Choose the z-axis $$z_{i}$$ of each frame through the axis of the joint.
 * Locate the origin of the frame at the intersection of $$z_{i}$$ and the
  common normal to $$z_{i}$$ and $$z_{i-1}$$.  If $$z_{i}$$ and $$z_{i-1}$$ intersect,
  this is the location of the origin.
 * Choose the x-axis $$x_{i}$$ along the common normal to $$z_{i}$$ and
  $$z_{i-1}$$.  If these z-axes intersect, choose $$x_{i}$$ to be $$z_{i-1}\times
  z_{i}$$.
 * Finally, choose $$y_{i}$$ to complete a right-handed coordinate frame.

Consider the three-link *planar* (z-axes are out of the page) manipulator having three revolute joints as below:

![three_link_planar.svg](../assets/img/three_link_planar.svg "Three-link planar manipulator with DH frames"){: .center-image}

| Link | θ | d | α | a |
|:----:|:----:|:----:|:----:|:-----:|
| 1 | θ<sub>1</sub> | 0 | 0 | a<sub>1</sub> |
| 2 | θ<sub>2</sub> | 0 | 0 | a<sub>2</sub> |
| 3 | θ<sub>3</sub> | 0 | 0 | a<sub>3</sub> |

In total, there are $$n$$ links and $$n+1$$ frames (one at each joint and one at the endeffector).  With the frames chosen as above, we can uniquely specify the manipulator geometry with four parameters:

 * $$\theta_{i}$$ is the angle between $$x_{i}$$ and $$x_{i-1}$$ about $$z_{i-1}$$. It's only variable for a **revolute joint**, otherwise it's a fixed offset angle.
 * $$d_{i}$$ is the distance between the origins of frames $$i$$ and $$i-1$$
  along the direction of axis $$z_{i-1}$$. It's only variable for a **prismatic joint**, otherwise it's a fixed distance offset. Obviously $$d_{i}=0$$ for a planar manipulator like the one above.
 * $$\alpha_{i}$$ is the angle between axes $$z_{i-1}$$ and $$z_{i}$$ about
  $$x_{i}$$; this is always a fixed offset angle.
 * $$a_{i}$$ is the distance between the axes $$z_{i-1}$$ and $$z_{i}$$ along the
  direction of axis $$x_{i}$$ (common normal between z-axes); this is always a fixed offset distance.

Note that we consider **each joint to have one DOF**, meaning only one of $$\theta_{i}$$ and $$d_{i}$$ can be variable for each link.  More complex joints having multiple DOFs (eg spherical joints like your shoulder) are treated as combinations of single-DOF joints sharing the same origin. More on that later.

#### Computing link transforms

In this setup, Link $$i$$ is located between Frames $$i$$ and $$i-1$$; the homogeneous
transformation between these frames as a general function of the DH parameters $$\{\theta,d,\alpha,a\}$$ is given by

$$
A_{i}^{i-1}(\theta_{i},d_{i},\alpha_{i},a_{i}) = 
\begin{pmatrix}
\cos{\theta_{i}} & -\sin{\theta_{i}}\cos{\alpha_{i}} & \sin{\theta_{i}}\sin{\alpha_{i}} &
a_{i}\cos{\theta_{i}}\\

\sin{\theta_{i}} & \cos{\theta_{i}}\cos{\alpha_{i}} & -\cos{\theta_{i}}\sin{\alpha_{i}} &
a_{i}\sin{\theta_{i}}\\

0 & \sin{\alpha_{i}} & \cos{\alpha_{i}} & d_{i}\\

0 & 0 & 0 & 1
\end{pmatrix}
$$

Consider again the three-link planar arm, for which we've highlighted the homogeneous transformation between the first and second link:

![three_link_planar_transform.svg](../assets/img/three_link_planar_transform.svg "Three-link planar manipulator link homogeneous transformation"){: .center-image}

Note that since $$\alpha_{2}=0$$ and $$d_{2}=0$$, the rotational and translational portions of the transformation matrix $$A_{2}^{1}$$ are two-dimensional, as expected for a planar arm. This is not the case for more complex manipulators, as we will see below.

The ultimate goal is generally to compute the endeffector pose (rotation + translation), since this is usually the point on the robot which interacts with the world. Since the three links are identical in this case, the homogeneous transformation between successive links is

$$
A_{i}^{i-1}(\theta_{i},a_{i}) = 
\begin{pmatrix}
c_{\theta_{i}} & -s_{\theta_{i}} & 0 & a_{i}c_{\theta_{i}}\\
s_{\theta_{i}} & c_{\theta_{i}} & 0 & a_{i}s_{\theta_{i}}\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{pmatrix}
$$

where we introduce $$\cos{x}=c_{x}$$ and $$\sin{x}=s_{x}$$ for brevity. The homogeneous transformation which describes the pose of the endeffector relative to the base is composed as intriduced earlier:

$$
A_{e}^{b}(\mathbf{\theta},\mathbf{a}) = A_{0}^{b}A_{1}^{0}(\theta_{1},a_{1})A_{2}^{1}(\theta_{2},a_{2})A_{e}^{2}(\theta_{3},a_{3})
$$

Substituting in the transform definitions (including the base to frame zero transform, which is the identity since they are identical frames) we have

$$
A_{e}^{b}(\mathbf{\theta},\mathbf{a})
=
\begin{pmatrix}
1 & 0 & 0 & 0\\
0 & 1 & 0 & 0\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{pmatrix}
\begin{pmatrix}
c_{\theta_{1}} & -s_{\theta_{1}} & 0 & a_{1}c_{\theta_{1}}\\
s_{\theta_{1}} & c_{\theta_{1}} & 0 & a_{1}s_{\theta_{1}}\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{pmatrix}
\begin{pmatrix}
c_{\theta_{2}} & -s_{\theta_{2}} & 0 & a_{2}c_{\theta_{2}}\\
s_{\theta_{2}} & c_{\theta_{2}} & 0 & a_{2}s_{\theta_{2}}\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{pmatrix}
\begin{pmatrix}
c_{\theta_{3}} & -s_{\theta_{3}} & 0 & a_{3}c_{\theta_{3}}\\
s_{\theta_{3}} & c_{\theta_{3}} & 0 & a_{3}s_{\theta_{3}}\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{pmatrix}
$$

which multiplies out to

$$
A_{e}^{b}(\mathbf{\theta},\mathbf{a})
=
\begin{pmatrix}
c_{\theta_{123}} & -s_{\theta_{123}} & 0 & a_{1}c_{\theta_{1}}+a_{2}c_{\theta_{12}}+a_{3}c_{\theta_{123}}\\
s_{\theta_{123}} & c_{\theta_{123}} & 0 & a_{1}s_{\theta_{1}}+a_{2}s_{\theta_{12}}+a_{3}s_{\theta_{123}}\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{pmatrix}
$$

where we introduce the notation $$c_{xy}=\cos{x}\cos{y}$$ and $$s_{xy}=\sin{x}\sin{y}$$ and so forth. Illustrated graphically:

![three_link_planar_endeff.svg](../assets/img/three_link_planar_endeff.svg "Three-link planar manipulator link homogeneous transformation"){: .center-image}

Whew! Let's check out some more complex manipulator structures now.

#### Case study 2: spherical arm

The *spherical arm* consists of two co-located revolute joints, followed by our first prismatic (translation) joint. Note that both $$\theta_{2}$$ and $$d_{2}$$ are nonzero; however, only one of these can be variable for a single joint. In this case, $$\theta_{2}$$ is the joint's DOF and $$d_{2}$$ is fixed. This is made clear from the use of a cylinder to illustrate the joint; a prismatic joint is shown as a cube, eg for the DOF $$d_{3}$$.

![spherical_arm.png](../assets/img/spherical_arm.png "Spherical manipulator DH link frames (credit: Siciliano et al)"){: .center-image}

| Link | θ | d | α | a |
|:----:|:----:|:----:|:----:|:-----:|
| 1 | θ<sub>1</sub> | 0 | -π/2 | 0 |
| 2 | θ<sub>2</sub> | d<sub>2</sub> | π/2 | 0 |
| 3 | 0 | d<sub>3</sub> | 0 | 0 |

#### Case study 3: anthropomorphic arm

The *anthropomorphic arm* is essentially a vertical two-link planar arm (latter two joints) with the first DOF providing rotation of the remainder of the structure within the (ground) plane.

![anthropomorphic_arm.png](../assets/img/anthropomorphic_arm.png "Anthropomorphic manipulator DH link frames (credit: Siciliano et al)"){: .center-image}

| Link | θ | d | α | a |
|:----:|:----:|:----:|:----:|:-----:|
| 1 | θ<sub>1</sub> | 0 | π/2 | 0 |
| 2 | θ<sub>2</sub> | 0 | 0 | a<sub>2</sub> |
| 3 | θ<sub>3</sub> | 0 | 0 | a<sub>3</sub> |

# Wrapping up

In this tutorial, we introduced the basics for defining manipulator link frames via Denavit-Hartenberg parameters, as well as computing the incremental homogeneous transformations between successive links. We worked through the specifics for a three-link planar arm and a more complex anthropomorphic arm. Next, we'll implement classes to define general manipulator kinematics based on this theory.