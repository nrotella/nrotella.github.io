---
layout: post
title: "3D Geometry for Robotics"
author: "Nick Rotella"
categories: journal
tags: [tutorial,python,qt,opengl,robotsim]
image: ARM.svg
---

In order to proceed with writing a simple robot simulator, there are a number of mathematical concepts that must be introduced before we can describe the motion of a manipulator.

In this post, I'll be pulling sections from [my PhD notes](https://nrotella.github.io/download/NicholasRotellaPhDNotes.pdf) which, in turn, were largely taken from the excellent introductory text [Modeling and Control of Robot Manipulators](https://www.springer.com/gp/book/9781852332211).

* Table of contents:
{:toc}

# Rigid body pose

The position and orientation (together, **pose**) of a rigid body in space can be
described with respect to a fixed coordinate frame in terms of a **translation** from the
fixed frame's origin and **rotation** about the axes of the fixed frame. Here, we let $$O$$ denote the fixed coordinate frame and $$O'$$ the body coordinate frame, as shown below.

![rigid_body_3d.svg](../assets/img/rigid_body_3d.svg "3d rigid body pose relative to fixed frame"){: .center-image}

As shown, the fixed coordinate frame with origin $$O$$ is specified by unit axes
$$\mathbf{x}$$, $$\mathbf{y}$$ and $$\mathbf{z}$$, while the body coordinate frame with
origin $$O'$$ be specified by unit axes $$\mathbf{x'}$$, $$\mathbf{y'}$$ and
$$\mathbf{z'}$$.

## Translation

The **translation** of frame $$O'$$ with respect to frame $$O$$ is

$$
\mathbf{o'} = o'_{x}\mathbf{x} + o'_{y}\mathbf{y} + o'_{z}\mathbf{z}
$$

which can be written, in terms of the axes of the fixed frame, as the vector
$$\mathbf{o'} = [o'_{x}, o'_{y}, o'_{z}]^{T}$$.

## Rotation

The **rotation** of the body frame with respect to the fixed frame is specified
by expressing the axes of the body frame in terms of the axes of the fixed
frame.  This yields

$$
\begin{align}
\mathbf{x'} &= x'_{x}\mathbf{x} + x'_{y}\mathbf{y} + x'_{z}\mathbf{z} \\
\mathbf{y'} &= y'_{x}\mathbf{x} + y'_{y}\mathbf{y} + y'_{z}\mathbf{z} \\
\mathbf{z'} &= z'_{x}\mathbf{x} + z'_{y}\mathbf{y} + z'_{z}\mathbf{z} \\
\end{align}
$$

These relations can be expressed compactly in the **rotation matrix**

$$
R =
\begin{pmatrix}
 & & \\
\mathbf{x'} & \mathbf{y'} & \mathbf{z'} \\
 & & 
\end{pmatrix} =
\begin{pmatrix}
x'_{x} & y'_{x} & z'_{x} \\
x'_{y} & y'_{y} & z'_{y} \\ 
x'_{z} & y'_{z} & z'_{z} \\
\end{pmatrix} = 
\begin{pmatrix}
\mathbf{x'}^{T}\mathbf{x} & \mathbf{y'}^{T}\mathbf{x} &\mathbf{z'}^{T}\mathbf{x} \\
\mathbf{x'}^{T}\mathbf{y} & \mathbf{y'}^{T}\mathbf{y} &\mathbf{z'}^{T}\mathbf{y} \\
\mathbf{x'}^{T}\mathbf{z} & \mathbf{y'}^{T}\mathbf{z} &\mathbf{z'}^{T}\mathbf{z}
\end{pmatrix}
$$

This matrix simply describes the body frame's axes in terms of the axes of the fixed frame. The rightmost expression can be verified by substituting in the definitions of $$\mathbf{x'}$$, $$\mathbf{y'}$$ and $$\mathbf{z'}$$ from above and using the fact that these axes vectors are *mutually orthogonal* (actually, mutually *orthonormal* since they are unit vectors).  This means that dot products between different axes vectors are zero, eg if we denote $$\mathbf{x'}$$, $$\mathbf{y'}$$ and $$\mathbf{z'}$$ instead by $$\mathbf{e_{1}}$$, $$\mathbf{e_{2}}$$ and $$\mathbf{e_{3}}$$ respectively:

$$
e_{i}^{T}e_{j} =
\begin{cases}
1 \quad \mbox{if} \quad i=j\\
0 \quad \mbox{if} \quad i\neq j
\end{cases}
$$

Also recall that the dot product can be written

$$
e_{i}^{T}e_{j}=||e_{i}||||e_{j}||\cos{\theta_{i,j}}
$$

where $$\theta_{i,j}$$ is the angle between the vectors. Since the axes of both body and fixed frames are unit vectors, we can rewrite the orientation of the body frame as

$$
R =
\begin{pmatrix}
\cos{\theta_{x',x}} & \cos{\theta_{y',x}} & \cos{\theta_{z',x}} \\
\cos{\theta_{x',y}} & \cos{\theta_{y',y}} & \cos{\theta_{z',y}} \\
\cos{\theta_{x',z}} & \cos{\theta_{y',z}} & \cos{\theta_{z',z}}
\end{pmatrix}
$$

where $$\theta_{i,j}$$ again describes the angle between two axes unit vectors of the fixed and body frames.  For this reason, the rotation matrix is sometimes referred to as the **direction cosine matrix** or **DCM** (where each element is referred to as a **direction cosine**).

![dcm.svg](../assets/img/dcm.svg "Direction Cosine Matrix elements, explained geometrically"){: .center-image}

Now, let's take a quick look at propertis of the rotation matrix and how it's used.

### Inverse rotations

If the rotation matrix $$R$$ represents the orientation of the body frame with respect to the fixed frame, then its **inverse** $$R^{-1}$$ represents the opposite rotation - the orientation of the fixed frame with respect to the body frame.

Recall from above that the columns of the rotation matrix were said to be *mutually orthogonal* - we call this an **orthogonal matrix** which has the special property that its inverse is equal to its transpose:

$$
R^{-1}=R^{T}
$$

This makes transforming between frames easy and computationally-efficient. In addition, as was mentioned above, the columns of a rotation matrix are all *unit vectors*.  This means its columns are *mutually orthonormal*, or in other words, $$R$$ is an **orthonormal matrix**.

### Representing a vector in different frames

Consider two frames with the same origin $$O$$ but different sets of unit
axes, as shown below.

![rot_mat.svg](../assets/img/rot_mat.svg "A 3d point described in two frames sharing the same origin $$O$$"){: .center-image}

A vector $$\mathbf{p}$$ can be described in terms of the first frame as
$$\mathbf{p} = [p_{x}, p_{y}, p_{z}]^{T}$$ and in terms of the second frame as
$$\mathbf{p'} = [p'_{x}, p'_{y}, p'_{z}]^{T}$$.  Since these vectors correspond to
the same point in space, we must have $$\mathbf{p} = \mathbf{p'}$$ and thus 

$$
\mathbf{p} = p'_{x}\mathbf{x'} + p'_{y}\mathbf{y'} + p'_{z}\mathbf{z'} =
\begin{pmatrix}
 & & \\
\mathbf{x'} & \mathbf{y'} & \mathbf{z'} \\
 & & \\
\end{pmatrix} \mathbf{p'}
$$
 
This is, from above, $$\mathbf{p} = R\mathbf{p'}$$; $$R$$ is the rotation matrix which
transforms the vector from the second frame to the first.  Since $$R$$ is
orthogonal, the inverse transformation is $$\mathbf{p'} = R^{T}\mathbf{p}$$.

Note that since $$R$$ is an orthogonal transformation we have

$$
||R\mathbf{p'}|| = (R\mathbf{p'})^{T}(R\mathbf{p'}) =
\mathbf{p'}^{T}R^{T}R\mathbf{p'} = \mathbf{p'}^{T}\mathbf{p'} = ||\mathbf{p'}|| 
$$

The matrix $$R$$ therefore does not change the norm of the vector but only its
direction, which is a property we expect from pure rotation.

### Composing multiple successive rotations

Consider a point $$\mathbf{p}$$ in space and its representations in the frames
$$O^{0}$$, $$O^{1}$$ and $$O^{2}$$ having a common origin $$O$$. We introduce the noration $$R_{i}^{j}$$ to denote
the rotation matrix describing the orientation of frame $$i$$ with respect to
frame $$j$$; then we have

$$
\mathbf{p^{1}} = R_{2}^{1}\mathbf{p^{2}}
$$

We also have, using the equation above, 

$$ 
\mathbf{p^{0}} = R_{1}^{0}\mathbf{p^{1}} = R_{1}^{0}R_{2}^{1}\mathbf{p^{2}}
$$

Finally, we can write 
$$
\mathbf{p^{0}} = R_{2}^{0}\mathbf{p^{2}} = R_{1}^{0}R_{2}^{1}\mathbf{p^{2}}
$$

From the above equation it is obvious that we must have

$$
R_{2}^{0} = R_{1}^{0}R_{2}^{1}
$$

Here it is interpreted that a frame aligned with $$O^{0}$$ is first rotated
into alignment with $$O^{1}$$ with $$R_{1}^{0}$$ and then this frame is rotated
into alignment with frame $$O^{2}$$ using $$R_{2}^{1}$$.  That is, **successive
rotations can be applied to a vector via post-multiplication of the
corresponding rotation matrices following the order of rotations**.

![multiple_rotations.svg](../assets/img/multiple_rotations.svg "Successive rotations relative to intermediate frames"){: .center-image}

The overall 
rotation is therefore expressed as the composition of partial rotations; each rotation 
is defined with respect to the axes of the *current frame in the sequence*. When each rotation is made instead with respect to the axes of a *fixed
frame*, the elementary rotation matrices are premultiplied in the order in which
the rotations are applied.

![multiple_rotations_fixed.svg](../assets/img/multiple_rotations_fixed.svg "Successive rotations relative to a fixed frame"){: .center-image}

---

There's a LOT more to be said about representing rotations in three dimensions, but that will have to wait for a dedicated post. For now, the point is that we understand what a rotation matrix is and how it's formed from the axes of the fixed frame and frame of interest (here, the frame attached to a rigid body).

## Homogeneous Transformations

We've seen what it means to express the **translation** and **rotation** of one frame with respect to another, but for the sake of convenience we'd like to be able to desribe the full **pose** in one mathematical expression.

Recall from above that a point $$p_{1}$$ on a rigid body in frame $$O^{1}$$ is fully specified with respect to a fixed reference frame $$O^{0}$$ by a vector $$\mathbf{o}_{1}^{0}$$ describing the origin
of Frame 1 with respect to Frame 0 and a rotation matrix $$R_{1}^{0}$$ describing
the orientation of Frame 1 with respect to Frame 0.

![rigid_body_3d_homogeneous.svg](../assets/img/rigid_body_3d_homogeneous.svg "3d rigid body pose relative to fixed frame"){: .center-image}

The position of point P in Frame 0 is then

$$
\mathbf{p}^{0} = \mathbf{o}_{1}^{0} + R_{1}^{0}\mathbf{p}^{1}
$$

This specifies a *coordinate transformation* of a vector between the two
frames.  The inverse transformation comes from premultiplying both sides of the
above equation by $${R_{1}^{0}}^{T}=R_{0}^{1}$$ and solving for $$p^{1}$$; this
leads to

$$
p^{1} = -R_{0}^{1}o_{1}^{0} + R_{0}^{1}p^{0}
$$

These transformations can be expressed in a more compact form by converting to
*homogeneous representations* given by

$$
\begin{align*}
\tilde{p}^{0} &= A_{1}^{0}\tilde{p}^{1} \\
&=
\begin{pmatrix}
R_{1}^{0} & o_{1}^{0} \\
0^{T} & 1
\end{pmatrix}
\begin{pmatrix}
p^{1} \\
1
\end{pmatrix}
\end{align*}
$$

and, for the inverse transform,

$$
\begin{align*}

\tilde{p}^{1} = A_{0}^{1}\tilde{p}_{0} &= (A_{1}^{0})^{-1}\tilde{p}^{0}\\
&=
\begin{pmatrix}
R_{0}^{1} & -R_{0}^{1}o_{1}^{0} \\
0^{T} & 1
\end{pmatrix}
\begin{pmatrix}
p^{0} \\
1
\end{pmatrix}
\end{align*}
$$

where $$\tilde{p} = [p, 1]^{T}$$ is a *homogeneous vector* and $$A_{0}^{1}$$ and $$A_{1}^{0}$$ are *homogeneous transformation matrices*.  It is important to note that, in general, homogeneous transformation matrices are NOT orthogonal, so unlike for rotation matrices we do not have $$A^{-1}=A^{T}$$.

Successive homogeneous transformations are composed in the same manner as for rotation matrices:

$$
\tilde{p}^{0} = A_{1}^{0}A_{2}^{1}\cdots A_{n}^{n-1}\tilde{p}^{n}
$$

![multiple_transforms.svg](../assets/img/multiple_transforms.svg "Successive homogeneous transformations relative to intermediate frames"){: .center-image}

As before, note that post-multiplication is used because these matrices are all defined with respect to the preceding frame in the chain - this will be useful in defining a systematic way to transform between links of a robot manipulator. When transformations are all defined relative to the same fixed frame, we pre-multiply them as was done for pure rotations above.

# Wrapping up

We introduced the basics for 3D geometry, including rotations, translations and homogeneous transforms. Next time, we'll build on this foundation to derive the kinematics of arbitrary robot manipulators.