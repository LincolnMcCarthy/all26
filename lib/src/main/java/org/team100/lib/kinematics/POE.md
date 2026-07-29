# Product of Exponentials

How to compute forward position and velocity kinematics
using the "Product of Exponentials" method.

## Forward position kinematics

The main reference appears to be Modern Robotics, here we're following
[Babaiasl](https://github.com/madibabaiasl/modern-robotics-course/wiki/Lesson-7:-Forward-Kinematics-of-Robot-Arms-Using-Screw-Theory).

The joints are expressed as unit twists,
which are 6-vectors.

```math
\mathcal{S}_1 ,  \cdots , \mathcal{S}_n
```

also written

```math
\hat{\xi}_1 ,  \cdots , \hat{\xi}_n
```

The configuration of each joint is a scalar:

```math
q_1 , \cdots , q_n
```

Thus the actual twists at each joint are:

```math
\mathcal{S}_1q_1 , \cdots , \mathcal{S}_nq_n
```
also written
```math
\hat{\xi}_1 q_1 ,  \cdots , \hat{\xi}_n q_n
```

Exponentiating, we obtain transforms for
each joint.  We use the affine (4x4)
matrix representation for each twist, and
indicate that with brackets:

```math
e^{[\mathcal{S}_1]q_1} , \cdots ,e^{[\mathcal{S}_n]q_n}
```

also written
```math
e^{\hat{\xi}_1q_1} , \cdots ,e^{\hat{\xi}_nq_n}
```

Each transform is a matrix,

```math
\begin{bmatrix}
e^{\hat{\omega}_1\theta_1} & t_1\\
0&1
\end{bmatrix}
,
\cdots
\begin{bmatrix}
e^{\hat{\omega}_n\theta_n} & t_n\\
0&1
\end{bmatrix}

```

The tool pose for all $q_i = 0$ is $M$.

The tool pose in general is:

```math
T(q) = e^{[\mathcal{S}_1]q_1} ... e^{[\mathcal{S}_{n-1}]q_{n-1}}e^{[\mathcal{S}_n]q_n} M
```

## Forward velocity kinematics

The PoE velocity kinematics are very confusing, because
the word "Jacobian" is used to mean more than one thing.

* Geometric Jacobian, ${}^0 \mathbf J$:
The linearization at a particular
configuration of the function relating configuration
velocity to the **tool point** velocity.
* Space Jacobian. ${}^0\mathbf J^v$:
The linearization at a particular
configuration of the function relating configuration
velocity to the **tool reference frame** velocity.

The former definition obviously includes the tool
itself, the latter obviously does not.

Given the space Jacobian, Corke says to apply

```math
{}^0 \mathbf J = \begin{pmatrix} \mathbf I_{3 \times 3} & -\left[{}^0\mathbf t_E\right]_\times \\ \mathbf 0_{3 \times 3} & \mathbf I_{3 \times 3} \end{pmatrix} {}^0\mathbf J^v
```

... but doesn't say anything else about it.

The space Jacobian, applied to joint velocities,
yields a twist in the global frame of the last joint
i.e. of the flange.  In this frame, the tool is a fixed
offset, so the transformation is another adjoint map.
What is that adjoint map?

## Example: 1R in SE(2)

Start with one revolute joint at the origin, with a
fixed tool offset of (2,0), so we can check the math
easily.

To construct the PoE forward kinematics, find the joint axis:

```math
S_1 = \begin{bmatrix}0\\0\\1\end{bmatrix}
```
The tool pose, in homogeneous form (this does not work when
representing pose as a vector):
```math
M = \begin{bmatrix}
1 & 0 & 2 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}
```
The forward position kinematics is the product of exponentials:
```math
\mathbf{x} = e^{[S_1]q_1} M
```
The matrix exponential of a twist is:
```math
e^{[S_1]q_1} =
\begin{bmatrix}
cos & -sin & x(1-cos) + y  sin\\
sin & cos & y(1-cos) - x sin \\
0 & 0 & 1
\end{bmatrix}
```
Evaluating at $q_1=0$, the matrix exponential is identity:
```math
\mathbf{x} =
\begin{bmatrix}
1 & 0 & 0 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
1 & 0 & 2 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}
=
\begin{bmatrix}
1 & 0 & 2 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}
```
so $\mathbf{x}$ is just the tool pose.

Evaluating at $q_1=\pi/2$, note the twist has
no translation so the exponential is just
the rotational part:
```math
\mathbf{x}
=
\begin{bmatrix}
0 & -1 & 0 \\
1 & 0 & 0 \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
1 & 0 & 2 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}
=
\begin{bmatrix}
0 & -1 & 0 \\
1 & 0 & 2 \\
0 & 0 & 1
\end{bmatrix}
```
which is the tool pose rotated $pi/2$.

__The Jacobian__

Each column of the space Jacobian is the screw axis of each joint,
transformed by the adjoints of the preceding joints.
In the 1R case, there are no preceding joints, and therefore
no adjoints; the space Jacobian is just the
(constant!) screw axis.

```math
\mathbf{J}^v(q) = 
\begin{bmatrix}
0 \\ 0 \\ 1
\end{bmatrix}
```

Obviously, applying this Jacobian to the joint velocity yields the velocity
of the tool **frame**, not the velocity of the tool **point**.
To get the velocity of the point, apply the adjoint
map of the tool point pose in the current configuration (not the
zero configuration!):

```math
\mathbf{J}(q) =
\mathrm{Ad}(\xi)\mathbf{J}^v(q) =
\begin{bmatrix}
R & -p^\perp \\
0 & 1
\end{bmatrix}
\mathbf{J}^v(q) = 
\begin{bmatrix}
cos & sin & -p_y \\
sin & cos & p_x \\
0 & 0 & 1
\end{bmatrix}
\mathbf{J}^v(q) 
```
or, at $q_1=0$, recalling the tool pose $(2, 0, 0)$:
```math
\mathbf{J}(q) =
\begin{bmatrix}
1 & 0 & 0 \\
0 & 1 & 2 \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
0 \\ 0 \\ 1
\end{bmatrix} =
\begin{bmatrix}
0 \\ 2 \\ 1
\end{bmatrix}
```
This fits the expectation: with radius 2, at angular velocity 1,
the tool is moving with y velocity +2, and rotating.

At $q_1=\pi/2$, the tool point has changed to $(0, 2, \pi/2)$,
so the adjoint has also changed:
```math
\mathbf{J}(q) =
\begin{bmatrix}
0 & -1 & -2 \\
1 & 0 & 0 \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
0 \\ 0 \\ 1
\end{bmatrix} =
\begin{bmatrix}
-2 \\ 0 \\ 1
\end{bmatrix}
```
This is again the expected value: radius 2 at $pi/2$ should
be moving with x velocity -2, and rotating.

Note that because the space Jacobian has no translational part,
the rotational part of the tool adjoint has no effect.

## Example: 2R in SE(2)

A slightly more complex example: one joint at the origin, one
at (1,0), tool point at (2,0).

Joint axis at the origin:

```math
S_1 = \begin{bmatrix}0\\0\\1\end{bmatrix}
```

Joint axis at (1, 0):
```math
S_2 =
\begin{bmatrix} 0 \\ -1 \\ 1 \end{bmatrix}

```

The tool pose at $q_1=q_2=0$:
```math
M = \begin{bmatrix}
1 & 0 & 2 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}
```

The forward position kinematics is the product of exponentials:

```math
\mathbf{x} = e^{[S_1]q_1} e^{[S_2]q_2} M
```

Again, using the matrix exponential of a twist:
```math
e^{[S_1]q_1} =
\begin{bmatrix}
cos & -sin & x(1-cos) + y  sin\\
sin & cos & y(1-cos) - x sin \\
0 & 0 & 1
\end{bmatrix}
```

Note also (Dellaert) that the exponential expressed as a
conjugate uses the instantaneous
axis, which is, of course, the joint:
```math
e^{[S]q} =
\begin{bmatrix}
1 & 0 & x \\
0 & 1 & y \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
cos & -sin & 0 \\
sin & cos & 0 \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
1 & 0 & -x \\
0 & 1 & -y \\
0 & 0 & 1
\end{bmatrix}
```

Evaluating at $q_1=q_2=0$. note the second exponential
translational part is zero because the angle is zero

```math
\mathbf{x}
=
\begin{bmatrix}
1 & 0 & 0 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
1 & 0 & 0 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}
 \begin{bmatrix}
1 & 0 & 2 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}
=
 \begin{bmatrix}
1 & 0 & 2 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}
```

This is the same result as before.

Evaluating at $q_1=0$, $q_2=\pi/2$, the translational
part plays a role:

```math
\mathbf{x}
=
\begin{bmatrix}
1 & 0 & 0 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
0 & -1 & -1 \\
1 & 0 & -1 \\
0 & 0 & 1
\end{bmatrix}
 \begin{bmatrix}
1 & 0 & 2 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}
=
\begin{bmatrix}
0 & -1 & -1 \\
1 & 0 & 1 \\
0 & 0 & 1
\end{bmatrix}
```

__TODO: this seems wrong.__


## References

* [Corke, Robotics Vision and Control 2011](https://weblibrary.mila.edu.my/upload/ebook/engineering/2011_Book_RoboticsVisionAndControl.pdf) (this is the 2011 edition which does not include the interesting part about the tool point jacobian)
* [Corke 2017](https://www.yumpu.com/en/document/read/63365638/2017-book-roboticsvisionandcontrol/267) the 2017 version
* [Lynch, Park 2019](https://hades.mech.northwestern.edu/images/2/25/MR-v2.pdf)
* [confusion post](https://robotics.stackexchange.com/questions/19744/the-jacobian-resulted-from-screw-method-is-different-from-analytical-one-exampl)
* [confusion post](https://robotics.stackexchange.com/questions/16829/relationship-between-the-velocity-twist-jacobian-and-the-spatial-velocity-jacobi) References page 247 of Corke, with a reply from Corke himself.
* [Dellaert](https://piazza.com/class_profile/get_resource/hpa4u5hmxk599/hsdw3z0329m3st) on serial link geometry