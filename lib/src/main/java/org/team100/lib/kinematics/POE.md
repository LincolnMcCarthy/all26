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

${}^0 \mathbf J = \begin{pmatrix} \mathbf I_{3 \times 3} & -\left[{}^0\mathbf t_E\right]_\times \\ \mathbf 0_{3 \times 3} & \mathbf I_{3 \times 3} \end{pmatrix} {}^0\mathbf J^v$,

## References

* [Corke, Robotics Vision and Control](https://weblibrary.mila.edu.my/upload/ebook/engineering/2011_Book_RoboticsVisionAndControl.pdf)
* [confusion post](https://robotics.stackexchange.com/questions/19744/the-jacobian-resulted-from-screw-method-is-different-from-analytical-one-exampl)
* [confusion post](https://robotics.stackexchange.com/questions/16829/relationship-between-the-velocity-twist-jacobian-and-the-spatial-velocity-jacobi) References page 247 of Corke, with a reply from Corke himself.
