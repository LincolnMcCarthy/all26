# Dynamics for arbitrary open chains

Following Modern Robotics ...

There's a worked example in the practice exercises, for 2R planar
in SE(3).

<img src="image.png" />

Here the mass is all at the ends of the links, to simplify the
math.

There are transformation matrices describing each link frame
in the zero configuration:

```math
M_1 = 
\begin{bmatrix}
1 & 0 & 0 & L_1 \\ 
0 & 1 & 0 & 0 \\ 
0 & 0 & 1 & 0 \\ 
0 & 0 & 0 & 1
\end{bmatrix}
,
M_2 = 
\begin{bmatrix}
1 & 0 & 0 & L_1+L_2 \\ 
0 & 1 & 0 & 0 \\ 
0 & 0 & 1 & 0 \\ 
0 & 0 & 0 & 1
\end{bmatrix}
```
And the transform from the first frame to the second:
```math
M_{12} = 
\begin{bmatrix}
1 & 0 & 0 & L_2 \\ 
0 & 1 & 0 & 0 \\ 
0 & 0 & 1 & 0 \\ 
0 & 0 & 0 & 1
\end{bmatrix}
```

There are screws for each axis (rotation-first here), first one is
just a rotation, second one is the twist at the end of the first
link.

```math
S_1 =
\begin{bmatrix}
0 \\
0 \\
1 \\
0 \\
0 \\
0
\end{bmatrix}
,
S_2 =
\begin{bmatrix}
0 \\
0 \\
1 \\
0 \\
-L_1 \\
0
\end{bmatrix}
```

To find these screws expressed in the link centers of mass,
$A_i$,
apply the adjoints of the inverses centers of mass transforms.

Given transform $T$,
```math
T =
\begin{bmatrix}
R & p \\
0 & 1
\end{bmatrix}
```
Remembering the definition of the adjoint:
```math
Ad_{x} =
\begin{bmatrix}
R & 0 \\
[p]_xR & R
\end{bmatrix}
```
where
```math
[p]_x =
\begin{bmatrix}
0 & -p_z & -p_y \\
p_z & 0 & -p_x \\
-p_y & p_x & 0
\end{bmatrix}
```

So

```math
Ad_{M_1^{-1}} =
\begin{bmatrix}
1 & 0 & 0 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & L_1 & 0 & 1 & 0 \\
0 & -L_1 & 0 & 0 & 0 & 1 \\
\end{bmatrix}
```
```math
Ad_{M_2^{-1}} =
\begin{bmatrix}
1 & 0 & 0 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & L_1+L_2 & 0 & 1 & 0 \\
0 & -L_1-L_2 & 0 & 0 & 0 & 1 \\
\end{bmatrix}
```
```math
A_1 =
Ad_{M_1^{-1}} S_1 = 
\begin{bmatrix}
0 \\
0 \\
1 \\
0 \\
L_1 \\
0
\end{bmatrix}
```
```math
A_2 =
Ad_{M_2^{-1}} S_2 = 
\begin{bmatrix}
0 \\
0 \\
1 \\
0 \\
L_2 \\
0
\end{bmatrix}
```
The gravity vector is 
```math
\mathbf{g} = 
\begin{bmatrix}
0 \\
g \\
0
\end{bmatrix}
```
where $g<0$.

Another way to express gravity is the acceleration of the
root, though here i think $g>0$.

```math
\dot{V}_0 =
\begin{bmatrix}
0 \\
0 \\
0 \\
0 \\
g \\
0
\end{bmatrix}
```

For each link, define the spatial inertia matrix $G_i$,
in the link reference frame, whose origin is at
the center of mass, so the inertia is zero:

```math
G_1 =
\begin{bmatrix}
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & m_1 & 0 & 0 \\
0 & 0 & 0 & 0 & m_1 & 0 \\
0 & 0 & 0 & 0 & 0 & m_1 \\
\end{bmatrix}
,
G_2 =
\begin{bmatrix}
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & m_2 & 0 & 0 \\
0 & 0 & 0 & 0 & m_2 & 0 \\
0 & 0 & 0 & 0 & 0 & m_2 \\
\end{bmatrix}

```

For each link, compute the transform from its predecessor to
its center of mass.  For that, we use the exponential
of the joint twist, which is
```math
e^{[S]\theta}
=
I + [S]\theta + [S]^2\frac{\theta^2}{2!} + \cdots
=
\begin{bmatrix}
e^{[\omega]\theta} & G(\theta)v \\
0 & 1
\end{bmatrix}
```
where
```math
G(\theta) = 
I\theta + (1-cos\theta)[\omega] + (1-sin\theta)[\omega]^2
```

So

```math
e^{[A_1]\theta_1} =
\begin{bmatrix}
cos(\theta_1) & -sin(\theta_1) & 0 & -L_1(1-cos(\theta_1)) \\ 
sin(\theta_1) & cos(\theta_1) & 0 & L_1sin(\theta_1) \\ 
0 & 0 & 1 & 0 \\ 
0 & 0 & 0 & 1
\end{bmatrix}

```

```math
T_{01}
=
M_1 e^{[A_1]\theta_1}
=
\begin{bmatrix}
1 & 0 & 0 & L_1 \\ 
0 & 1 & 0 & 0 \\ 
0 & 0 & 1 & 0 \\ 
0 & 0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
cos(\theta_1) & -sin(\theta_1) & 0 & -L_1(1-cos(\theta_1)) \\ 
sin(\theta_1) & cos(\theta_1) & 0 & L_1sin(\theta_1) \\ 
0 & 0 & 1 & 0 \\ 
0 & 0 & 0 & 1
\end{bmatrix}
```
```math
T_{01}
=
\begin{bmatrix}
cos(\theta_1) & -sin(\theta_1) & 0 & L_1 cos(\theta_1) \\
sin(\theta_1) & cos(\theta_1) & 0 & L_1 sin(\theta_1) \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
```

Compute the velocity, $V_1$, of the first link in its own
frame:

```math
V_1 = Ad_{T_{10}}V_0 + A_1\dot{\theta}_1
```
Since $V_0 = 0$,
```math
V_1 = 
\begin{bmatrix}
0 \\
0 \\
1 \\
0 \\
L_1 \\
0
\end{bmatrix}
\dot{\theta}_1
=
\begin{bmatrix}
0 \\
0 \\
\dot{\theta}_1 \\
0 \\
L_1 \dot{\theta}_1
\\
0
\end{bmatrix}
```

Compute the acceleration, $\dot{V}_1$, of the first link in
its own frame (note the Lie bracket).

```math
\dot{V}_1 =
Ad_{T_{10}}\dot{V}_0 + [V_1,A_1]\dot{\theta}_1 + A_1\ddot{\theta}_1
```
Recall the acceleration of the base involves $g$:

```math
\dot{V}_1 =
\begin{bmatrix}
0 \\
0 \\
\ddot{\theta}_1 \\
g sin (\theta_1) \\
g cos (\theta_1) + L_1\ddot{\theta}_1 \\
0
\end{bmatrix}
```

Repeating for the second link:
```math
T_{12} =
\begin{bmatrix}
cos(\theta_2) & -sin(\theta_2) & 0 & L_2cos(\theta_2) \\
sin(\theta_2) & cos(\theta_2) & 0 & L_2sin(\theta_2) \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
```

```math
V_2 =
\begin{bmatrix}
0 \\
0 \\
\dot{\theta}_1 + \dot{\theta}_2 \\
L_1 sin(\theta_2)\dot{\theta}_1 \\
(L_2 + L_1 cos(theta_2)) \dot{\theta}_1 + L_2\dot{\theta}_2
\\
0
\end{bmatrix}
```

```math
\dot{V}_1 =
\begin{bmatrix}
0 \\
0 \\
\ddot{\theta}_1 +\ddot{\theta}_2 \\
g sin (\theta_1+\theta_2) + L_1 cos(\theta_2)\dot{\theta}_1\dot{\theta}_2 + L_1sin(\theta_2))\ddot{\theta}_1\\
g cos (\theta_1+\theta_2) -L_1sin(\theta_2)\dot{\theta}_1\dot{\theta}_2 + (L_2 + L_1cos(\theta_2)\ddot{\theta}_1 + L_2\ddot{\theta}_2\\
0
\end{bmatrix}
```

Then work backwards from the end-effector (which has the
same frame as link 2).)


## References

* [Sola et al 2021](https://arxiv.org/pdf/1812.01537) Lie group theory, condensed and with robotics applications
* [Modern Robotics homepage](https://hades.mech.northwestern.edu/index.php/Modern_Robotics)}
* [Modern Robotics book](https://hades.mech.northwestern.edu/images/2/25/MR-v2.pdf)
* [Modern Robotics code](https://github.com/NxRLab/ModernRobotics) but no tests :(
* [RoboKitPy](https://github.com/foiegreis/RoboKitPy/tree/main) kinda the same, refactored a bit (still no tests)
* [Exercises including a 2R worked example](https://hades.mech.northwestern.edu/images/archive/0/00/20181130042048!ME449-practice.pdf)