# Six-DOF Kinematics

<img src="image.png" width=400/>

Analytic kinematics for the common 6R linkage,
where the three wrist axes intersect, i.e. a
"spherical wrist".  The sequence of joint orientations
is "Y-P-P-R-P-R".

The spherical wrist (left) compared with an
offset design (right):
<img src="compare.png" width=400/>

This design allows us to decouple the kinematics
problem into a "postional part", solving for the
translation of the wrist center, and an "orientation
part", solving for the rotation of the wrist.

A classic example of this design is the
["PUMA" arm](https://en.wikipedia.org/wiki/Programmable_Universal_Machine_for_Assembly):

<img src="puma.png" width=400 />

## Forward position kinematics

The forward kinematics simply composes the joint transforms.

## Inverse position kinematics

For an end-effector pose $(\mathbf{t}, \mathbf{R})$:

First find the wrist position, by backing along the tool.  The
tool axis is z, so the tool vector $\mathbf{b}$ is:

```math
\mathbf{b} = l \cdot \mathbf{R} \cdot
\begin{bmatrix}
0\\0\\1
\end{bmatrix}
```

```math
\mathbf{w} = \mathbf{t} - \mathbf{b}
```

The first three joints are as follows:

* $\theta_1$ is called the "swing" or "waist" axis.  It is at the base,
  with z axis pointing up, usually implemented in a real robot with a
  large slewing bearing, but in our robot this will likely be the
  rotational axis of the swerve drivetrain.
* $\theta_2$ is the "shoulder" or "boom" axis, with z-axis pointing
  to the global right.  In heavy industrial robots this axis is often pushed
  forward, rather than being coincident with $\theta_1$, in order to
  increase reach.
* $\theta_3$ is the "elbow" or "stick" axis, with axis parallel to the
  shoulder.

Note that the definition of the "zero" of each joint will affect
the actual value of $\theta_i$; it is common for zeros of the
first three joints to be as follows:

* $\theta_1 = 0$ : shoulder joint axis pointing -y
* $\theta_2 = 0$ : upper arm pointing parallel to +x
* $\theta_3 = 0$ : lower arm pointing parallel to +x

The positional kinematics of the first three joints, to the wrist origin,
can be solved as in the [RR case](../rr/README.md).

Beware of the singularity where the wrist is inline with
the base joint.

Given the solution for position, we can compute the rotation of
the wrist parent, ${}_0^3\mathbf{R}$

The desired rotation is the composition of the wrist parent and
the wrist joint:

```math
\mathbf{R} = {}_0^3\mathbf{R} {}_3^6\mathbf{R}
```
So we get the wrist rotation:
```math
{}_3^6\mathbf{R} =  {}_0^3\mathbf{R}^{-1} \mathbf{R}  

```
The wrist rotation is implemented using the "roll-pitch-roll" ("RPR")
arrangement:

* $\theta_4$ is called "wrist roll" or "precession".  Its axis is inline with the
  forearm, so it rolls the whole wrist assembly relative to
  the forearm.  This joint is actually often built into the elbow, so
  it rotates the entire forearm.
* $\theta_5$ is called "wrist pitch" or "nutation".  Its axis is perpendicular
  to the forearm.
* $\theta_6$ is called "tool roll" or "spin".  Its axis is inline with the tool
  itself, so it rotates the tool.

The "zero" of these joints is usually as follows:

* $\theta_4 = 0$ : pitch joint axis points parallel to -y
* $\theta_5 = 0$ : tool is pointing parallel to +x
* $\theta_6 = 0$ : tool is rotated in the middle of its range

Using these zeros, the whole arm is stretching in the +x dirction

All three of these joints intersect at the same origin,
which is why the wrist is called "spherical."

The decomposition of ${}_3^r\mathbf{R}$ into the components
$\theta_4$, $\theta_5$, and $\theta_6$ is called the "ZYZ Euler Angles"
or "Proper Euler Angles".

Or, maybe it's "ZXZ," since the pitch axis zero is horizontal?

Following [ETH](https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/KinematicsSingleBody.pdf), this is the ZYZ case:

```math
\theta_4 = atan2(r_{23}, r_{13})\\
```
```math
\theta_5 = atan2 \left(  \sqrt{r_{13}^2 + r_{23}^2}, r_{33} \right)
```
```math
\theta_6 = atan2 (r_{32}, -r_{31})
```

and this is the ZXZ case:

```math
\theta_4 = atan2(r_{13}, -r_{23})\\
```
```math
\theta_5 = atan2 \left(  \sqrt{r_{13}^2 + r_{23}^2}, r_{33} \right)
```
```math
\theta_6 = atan2 (r_{31}, r_{32})
```

Note that if $r_{33}$ is 1, the two "roll"
joints are collinear, which is a singularity. In this case, do
something convenient (e.g. note the most-recent setting of one
of the joints, keep it the same, and move the other).


## References

* [Miranda 2017](https://ljvmiranda921.github.io/notebook/2017/01/25/forward-kinematics-stanford-manipulator/)
* [Kalaycioglu et al 2024](https://arxiv.org/pdf/2410.22582)
* [Addison 2020](https://automaticaddison.com/the-ultimate-guide-to-inverse-kinematics-for-6dof-robot-arms/)
* [Boschetti 2023](https://www.mdpi.com/2075-1702/11/2/306) comparing spherical and offset wrists; main insight is that q5 is limited to +/- pi/2 in the usual "spherical" design.
* [iris FK](https://wanxinjin.github.io/asu-robotics/lec6-8/fk.html)
* [iris IK](https://wanxinjin.github.io/asu-robotics/lec9/ik.html)
* [Shah 2019](https://surilshah.weebly.com/uploads/1/1/4/6/11462120/robotics_l9_inverse_kinematics2.pdf) presentation, I think this contains an error?
* [Ngo](https://orionquest.github.io/CS428/scribe_notes/lecture7_notes_Duc_Ngo.pdf) notes
* [ETH notes 2016](https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/KinematicsSingleBody.pdf) <== the best reference.
* [ETH full notes](https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf)