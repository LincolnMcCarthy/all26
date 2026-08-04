# math

In the SE(2) and SE(3) spline classes, we calculate the curvature vector.

https://en.wikipedia.org/wiki/Curvature#Curvature_vector

See `HolonomicSplineSE2.K()` and `HolonomicSplineSE3.K()`

The curvature vector is the path-length-derivative of the unit tangent
vector. It's an R3 vector that lies in the plane
perpendicular to the tangent vector, T, i.e. the course.
 
It points in the direction of the center of curvature.

https://en.wikipedia.org/wiki/Center_of_curvature
 
Its magnitude is "κ", 1/radius of osculating circle, rad/m

https://en.wikipedia.org/wiki/Osculating_circle

## calculating the curvature vector

We'll use an arbitrary parameterization over parameter $s$  for each point, $\boldsymbol{P}$ on the curve:

```math
\boldsymbol{P}
=
\boldsymbol{\gamma}(s)
```

The _unit_ tangent vector, $\boldsymbol{T}$, is
the normalized derivative:

```math
\boldsymbol{T}
=
\frac
    {\boldsymbol{\gamma}'(s)}
    {\left\| \boldsymbol{\gamma}'(s) \right\| }
```

The curvature vector, $\boldsymbol{K}$, is the
derivative of the unit tangent vector with respect
to arc length, which is just the second derivative
of position, but only the
component perpendicular to $\boldsymbol{T}$,
so subtract the parallel component:
```math
\boldsymbol{K}
=
  \frac 
    {\boldsymbol{\gamma}''(s)}
    {\left\| \boldsymbol{\gamma}'(s) \right\|^2}
- 
  \boldsymbol{T}
\left (
  \boldsymbol{T}
  \cdot 
  \frac 
    {\boldsymbol{\gamma}''(s)}
    {\left\| \boldsymbol{\gamma}'(s) \right\|^2}
\right)
```

There are other ways to compute this vector, for example using the cross product,
but the cross product is kind of an $\mathbb{R}^3$ specialty; this method works for both
$\mathbb{R}^2$ and $\mathbb{R}^3$.