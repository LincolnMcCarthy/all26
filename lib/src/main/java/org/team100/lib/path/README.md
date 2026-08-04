# Paths

A path is a list of points, constructed by sampling a spline -- the
spline parameter is retained for convenience.

In SE(2) (the space Pose2d lives in), there are a few more fields, e.g.
the curvature of the path. There's a factory, PathFactorySE2, which
chooses parameter values so that the straight parts don't have too
many points, but the curved parts have more.

There are also versions of these for SE(3), the Pose3d manifold.