package org.team100.lib.trajectory.se3.constraint;

import org.team100.lib.path.se3.PathSE3Point;

public class ConstantConstraintSE3 implements TimingConstraintSE3 {

    @Override
    public double maxV(PathSE3Point state) {
        return 1.0;
    }

    @Override
    public double maxAccel(PathSE3Point state, double velocityM_S) {
        return 1.0;
    }

    @Override
    public double maxDecel(PathSE3Point state, double velocityM_S) {
        return 1.0;
    }

}
