package org.team100.lib.kinematics.rrr_se2;

import java.util.List;

import org.team100.lib.geometry.rrr.RRRAcceleration;
import org.team100.lib.geometry.rrr.RRRConfig;
import org.team100.lib.geometry.rrr.RRRVelocity;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Planar 3R mechanism in SE2, using Product of Exponentials.
 * 
 * Zero position is extended along x.
 */
public class RRRKinematicsPoE {

    public RRRKinematicsPoE() {
    }

    public Pose2d forward(RRRConfig q) {
        return null;
    }

    public VelocitySE2 forward(RRRConfig q, RRRVelocity qdot) {
        return null;
    }

    public AccelerationSE2 forward(RRRConfig q, RRRVelocity qdot, RRRAcceleration qddot) {
        return null;
    }

    public List<RRRConfig> inverse(Pose2d x) {
        return null;
    }

    public RRRVelocity inverse(RRRConfig q, VelocitySE2 xdot) {
        return null;
    }

    public RRRAcceleration inverse(RRRConfig q, VelocitySE2 xdot, AccelerationSE2 xddot) {
        return null;
    }

}
