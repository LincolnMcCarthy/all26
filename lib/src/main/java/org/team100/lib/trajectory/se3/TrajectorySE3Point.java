package org.team100.lib.trajectory.se3;

import org.team100.lib.path.se3.PathSE3Point;
import org.team100.lib.util.Math100;

public class TrajectorySE3Point {
    private static final boolean DEBUG = false;

    private final PathSE3Point m_point;
    private final double m_timeS;
    private final double m_velocityM_S;
    private final double m_accelM_S_S;

    public TrajectorySE3Point(
            PathSE3Point point,
            double t,
            double velocity,
            double acceleration) {
        m_point = point;
        m_timeS = t;
        m_velocityM_S = velocity;
        m_accelM_S_S = acceleration;
    }

    /** path point */
    public PathSE3Point point() {
        return m_point;
    }

    /** Instant this point is reached, seconds */
    public double time() {
        return m_timeS;
    }

    /** Instantaneous pathwise velocity, m/s */
    public double velocity() {
        return m_velocityM_S;
    }

    /** Instantaneous pathwise (not centripetal) acceleration, m/s^2 */
    public double accel() {
        return m_accelM_S_S;
    }

    @Override
    public String toString() {
        return String.format("TrajectorySE3Point [ %s,  %5.3f, %5.3f, %5.3f ]",
                point(), time(), velocity(), accel());
    }

    @Override
    public boolean equals(final Object other) {
        if (!(other instanceof TrajectorySE3Point)) {
            if (DEBUG)
                System.out.println("wrong type");
            return false;
        }
        TrajectorySE3Point ts = (TrajectorySE3Point) other;
        if (!point().equals(ts.point())) {
            if (DEBUG)
                System.out.println("wrong state");
            return false;
        }
        if (!Math100.epsilonEquals(time(), ts.time())) {
            if (DEBUG)
                System.out.println("wrong time");
            return false;
        }
        return true;
    }

}
