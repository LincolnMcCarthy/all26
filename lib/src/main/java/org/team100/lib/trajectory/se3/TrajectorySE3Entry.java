package org.team100.lib.trajectory.se3;

import org.team100.lib.path.se3.PathSE3Parameter;

/**
 * Represents a state within a trajectory in SE(3).
 */
public class TrajectorySE3Entry {
    static final boolean DEBUG = false;

    /** Source of the point */
    private final PathSE3Parameter m_parameter;
    /** Path point and timing, velocity, acceleration */
    private final TrajectorySE3Point m_point;

    public TrajectorySE3Entry(PathSE3Parameter parameter, TrajectorySE3Point point) {
        m_parameter = parameter;
        m_point = point;
    }

    public PathSE3Parameter parameter() {
        return m_parameter;
    }

    public TrajectorySE3Point point() {
        return m_point;
    }

    @Override
    public String toString() {
        return String.format("[ %s ]",
                m_point.toString());
    }

    @Override
    public boolean equals(final Object other) {
        if (!(other instanceof TrajectorySE3Entry)) {
            if (DEBUG)
                System.out.println("wrong type");
            return false;
        }
        TrajectorySE3Entry ts = (TrajectorySE3Entry) other;
        if (!m_point.equals(ts.m_point)) {
            if (DEBUG)
                System.out.println("wrong point");
            return false;
        }
        return true;
    }

}
