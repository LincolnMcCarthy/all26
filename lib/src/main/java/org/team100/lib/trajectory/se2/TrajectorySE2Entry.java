package org.team100.lib.trajectory.se2;

import org.team100.lib.path.se2.PathSE2Parameter;

/**
 * Represents a state within a trajectory in SE(2).
 */
public class TrajectorySE2Entry {
    private static final boolean DEBUG = false;

    /** Source of the point */
    private final PathSE2Parameter m_parameter;
    /** Path point and timing, velocity, acceleration */
    private final TrajectorySE2Point m_point;

    public TrajectorySE2Entry(PathSE2Parameter parameter, TrajectorySE2Point point) {
        m_parameter = parameter;
        m_point = point;
    }

    public PathSE2Parameter parameter() {
        return m_parameter;
    }

    public TrajectorySE2Point point() {
        return m_point;
    }

    @Override
    public String toString() {
        return String.format("[ %s ]",
                m_point.toString());
    }

    @Override
    public boolean equals(final Object other) {
        if (!(other instanceof TrajectorySE2Entry)) {
            if (DEBUG)
                System.out.println("wrong type");
            return false;
        }
        TrajectorySE2Entry ts = (TrajectorySE2Entry) other;
        if (!m_point.equals(ts.m_point)) {
            if (DEBUG)
                System.out.println("wrong point");
            return false;
        }
        return true;
    }
}
