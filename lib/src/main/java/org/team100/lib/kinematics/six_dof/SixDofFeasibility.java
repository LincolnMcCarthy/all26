package org.team100.lib.kinematics.six_dof;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.geometry.six_dof.SixDofPose;

/** Selects feasible configurations. */
public class SixDofFeasibility {
    private final SixDofKinematics m_k;

    public SixDofFeasibility(SixDofKinematics k) {
        m_k = k;
    }

    /** Return the list without infeasible configurations. */
    public List<SixDofConfig> filter(List<SixDofConfig> ql) {
        List<SixDofConfig> result = new ArrayList<>();
        for (SixDofConfig q : ql) {
            if (!qRange(q))
                continue;
            if (!pRange(q))
                continue;
            result.add(q);
        }
        return result;
    }

    /** True if the joints configurations are in their allowed ranges. */
    boolean qRange(SixDofConfig q) {
        // Filter joint limits.
        // Swerve base axis is unlimited.
        // Shoulder cannot go below horizon.
        if (q.q2() < 0.1 || q.q2() > 3)
            return false;
        // Elbow cannot pass through shoulder.
        if (q.q3() < -3 || q.q3() > 3)
            return false;
        // Wrist roll is unlimited.
        // Wrist pitch cannot go backwards.
        if (q.q5() < -1.5 || q.q5() > 1.5)
            return false;
        // Tool roll is unlimited.
        return true;
    }

    /**
     * True if the joint cartesian positions are ok,
     * which for now means "above the floor".
     */
    boolean pRange(SixDofConfig q) {
        SixDofPose p = m_k.forward(q);
        // p1 z is fixed
        if (p.p2().getZ() < 0)
            return false;
        if (p.p3().getZ() < 0)
            return false;
        if (p.p4().getZ() < 0)
            return false;
        if (p.p5().getZ() < 0)
            return false;
        if (p.p6().getZ() < 0)
            return false;
        return true;
    }
}
