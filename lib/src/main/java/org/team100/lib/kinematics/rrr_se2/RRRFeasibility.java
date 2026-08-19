package org.team100.lib.kinematics.rrr_se2;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.rrr.RRRConfig;
import org.team100.lib.geometry.rrr.RRRPose;

/** Selects feasible configurations. */
public class RRRFeasibility {
    private static final boolean DEBUG = false;

    private final RRRKinematicsPoE m_k;

    public RRRFeasibility(RRRKinematicsPoE k) {
        m_k = k;
    }

    public List<RRRConfig> filter(List<RRRConfig> ql) {
        List<RRRConfig> result = new ArrayList<>();
        for (RRRConfig q : ql) {
            if (!qRange(q)) {
                continue;
            }
            if (!xRange(q)) {
                continue;
            }
            result.add(q);
        }
        return result;
    }

    /**
     * True if the joints configurations are in their allowed ranges.
     * 
     * TODO: make this match the real mechanism.
     */
    boolean qRange(RRRConfig q) {
        if (q.q1() < -Math.PI / 2 || q.q1() > Math.PI / 2) {
            if (DEBUG)
                System.out.printf("q1 out of range %s\n", q.q1());
            return false;
        }
        if (q.q2() < -3 || q.q2() > 3) {
            if (DEBUG)
                System.out.printf("q2 out of range %s\n", q.q2());
            return false;
        }
        if (q.q3() < -3 || q.q3() > 3) {
            if (DEBUG)
                System.out.printf("q3 out of range %s\n", q.q3());
            return false;
        }
        return true;
    }

    /**
     * True if the joint workspace positions are ok.
     * 
     * TODO: make a real work envelope
     */
    boolean xRange(RRRConfig q) {
        RRRPose x = m_k.forward(q);
        if (x.p4().getX() < 0) {
            if (DEBUG)
                System.out.printf("p4 out of range %s\n", x.p4());
            return false;
        }
        return true;
    }
}
