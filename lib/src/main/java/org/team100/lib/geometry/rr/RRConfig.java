package org.team100.lib.geometry.rr;

import org.team100.lib.geometry.rrr.RRRConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;

/**
 * Joint configuration for the RR example.
 * 
 * @param q1 rotation of joint 1 ("proximal", "shoulder"), CCW rad from x
 * @param q2 rotation of joint 2 ("distal", "elbow"),CCW rad from link 1
 */
public record RRConfig(double q1, double q2) {

    /**
     * For now, euclidean with weights.
     * 
     * You can change these weights to change how configs are selected, based on
     * their "nearness" to the current pose.
     * 
     * See https://arxiv.org/pdf/1808.03891
     */
    public double distance(RRConfig other) {
        double l2 = 0;
        // shoulder movements are expensive
        l2 += 3.0 * Math.pow(q1 - other.q1, 2);
        // elbow movements are less expensive
        l2 += 2.0 * Math.pow(q2 - other.q2, 2);
        return Math.sqrt(l2);
    }

    /** Interpolate in configuration space, never crossing pi. */
    public static RRConfig interpolate(RRConfig a, RRConfig b, double s) {
        return new RRConfig(
                MathUtil.interpolate(a.q1(), b.q1(), s),
                MathUtil.interpolate(a.q2(), b.q2(), s));
    }

    public Vector<N2> toVector() {
        return VecBuilder.fill(q1, q2);
    }
}
