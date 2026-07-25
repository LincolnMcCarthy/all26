package org.team100.lib.geometry.six_dof;

/**
 * @param q1 base/swing
 * @param q2 shoulder/boom
 * @param q3 elbow/stick
 * @param q4 wrist roll
 * @param q5 wrist pitch
 * @param q6 tool roll
 */
public record SixDofConfig(double q1, double q2, double q3, double q4, double q5, double q6) {

    /**
     * For now, euclidean with weights.
     * see https://arxiv.org/pdf/1808.03891
     */
    public double distance(SixDofConfig other) {
        double l2 = 0;
        l2 += 10.0 * Math.pow(q1 - other.q1, 2);
        l2 += 5.0 * Math.pow(q2 - other.q2, 2);
        l2 += 2.0 * Math.pow(q3 - other.q3, 2);
        l2 += 1.0 * Math.pow(q4 - other.q4, 2);
        l2 += 1.0 * Math.pow(q5 - other.q5, 2);
        l2 += 1.0 * Math.pow(q6 - other.q6, 2);

        return Math.sqrt(l2);

    }
}
