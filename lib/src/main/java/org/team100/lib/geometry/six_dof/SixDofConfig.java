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

}
