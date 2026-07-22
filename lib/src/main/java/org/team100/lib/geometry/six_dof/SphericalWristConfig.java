package org.team100.lib.geometry.six_dof;

/** Because this is used in the 6dof arm, we use q4,5,6, not 1,2,3 */
public record SphericalWristConfig(double q4, double q5, double q6) {
}
