package org.team100.lib.geometry.canfield;

/**
 * The Canfield joint angles are measured with zero pointing
 * at the center of the basal plate, so all useful angles are
 * greater than pi/2.
 */
public record CanfieldConfig(double q1, double q2, double q3) {

}
