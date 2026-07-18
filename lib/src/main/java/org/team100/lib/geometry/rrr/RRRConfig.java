package org.team100.lib.geometry.rrr;

/**
 * 3R config
 * 
 * NOTE!  Coordinates are different from other planar examples.
 * 
 * The arm is in the XZ plane.
 * 
 * The rotational zero is along +x
 * 
 * @param q1 shoulder rotation, "up" is negative
 * @param q2 elbow rotation, "up" is negative
 * @param q3 wrist rotation, "up" is negative 
 */
public record RRRConfig(double q1, double q2, double q3) {

}
