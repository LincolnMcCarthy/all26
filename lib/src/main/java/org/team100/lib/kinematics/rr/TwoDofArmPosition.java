package org.team100.lib.kinematics.rr;

import org.wpilib.math.geometry.Translation2d;

/**
 * Position of each joint.
 */
public record TwoDofArmPosition(Translation2d p1, Translation2d p2) {
}