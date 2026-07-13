package org.team100.lib.geometry;

import org.wpilib.math.geometry.Translation2d;

public record StateR2(Translation2d position, GlobalVelocityR2 velocity) {
}
