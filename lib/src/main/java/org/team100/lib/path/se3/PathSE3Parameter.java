package org.team100.lib.path.se3;

import org.team100.lib.spline.se3.SplineSE3;

/** A point on a spline. */
public record PathSE3Parameter(SplineSE3 spline, double s) {
}
