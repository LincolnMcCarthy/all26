package org.team100.lib.path.se2;

import org.team100.lib.spline.se2.ISplineSE2;

/** A point on a spline. */
public record PathSE2Parameter(ISplineSE2 spline, double s) {
}