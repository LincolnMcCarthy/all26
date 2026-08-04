package org.team100.lib.geometry.six_dof;

import org.team100.lib.geometry.GeometryUtil;

/** Because this is used in the 6dof arm, we use q4,5,6, not 1,2,3 */
public record SphericalWristConfig(double q4, double q5, double q6) {
    /**
     * Return the other equivalent solution, which reflects the pitch axis and uses
     * the opposite roll angles.
     */
    public SphericalWristConfig flip() {
        return new SphericalWristConfig(
                GeometryUtil.flip(q4), -q5, GeometryUtil.flip(q6));
    }
}
