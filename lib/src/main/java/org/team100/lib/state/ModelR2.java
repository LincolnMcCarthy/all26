package org.team100.lib.state;

import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Represents planar position only. */
public class ModelR2 {
    private final ModelR1 m_x;
    private final ModelR1 m_y;

    public ModelR2(ModelR1 x, ModelR1 y) {
        m_x = x;
        m_y = y;
    }

    public ModelR2(Translation2d x, VelocityR2 v) {
        this(
                new ModelR1(x.getX(), v.x()),
                new ModelR1(x.getY(), v.y()));
    }

    /** Motionless with the specified pose */
    public ModelR2(Translation2d x) {
        this(x, VelocityR2.ZERO);
    }

    /** Motionless at the origin */
    public ModelR2() {
        this(new ModelR1(), new ModelR1());
    }

    public ControlR2 control() {
        return new ControlR2(m_x.control(), m_y.control());
    }

    /** Component-wise difference (not geodesic) */
    public ModelR2 minus(ModelR2 other) {
        return new ModelR2(x().minus(other.x()), y().minus(other.y()));
    }

    /** Component-wise sum (not geodesic) */
    public ModelR2 plus(ModelR2 other) {
        return new ModelR2(x().plus(other.x()), y().plus(other.y()));
    }

    /**
     * Use the current velocity to evolve the position of each dimension
     * independently.
     * 
     * This does not describe geodesic paths in SE(2). For that, see Twist2d.
     */
    public ModelR2 evolve(double dt) {
        return new ModelR2(m_x.evolve(dt), m_y.evolve(dt));
    }

    /** All dimensions position and velocity are within (the same) tolerance */
    public boolean near(ModelR2 other, double tolerance) {
        return x().near(other.x(), tolerance)
                && y().near(other.y(), tolerance);
    }

    /** Translation of the pose. */
    public Translation2d translation() {
        return new Translation2d(m_x.x(), m_y.x());
    }

    public VelocityR2 velocityR2() {
        return new VelocityR2(m_x.v(), m_y.v());
    }

    public ModelR1 x() {
        return m_x;
    }

    public ModelR1 y() {
        return m_y;
    }

    public String toString() {
        return "ModelR2(" + m_x + ", " + m_y + ")";
    }

}
