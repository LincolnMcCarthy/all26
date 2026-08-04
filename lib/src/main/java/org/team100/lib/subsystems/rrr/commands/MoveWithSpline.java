package org.team100.lib.subsystems.rrr.commands;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.geometry.rn.WaypointRn;
import org.team100.lib.geometry.rrr.RRRConfig;
import org.team100.lib.geometry.rrr.RRRVelocity;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.reference.rn.PositionReferenceControllerRn;
import org.team100.lib.reference.rn.SplineReferenceRn;
import org.team100.lib.spline.rn.SplineRn;
import org.team100.lib.subsystems.rrr.RRRArm;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;

/**
 * Generate a spline in R3, in joint space, and follow it.
 * 
 * The endpoints of the spline are chosen using the SE(2) direction.
 * 
 * * The benefit is that it is immune to interior singularities and joint
 * limits.
 * 
 * * The drawback is that it doesn't follow any particular workspace path --
 * it's better than the simple profiled method, due to the care in choosing
 * endpoint directions, but in between, there's nothing to make the path do
 * anything in particular (e.g. be straight).
 */
public class MoveWithSpline extends MoveAndHold {
    private final LoggerFactory m_log;
    private final RRRArm m_arm;
    private final Pose2d m_goal;
    // TODO: this is not really velocity
    private final VelocitySE2 m_goalv;
    /** Non-null when the command is running, otherwise null. */
    private PositionReferenceControllerRn m_referenceController;

    public MoveWithSpline(
            LoggerFactory parent,
            RRRArm arm,
            Pose2d goal,
            VelocitySE2 goalv) {
        m_log = parent.type(this);
        m_arm = arm;
        m_goal = goal;
        m_goalv = goalv;
        // Check feasibility in constructor to avoid later exception.
        if (m_arm.config(m_goal) == null)
            throw new IllegalArgumentException("infeasible goal");

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        Pose2d start = m_arm.pose();
        Translation2d currTranslation = start.getTranslation();
        Rotation2d courseToGoal = m_goal.getTranslation().minus(currTranslation).getAngle();
        VelocitySE2 dx0 = VelocitySE2.fromAngle(courseToGoal);
        RRRConfig q0 = m_arm.getConfig();
        RRRVelocity q0dot = m_arm.qdot(q0, dx0);

        // because we don't know if each joint will want to go
        // positive or negative (to avoid infeasibilities), we don't
        // really know what to use here, for arbitrary starting location.
        // TODO: this really only works if you specify it.
        WaypointRn<N3> p0 = new WaypointRn<>(q0.toVector(), q0dot.toVector());
        // WaypointRn<N3> p0 = new WaypointRn<>(q0.toVector(), VecBuilder.fill(0, 0, 0));

        RRRConfig q1 = m_arm.config(m_goal);
        RRRVelocity q1dot = m_arm.qdot(q1, m_goalv);
        WaypointRn<N3> p1 = new WaypointRn<>(q1.toVector(), q1dot.toVector());

        SplineRn<N3> spline = new SplineRn<>(Nat.N3(), p0, p1);
        SplineReferenceRn<N3> reference = new SplineReferenceRn<>(
                spline, 1);
        m_referenceController = new PositionReferenceControllerRn(
                m_arm, reference);
    }

    @Override
    public void execute() {
        m_referenceController.execute();
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.stop();
        m_referenceController = null;
    }

    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public double toGo() {
        return 0;
    }

}
