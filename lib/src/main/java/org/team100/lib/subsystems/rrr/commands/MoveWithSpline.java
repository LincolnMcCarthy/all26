package org.team100.lib.subsystems.rrr.commands;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.geometry.rn.WaypointRn;
import org.team100.lib.geometry.rrr.RRRConfig;
import org.team100.lib.geometry.rrr.RRRVelocity;
import org.team100.lib.geometry.se2.DirectionSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.reference.rn.PositionReferenceControllerRn;
import org.team100.lib.reference.rn.SplineReferenceRn;
import org.team100.lib.spline.rn.SplineRn;
import org.team100.lib.subsystems.rrr.RRRArm;

import edu.wpi.first.math.Nat;
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
    private static final boolean DEBUG = false;
    private final LoggerFactory m_log;
    private final RRRArm m_arm;
    private final DirectionSE2 m_startv;
    private final Pose2d m_goal;
    private final DirectionSE2 m_goalv;
    /** Non-null when the command is running, otherwise null. */
    private PositionReferenceControllerRn m_referenceController;

    public MoveWithSpline(
            LoggerFactory parent,
            RRRArm arm,
            DirectionSE2 startv,
            Pose2d goal,
            DirectionSE2 goalv) {
        m_log = parent.type(this);
        m_arm = arm;
        m_startv = startv;
        m_goal = goal;
        m_goalv = goalv;
        // Check feasibility in constructor to avoid later exception.
        if (m_arm.config(m_goal) == null)
            throw new IllegalArgumentException("infeasible goal");

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        // Pose2d start = m_arm.pose();
        // Translation2d currTranslation = start.getTranslation();
        // Rotation2d courseToGoal = m_goal.getTranslation().minus(currTranslation).getAngle();
        // VelocitySE2 dx0 = VelocitySE2.fromAngle(courseToGoal);
        RRRConfig q0 = m_arm.getConfig();
        // RRRVelocity q0dot = m_arm.qdot(q0, dx0);
        RRRVelocity q0dot = m_arm.qdot(q0, VelocitySE2.fromDirection(m_startv, 1));

        // because we don't know if each joint will want to go
        // positive or negative (to avoid infeasibilities), we don't
        // really know what to use here, for arbitrary starting location.
        // TODO: this really only works if you specify it.
        WaypointRn<N3> p0 = new WaypointRn<>(q0.toVector(), q0dot.toVector(), 2);
        // WaypointRn<N3> p0 = new WaypointRn<>(q0.toVector(), VecBuilder.fill(0, 0,
        // 0));

        RRRConfig q1 = m_arm.config(m_goal);
        RRRVelocity q1dot = m_arm.qdot(q1, VelocitySE2.fromDirection(m_goalv, 1));
        WaypointRn<N3> p1 = new WaypointRn<>(q1.toVector(), q1dot.toVector(), 2);

        if (DEBUG) {
            System.out.printf("p0 %s p1 %s\n", p0, p1);
        }

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
        return m_referenceController != null && m_referenceController.isDone();
    }

    // TODO: implement toGo
    @Override
    public double toGo() {
        return 0;
    }

}
