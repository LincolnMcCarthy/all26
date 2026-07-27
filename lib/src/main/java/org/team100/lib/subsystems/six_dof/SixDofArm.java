package org.team100.lib.subsystems.six_dof;

import java.util.List;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.geometry.six_dof.SixDofPose;
import org.team100.lib.kinematics.six_dof.SixDofFeasibility;
import org.team100.lib.kinematics.six_dof.SixDofKinematics;
import org.team100.lib.kinematics.six_dof.SixDofKinematicsPoE;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.subsystems.six_dof.commands.MoveWithProfile;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Six-DOF arm, for training.
 */
public class SixDofArm extends SubsystemBase {
    private static final boolean DEBUG = false;

    final SixDofKinematics m_kinematics;
    final SixDofFeasibility m_feasibility;
    private final BareMotor m_q1;
    private final BareMotor m_q2;
    private final BareMotor m_q3;
    private final BareMotor m_q4;
    private final BareMotor m_q5;
    private final BareMotor m_q6;

    public SixDofArm(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);

        // m_kinematics = new SixDofKinematicsAnalytic(0.1, 0.3, 0.3, 0.1);
        m_kinematics = new SixDofKinematicsPoE(0.1, 0.3, 0.3, 0.1);
        m_feasibility = new SixDofFeasibility(m_kinematics);

        m_q1 = new SimulatedBareMotor(log, 600);
        m_q2 = new SimulatedBareMotor(log, 600);
        m_q3 = new SimulatedBareMotor(log, 600);
        m_q4 = new SimulatedBareMotor(log, 600);
        m_q5 = new SimulatedBareMotor(log, 600);
        m_q6 = new SimulatedBareMotor(log, 600);
    }

    @Override
    public void periodic() {
        m_q1.periodic();
        m_q2.periodic();
        m_q3.periodic();
        m_q4.periodic();
        m_q5.periodic();
        m_q6.periodic();
    }

    /**
     * @param p tool center point pose, aimed at +z
     */
    public SixDofConfig config(Pose3d p) {
        SixDofConfig q0 = getConfig();
        List<SixDofConfig> qAll = m_kinematics.inverse(p, q0.q1(), q0.q4());
        List<SixDofConfig> qFeasible = m_feasibility.filter(qAll);
        if (qFeasible.isEmpty()) {
            System.out.println("infeasible pose " + StrUtil.poseStr(p));
            return null;
        }
        return getBest(qFeasible, q0);
    }

    public void setPosition(Pose3d p) {
        SixDofConfig q = config(p);
        if (q == null)
            return;
        setConfig(q);
    }

    /** TODO: velocity and force in config space. */
    public void setConfig(SixDofConfig q) {
        m_q1.setUnwrappedPosition(q.q1(), 0, 0);
        m_q2.setUnwrappedPosition(q.q2(), 0, 0);
        m_q3.setUnwrappedPosition(q.q3(), 0, 0);
        m_q4.setUnwrappedPosition(q.q4(), 0, 0);
        m_q5.setUnwrappedPosition(q.q5(), 0, 0);
        m_q6.setUnwrappedPosition(q.q6(), 0, 0);
    }

    SixDofConfig getBest(List<SixDofConfig> qAll, SixDofConfig q0) {
        double closest = Double.POSITIVE_INFINITY;
        SixDofConfig best = qAll.get(0);
        for (SixDofConfig q : qAll) {
            double d = q0.distance(q);
            if (DEBUG)
                System.out.printf("q0 %s q %s distance %6.3f\n", q0, q, d);
            if (d < closest) {
                closest = d;
                best = q;
            }
        }
        return best;
    }

    public SixDofConfig getConfig() {
        return new SixDofConfig(
                m_q1.getUnwrappedPositionRad(),
                m_q2.getUnwrappedPositionRad(),
                m_q3.getUnwrappedPositionRad(),
                m_q4.getUnwrappedPositionRad(),
                m_q5.getUnwrappedPositionRad(),
                m_q6.getUnwrappedPositionRad());
    }

    public SixDofPose getPose() {
        return pose(getConfig());
    }

    private SixDofPose pose(SixDofConfig q) {
        return m_kinematics.forward(q);
    }

    public Command warp0() {
        return run(() -> setConfig(new SixDofConfig(0, 0, 0, 0, 0, 0)));
    }

    public Command warp1() {
        return run(() -> setConfig(new SixDofConfig(0, 1, -1, 0, -1, 0)));
    }

    public MoveAndHold move0() {
        return new MoveWithProfile(this, pose(new SixDofConfig(0, 0, 0, 0, 0, 0)).p7());
    }

    public MoveAndHold move1() {
        return new MoveWithProfile(this, pose(new SixDofConfig(0, 1, -1, 0, -1, 0)).p7());
    }

    public MoveAndHold move(Pose3d goal) {
        return new MoveWithProfile(this, goal);
    }

}
