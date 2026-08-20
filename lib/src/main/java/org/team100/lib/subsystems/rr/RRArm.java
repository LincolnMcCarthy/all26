package org.team100.lib.subsystems.rr;

import java.util.List;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rr.RRVelocity;
import org.team100.lib.kinematics.rr.RRFeasibility;
import org.team100.lib.kinematics.rr.RRKinematics;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.sim.SimulatedBareMotor;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ControlR2;
import org.team100.lib.state.ModelR1;
import org.team100.lib.state.ModelR2;
import org.team100.lib.subsystems.r2.PositionSubsystemR2;
import org.team100.lib.subsystems.rn.PositionSubsystemRn;
import org.team100.lib.subsystems.rr.commands.MoveWithProfile;
import org.team100.lib.subsystems.rr.commands.MoveWithSpline;
import org.team100.lib.subsystems.rr.commands.MoveWithTrajectoryR2;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Planar RR arm, for training.
 */
public class RRArm extends SubsystemBase
        implements PositionSubsystemR2, PositionSubsystemRn {
    private final LoggerFactory m_log;
    final RRKinematics m_kinematics;
    final RRFeasibility m_feasibility;
    private final BareMotor m_q1;
    private final BareMotor m_q2;

    public RRArm(LoggerFactory parent) {
        m_log = parent.type(this);
        m_kinematics = new RRKinematics(0.3, 0.3);
        m_feasibility = new RRFeasibility(m_kinematics);
        m_q1 = new SimulatedBareMotor(m_log.name("q1"), 600);
        m_q2 = new SimulatedBareMotor(m_log.name("q2"), 600);
    }

    @Override
    public void periodic() {
        m_q1.periodic();
        m_q2.periodic();
    }

    /** TODO: velocity and force in config space. */
    public void setConfig(RRConfig q) {
        m_q1.setUnwrappedPosition(q.q1(), 0, 0);
        m_q2.setUnwrappedPosition(q.q2(), 0, 0);
    }

    /**
     * Choose the feasible config closest to the current config.
     * 
     * @param p tool center point translation
     */
    public RRConfig config(Translation2d p) {
        RRConfig q0 = getConfig();
        List<RRConfig> qAll = m_kinematics.inverse(p, q0.q1());
        if (qAll.isEmpty()) {
            System.out.println("no solution for pose " + StrUtil.transStr(p));
            return null;
        }
        List<RRConfig> qFeasible = m_feasibility.filter(qAll);
        if (qFeasible.isEmpty()) {
            System.out.println("infeasible pose " + StrUtil.transStr(p));
            return null;
        }
        return getBest(qFeasible, q0);
    }

    public RRVelocity qdot(RRConfig q, VelocityR2 xdot) {
        return m_kinematics.inverse(q, xdot);
    }

    /** Current configuration. */
    public RRConfig getConfig() {
        return new RRConfig(
                m_q1.getUnwrappedPositionRad(),
                m_q2.getUnwrappedPositionRad());
    }

    /**
     * Choose config "closest" to q0, using the (non-Euclidean) config distance
     * metric.
     */
    RRConfig getBest(List<RRConfig> qAll, RRConfig q0) {
        double closest = Double.POSITIVE_INFINITY;
        RRConfig best = qAll.get(0);
        for (RRConfig q : qAll) {
            double d = q0.distance(q);
            if (d < closest) {
                closest = d;
                best = q;
            }
        }
        return best;
    }

    public Translation2d pose() {
        return pose(getConfig());
    }

    public Translation2d pose(RRConfig q) {
        return m_kinematics.forward(q).p2();
    }

    public void stop() {
        m_q1.stop();
        m_q2.stop();
    }

    // COMMANDS

    public Command warp0() {
        return run(() -> setConfig(new RRConfig(0, 0)));
    }

    public Command warp1() {
        return run(() -> setConfig(new RRConfig(1, -1)));
    }

    public MoveAndHold moveProfiled(ProfileR1 profile, Translation2d goal) {
        return new MoveWithProfile(this, profile, goal);
    }

    public MoveAndHold moveTrajSE2(Pose2d goal, double speed) {
        return new MoveWithTrajectoryR2(m_log, this, goal, speed);
    }

    public MoveAndHold moveSplined(VelocityR2 x0dot, Translation2d x1, VelocityR2 x1dot) {
        return new MoveWithSpline(m_log, this, x0dot, x1, x1dot);
    }

    @Override
    public ModelR2 getState() {
        // TODO: add velocity
        return new ModelR2(pose());
    }

    @Override
    public List<ModelR1> getStateRn() {
        RRConfig q = getConfig();
        return List.of(
                new ModelR1(q.q1()),
                new ModelR1(q.q2()));
    }

    /** Ignores rotation */
    @Override
    public void set(ControlR2 setpoint) {
        // TODO: add velocity and acceleration.
        setConfig(config(setpoint.translation()));
    }

    @Override
    public void setRn(List<ControlR1> setpoint) {
        RRConfig q = new RRConfig(
                setpoint.get(0).x(),
                setpoint.get(1).x());
        setConfig(q);
    }
}
