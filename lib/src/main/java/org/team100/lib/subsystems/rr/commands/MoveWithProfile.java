package org.team100.lib.subsystems.rr.commands;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.profile.r1.ProfileR1;
import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ModelR1;
import org.team100.lib.subsystems.rr.RRArm;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Move the arm to the goal, endlessly.
 * 
 * Uses a single profile.
 * 
 * Interpolates each joint separately, so this is
 * immune to singularities (except at the endpoints),
 * but it might exceed the workspace limits.
 */
public class MoveWithProfile extends MoveAndHold {

    private final RRArm m_arm;
    private final Translation2d m_goal;
    /** Profile walks from 0 to 1. */
    private final ProfileR1 m_profile;

    // this is the config at initialize
    private RRConfig m_start;
    // config goal depends on start, to choose the closest one.
    private RRConfig m_configGoal;

    private ControlR1 m_setpoint;
    // for now this is always 1.
    private ModelR1 m_profileGoal;

    public MoveWithProfile(RRArm arm, ProfileR1 profile, Translation2d goal) {
        m_arm = arm;
        m_goal = goal;
        // Check feasibility in constructor to avoid later exception.
        if (m_arm.config(m_goal) == null)
            throw new IllegalArgumentException("infeasible goal");
        m_profile = profile;
        addRequirements(arm);
    }

    @Override
    public void initialize() {

        m_start = m_arm.getConfig();
        m_configGoal = m_arm.config(m_goal);
        // l1 norm treats all joints the same
        // RRConfig.distance weighs the root higher
        // TODO: which is better?
        // double distance =
        // Metrics.l1Norm(m_start.toVector().minus(m_configGoal.toVector()));
        double distance = m_start.distance(m_configGoal);
        if (m_configGoal == null)
            throw new IllegalArgumentException(
                    "infeasible goal: " + StrUtil.transStr(m_goal));
        m_setpoint = new ControlR1();
        // scale the profile to the norm
        m_profileGoal = new ModelR1(distance, 0);
    }

    @Override
    public void execute() {
        double distance = m_profileGoal.x();
        if (distance < 1e-6)
            return;
        m_setpoint = m_profile.calculate(
                TimedRobot100.LOOP_PERIOD_S,
                m_setpoint,
                m_profileGoal);

        double s = m_setpoint.x() / distance;

        RRConfig c = RRConfig.interpolate(m_start, m_configGoal, s);
        m_arm.setConfig(c);
    }

    @Override
    public boolean isDone() {
        return toGo() < 0.01;
    }

    @Override
    public double toGo() {
        return m_arm.getConfig().distance(m_configGoal);
    }

}
