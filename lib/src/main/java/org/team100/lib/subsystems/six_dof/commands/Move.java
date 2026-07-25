package org.team100.lib.subsystems.six_dof.commands;

import org.team100.lib.subsystems.six_dof.SixDofArm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Move the arm to the goal.
 * 
 * Interpolates each joint separately, so this is
 * immune to singularities (except at the endpoints),
 * but it might exceed the workspace limits, e.g.
 * by hitting the floor.
 */
public class Move extends Command {

    private final SixDofArm m_arm;
    private final Pose3d m_goal;

    private Pose3d m_start;

    public Move(SixDofArm arm, Pose3d goal) {
        m_arm = arm;
        m_goal = goal;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_start = m_arm.getPose().p7();
    }

    @Override
    public void execute() {
    }

}
