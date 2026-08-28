package org.team100.lib.subsystems.six_dof.commands;

/**
 * Move the arm to the goal.
 * 
 * Uses a trajectory in configuration space, so it is
 * immune to singularities (except at the endpoints).
 * 
 * Choose trajectory control points to avoid workspace
 * limits.
 */
public class MoveWithConfigTrajectory {

    public MoveWithConfigTrajectory() {

    }

}
