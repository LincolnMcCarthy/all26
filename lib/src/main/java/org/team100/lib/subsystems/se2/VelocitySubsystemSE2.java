package org.team100.lib.subsystems.se2;

import org.team100.lib.state.VelocityControlSE2;

/**
 * A planar subsystem controlled by velocity.
 * 
 * We use this interface for robot movement on the floor, where the three
 * dimensions are the dimensions of Pose2d: x, y, and theta.
 */
public interface VelocitySubsystemSE2 extends SubsystemSE2 {

    /**
     * Velocity and acceleration in SE2.
     * 
     * Subsystems are expected to compute the (generalized)
     * force required to meet this setpoint, using the dynamics
     * of the mechanism.
     * 
     * @param setpoint for the next timestep.
     */
    void set(VelocityControlSE2 setpoint);
}