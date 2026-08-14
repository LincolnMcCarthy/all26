package org.team100.lib.dynamics.r;

public interface RDynamics {

    /** Compute effort (torque) for each joint, with zero tip force. */
    REffort effort(
            RConfig q,
            RVelocity qdot,
            RAcceleration qddot);

    /** Compute acceleration for each joint. */
    RAcceleration qddot(
            RConfig q,
            RVelocity qdot,
            REffort effort);

}