package org.team100.lib.dynamics.rr;

import org.team100.lib.geometry.rr.RRAcceleration;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rr.RRVelocity;

public interface RRDynamics {

    /** Compute effort (torque) for each joint, with zero tip force. */
    RREffort effort(
            RRConfig q,
            RRVelocity qdot,
            RRAcceleration qddot);

    /** Compute acceleration for each joint. */
    RRAcceleration qddot(
            RRConfig q,
            RRVelocity qdot,
            RREffort effort);

}