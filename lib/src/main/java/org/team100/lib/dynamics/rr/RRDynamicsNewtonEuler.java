package org.team100.lib.dynamics.rr;

/**
 * Port of Modern Robotics dynamics code.
 * 
 * https://github.com/NxRLab/ModernRobotics/blob/master/packages/Python/modern_robotics/core.py
 */
public class RRDynamicsNewtonEuler {

    /**
     * Computes forward dynamics in the space frame for an open chain robot.
     * 
     * This function computes ddthetalist by solving:
     * Mlist(thetalist) * ddthetalist = taulist - c(thetalist,dthetalist) -
     * g(thetalist) - Jtr(thetalist) * Ftip
     * 
     * 
     * @param thetalist:  A list of joint variables
     * @param dthetalist: A list of joint rates
     * @param taulist:    An n-vector of joint forces/torques
     * @param g:          Gravity vector g
     * @param Ftip:       Spatial force applied by the end-effector expressed in
     *                    frame {n+1}
     * @param Mlist:      List of link frames i relative to i-1 at the home position
     * @param Glist:      Spatial inertia matrices Gi of the links
     * @param Slist:      Screw axes Si of the joints in a space frame, in the
     *                    format of a matrix with axes as the columns
     * @return: The resulting joint accelerations
     */
    void ForwardDynamics(
            int thetalist,
            int dthetalist,
            int taulist,
            int g,
            int Ftip,
            int Mlist,
            int Glist,
            int Slist) {
    }

    /**
     * Computes inverse dynamics in the space frame for an open chain robot.
     * 
     * This function uses forward-backward Newton-Euler iterations to solve the
     * equation:
     * 
     * taulist = Mlist(thetalist)ddthetalist + c(thetalist,dthetalist) +
     * g(thetalist) + Jtr(thetalist)Ftip
     * 
     * @param thetalist   n-vector of joint variables
     * @param dthetalist  n-vector of joint rates
     * @param ddthetalist n-vector of joint accelerations
     * @param g           Gravity vector g
     * @param Ftip        Spatial force applied by the end-effector expressed in
     *                    frame {n+1}
     * @param Mlist       List of link frames {i} relative to {i-1} at the home
     *                    position
     * @param Glist       Spatial inertia matrices Gi of the links
     * @param Slist       Screw axes Si of the joints in a space frame, in the
     *                    format of a matrix with axes as the columns
     * @return The n-vector of required joint forces/torques
     */
    void InverseDynamics(
            int thetalist,
            int dthetalist,
            int ddthetalist,
            int g,
            int Ftip,
            int Mlist,
            int Glist, int Slist) {

    }
}
