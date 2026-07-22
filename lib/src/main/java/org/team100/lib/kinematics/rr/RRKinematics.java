package org.team100.lib.kinematics.rr;

import org.team100.lib.geometry.r2.AccelerationR2;
import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.geometry.rr.RRAcceleration;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rr.RRPosition;
import org.team100.lib.geometry.rr.RRVelocity;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;

/**
 * Planar serial RR arm kinematics: two revolute joints and two links.
 * 
 * Note these kinematics always choose the "elbow up" configuration,
 * i.e. the distal joint prefers negative values.
 * 
 * Refer to the diagram:
 * https://docs.google.com/document/d/1B6vGPtBtnDSOpfzwHBflI8-nn98W9QvmrX78bon8Ajw
 */
public class RRKinematics {
    private static final boolean DEBUG = true;

    // we use these containers instead of lists so that there's no
    // question about which solution is which.
    public record ConfigSolution(RRConfig elbowUp, RRConfig elbowDown) {
    }

    public record VelocitySolution(RRVelocity elbowUp, RRVelocity elbowDown) {
    }

    public record AccelerationSolution(RRAcceleration elbowUp, RRAcceleration elbowDown) {
    }

    /** Proximal link length, meters. */
    private final double l1;
    /** Distal link length, meters. */
    private final double l2;

    /**
     * @param l1 Proximal link length, meters.
     * @param l2 Distal link length, meters.
     */
    public RRKinematics(double l1, double l2) {
        this.l1 = l1;
        this.l2 = l2;
    }

    /**
     * Forward position kinematics: cartesian position from joint configuration.
     * 
     * x = f(q)
     */
    public RRPosition forward(RRConfig q) {
        double x1 = l1 * Math.cos(q.q1());
        double y1 = l1 * Math.sin(q.q1());
        double x2 = x1 + l2 * Math.cos(q.q2() + q.q1());
        double y2 = y1 + l2 * Math.sin(q.q2() + q.q1());
        return new RRPosition(
                new Translation2d(x1, y1),
                new Translation2d(x2, y2));
    }

    /**
     * Forward velocity kinematics: cartesian velocity from joint configuration and
     * velocity.
     * 
     * \dot{x} = J \dot{q}
     */
    public VelocityR2 forward(RRConfig q, RRVelocity qdot) {
        Matrix<N2, N2> J = J(q);
        return VelocityR2.fromVector2(J.times(qdot.toVector()));
    }

    /**
     * Forward acceleration kinematics.
     * 
     * \ddot{x} = \dot{J} \dot{q} + J \ddot{q}
     */
    public AccelerationR2 forward(
            RRConfig q, RRVelocity qdot, RRAcceleration qddot) {
        Matrix<N2, N2> J = J(q);
        Matrix<N2, N2> Jdot = Jdot(q, qdot);
        return AccelerationR2.fromVector(
                Jdot.times(qdot.toVector()).plus(J.times(qddot.toVector())));
    }

    /**
     * Inverse position kinematics: joint configuration from cartesian position.
     * 
     * q = f(x)
     * 
     * Note there are two possible configurations, "elbow up" and "elbow down",
     * so this returns a list.
     * 
     * Refer to the diagram, or README.md
     * https://docs.google.com/document/d/1B6vGPtBtnDSOpfzwHBflI8-nn98W9QvmrX78bon8Ajw
     */
    public ConfigSolution inverse(Translation2d x) {
        if (DEBUG)
            System.out.printf("t %s\n", StrUtil.transStr(x));
        // Use law of cosines.
        double r = x.getNorm();
        double gamma = Math.atan2(x.getY(), x.getX());
        double c1 = (r * r + l1 * l1 - l2 * l2) / (2 * r * l1);
        double beta = Math.acos(c1);
        double c2 = (l1 * l1 + l2 * l2 - r * r) / (2 * l1 * l2);
        double alpha = Math.acos(c2);

        if (Double.isNaN(alpha) || Double.isNaN(beta) || Double.isNaN(gamma))
            throw new IllegalArgumentException(String.format("invalid two-dof parameter %s", x));

        // There are two solutions, "elbow up" ...
        double q1up = MathUtil.angleModulus(gamma + beta);
        double q2up = MathUtil.angleModulus(alpha + Math.PI);
        // ... and "elbow down":
        double q1down = MathUtil.angleModulus(gamma - beta);
        double q2down = -q2up;

        return new ConfigSolution(
                new RRConfig(q1up, q2up), new RRConfig(q1down, q2down));
    }

    /**
     * Inverse velocity kinematics.
     * 
     * \dot{q} = J^{-1} \dot{x}
     */
    public VelocitySolution inverse(Translation2d x, VelocityR2 xdot) {
        ConfigSolution q = inverse(x);

        Matrix<N2, N2> JinvUp = Jinv(q.elbowUp);
        RRVelocity vUp = RRVelocity.fromVector(JinvUp.times(xdot.toVector()));

        Matrix<N2, N2> JinvDown = Jinv(q.elbowDown);
        RRVelocity vDown = RRVelocity.fromVector(JinvDown.times(xdot.toVector()));

        return new VelocitySolution(vUp, vDown);
    }

    /**
     * Inverse acceleration kinematics.
     * 
     * \ddot{q} = J^{-1}(\ddot{x} - \dot{J} J^{-1} \dot{x})
     * 
     * See doc/README.md equation 9
     */
    public AccelerationSolution inverse(Translation2d x, VelocityR2 xdot, AccelerationR2 xddot) {
        ConfigSolution s = inverse(x);

        RRAcceleration up = getJdot(xdot, xddot, s.elbowUp);
        RRAcceleration down = getJdot(xdot, xddot, s.elbowDown);

        return new AccelerationSolution(up, down);
    }

    private RRAcceleration getJdot(VelocityR2 xdot, AccelerationR2 xddot, RRConfig q) {
        Matrix<N2, N2> Jinv = Jinv(q);
        RRVelocity qdot = RRVelocity.fromVector(Jinv.times(xdot.toVector()));
        Matrix<N2, N2> Jdot = Jdot(q, qdot);
        return RRAcceleration.fromVector(
                Jinv.times(
                        xddot.toVector().minus(
                                Jdot.times(Jinv.times(xdot.toVector())))));
    }

    ////////////////////////////////////////////////////////////////////

    /**
     * End-effector Jacobian.
     */
    private Matrix<N2, N2> J(RRConfig q) {
        double s1 = Math.sin(q.q1());
        double c1 = Math.cos(q.q1());
        double s12 = Math.sin(q.q1() + q.q2());
        double c12 = Math.cos(q.q1() + q.q2());
        return MatBuilder.fill(Nat.N2(), Nat.N2(),
                -l1 * s1 - l2 * s12, -l2 * s12, //
                l1 * c1 + l2 * c12, l2 * c12);
    }

    /**
     * Time-derivative of the end-effector Jacobian.
     */
    private Matrix<N2, N2> Jdot(RRConfig q, RRVelocity qdot) {
        double s1 = Math.sin(q.q1());
        double c1 = Math.cos(q.q1());
        double s12 = Math.sin(q.q1() + q.q2());
        double c12 = Math.cos(q.q1() + q.q2());
        double q1dot = qdot.q1dot();
        double q2dot = qdot.q2dot();
        return MatBuilder.fill(Nat.N2(), Nat.N2(), //
                -l1 * c1 * q1dot - l2 * c12 * (q1dot + q2dot), -l1 * c12 * (q1dot + q2dot), //
                -l1 * s1 * q1dot - l2 * s12 * (q1dot + q2dot), -l2 * s12 * (q1dot + q2dot));
    }

    /**
     * Inverse Jacobian, or zero if singular.
     */
    private Matrix<N2, N2> Jinv(RRConfig q) {
        Matrix<N2, N2> J = J(q);
        if (Math.abs(J.det()) < 1e-3) {
            // not invertible
            System.out.printf("WARNING: zero jacobian for config %s\n", q.toString());
            return new Matrix<>(Nat.N2(), Nat.N2());
        }
        return J.inv();
    }
}