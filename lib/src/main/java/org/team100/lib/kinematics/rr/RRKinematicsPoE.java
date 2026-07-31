package org.team100.lib.kinematics.rr;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.r2.AccelerationR2;
import org.team100.lib.geometry.rr.RRAcceleration;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rr.RRPose;
import org.team100.lib.geometry.rr.RRVelocity;
import org.team100.lib.geometry.se2.AdjointSE2;
import org.team100.lib.geometry.se2.VelocitySE2;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

/**
 * Using the PoE method, with SE2 poses, not just
 * R2 translations.
 */
public class RRKinematicsPoE {
    /** Proximal link length, meters. */
    private final double l1;
    /** Distal link length, meters. */
    private final double l2;
    private final Pose2d M1;
    private final Pose2d M2;
    private final Pose2d M3;
    private final Twist2d S1;
    private final Twist2d S2;

    public RRKinematicsPoE(double l1, double l2) {
        this.l1 = l1;
        this.l2 = l2;
        S1 = new Twist2d(0, 0, 1);
        S2 = new Twist2d(0, -1, 1);
        M1 = new Pose2d(0, 0, Rotation2d.kZero);
        M2 = new Pose2d(l1, 0, Rotation2d.kZero);
        M3 = new Pose2d(l1 + l2, 0, Rotation2d.kZero);
    }

    public RRPose forward(RRConfig q) {
        Pose2d eS1q1 = GeometryUtil.exp(S1, q.q1());
        Pose2d eS2q2 = GeometryUtil.exp(S2, q.q2());

        Pose2d p1 = eS1q1;
        Pose2d p2 = GeometryUtil.compose(p1, eS2q2);

        return new RRPose(
                M1,
                GeometryUtil.compose(p1, M2),
                GeometryUtil.compose(p2, M3));
    }

    public VelocitySE2 forward(RRConfig q, RRVelocity qdot) {
        Matrix<N3, N2> J = J(q);
        return VelocitySE2.fromVector(J.times(qdot.toVector()));
    }

    /** Construct the Jacobian. */
    Matrix<N3, N2> J(RRConfig q) {
        Pose2d eS1q1 = GeometryUtil.exp(S1, q.q1());
        Pose2d eS2q2 = GeometryUtil.exp(S2, q.q2());
        Pose2d p1 = eS1q1;
        Pose2d p2 = GeometryUtil.compose(p1, eS2q2);
        Pose2d tcp = GeometryUtil.compose(p2, M3);

        // first column is just the q1 axis
        Vector<N3> J1 = GeometryUtil.toVec(S1);
        // second column is the q2 axis transformed by the q1 adjoint
        Vector<N3> J2 = new Vector<>(AdjointSE2.ad(p1).times(GeometryUtil.toVec(S2)));
        // Space Jacobian
        Matrix<N3, N2> Jv = new Matrix<>(Nat.N3(), Nat.N2());
        Jv.assignBlock(0, 0, J1);
        Jv.assignBlock(0, 1, J2);
        // Tool translation
        Matrix<N3, N3> t = MatBuilder.fill(Nat.N3(), Nat.N3(), //
                1, 0, -tcp.getY(), //
                0, 1, tcp.getX(), //
                0, 0, 1);
        return t.times(Jv);
    }

    public AccelerationR2 forward(
            RRConfig q, RRVelocity qdot, RRAcceleration qddot) {
        // TODO: finish this
        return null;
    }
}
