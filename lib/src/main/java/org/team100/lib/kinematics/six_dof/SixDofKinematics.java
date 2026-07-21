package org.team100.lib.kinematics.six_dof;

import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.kinematics.rr.RRKinematics;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

/**
 * Kinematics of six-DOF all-revolute arm with spherical wrist, e.g. PUMA.
 * 
 * In this implementation, the joint axis is always +z, and the link transform
 * is arranged to make that work.
 */
public class SixDofKinematics {
    private static final boolean DEBUG = true;
    /** height of the shoulder */
    private final double base;
    /** Boom length between shoulder and elbow */
    private final double boom;
    /** Stick length from elbow to wrist */
    private final double stick;
    /** Tool length from wrist origin. */
    private final double tool;
    /** For solving the positional subproblem */
    private final RRKinematics twodof;

    public SixDofKinematics() {
        base = 0.25;
        boom = 0.75;
        stick = 0.75;
        tool = 0.15;
        twodof = new RRKinematics(boom, stick);
    }

    public Pose3d forward(SixDofConfig q) {
        Pose3d p1 = Pose3d.kZero.plus(t1(q.q1()));
        Pose3d p2 = p1.plus(t2(q.q2()));
        Pose3d p3 = p2.plus(t3(q.q3()));
        Pose3d p4 = p3.plus(t4(q.q4()));
        Pose3d p5 = p4.plus(t5(q.q5()));
        Pose3d p6 = p5.plus(t6(q.q6()));
        Pose3d tcp = p6.plus(tool());
        if (DEBUG) {
            System.out.printf("p1  %s\n", StrUtil.poseStr2(p1));
            System.out.printf("p2  %s\n", StrUtil.poseStr2(p2));
            System.out.printf("p3  %s\n", StrUtil.poseStr2(p3));
            System.out.printf("p4  %s\n", StrUtil.poseStr2(p4));
            System.out.printf("p5  %s\n", StrUtil.poseStr2(p5));
            System.out.printf("p6  %s\n", StrUtil.poseStr2(p6));
            System.out.printf("tcp %s\n", StrUtil.poseStr2(tcp));
        }
        return tcp;
    }

    /**
     * Similar to the Lynx arm case.
     * For now this ignores the singularity on the swing axis.
     */
    public SixDofConfig inverse(Pose3d p) {
        Translation3d t = p.getTranslation();
        Rotation3d R = p.getRotation();

        // Tool translation.
        Translation3d b = new Translation3d(tool, p.getRotation());
        // Wrist origin.
        Translation3d w = t.minus(b);
        double q1 = w.toTranslation2d().getAngle().getRadians();

        double hypot = Math.hypot(w.getX(), w.getY()) * Math.signum(w.getX());
        // 3d z becomes y, offset by the shoulder height.
        double twoDofY = w.getZ() - base;
        Translation2d twoDofEnd = new Translation2d(
                hypot,
                twoDofY);
        RRConfig twoDofConfig = twodof.inverse(twoDofEnd);
        double q2 = twoDofConfig.q1();
        double q3 = twoDofConfig.q2();

        // now compute the full pose at the wrist parent.
        Pose3d p1 = Pose3d.kZero.plus(t1(q1));
        Pose3d p2 = p1.plus(t2(q2));
        Pose3d p3 = p2.plus(t3(q3));

        // The rotation of the wrist parent
        Rotation3d R03 = p3.getRotation();

        // The wrist rotation
        Rotation3d R36 = R.relativeTo(R03);

        Matrix<N3, N3> r = R36.toMatrix();
        // ZXZ case
        double q4 = Math.atan2(r.get(1, 3), -r.get(2, 3));
        double q5 = Math.atan2(Math.sqrt(Math.pow(r.get(1, 3), 2) + Math.pow(r.get(2, 3), 2)), r.get(3, 3));
        double q6 = Math.atan2(r.get(3, 1), r.get(3, 2));

        // Decompose the rotation into roll-pitch-roll components.

        return new SixDofConfig(q1, q2, q3, q4, q5, q6);
    }

    private Transform3d t1(double q1) {
        // swing, no offset, rotate around z
        return new Transform3d(
                Translation3d.kZero,
                Rotation3d.kZero).plus(R(q1));
    }

    private Transform3d t2(double q2) {
        // up 25 cm, shoulder axis points right
        return new Transform3d(
                new Translation3d(0, 0, base),
                new Rotation3d(Math.PI / 2, 0, 0)).plus(R(q2));
    }

    private Transform3d t3(double q3) {
        // out 75 cm, parallel axis
        return new Transform3d(
                new Translation3d(boom, 0, 0),
                Rotation3d.kZero).plus(R(q3));
    }

    private Transform3d t4(double q4) {
        // out 75cm, wrist roll points out
        return new Transform3d(
                new Translation3d(stick, 0, 0),
                new Rotation3d(0, Math.PI / 2, 0)).plus(R(q4));
    }

    private Transform3d t5(double q5) {
        // no offset, wrist pitch parallel to the other pitch axes
        return new Transform3d(
                Translation3d.kZero,
                new Rotation3d(0, -Math.PI / 2, 0)).plus(R(q5));
    }

    private Transform3d t6(double q6) {
        // no offset, roll points out
        return new Transform3d(
                Translation3d.kZero,
                new Rotation3d(0, Math.PI / 2, 0)).plus(R(q6));
    }

    private Transform3d tool() {
        // tool is just offset
        return new Transform3d(
                new Translation3d(0, 0, tool),
                Rotation3d.kZero);
    }

    /** Rotate in child frame */
    private Transform3d R(double q) {
        return new Transform3d(Translation3d.kZero, new Rotation3d(v(0, 0, 1), q));
    }

    /** convenience method for vector */
    private Vector<N3> v(double x, double y, double z) {
        return VecBuilder.fill(x, y, z);
    }

}
