package org.team100.lib.kinematics.six_dof;

import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.kinematics.rr.RRKinematics;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.MathUtil;
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

    record WristConfig(double q4, double q5, double q6) {
    }

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
     * 
     * If q5 is zero, the wrist is in a singularity, and q4 and q6 should be handled
     * differently.
     */
    public SixDofConfig inverse(Pose3d p) {
        Translation3d t = p.getTranslation();
        if (DEBUG)
            System.out.printf("t %s\n", StrUtil.transStr(t));

        Rotation3d R = p.getRotation();

        // Tool translation = tool translation in tool frame, rotated by R.
        Translation3d b = new Translation3d(0, 0, tool).rotateBy(R);
        // Wrist origin = start at tool point, walk backwards along tool.
        Translation3d w = t.minus(b);
        if (DEBUG)
            System.out.printf("w %s\n", StrUtil.transStr(w));
        // Note: IEEE 754 defined atan2(0,0) as 0 in 1985.  It's wrong.
        Translation2d w2d = w.toTranslation2d();
        if (w2d.getNorm() < 1e-3) {
            if (DEBUG)
                System.out.println("base singularity");
        }
        // Swing joint = wrist origin must be in the swing plane.
        double q1 = w2d.getAngle().getRadians();
        // Horizontal distance from base to wrist.
        double twoDofX = Math.hypot(w.getX(), w.getY()) * Math.signum(w.getX());
        // Vertical distance from base to wrist..
        double twoDofY = w.getZ() - base;
        // RR sub-problem
        Translation2d twoDofEnd = new Translation2d(twoDofX, twoDofY);
        RRConfig twoDofConfig = twodof.inverse(twoDofEnd);
        double q2 = twoDofConfig.q1();
        double q3 = twoDofConfig.q2();

        // Each joint pose up to the wrist.
        Pose3d p1 = Pose3d.kZero.plus(t1(q1));
        Pose3d p2 = p1.plus(t2(q2));
        Pose3d p3 = p2.plus(t3(q3));
        // The pose when the wrist roll is zero.
        Pose3d p4 = p3.plus(t4(0));
        // The rotation for zero wrist roll.
        Rotation3d R04 = p4.getRotation();
        if (DEBUG)
            System.out.printf("R04 %s\n", StrUtil.rotStr(R04));

        // The RPR wrist rotation is whatever is left.
        Rotation3d R36 = R.relativeTo(R04);
        WristConfig wq = wristInverse(R36);

        return new SixDofConfig(q1, q2, q3, wq.q4, wq.q5, wq.q6);
    }

    /**
     * Decomposition of R into ZXZ Euler angles.
     * 
     * If q5 is zero, the wrist is in a singularity, and q4 and q6 should be handled
     * differently.
     *
     * https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/KinematicsSingleBody.pdf
     */
    static WristConfig wristInverse(Rotation3d R) {
        if (DEBUG)
            System.out.printf("R %s\n", StrUtil.rotStr(R));
        Matrix<N3, N3> r = R.toMatrix();
        if (DEBUG)
            System.out.printf("R %s\n", StrUtil.matStr(r));
        double r13 = r.get(0, 2);
        double r23 = r.get(1, 2);
        double r31 = r.get(2, 0);
        double r32 = r.get(2, 1);
        double r33 = r.get(2, 2);

        if (MathUtil.isNear(1, r33, 1e-2)) {
            if (DEBUG)
                System.out.println("wrist singularity");
        }

        double q4 = Math.atan2(r13, -r23);
        // Negative sign here because our convention for the orientation of q5 is
        // opposite the ZXZ convention.
        double q5 = -1.0 * Math.atan2(Math.sqrt(Math.pow(r13, 2) + Math.pow(r23, 2)), r33);
        double q6 = Math.atan2(r31, r32);
        return new WristConfig(q4, q5, q6);
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
