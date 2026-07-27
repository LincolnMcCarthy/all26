package org.team100.lib.kinematics.six_dof;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.geometry.six_dof.SixDofPose;
import org.team100.lib.geometry.six_dof.SphericalWristConfig;
import org.team100.lib.kinematics.rr.RRKinematics;
import org.team100.lib.kinematics.rrr_so3.SphericalWristKinematics;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.numbers.N3;

/**
 * Six-DOF kinematics using the Modern Robotics approach.
 * 
 * The tool axis is +x at zero config.
 */
public class SixDofKinematicsPoE implements SixDofKinematics {
    private static final boolean DEBUG = true;

    // Joint positions, in global frame, at zero config
    private final Pose3d M1;
    private final Pose3d M2;
    private final Pose3d M3;
    private final Pose3d M4;
    private final Pose3d M5;
    private final Pose3d M6;
    private final Pose3d M7;
    // Screw axes, in global frame, at zero config
    private final Twist3d S1;
    private final Twist3d S2;
    private final Twist3d S3;
    private final Twist3d S4;
    private final Twist3d S5;
    private final Twist3d S6;
    /** For solving the positional subproblem */
    private final RRKinematics rrk;
    /** For solving the wrist */
    private final SphericalWristKinematics wk;
    /** Height of the shoulder */
    private final double base;
    /** Tool length from wrist origin, for IK */
    private final double tool;

    public SixDofKinematicsPoE(double base, double boom, double stick, double tool) {
        // base
        M1 = new Pose3d(0, 0, 0, Rotation3d.kZero);
        // shoulder
        M2 = new Pose3d(0, 0, base, Rotation3d.kZero);
        // elbow
        M3 = new Pose3d(boom, 0, base, Rotation3d.kZero);
        // wrist pointing at +x
        M4 = new Pose3d(boom + stick, 0, base, Rotation3d.kZero);
        // wrist pointing at +x
        M5 = new Pose3d(boom + stick, 0, base, Rotation3d.kZero);
        // wrist pointing at +x, this is tool flange
        M6 = new Pose3d(boom + stick, 0, base, Rotation3d.kZero);
        // tool point, is pointing at +x, at full extension
        M7 = new Pose3d(boom + stick + tool, 0, base, Rotation3d.kZero);
        // joint 1 (base) is at the origin, around z
        S1 = S(VecBuilder.fill(0, 0, 1), VecBuilder.fill(0, 0, 0));
        // joint 2 (shoulder) is at z=base, around -y
        S2 = S(VecBuilder.fill(0, -1, 0), VecBuilder.fill(0, 0, base));
        // joint 3 (elbow) is at x=boom, z=base, around -y
        S3 = S(VecBuilder.fill(0, -1, 0), VecBuilder.fill(boom, 0, base));
        // joint 4 (wrist roll) is at x=boom+stick, z=base, around +x
        S4 = S(VecBuilder.fill(1, 0, 0), VecBuilder.fill(boom + stick, 0, base));
        // joint 5 (wrist pitch) is at x=boom+stick, z=base, around -y
        S5 = S(VecBuilder.fill(0, -1, 0), VecBuilder.fill(boom + stick, 0, base));
        // joint 6 (tool roll) is at x=boom+stick, z=base, around +x
        S6 = S(VecBuilder.fill(1, 0, 0), VecBuilder.fill(boom + stick, 0, base));
        this.base = base;
        this.tool = tool;

        rrk = new RRKinematics(boom, stick);
        wk = new SphericalWristKinematics();
    }

    /** Compose exponentials for each joint pose. */
    @Override
    public SixDofPose forward(SixDofConfig q) {
        Pose3d eS1q1 = GeometryUtil.exp(S1, q.q1());
        Pose3d eS2q2 = GeometryUtil.exp(S2, q.q2());
        Pose3d eS3q3 = GeometryUtil.exp(S3, q.q3());
        Pose3d eS4q4 = GeometryUtil.exp(S4, q.q4());
        Pose3d eS5q5 = GeometryUtil.exp(S5, q.q5());
        Pose3d eS6q6 = GeometryUtil.exp(S6, q.q6());
        Pose3d p1 = eS1q1;
        Pose3d p2 = GeometryUtil.compose(p1, eS2q2);
        Pose3d p3 = GeometryUtil.compose(p2, eS3q3);
        Pose3d p4 = GeometryUtil.compose(p3, eS4q4);
        Pose3d p5 = GeometryUtil.compose(p4, eS5q5);
        Pose3d p6 = GeometryUtil.compose(p5, eS6q6);
        // return *all* the poses
        return new SixDofPose(
                M1,
                GeometryUtil.compose(p1, M2),
                GeometryUtil.compose(p2, M3),
                GeometryUtil.compose(p3, M4),
                GeometryUtil.compose(p4, M5),
                GeometryUtil.compose(p5, M6),
                GeometryUtil.compose(p6, M7));
    }

    /**
     * Inverse position kinematics: joint configs from cartesian pose.
     *
     * Zero, one, two, four, or eight solutions.
     * 
     * For defaults, use the previous value, or null if you have no idea (and in
     * that case, catch the exception that may occur).
     * 
     * @param p         Tool point pose.
     * @param q1Default In case of base singularity.
     * @param q4Default In case of wrst singularity.
     */
    @Override
    public List<SixDofConfig> inverse(Pose3d p, Double q1Default, Double q4Default) {
        Translation3d t = p.getTranslation();
        if (DEBUG)
            System.out.printf("t %s\n", StrUtil.transStr(t));

        // Wrist rotation is tool rotation.
        Rotation3d R = p.getRotation();

        // Tool translation = tool translation in tool frame, rotated by R.
        Translation3d b = new Translation3d(tool, 0, 0).rotateBy(R);
        // Wrist origin = start at tool point, walk backwards along tool.
        Translation3d w = t.minus(b);
        if (DEBUG)
            System.out.printf("w %s\n", StrUtil.transStr(w));
        Translation2d w2d = w.toTranslation2d();
        // One or two swing options
        List<Double> q1List = getQ1(w2d, q1Default);
        if (DEBUG)
            System.out.printf("swing options %d\n", q1List.size());
        List<SixDofConfig> result = new ArrayList<>();
        for (double q1 : q1List) {
            if (DEBUG)
                System.out.printf("swing %f\n", q1);
            List<RRConfig> rrs = rrConfig(w, q1);
            if (DEBUG)
                System.out.printf("RR options %d\n", rrs.size());
            for (RRConfig rr : rrs) {
                double q2 = rr.q1();
                double q3 = rr.q2();
                if (DEBUG)
                    System.out.printf("q2 %f q3 %f\n ", q2, q3);
                List<SphericalWristConfig> wqs = wristQ(R, wristOrigin(q1, q2, q3), q4Default);
                if (DEBUG)
                    System.out.printf("wrist options %d\n", wqs.size());
                for (SphericalWristConfig wq : wqs) {
                    if (DEBUG)
                        System.out.printf("q4 %f q5 %f q6 %f\n", wq.q4(), wq.q5(), wq.q6());
                    result.add(new SixDofConfig(q1, q2, q3, wq.q4(), wq.q5(), wq.q6()));
                }
            }
        }
        return result;
    }

    /**
     * Screw axis
     * 
     * @param So S_omega, the axis of rotation in the global frame
     * @param a  any point on the axis
     * @return The screw axis of the joint, in the global frame.
     */
    static Twist3d S(Vector<N3> So, Vector<N3> a) {
        Vector<N3> Sv = Vector.cross(a, So);
        return new Twist3d(Sv.get(0), Sv.get(1), Sv.get(2), So.get(0), So.get(1), So.get(2));
    }

    /**
     * Swing joint. Wrist origin must be in the swing plane. One or two solutions.
     * 
     * In the non-singular case, there are two alternatives here: the "no-flip"
     * case, shoulder near zero, and the "flip" case, with the base pointing the
     * opposite way and the shoulder pointing "back" to the same result.
     * 
     * @param w         Wrist position in the xy plane
     * @param q1Default Used if the position is the origin. A good choice would be
     *                  the previous value of q1. If you have no idea, pass null and
     *                  catch the exception.
     */
    static List<Double> getQ1(Translation2d w, Double q1Default) {
        if (w.getNorm() < 1e-3) {
            if (DEBUG)
                System.out.println("base singularity");
            if (q1Default == null)
                throw new IllegalArgumentException("q1Default is null");
            // in this case we don't do both alternatives, just the one default.
            return List.of(q1Default);
        }
        double radians = w.getAngle().getRadians();
        return List.of(radians, MathUtil.angleModulus(radians + Math.PI));
    }

    /**
     * 0, 1, or 2 solutions
     * 
     * @param w  wrist position
     * @param q1 swing configuration
     */
    private List<RRConfig> rrConfig(Translation3d w, double q1) {
        // Is this the "inline" or the "flip" case?
        Rotation2d rot = w.toTranslation2d().getAngle();
        double signum = 0;
        if (MathUtil.isNear(q1, rot.getRadians(), 1e-3))
            // forward
            signum = 1;
        else
            // backward
            signum = -1;
        // Horizontal distance from base to wrist.
        double x = Math.hypot(w.getX(), w.getY()) * signum;
        // Vertical distance from base to wrist..
        double y = w.getZ() - base;
        // RR sub-problem.
        Translation2d end = new Translation2d(x, y);
        // Find the RR configs
        return rrk.inverse(end);
    }

    /**
     * Wrist config. One (if singular) or two solutions.
     * 
     * @param R         tool origin rotation
     * @param R04       wrist origin rotation
     * @param q4Default in case of singularity, pass null if you have no idea.
     */
    private List<SphericalWristConfig> wristQ(Rotation3d R, Rotation3d R04, Double q4Default) {
        // The RPR wrist rotation is whatever is left.
        Rotation3d R36 = R.relativeTo(R04);
        List<SphericalWristConfig> wq = wk.inverse(R36, q4Default);
        return wq;
    }

    /** The rotation of the wrist origin */
    private Rotation3d wristOrigin(double q1, double q2, double q3) {
        // Each joint pose up to the wrist.

        Pose3d eS1q1 = GeometryUtil.exp(S1, q1);
        Pose3d eS2q2 = GeometryUtil.exp(S2, q2);
        Pose3d eS3q3 = GeometryUtil.exp(S3, q3);
        Pose3d eS4q4 = GeometryUtil.exp(S4, 0);

        Pose3d p1 = eS1q1;
        Pose3d p2 = GeometryUtil.compose(p1, eS2q2);
        Pose3d p3 = GeometryUtil.compose(p2, eS3q3);
        // Wrist origin.
        Pose3d p4 = GeometryUtil.compose(p3, eS4q4);

        // The rotation for zero wrist roll.
        Rotation3d R04 = p4.getRotation();
        if (DEBUG)
            System.out.printf("R04 %s\n", StrUtil.rotStr(R04));
        return R04;
    }

}
