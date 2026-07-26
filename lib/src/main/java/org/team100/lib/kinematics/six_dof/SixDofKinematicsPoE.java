package org.team100.lib.kinematics.six_dof;

import java.util.List;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.geometry.six_dof.SixDofPose;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.numbers.N3;

/**
 * Six-DOF kinematics using the Modern Robotics approach.
 * 
 * This is different from the other implementation because we use "x" as the
 * tool axis here, because it makes the whole thing easier to understand.
 */
public class SixDofKinematicsPoE implements SixDofKinematics {
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
    }

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

    @Override
    public List<SixDofConfig> inverse(Pose3d p, Double q1Default, Double q4Default) {
        return null;
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

}
