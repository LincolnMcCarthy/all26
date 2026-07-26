package org.team100.lib.kinematics.six_dof;

import java.util.List;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.geometry.six_dof.SixDofPose;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.numbers.N3;

/**
 * Six-DOF kinematics using the Modern Robotics approach.
 * 
 * This is different from the other implementation because we use "x" as the
 * tool axis here, because it makes the whole thing easier to understand.
 */
public class SixDofKinematicsPoE implements SixDofKinematics {

    // Tool center point, in global frame, at zero config
    private final Pose3d M;
    // Screw axes, in global frame, at zero config
    private final Twist3d S1;
    private final Twist3d S2;
    private final Twist3d S3;
    private final Twist3d S4;
    private final Twist3d S5;
    private final Twist3d S6;

    public SixDofKinematicsPoE(double base, double boom, double stick, double tool) {
        // tool is pointing at +x, at full extension
        M = new Pose3d(boom + stick + tool, 0, base, Rotation3d.kZero);
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
        Pose3d eS1q1 = Pose3d.kZero.exp(GeometryUtil.scale(S1, q.q1()));
        Pose3d p1 = eS1q1;
        Pose3d eS2q2 = Pose3d.kZero.exp(GeometryUtil.scale(S2, q.q2()));
        Pose3d p2 = p1.transformBy(new Transform3d(Pose3d.kZero, eS2q2));
        Pose3d eS3q3 = Pose3d.kZero.exp(GeometryUtil.scale(S3, q.q3()));
        Pose3d p3 = p2.transformBy(new Transform3d(Pose3d.kZero, eS3q3));
        Pose3d `eS4q4 = Pose3d.kZero.exp(GeometryUtil.scale(S4, q.q4()));
        Pose3d p4 = p3.transformBy(new Transform3d(Pose3d.kZero, eS4q4));
        Pose3d eS5q5 = Pose3d.kZero.exp(GeometryUtil.scale(S5, q.q5()));
        Pose3d p5 = p4.transformBy(new Transform3d(Pose3d.kZero, eS5q5));
        Pose3d eS6q6 = Pose3d.kZero.exp(GeometryUtil.scale(S6, q.q6()));
        Pose3d p6 = p5.transformBy(new Transform3d(Pose3d.kZero, eS6q6));
        Pose3d p7 = p6.transformBy(new Transform3d(Pose3d.kZero, M));
        return new SixDofPose(
            p1, p2, p3, p4, p5, p6, p7);
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
