package org.team100.lib.kinematics.rrr_so3;

import java.util.List;

import org.team100.lib.geometry.six_dof.SphericalWristConfig;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

/**
 * The RPR spherical wrist has three joints with intersecting axes:
 * 
 * * roll: rotates around z
 * * pitch: bends the z axis
 * * roll: rotates around z again
 */
public class SphericalWristKinematics {
    private static final boolean DEBUG = true;

    /** Forward kinematics is simply a composition of rotations. */
    public Rotation3d forward(SphericalWristConfig q) {
        Pose3d p4 = Pose3d.kZero.plus(R(q.q4()));
        Pose3d p5 = p4.plus(o5()).plus(R(q.q5()));
        Pose3d p6 = p5.plus(o6()).plus(R(q.q6()));
        if (DEBUG) {
            System.out.printf("p4  %s\n", StrUtil.poseStr2(p4));
            System.out.printf("p5  %s\n", StrUtil.poseStr2(p5));
            System.out.printf("p6  %s\n", StrUtil.poseStr2(p6));
        }
        return p6.getRotation();
    }

    /**
     * Decomposition of R into ZXZ Euler angles.
     * 
     * If q5 is zero, the wrist is in a singularity, and q4 and q6 should be handled
     * differently.
     * 
     * There are two (nonsingular) configurations for every rotation, so this returns a list.
     * 
     * I'm not sure the names of these two configurations have any significance,
     * so they're just in a list.
     *
     * https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/KinematicsSingleBody.pdf
     */
    public List<SphericalWristConfig> inverse(Rotation3d R) {
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
            // TODO: handle this case
        }

        double q4 = Math.atan2(r13, -r23);
        // Negative sign here because our convention for the orientation of q5 is
        // opposite the ZXZ convention.
        double q5 = -1.0 * Math.atan2(Math.sqrt(Math.pow(r13, 2) + Math.pow(r23, 2)), r33);
        double q6 = Math.atan2(r31, r32);

        // There are two solutions, the one we found...
        SphericalWristConfig s1 = new SphericalWristConfig(q4, q5, q6);
        // ...and the "flipped" solution, q4+pi, -q5, q6+pi.
        SphericalWristConfig s2 = new SphericalWristConfig(
                MathUtil.angleModulus(q4 + Math.PI),
                -q5,
                MathUtil.angleModulus(q6 + Math.PI));
        return List.of(s1, s2);
    }

    /** Origin of joint 5: no offset, parallel to other pitch axes. */
    private Transform3d o5() {
        return new Transform3d(
                Translation3d.kZero,
                new Rotation3d(0, -Math.PI / 2, 0));
    }

    /** Origin of joint 6: no offset, roll points out. */
    private Transform3d o6() {
        return new Transform3d(
                Translation3d.kZero,
                new Rotation3d(0, Math.PI / 2, 0));
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
