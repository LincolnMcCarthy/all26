package org.team100.lib.kinematics.six_dof;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.geometry.six_dof.SixDofPose;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.numbers.N3;

public class SixDofKinematicsPoETest {
    /**
     * Example from
     * https://github.com/madibabaiasl/modern-robotics-course/wiki/Lesson-7:-Forward-Kinematics-of-Robot-Arms-Using-Screw-Theory
     */
    @Test
    void testS1() {
        // axis +z
        Vector<N3> So = VecBuilder.fill(0, 0, 1);
        // at the origin
        Vector<N3> a = VecBuilder.fill(0, 0, 0);
        Twist3d S = SixDofKinematicsPoE.S(So, a);
        verify(new Twist3d(0, 0, 0, 0, 0, 1), S);
    }

    @Test
    void testS2() {
        // axis +z
        Vector<N3> So = VecBuilder.fill(0, 0, 1);
        // offset by (say) x=1
        Vector<N3> a = VecBuilder.fill(1, 0, 0);
        Twist3d S = SixDofKinematicsPoE.S(So, a);
        verify(new Twist3d(0, -1, 0, 0, 0, 1), S);
    }

    @Test
    void testS3() {
        // axis +z
        Vector<N3> So = VecBuilder.fill(0, 0, 1);
        // offset by (say) x=2
        Vector<N3> a = VecBuilder.fill(2, 0, 0);
        Twist3d S = SixDofKinematicsPoE.S(So, a);
        verify(new Twist3d(0, -2, 0, 0, 0, 1), S);
    }

    @Test
    void testForward1() {
        SixDofKinematicsPoE k = new SixDofKinematicsPoE(0.25, 0.75, 0.75, 0.15);
        SixDofPose p = k.forward(new SixDofConfig(0, 0, 0, 0, 0, 0));
        verify(new Pose3d(0, 0, 0, Rotation3d.kZero), p.p1());
        verify(new Pose3d(0, 0, 0.25, Rotation3d.kZero), p.p2());
        verify(new Pose3d(0.75, 0, 0.25, Rotation3d.kZero), p.p3());
        verify(new Pose3d(1.5, 0, 0.25, Rotation3d.kZero), p.p4());
        verify(new Pose3d(1.5, 0, 0.25, Rotation3d.kZero), p.p5());
        verify(new Pose3d(1.5, 0, 0.25, Rotation3d.kZero), p.p6());
        // note the zero rotation, different than the other implementation.
        verify(new Pose3d(1.65, 0, 0.25, Rotation3d.kZero), p.p7());
    }

    @Test
    void testForward1a() {
        SixDofKinematicsPoE k = new SixDofKinematicsPoE(0.25, 0.75, 0.75, 0.15);
        SixDofPose p = k.forward(new SixDofConfig(Math.PI / 2, 0, 0, 0, 0, 0));
        verify(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)), p.p1());
        verify(new Pose3d(0, 0, 0.25, new Rotation3d(0, 0, Math.PI / 2)), p.p2());
        verify(new Pose3d(0, 0.75, 0.25, new Rotation3d(0, 0, Math.PI / 2)), p.p3());
        verify(new Pose3d(0, 1.5, 0.25, new Rotation3d(0, 0, Math.PI / 2)), p.p4());
        verify(new Pose3d(0, 1.5, 0.25, new Rotation3d(0, 0, Math.PI / 2)), p.p5());
        verify(new Pose3d(0, 1.5, 0.25, new Rotation3d(0, 0, Math.PI / 2)), p.p6());
        // note the zero rotation, different than the other implementation.
        verify(new Pose3d(0, 1.65, 0.25, new Rotation3d(0, 0, Math.PI / 2)), p.p7());
    }

    @Test
    void testForward2() {
        // point up
        SixDofKinematics k = new SixDofKinematicsPoE(0.25, 0.75, 0.75, 0.15);
        SixDofConfig q = new SixDofConfig(
                0, // yaw +x
                Math.PI / 2, // shoulder up
                -Math.PI / 2, // elbow out
                0, // use pitch axis for pitch
                Math.PI / 2, // pitch up
                0);
        SixDofPose p = k.forward(q);
        // tool is pointing up
        verify(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)), p.p1());
        verify(new Pose3d(0, 0, 0.25, new Rotation3d(0, 0, 0)), p.p2());
        verify(new Pose3d(0, 0, 1.0, new Rotation3d(0, -Math.PI / 2, 0)), p.p3());
        verify(new Pose3d(0.75, 0, 1.0, new Rotation3d(0, 0, 0)), p.p4());
        verify(new Pose3d(0.75, 0, 1.0, new Rotation3d(0, 0, 0)), p.p5());
        verify(new Pose3d(0.75, 0, 1.0, new Rotation3d(0, -Math.PI / 2, 0)), p.p6());
        verify(new Pose3d(0.75, 0, 1.15, new Rotation3d(0, -Math.PI / 2, 0)), p.p7());
    }

    void verify(Twist3d expected, Twist3d actual) {
        assertEquals(expected.dx, actual.dx, 1e-3);
        assertEquals(expected.dy, actual.dy, 1e-3);
        assertEquals(expected.dz, actual.dz, 1e-3);
        assertEquals(expected.rx, actual.rx, 1e-3);
        assertEquals(expected.ry, actual.ry, 1e-3);
        assertEquals(expected.rz, actual.rz, 1e-3);
    }

    void verify(Pose3d expected, Pose3d actual) {
        assertEquals(expected.getX(), actual.getX(), 1e-3, "x");
        assertEquals(expected.getY(), actual.getY(), 1e-3, "y");
        assertEquals(expected.getZ(), actual.getZ(), 1e-3, "z");
        assertEquals(expected.getRotation().getX(), actual.getRotation().getX(), 1e-3, "rx");
        assertEquals(expected.getRotation().getY(), actual.getRotation().getY(), 1e-3, "ry");
        assertEquals(expected.getRotation().getZ(), actual.getRotation().getZ(), 1e-3, "rz");
    }

    void verify(SixDofConfig expected, SixDofConfig actual) {
        assertEquals(expected.q1(), actual.q1(), 1e-3, "q1");
        assertEquals(expected.q2(), actual.q2(), 1e-3, "q2");
        assertEquals(expected.q3(), actual.q3(), 1e-3, "q3");
        assertEquals(expected.q4(), actual.q4(), 1e-3, "q4");
        assertEquals(expected.q5(), actual.q5(), 1e-3, "q5");
        assertEquals(expected.q6(), actual.q6(), 1e-3, "q6");
    }
}
