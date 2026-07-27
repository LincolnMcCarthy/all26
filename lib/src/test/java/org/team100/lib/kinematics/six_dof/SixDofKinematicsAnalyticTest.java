package org.team100.lib.kinematics.six_dof;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.geometry.six_dof.SixDofPose;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

public class SixDofKinematicsAnalyticTest {
    @Test
    void testForward1() {
        // all zeros => arm is extended along +x
        SixDofKinematics k = new SixDofKinematicsAnalytic(0.25, 0.75, 0.75, 0.15);
        SixDofConfig q = new SixDofConfig(0, 0, 0, 0, 0, 0);
        SixDofPose p = k.forward(q);
        verify(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)), p.p1());
        verify(new Pose3d(0, 0, 0.25, new Rotation3d(0, 0, 0)), p.p2());
        verify(new Pose3d(0.75, 0, 0.25, new Rotation3d(0, 0, 0)), p.p3());
        verify(new Pose3d(1.5, 0, 0.25, new Rotation3d(0, 0, 0)), p.p4());
        verify(new Pose3d(1.5, 0, 0.25, new Rotation3d(0, 0, 0)), p.p5());
        verify(new Pose3d(1.5, 0, 0.25, new Rotation3d(0, 0, 0)), p.p6());
        verify(new Pose3d(1.65, 0, 0.25, new Rotation3d(0, 0, 0)), p.p7());
    }

    @Test
    void testForward2() {
        // point up
        SixDofKinematics k = new SixDofKinematicsAnalytic(0.25, 0.75, 0.75, 0.15);
        SixDofConfig q = new SixDofConfig(
                0, // yaw +x
                Math.PI / 2, // shoulder up
                -Math.PI / 2, // elbow out
                0, // use pitch axis for pitch
                Math.PI / 2, // pitch up
                0);
        // tool is pointing up
        SixDofPose p = k.forward(q);
        verify(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)), p.p1());
        verify(new Pose3d(0, 0, 0.25, new Rotation3d(0, -Math.PI / 2, 0)), p.p2());
        verify(new Pose3d(0, 0, 1.0, new Rotation3d(0, 0, 0)), p.p3());
        verify(new Pose3d(0.75, 0, 1.0, new Rotation3d(0, 0, 0)), p.p4());
        verify(new Pose3d(0.75, 0, 1.0, new Rotation3d(0, -Math.PI / 2, 0)), p.p5());
        verify(new Pose3d(0.75, 0, 1.0, new Rotation3d(0, -Math.PI / 2, 0)), p.p6());
        verify(new Pose3d(0.75, 0, 1.15, new Rotation3d(0, -Math.PI / 2, 0)), p.p7());
    }

    @Test
    void testInverse1() {
        // This is the wrist singularity and the elbow singularity
        SixDofKinematics k = new SixDofKinematicsAnalytic(0.25, 0.75, 0.75, 0.15);
        // tool (x) points at global +x 
        Pose3d p = new Pose3d(1.65, 0, 0.25,
                new Rotation3d(0, 0, 0));
        List<SixDofConfig> q = k.inverse(p, null, 1.0);
        assertEquals(2, q.size());
        // the q6 are different because of the default q4
        verify(new SixDofConfig(0, 0, 0, 1, 0, -1), q.get(0));
        verify(new SixDofConfig(3.141, 3.141, 0, 1, 0, 2.141), q.get(1));
    }

    @Test
    void testInverse2() {
        SixDofKinematics k = new SixDofKinematicsAnalytic(0.25, 0.75, 0.75, 0.15);
        // tool (x) points at global -z
        Pose3d p = new Pose3d(0.5, 0, 0.5,
                new Rotation3d(0, Math.PI/2, 0));
        List<SixDofConfig> q = k.inverse(p, null, null);
        assertEquals(8, q.size());
        verify(new SixDofConfig(0, 1.804, -2.259, 0, -1.116, 0), q.get(0));
        verify(new SixDofConfig(0, 1.804, -2.259, 3.141, 1.116, 3.141), q.get(1));
        verify(new SixDofConfig(0, -0.455, 2.259, 3.141, -2.908, 3.141), q.get(2));
        verify(new SixDofConfig(0, -0.455, 2.259, 0, 2.908, 0), q.get(3));

        verify(new SixDofConfig(3.141, -2.687, -2.259, 0, -2.908, 3.141), q.get(4));
        verify(new SixDofConfig(3.141, -2.687, -2.259, 3.141, 2.908, 0), q.get(5));
        verify(new SixDofConfig(3.141, 1.337, 2.259, -3.141, -1.116, 0), q.get(6));
        verify(new SixDofConfig(3.141, 1.337, 2.259, 0, 1.116, 3.141), q.get(7));

    }

    @Test
    void testInverse3() {
        SixDofKinematics k = new SixDofKinematicsAnalytic(0.25, 0.75, 0.75, 0.15);
        // tool (x) points at global +x
        // note position offset +y, wrist should be at (0.5,0.5,0.5)
        Pose3d p = new Pose3d(0.65, 0.5, 0.5,
                new Rotation3d(0, 0, 0));
        List<SixDofConfig> q = k.inverse(p, null, null);
        assertEquals(8, q.size());

        verify(new SixDofConfig(0.785, 1.387, -2.094, -2.147, -1.003, 2.451), q.get(0));
        verify(new SixDofConfig(0.785, 1.387, -2.094, 0.994, 1.003, -0.691), q.get(1));
        // elbow-down cases
        verify(new SixDofConfig(0.785, -0.707, 2.094, -0.794, -1.441, 0.131), q.get(2));
        verify(new SixDofConfig(0.785, -0.707, 2.094, 2.348, 1.441, -3.011), q.get(3));
    }

    @Test
    void testInverse4a() {
        // This is the base singularity with no default
        SixDofKinematics k = new SixDofKinematicsAnalytic(0.25, 0.75, 0.75, 0.15);
        // tool (x) points at global +x
        Pose3d p = new Pose3d(0.15, 0, 1,
                new Rotation3d(0, 0, 0));
        assertThrows(IllegalArgumentException.class, () -> k.inverse(p, null, null));
    }

    @Test
    void testInverse4b() {
        // This is the base singularity with a default, a good example of various
        // solutions using the base default.
        SixDofKinematics k = new SixDofKinematicsAnalytic(0.25, 0.75, 0.75, 0.15);
        // tool (x) points at global +x
        Pose3d p = new Pose3d(0.15, 0, 1,
                new Rotation3d(0, 0, 0));
        List<SixDofConfig> q = k.inverse(p, 1.0, null);
        assertEquals(4, q.size());

        // elbow-up
        verify(new SixDofConfig(1.0, 2.618, -2.094, -1.260, -1.084, 0.969), q.get(0));
        verify(new SixDofConfig(1.0, 2.618, -2.094, 1.881, 1.084, -2.172), q.get(1));
        // elbow-down
        verify(new SixDofConfig(1.0, 0.523, 2.094, -1.260, -2.058, -0.969), q.get(2));
        verify(new SixDofConfig(1.0, 0.523, 2.094, 1.881, 2.058, 2.172), q.get(3));

    }

    @Test
    void testInverse5() {
        // This is reaching "back" behind the base singularity.
        SixDofKinematics k = new SixDofKinematicsAnalytic(0.25, 0.75, 0.75, 0.15);
        // tool (x) points at global +x
        Pose3d p = new Pose3d(0, 0, 1,
                new Rotation3d(0, 0, 0));
        List<SixDofConfig> q = k.inverse(p, null, null);
        assertEquals(8, q.size());

        // flip, elbow up
        verify(new SixDofConfig(3.141, 2.409, -2.071, -3.141, -2.804, 0), q.get(0));
        verify(new SixDofConfig(3.141, 2.409, -2.071, 0, 2.804, 3.141), q.get(1));

        // flip, elbow down
        verify(new SixDofConfig(3.141, 0.338, 2.071, -3.141, -0.732, 0), q.get(2));
        verify(new SixDofConfig(3.141, 0.338, 2.071, 0, 0.732, 3.141), q.get(3));

        // noflip, elbow
        verify(new SixDofConfig(0, 2.804, -2.071, 0, -0.732, 0), q.get(4));
        verify(new SixDofConfig(0, 2.804, -2.071, 3.141, 0.732, 3.141), q.get(5));

        verify(new SixDofConfig(0, 0.732, 2.071, 0, -2.804, 0), q.get(6));
        verify(new SixDofConfig(0, 0.732, 2.071, 3.141, 2.804, 3.141), q.get(7));
    }

    @Test
    void testQ1() {
        List<Double> q1s = SixDofKinematicsAnalytic.getQ1(new Translation2d(1, 1), null);
        assertEquals(2, q1s.size());
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
