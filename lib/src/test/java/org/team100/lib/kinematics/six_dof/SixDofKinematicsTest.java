package org.team100.lib.kinematics.six_dof;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.six_dof.SixDofConfig;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class SixDofKinematicsTest {
    @Test
    void testForward1() {
        // all zeros => arm is extended along +x
        SixDofKinematics k = new SixDofKinematics();
        SixDofConfig q = new SixDofConfig(0, 0, 0, 0, 0, 0);
        Pose3d p = k.forward(q);
        verify(new Pose3d(1.65, 0, 0.25, new Rotation3d(Math.PI / 2, 0, Math.PI / 2)), p);
    }

    @Test
    void testForward2() {
        // make the wrist frame parallel with the world frame
        SixDofKinematics k = new SixDofKinematics();
        SixDofConfig q = new SixDofConfig(
                0, // yaw +x
                Math.PI / 2, // shoulder up
                -Math.PI / 2, // elbow out
                0, // use pitch axis for pitch
                Math.PI / 2, // pitch up so wrist axes are parallel
                -Math.PI / 2);
        Pose3d p = k.forward(q);
        // tool is pointing up
        verify(new Pose3d(0.75, 0, 1.15, new Rotation3d(0, 0, 0)), p);
    }

    void verify(Pose3d expected, Pose3d actual) {
        assertEquals(expected.getX(), actual.getX(), 1e-3, "x");
        assertEquals(expected.getY(), actual.getY(), 1e-3, "y");
        assertEquals(expected.getZ(), actual.getZ(), 1e-3, "z");
        assertEquals(expected.getRotation().getX(), actual.getRotation().getX(), 1e-3, "rx");
        assertEquals(expected.getRotation().getY(), actual.getRotation().getY(), 1e-3, "ry");
        assertEquals(expected.getRotation().getZ(), actual.getRotation().getZ(), 1e-3, "rz");
    }
}
