package org.team100.lib.kinematics.canfield;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.canfield.CanfieldConfig;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class CanfieldKinematicsTest {
    @Test
    void testForwardUp() {
        CanfieldKinematics k = new CanfieldKinematics(0.5, 1);
        // all same, point up
        CanfieldConfig q = new CanfieldConfig(2, 2, 2);
        Pose3d p = k.forward(q);
        verify(new Pose3d(0, 0, 1.818, new Rotation3d()), p);
    }

    @Test
    void testForwardX1() {
        CanfieldKinematics k = new CanfieldKinematics(0.5, 1);
        // x low, others high -> point at +x
        CanfieldConfig q = new CanfieldConfig(2.1, 1.9, 1.9);
        Pose3d p = k.forward(q);
        verify(new Pose3d(0.107, 0, 1.837, new Rotation3d(0, 0.117, 0)), p);
    }

    @Test
    void testForwardX2() {
        CanfieldKinematics k = new CanfieldKinematics(0.5, 1);
        // x low, others high -> point at +x
        CanfieldConfig q = new CanfieldConfig(3, 1.6, 1.6);
        Pose3d p = k.forward(q);
        verify(new Pose3d(0.687, 0, 1.404, new Rotation3d(0, 0.910, 0)), p);
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
