package org.team100.lib.kinematics.six_dof;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.six_dof.SixDofConfig;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class SixDofKinematicsTest {
    @Test
    void testForward1() {
        // all zeros => arm is extended along +x
        SixDofKinematics k = new SixDofKinematics(0.25, 0.75, 0.75, 0.15);
        SixDofConfig q = new SixDofConfig(0, 0, 0, 0, 0, 0);
        Pose3d p = k.forward(q);
        verify(new Pose3d(1.65, 0, 0.25, new Rotation3d(Math.PI / 2, 0, Math.PI / 2)), p);
    }

    @Test
    void testForward2() {
        // make the wrist frame parallel with the world frame
        SixDofKinematics k = new SixDofKinematics(0.25, 0.75, 0.75, 0.15);
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

    @Test
    void testInverse1() {
        // This is the wrist singularity and the elbow singularity,
        // so just one solution.
        SixDofKinematics k = new SixDofKinematics(0.25, 0.75, 0.75, 0.15);
        // tool (z) points at global +x with tool x at global -y
        Pose3d p = new Pose3d(1.65, 0, 0.25,
                new Rotation3d(Math.PI / 2, 0, Math.PI / 2));
        List<SixDofConfig> q = k.inverse(p, null, 1.0);
        assertEquals(1, q.size());
        verify(new SixDofConfig(0, 0, 0, 1, 0, -1), q.get(0));
    }

    @Test
    void testInverse2() {
        SixDofKinematics k = new SixDofKinematics(0.25, 0.75, 0.75, 0.15);
        // tool (z) points at global -z, tool x at global -y
        Pose3d p = new Pose3d(0.5, 0, 0.5,
                new Rotation3d(Math.PI, 0, Math.PI / 2));
        List<SixDofConfig> q = k.inverse(p, null, null);
        assertEquals(4, q.size());

        // elbow-up cases
        verify(new SixDofConfig(0, // along +x
                1.804, // slightly back
                -2.259, // down from the elbow
                0, // zero means the "pitch" is really global pitch
                -1.116, // aim tool down
                0), // no tool rotation
                q.get(0));
        verify(new SixDofConfig(0, // along +x
                1.804, // slightly back
                -2.259, // down from the elbow
                3.141, // zero means the "pitch" is really global pitch
                1.116, // aim tool down
                3.141), // no tool rotation
                q.get(1));
        // elbow-down cases
        verify(new SixDofConfig(0, // along +x
                -0.455, // slightly down
                2.259, // up from the elbow
                -3.141, // invert the wrist
                -2.908, // aim tool down
                -3.141), // rotate tool back
                q.get(2));
        verify(new SixDofConfig(0, // along +x
                -0.455, // slightly down
                2.259, // up from the elbow
                0, // zero means the "pitch" is really global pitch
                2.908, // opposite
                0), // no tool rotation
                q.get(3));
    }

    @Test
    void testInverse3() {
        SixDofKinematics k = new SixDofKinematics(0.25, 0.75, 0.75, 0.15);
        // tool (z) points at global +x, tool x at global -y
        // note position offset +y, wrist should be at (0.5,0.5,0.5)
        Pose3d p = new Pose3d(0.65, 0.5, 0.5,
                new Rotation3d(Math.PI / 2, 0, Math.PI / 2));
        List<SixDofConfig> q = k.inverse(p, null, null);
        assertEquals(4, q.size());

        verify(new SixDofConfig(
                0.785, // diagonal, +x+y
                1.387, // slightly forward
                -2.094, // down from the elbow
                -2.147, // the q5 axis is pointing kinda -x+y+z
                -1.003, // total bend is around 60?
                2.451), // turn back so tool is correct
                q.get(0));
        verify(new SixDofConfig(
                0.785, // diagonal, +x+y
                1.387, // slightly forward
                -2.094, // down from the elbow
                0.994, // the q5 axis is pointing kinda -x+y+z
                1.003, // total bend is around 60?
                -0.691), // turn back so tool is correct
                q.get(1));
        // elbow-down cases
        verify(new SixDofConfig(
                0.785, // diagonal, +x+y
                -0.707, // slightly down
                2.094, // up from the elbow
                -0.794, //
                -1.441, //
                0.131), // turn back so tool is correct
                q.get(2));
        verify(new SixDofConfig(
                0.785, // diagonal, +x+y
                -0.707, // slightly down
                2.094, // up from the elbow
                2.348, // the q5 axis is pointing kinda -x+y+z
                1.441, // total bend is around 60?
                -3.011), // turn back so tool is correct
                q.get(3));
    }

    @Test
    void testInverse4a() {
        // This is the base singularity with no default
        SixDofKinematics k = new SixDofKinematics(0.25, 0.75, 0.75, 0.15);
        // tool (z) points at global +x with tool x at global -y
        Pose3d p = new Pose3d(0.15, 0, 1,
                new Rotation3d(Math.PI / 2, 0, Math.PI / 2));
        assertThrows(IllegalArgumentException.class, () -> k.inverse(p, null, null));
    }

    @Test
    void testInverse4b() {
        // This is the base singularity with a default, a good example of various
        // solutions using the base default.
        SixDofKinematics k = new SixDofKinematics(0.25, 0.75, 0.75, 0.15);
        // tool (z) points at global +x with tool x at global -y
        Pose3d p = new Pose3d(0.15, 0, 1,
                new Rotation3d(Math.PI / 2, 0, Math.PI / 2));
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
        // Currently the base swings all the way around, and the
        // wrist goes "backward".
        // TODO: fix this case.
        SixDofKinematics k = new SixDofKinematics(0.25, 0.75, 0.75, 0.15);
        // tool (z) points at global +x with tool x at global -y
        Pose3d p = new Pose3d(0, 0, 1,
                new Rotation3d(Math.PI / 2, 0, Math.PI / 2));
        List<SixDofConfig> q = k.inverse(p, null, null);
        assertEquals(4, q.size());

        // note q1=pi :(
        // note q5 is very sharp
        // elbow-up
        verify(new SixDofConfig(3.141, 2.804, -2.071, -3.141, -2.409, 0), q.get(0));
        verify(new SixDofConfig(3.141, 2.804, -2.071, 0, 2.409, 3.141), q.get(1));
        // elbow-down
        verify(new SixDofConfig(3.141, 0.732, 2.071, 3.141, -0.338, 0), q.get(2));
        verify(new SixDofConfig(3.141, 0.732, 2.071, 0, 0.338, 3.141), q.get(3));
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
