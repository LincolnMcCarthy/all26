package org.team100.lib.testing;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Map;

import org.team100.lib.geometry.lynx_arm.LynxArmConfig;
import org.team100.lib.geometry.lynx_arm.LynxArmPose;
import org.team100.lib.geometry.pr.PRAcceleration;
import org.team100.lib.geometry.pr.PRVelocity;
import org.team100.lib.geometry.r2.AccelerationR2;
import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.geometry.rr.RRAcceleration;
import org.team100.lib.geometry.rr.RRPose;
import org.team100.lib.geometry.rr.RRVelocity;
import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.geometry.six_dof.SphericalWristConfig;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N6;

public class TestUtil {
    private static final boolean DEBUG = false;

    public static void verify(Map<String, Double> expected, Map<String, Double> actual) {
        assertEquals(expected.size(), actual.size());
        for (String key : expected.keySet()) {
            assertEquals(expected.get(key), actual.get(key), 1e-3, key);
        }
    }

    public static void verify(Pose3d expected, Map<String, Pose3d> poses, String name) {
        Pose3d actual = poses.get(name);
        verify(expected, actual);
    }

    public static void verify(Pose2d expected, Pose2d actual) {
        if (DEBUG)
            System.out.printf("expected %s actual %s\n", StrUtil.poseStr(expected), StrUtil.poseStr(actual));
        assertEquals(expected.getX(), actual.getX(), 1e-3, "x");
        assertEquals(expected.getY(), actual.getY(), 1e-3, "y");
        assertEquals(expected.getRotation().getRadians(), actual.getRotation().getRadians(), 1e-3, "r");
    }

    public static void verify(Pose3d expected, Pose3d actual) {
        if (DEBUG)
            System.out.printf("expected %s actual %s\n", StrUtil.poseStr(expected), StrUtil.poseStr(actual));
        assertEquals(expected.getX(), actual.getX(), 1e-3, "x");
        assertEquals(expected.getY(), actual.getY(), 1e-3, "y");
        assertEquals(expected.getZ(), actual.getZ(), 1e-3, "z");
        assertEquals(expected.getRotation().getX(), actual.getRotation().getX(), 1e-3, "rx");
        assertEquals(expected.getRotation().getY(), actual.getRotation().getY(), 1e-3, "ry");
        assertEquals(expected.getRotation().getZ(), actual.getRotation().getZ(), 1e-3, "rz");
    }

    public static void verify(Transform3d expected, Transform3d actual) {
        assertEquals(expected.getX(), actual.getX(), 1e-3, " x");
        assertEquals(expected.getY(), actual.getY(), 1e-3, " y");
        assertEquals(expected.getZ(), actual.getZ(), 1e-3, " z");
        assertEquals(expected.getRotation().getX(), actual.getRotation().getX(), 1e-3, " rot x");
        assertEquals(expected.getRotation().getY(), actual.getRotation().getY(), 1e-3, " rot y");
        assertEquals(expected.getRotation().getZ(), actual.getRotation().getZ(), 1e-3, " rot z");
    }

    public static void verify(Twist2d t, Matrix<N3, N1> v) {
        assertEquals(t.dx, v.get(0, 0), 1e-3, "x");
        assertEquals(t.dy, v.get(1, 0), 1e-3, "y");
        assertEquals(t.dtheta, v.get(2, 0), 1e-3, "r");
    }

    public static void verify(Twist3d t, Matrix<N6, N1> v) {
        assertEquals(t.dx, v.get(0, 0), 1e-3, "x");
        assertEquals(t.dy, v.get(1, 0), 1e-3, "y");
        assertEquals(t.dz, v.get(2, 0), 1e-3, "z");
        assertEquals(t.rx, v.get(3, 0), 1e-3, "rx");
        assertEquals(t.ry, v.get(4, 0), 1e-3, "ry");
        assertEquals(t.rz, v.get(5, 0), 1e-3, "rz");
    }

    public static void verify(Twist3d expected, Twist3d actual) {
        assertEquals(expected.dx, actual.dx, 1e-3);
        assertEquals(expected.dy, actual.dy, 1e-3);
        assertEquals(expected.dz, actual.dz, 1e-3);
        assertEquals(expected.rx, actual.rx, 1e-3);
        assertEquals(expected.ry, actual.ry, 1e-3);
        assertEquals(expected.rz, actual.rz, 1e-3);
    }

    public static void verify(Matrix<N6, N6> a, Matrix<N6, N6> b) {
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                assertEquals(a.get(i, j), b.get(i, j),
                        String.format("(%d, %d) a %f b %f", i, j, a.get(i, j), b.get(i, j)));
            }
        }
    }

    public static void verify(SixDofConfig expected, SixDofConfig actual) {
        assertEquals(expected.q1(), actual.q1(), 1e-3, "q1");
        assertEquals(expected.q2(), actual.q2(), 1e-3, "q2");
        assertEquals(expected.q3(), actual.q3(), 1e-3, "q3");
        assertEquals(expected.q4(), actual.q4(), 1e-3, "q4");
        assertEquals(expected.q5(), actual.q5(), 1e-3, "q5");
        assertEquals(expected.q6(), actual.q6(), 1e-3, "q6");
    }

    public static void verify(LynxArmPose expected, LynxArmPose actual) {
        assertEquals(expected.p1(), actual.p1(), "fwd p1");
        assertEquals(expected.p2(), actual.p2(), "fwd p2");
        assertEquals(expected.p3(), actual.p3(), "fwd p3");
        assertEquals(expected.p4(), actual.p4(), "fwd p4");
        assertEquals(expected.p5(), actual.p5(), "fwd p5");
        assertEquals(expected.p6(), actual.p6(), "fwd p6");
    }

    public static void verify(SphericalWristConfig expected, SphericalWristConfig actual) {
        assertEquals(expected.q4(), actual.q4(), 1e-3, "q4");
        assertEquals(expected.q5(), actual.q5(), 1e-3, "q5");
        assertEquals(expected.q6(), actual.q6(), 1e-3, "q6");
    }

    public static void verify(Rotation3d expected, Rotation3d actual) {
        double d = expected.getQuaternion().norm() * actual.getQuaternion().norm();
        double dot = expected.getQuaternion().dot(actual.getQuaternion());
        assertTrue(Math.abs(Math.abs(dot) - d) < 1e-3, StrUtil.rotStr(actual));
    }

    public static void print(Pose3d p) {
        System.out.printf("x %6.3f y %6.3f z %6.3f r %6.3f p %6.3f y %6.3f\n",
                p.getX(), p.getY(), p.getZ(),
                p.getRotation().getX(), p.getRotation().getY(), p.getRotation().getZ());
    }

    public static void verify(LynxArmConfig expected, LynxArmConfig actual) {
        assertEquals(expected.swing().getAsDouble(), actual.swing().getAsDouble(), 1e-3, "swing");
        assertEquals(expected.boom(), actual.boom(), 1e-3, "boom");
        assertEquals(expected.stick(), actual.stick(), 1e-3, "stick");
        assertEquals(expected.wrist(), actual.wrist(), 1e-3, "wrist");
        assertEquals(expected.twist().getAsDouble(), actual.twist().getAsDouble(), 1e-3, "twist");
    }

    public static void verify(LynxArmConfig q, double a, double b, double c, double d, double e) {
        assertEquals(a, q.swing().getAsDouble(), 1e-3);
        assertEquals(b, q.boom(), 1e-3);
        assertEquals(c, q.stick(), 1e-3);
        assertEquals(d, q.wrist(), 1e-3);
        assertEquals(e, q.twist().getAsDouble(), 1e-3);
    }

    public static void verify(VelocityR2 expected, VelocityR2 actual) {
        assertEquals(expected.x(), actual.x(), 1e-3);
        assertEquals(expected.y(), actual.y(), 1e-3);
    }

    public static void verify(AccelerationR2 expected, AccelerationR2 actual) {
        assertEquals(expected.x(), actual.x(), 1e-3);
        assertEquals(expected.y(), actual.y(), 1e-3);
    }

    public static void verify(RRVelocity expected, RRVelocity actual) {
        assertEquals(expected.q1dot(), actual.q1dot(), 1e-3);
        assertEquals(expected.q2dot(), actual.q2dot(), 1e03);
    }

    public static void verify(RRAcceleration expected, RRAcceleration actual) {
        assertEquals(expected.q1ddot(), actual.q1ddot(), 1e-3);
        assertEquals(expected.q2ddot(), actual.q2ddot(), 1e03);
    }

    public static void verify(PRVelocity expected, PRVelocity v) {
        assertEquals(expected.q1dot(), v.q1dot(), 1e-3);
        assertEquals(expected.q2dot(), v.q2dot(), 1e03);
    }

    public static void verify(PRAcceleration expected, PRAcceleration a) {
        assertEquals(expected.q1ddot(), a.q1ddot(), 1e-3);
        assertEquals(expected.q2ddot(), a.q2ddot(), 1e03);
    }

    public static void verify(RRPose expected, RRPose actual) {
        verify(expected.p1(), actual.p1());
    }

}
