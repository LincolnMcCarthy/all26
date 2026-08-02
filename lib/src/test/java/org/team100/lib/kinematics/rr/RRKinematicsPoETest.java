package org.team100.lib.kinematics.rr;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.rr.RRAcceleration;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rr.RRPose;
import org.team100.lib.geometry.rr.RRVelocity;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.testing.TestUtil;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

/** Test cases from POE.md */
public class RRKinematicsPoETest {

    @Test
    void test00() {
        TestUtil.verify(new Twist2d(0, 0, 1), RRKinematicsPoE.S(new Translation2d(0, 0)));
        TestUtil.verify(new Twist2d(0, -1, 1), RRKinematicsPoE.S(new Translation2d(1, 0)));
    }

    @Test
    void testp0() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // position
        RRPose p = k.forward(new RRConfig(0, 0));
        TestUtil.verify(new RRPose(
                new Pose2d(),
                new Pose2d(1, 0, Rotation2d.kZero),
                new Pose2d(2, 0, Rotation2d.kZero)), p);
    }

    @Test
    void testp1() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // position
        RRPose p = k.forward(new RRConfig(0, Math.PI / 2));
        TestUtil.verify(new RRPose(
                new Pose2d(),
                new Pose2d(1, 0, Rotation2d.kZero),
                new Pose2d(1, 1, Rotation2d.kCCW_Pi_2)), p);
    }

    @Test
    void testv1() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // velocity but not moving
        VelocitySE2 v = k.forward(new RRConfig(0, 0), new RRVelocity(0, 0));
        // no velocity
        TestUtil.verify(new VelocitySE2(0, 0, 0), v);
    }

    @Test
    void testv2() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // velocity: extended, rotating at shoulder
        VelocitySE2 v = k.forward(new RRConfig(0, 0), new RRVelocity(1, 0));
        // +y (more) and +theta
        TestUtil.verify(new VelocitySE2(0, 2, 1), v);
    }

    @Test
    void testv3() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // velocity: extended, rotating at elbow
        VelocitySE2 v = k.forward(new RRConfig(0, 0), new RRVelocity(0, 1));
        // +y (less) and +theta
        TestUtil.verify(new VelocitySE2(0, 1, 1), v);
    }

    @Test
    void testv4() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // velocity: bent, not moving
        VelocitySE2 v = k.forward(new RRConfig(0, Math.PI / 2), new RRVelocity(0, 0));
        // no movement
        TestUtil.verify(new VelocitySE2(0, 0, 0), v);
    }

    @Test
    void testv5() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // velocity: bent, moving at shoulder
        VelocitySE2 v = k.forward(new RRConfig(0, Math.PI / 2), new RRVelocity(1, 0));
        // diagonal motion (and +theta)
        TestUtil.verify(new VelocitySE2(-1, 1, 1), v);
    }

    @Test
    void testv6() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // velocity: bent, moving at elbow
        VelocitySE2 v = k.forward(new RRConfig(0, Math.PI / 2), new RRVelocity(0, 1));
        // motion -x and +theta
        TestUtil.verify(new VelocitySE2(-1, 0, 1), v);
    }

    @Test
    void testv7() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // velocity: both joints bent, moving at elbow
        VelocitySE2 v = k.forward(new RRConfig(Math.PI / 2, Math.PI / 2), new RRVelocity(0, 1));
        // motion -y and +theta
        TestUtil.verify(new VelocitySE2(0, -1, 1), v);
    }

    @Test
    void testForwardV() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // velocity examples from the other test
        VelocitySE2 xdot = k.forward(new RRConfig(0, 0), new RRVelocity(0, 0));
        TestUtil.verify(new VelocitySE2(0, 0, 0), xdot);
        xdot = k.forward(new RRConfig(0, 0), new RRVelocity(1, 0));
        TestUtil.verify(new VelocitySE2(0, 2, 1), xdot);
        xdot = k.forward(new RRConfig(0, 0), new RRVelocity(0, 1));
        TestUtil.verify(new VelocitySE2(0, 1, 1), xdot);
        xdot = k.forward(new RRConfig(Math.PI / 2, -Math.PI / 2), new RRVelocity(0, 0));
        TestUtil.verify(new VelocitySE2(0, 0, 0), xdot);
        xdot = k.forward(new RRConfig(Math.PI / 2, -Math.PI / 2), new RRVelocity(1, 0));
        TestUtil.verify(new VelocitySE2(-1, 1, 1), xdot);
        xdot = k.forward(new RRConfig(Math.PI / 2, -Math.PI / 2), new RRVelocity(0, 1));
        TestUtil.verify(new VelocitySE2(0, 1, 1), xdot);
    }

    @Test
    void testa0() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // bent, rotating at shoulder, no joint accel
        AccelerationSE2 xddot = k.forward(
                new RRConfig(0, Math.PI / 2),
                new RRVelocity(1, 0),
                new RRAcceleration(0, 0));
        // centripetal pulling in
        TestUtil.verify(new AccelerationSE2(-1, -1, 0), xddot);
    }

    @Test
    void testa1() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // as above, bent the other way
        AccelerationSE2 xddot = k.forward(
                new RRConfig(0, -Math.PI / 2),
                new RRVelocity(1, 0),
                new RRAcceleration(0, 0));
        // centripetal pulling in
        TestUtil.verify(new AccelerationSE2(-1, 1, 0), xddot);
    }

    @Test
    void testForwardA() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // acceleration examples from the other test
        AccelerationSE2 xddot = k.forward(
                new RRConfig(0, 0),
                new RRVelocity(0, 0),
                new RRAcceleration(0, 0));
        TestUtil.verify(new AccelerationSE2(0, 0, 0), xddot);
        // move shoulder: centripetal towards shoulder
        xddot = k.forward(
                new RRConfig(Math.PI / 2, -Math.PI / 2),
                new RRVelocity(1, 0),
                new RRAcceleration(0, 0));
        TestUtil.verify(new AccelerationSE2(-1, -1, 0), xddot);
        // move elbow: centripetal towards elbow
        xddot = k.forward(
                new RRConfig(Math.PI / 2, -Math.PI / 2),
                new RRVelocity(0, 1),
                new RRAcceleration(0, 0));
        TestUtil.verify(new AccelerationSE2(-1, 0, 0), xddot);
    }

    @Test
    void test9() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        Matrix<N3, N2> J = k.J(new RRConfig(0, 0));
        TestUtil.verify(MatBuilder.fill(Nat.N3(), Nat.N2(), //
                0, 0, //
                2, 1, //
                1, 1), J);
    }

    @Test
    void test9v() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        Matrix<N3, N2> Jv = k.Jv(new RRConfig(0, 0));
        TestUtil.verify(MatBuilder.fill(Nat.N3(), Nat.N2(), //
                0, 0, //
                0, -1, //
                1, 1), Jv);
    }

    @Test
    void test10() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        Matrix<N3, N2> J = k.J(new RRConfig(0, Math.PI / 2));
        TestUtil.verify(MatBuilder.fill(Nat.N3(), Nat.N2(), //
                -1, -1, //
                1, 0, //
                1, 1), J);
    }

    @Test
    void test11() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // arm is extended, q1 is turning, q2 is not.
        // centripetal acceleration towards origin
        Matrix<N3, N2> Jdot = k.Jdot(
                new RRConfig(0, 0),
                new RRVelocity(1, 0));
        TestUtil.verify(MatBuilder.fill(Nat.N3(), Nat.N2(), //
                -2, -1, //
                0, 0, //
                0, 0), Jdot);
    }

    @Test
    void test11v() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // arm is extended, q1 is turning, q2 is not.
        // centripetal acceleration towards origin
        Matrix<N3, N2> Jdotv = k.Jdotv(
                new RRConfig(0, 0),
                new RRVelocity(1, 0));
        TestUtil.verify(MatBuilder.fill(Nat.N3(), Nat.N2(), //
                0, 1, //
                0, 0, //
                0, 0), Jdotv);
    }

    @Test
    void test12() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // arm is bent, not moving
        Matrix<N3, N2> Jdot = k.Jdot(
                new RRConfig(0, Math.PI / 2),
                new RRVelocity(0, 0));
        TestUtil.verify(MatBuilder.fill(Nat.N3(), Nat.N2(), //
                0, 0, //
                0, 0, //
                0, 0), Jdot);
    }

    @Test
    void test13() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // arm is bent, moving
        Matrix<N3, N2> Jdot = k.Jdot(
                new RRConfig(0, Math.PI / 2),
                new RRVelocity(1, 0));
        TestUtil.verify(MatBuilder.fill(Nat.N3(), Nat.N2(), //
                -1, 0, //
                -1, -1, //
                0, 0), Jdot);
    }
}
