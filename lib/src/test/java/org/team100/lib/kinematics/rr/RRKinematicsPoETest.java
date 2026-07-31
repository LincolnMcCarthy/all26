package org.team100.lib.kinematics.rr;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rr.RRPose;
import org.team100.lib.geometry.rr.RRVelocity;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.testing.TestUtil;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

/** Test cases from POE.md */
public class RRKinematicsPoETest {
    @Test
    void test0() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        RRPose p = k.forward(new RRConfig(0, 0));
        TestUtil.verify(new RRPose(
                new Pose2d(),
                new Pose2d(1, 0, Rotation2d.kZero),
                new Pose2d(2, 0, Rotation2d.kZero)), p);
    }

    @Test
    void test1() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        RRPose p = k.forward(new RRConfig(0, Math.PI / 2));
        TestUtil.verify(new RRPose(
                new Pose2d(),
                new Pose2d(1, 0, Rotation2d.kZero),
                new Pose2d(1, 1, Rotation2d.kCCW_Pi_2)), p);
    }

    @Test
    void test2() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        VelocitySE2 v = k.forward(new RRConfig(0, 0), new RRVelocity(0, 0));
        TestUtil.verify(new VelocitySE2(0, 0, 0), v);
    }

    @Test
    void test3() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        VelocitySE2 v = k.forward(new RRConfig(0, 0), new RRVelocity(1, 0));
        TestUtil.verify(new VelocitySE2(0, 2, 1), v);
    }

    @Test
    void test4() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        VelocitySE2 v = k.forward(new RRConfig(0, 0), new RRVelocity(0, 1));
        TestUtil.verify(new VelocitySE2(0, 1, 1), v);
    }

    @Test
    void test5() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        VelocitySE2 v = k.forward(new RRConfig(0, Math.PI / 2), new RRVelocity(0, 0));
        TestUtil.verify(new VelocitySE2(0, 0, 0), v);
    }

    @Test
    void test6() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        VelocitySE2 v = k.forward(new RRConfig(0, Math.PI / 2), new RRVelocity(1, 0));
        TestUtil.verify(new VelocitySE2(-1, 1, 1), v);
    }

    @Test
    void test7() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        VelocitySE2 v = k.forward(new RRConfig(0, Math.PI / 2), new RRVelocity(0, 1));
        TestUtil.verify(new VelocitySE2(-1, 0, 1), v);
    }

    @Test
    void test8() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        VelocitySE2 v = k.forward(new RRConfig(Math.PI / 2, Math.PI / 2), new RRVelocity(0, 1));
        TestUtil.verify(new VelocitySE2(0, -1, 1), v);
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
        // arm is extended, q1 is turning, q2 is not
        Matrix<N3, N2> J = k.Jdot(
                new RRConfig(0, 0),
                new RRVelocity(1, 0));
        // TODO: this seems wrong
        TestUtil.verify(MatBuilder.fill(Nat.N3(), Nat.N2(), //
                0, 0, //
                2, 1, //
                1, 1), J);
    }

    @Test
    void test12() {
        RRKinematicsPoE k = new RRKinematicsPoE(1, 1);
        // arm is bent, not moving
        Matrix<N3, N2> J = k.Jdot(
                new RRConfig(0, Math.PI / 2),
                new RRVelocity(0, 0));
        TestUtil.verify(MatBuilder.fill(Nat.N3(), Nat.N2(), //
                -1, -1, //
                1, 0, //
                1, 1), J);
    }
}
