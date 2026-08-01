package org.team100.lib.kinematics.rr;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.r2.AccelerationR2;
import org.team100.lib.geometry.r2.VelocityR2;
import org.team100.lib.geometry.rr.RRAcceleration;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rr.RRPosition;
import org.team100.lib.geometry.rr.RRVelocity;
import org.team100.lib.testing.TestUtil;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;

public class RRKinematicsTest {
    private static final double DELTA = 0.001;

    @Test
    void testf1() {
        // stretched along x
        RRKinematics k = new RRKinematics(1, 1);
        RRPosition p = new RRPosition(
                new Translation2d(1, 0), new Translation2d(2, 0));
        RRConfig q = new RRConfig(0, 0);

        RRPosition actual1 = k.forward(q);
        assertEquals(p.p1(), actual1.p1(), "fwd p1");
        assertEquals(p.p2(), actual1.p2(), "fwd p2");
        List<RRConfig> qq = k.inverse(p.p2());
        assertEquals(1, qq.size());
        assertEquals(0, qq.get(0).q1(), DELTA, "inv q1");
        assertEquals(0, qq.get(0).q2(), DELTA, "inv q2");
    }

    @Test
    void testf2() {
        // up and then out
        RRKinematics k = new RRKinematics(1, 1);
        RRPosition p = new RRPosition(
                new Translation2d(0, 1), new Translation2d(1, 1));
        RRConfig q = new RRConfig(Math.PI / 2, -1 * Math.PI / 2);

        RRPosition actual1 = k.forward(q);
        assertEquals(p.p1(), actual1.p1(), "fwd p1");
        assertEquals(p.p2(), actual1.p2(), "fwd p2");
        List<RRConfig> qq = k.inverse(p.p2());
        assertEquals(2, qq.size());

        assertEquals(Math.PI / 2, qq.get(0).q1(), DELTA, "inv q1");
        assertEquals(-Math.PI / 2, qq.get(0).q2(), DELTA, "inv q2");
        assertEquals(0, qq.get(1).q1(), DELTA, "inv q1");
        assertEquals(Math.PI / 2, qq.get(1).q2(), DELTA, "inv q2");

    }

    @Test
    void testf3() {
        // equilateral triangle, first link up
        RRKinematics k = new RRKinematics(1, 1);
        RRPosition p = new RRPosition(
                new Translation2d(0, 1), new Translation2d(Math.sqrt(3) / 2, 0.5));
        RRConfig q = new RRConfig(Math.PI / 2, -2 * Math.PI / 3);

        RRPosition actual1 = k.forward(q);
        assertEquals(p.p1(), actual1.p1(), "fwd p1");
        assertEquals(p.p2(), actual1.p2(), "fwd p2");
        List<RRConfig> qq = k.inverse(p.p2());
        assertEquals(2, qq.size());

        assertEquals(Math.PI / 2, qq.get(0).q1(), DELTA, "inv q1");
        assertEquals(-2.094, qq.get(0).q2(), DELTA, "inv q2");
        assertEquals(-0.523, qq.get(1).q1(), DELTA, "inv q1");
        assertEquals(2.094, qq.get(1).q2(), DELTA, "inv q2");

    }

    @Test
    void test4() {
        // vertical equilateral triangle
        RRKinematics k = new RRKinematics(1, 1);
        RRPosition p = new RRPosition(
                new Translation2d(-Math.sqrt(3) / 2, 0.5), new Translation2d(0, 1));
        RRConfig q = new RRConfig(5 * Math.PI / 6, -2 * Math.PI / 3);

        RRPosition actual1 = k.forward(q);
        assertEquals(p.p1(), actual1.p1(), "fwd p1");
        assertEquals(p.p2(), actual1.p2(), "fwd p2");
        List<RRConfig> qq = k.inverse(p.p2());
        assertEquals(2, qq.size());

        assertEquals(2.618, qq.get(0).q1(), DELTA, "inv q1");
        assertEquals(-2.094, qq.get(0).q2(), DELTA, "inv q2");
        assertEquals(0.524, qq.get(1).q1(), DELTA, "inv q1");
        assertEquals(2.094, qq.get(1).q2(), DELTA, "inv q2");
    }

    @Test
    void test5() {
        // behind
        RRKinematics k = new RRKinematics(1, 1);
        RRPosition p = new RRPosition(
                new Translation2d(-1, 0), new Translation2d(-1, 1));
        RRConfig q = new RRConfig(Math.PI, -Math.PI / 2);

        RRPosition actual1 = k.forward(q);
        assertEquals(p.p1(), actual1.p1(), "fwd p1");
        assertEquals(p.p2(), actual1.p2(), "fwd p2");
        List<RRConfig> qq = k.inverse(p.p2());
        assertEquals(2, qq.size());

        assertEquals(3.141, qq.get(0).q1(), DELTA, "inv q1");
        assertEquals(-1.571, qq.get(0).q2(), DELTA, "inv q2");
        assertEquals(1.571, qq.get(1).q1(), DELTA, "inv q1");
        assertEquals(1.571, qq.get(1).q2(), DELTA, "inv q2");

    }

    @Test
    void testForwardV() {
        RRKinematics k = new RRKinematics(1, 1);
        VelocityR2 xdot = k.forward(new RRConfig(0, 0), new RRVelocity(0, 0));
        TestUtil.verify(new VelocityR2(0, 0), xdot);
        xdot = k.forward(new RRConfig(0, 0), new RRVelocity(1, 0));
        TestUtil.verify(new VelocityR2(0, 2), xdot);
        xdot = k.forward(new RRConfig(0, 0), new RRVelocity(0, 1));
        TestUtil.verify(new VelocityR2(0, 1), xdot);
        xdot = k.forward(new RRConfig(Math.PI / 2, -Math.PI / 2), new RRVelocity(0, 0));
        TestUtil.verify(new VelocityR2(0, 0), xdot);
        xdot = k.forward(new RRConfig(Math.PI / 2, -Math.PI / 2), new RRVelocity(1, 0));
        TestUtil.verify(new VelocityR2(-1, 1), xdot);
        xdot = k.forward(new RRConfig(Math.PI / 2, -Math.PI / 2), new RRVelocity(0, 1));
        TestUtil.verify(new VelocityR2(0, 1), xdot);
    }

    @Test
    void testInverseV() {
        // TODO: add elbow-down cases.
        RRKinematics k = new RRKinematics(1, 1);
        RRVelocity qdot = k.inverse(
                new Translation2d(2, 0),
                new VelocityR2(0, 0)).get(0);
        TestUtil.verify(new RRVelocity(0, 0), qdot);
        qdot = k.inverse(
                new Translation2d(2, 0),
                new VelocityR2(1, 0)).get(0);
        TestUtil.verify(new RRVelocity(0, 0), qdot);
        qdot = k.inverse(
                new Translation2d(2, 0),
                new VelocityR2(0, 1)).get(0);
        TestUtil.verify(new RRVelocity(0, 1), qdot);
        qdot = k.inverse(
                new Translation2d(1, 1),
                new VelocityR2(0, 0)).get(0);
        TestUtil.verify(new RRVelocity(0, 0), qdot);
        qdot = k.inverse(
                new Translation2d(1, 1),
                new VelocityR2(1, 0)).get(0);
        TestUtil.verify(new RRVelocity(-1, 0), qdot);
        qdot = k.inverse(
                new Translation2d(1, 1),
                new VelocityR2(0, 1)).get(0);
        TestUtil.verify(new RRVelocity(0, 1), qdot);
        qdot = k.inverse(
                new Translation2d(1, 1),
                new VelocityR2(-1, 1)).get(0);
        TestUtil.verify(new RRVelocity(1, 0), qdot);
    }

    @Test
    void testInverseA() {
        // TODO: add elbow-down cases
        RRKinematics k = new RRKinematics(1, 1);
        RRAcceleration qddot = k.inverse(
                new Translation2d(1, 1),
                new VelocityR2(0, 0),
                new AccelerationR2(0, 0)).get(0);
        TestUtil.verify(new RRAcceleration(0, 0), qddot);
        // steady +x does not require accel.
        qddot = k.inverse(
                new Translation2d(1, 1),
                new VelocityR2(1, 0),
                new AccelerationR2(0, 0)).get(0);
        TestUtil.verify(new RRAcceleration(0, 0), qddot);
        // steady +y requires shoulder accel
        qddot = k.inverse(
                new Translation2d(1, 1),
                new VelocityR2(0, 1),
                new AccelerationR2(0, 0)).get(0);
        TestUtil.verify(new RRAcceleration(-1, 0), qddot);
    }

    @Test
    void testForwardA() {
        RRKinematics k = new RRKinematics(1, 1);
        AccelerationR2 xddot = k.forward(
                new RRConfig(0, 0),
                new RRVelocity(0, 0),
                new RRAcceleration(0, 0));
        TestUtil.verify(new AccelerationR2(0, 0), xddot);
        // move shoulder: centripetal towards shoulder
        xddot = k.forward(
                new RRConfig(Math.PI / 2, -Math.PI / 2),
                new RRVelocity(1, 0),
                new RRAcceleration(0, 0));
        TestUtil.verify(new AccelerationR2(-1, -1), xddot);
        // move elbow: centripetal towards elbow
        xddot = k.forward(
                new RRConfig(Math.PI / 2, -Math.PI / 2),
                new RRVelocity(0, 1),
                new RRAcceleration(0, 0));
        TestUtil.verify(new AccelerationR2(-1, 0), xddot);
    }

    @Test
    void testJ0() {
        RRKinematics k = new RRKinematics(1, 1);
        // extended
        RRConfig q = new RRConfig(0, 0);
        Matrix<N2, N2> J = k.J(q);
        TestUtil.verify(MatBuilder.fill(Nat.N2(), Nat.N2(), //
                0, 0, //
                2, 1), J);
    }

    @Test
    void test11() {
        RRKinematics k = new RRKinematics(1, 1);
        // arm is extended, q1 is turning, q2 is not
        // centripetal acceleration towards origin
        Matrix<N2, N2> Jdot = k.Jdot(
                new RRConfig(0, 0),
                new RRVelocity(1, 0));
        TestUtil.verify(MatBuilder.fill(Nat.N2(), Nat.N2(), //
                -2, -1, //
                0, 0), Jdot);
    }

    @Test
    void test12() {
        RRKinematics k = new RRKinematics(1, 1);
        // arm is bent, not moving: no acceleration
        Matrix<N2, N2> Jdot = k.Jdot(
                new RRConfig(0, Math.PI / 2),
                new RRVelocity(0, 0));
        TestUtil.verify(MatBuilder.fill(Nat.N2(), Nat.N2(), //
                0, 0, //
                0, 0), Jdot);
    }

    @Test
    void test13() {
        RRKinematics k = new RRKinematics(1, 1);
        // arm is bent, moving => coriolis force
        Matrix<N2, N2> Jdot = k.Jdot(
                new RRConfig(0, Math.PI / 2),
                new RRVelocity(1, 0));
        TestUtil.verify(MatBuilder.fill(Nat.N2(), Nat.N2(), //
                -1, 0, //
                -1, -1), Jdot);
    }

}