package org.team100.lib.kinematics.rrr_se2;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.rrr.RRRAcceleration;
import org.team100.lib.geometry.rrr.RRRConfig;
import org.team100.lib.geometry.rrr.RRRVelocity;
import org.team100.lib.geometry.se2.AccelerationSE2;
import org.team100.lib.geometry.se2.VelocitySE2;
import org.team100.lib.testing.TestUtil;

import edu.wpi.first.math.geometry.Pose2d;

public class RRRKinematicsPoETest {
    @Test
    void test0() {
        RRRKinematicsPoE k = new RRRKinematicsPoE();
        RRRConfig q = new RRRConfig(0, 0, 0);
        Pose2d x = k.forward(q);
        TestUtil.verify(new Pose2d(), x);
    }

    @Test
    void test1() {
        RRRKinematicsPoE k = new RRRKinematicsPoE();
        RRRConfig q = new RRRConfig(0, 0, 0);
        RRRVelocity qdot = new RRRVelocity(0, 0, 0);
        VelocitySE2 xdot = k.forward(q, qdot);
        TestUtil.verify(new VelocitySE2(), xdot);
    }

    @Test
    void test2() {
        RRRKinematicsPoE k = new RRRKinematicsPoE();
        RRRConfig q = new RRRConfig(0, 0, 0);
        RRRVelocity qdot = new RRRVelocity(0, 0, 0);
        RRRAcceleration qddot = new RRRAcceleration(0, 0, 0);
        AccelerationSE2 xddot = k.forward(q, qdot, qddot);
        TestUtil.verify(new AccelerationSE2(), xddot);
    }

    @Test
    void test3() {
        RRRKinematicsPoE k = new RRRKinematicsPoE();
        Pose2d x = new Pose2d();
        List<RRRConfig> q = k.inverse(x);
        assertEquals(1, q.size());
        TestUtil.verify(new RRRConfig(0, 0, 0), q.get(0));
    }

    @Test
    void test4() {
        RRRKinematicsPoE k = new RRRKinematicsPoE();
        RRRConfig q = new RRRConfig(0, 0, 0);
        VelocitySE2 xdot = new VelocitySE2(0, 0, 0);
        RRRVelocity qdot = k.inverse(q, xdot);
        TestUtil.verify(new RRRVelocity(0, 0, 0), qdot);
    }

    @Test
    void test5() {
        RRRKinematicsPoE k = new RRRKinematicsPoE();
        RRRConfig q = new RRRConfig(0, 0, 0);
        VelocitySE2 xdot = new VelocitySE2(0, 0, 0);
        AccelerationSE2 xddot = new AccelerationSE2(0, 0, 0);
        RRRAcceleration qdot = k.inverse(q, xdot, xddot);
        TestUtil.verify(new RRRAcceleration(0, 0, 0), qdot);
    }
}
