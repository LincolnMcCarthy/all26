package org.team100.lib.kinematics.rr;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rr.RRPose;
import org.team100.lib.testing.TestUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
}
