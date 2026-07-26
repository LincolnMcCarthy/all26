package org.team100.lib.kinematics.six_dof;

import java.util.List;

import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.geometry.six_dof.SixDofPose;

import edu.wpi.first.math.geometry.Pose3d;

public interface SixDofKinematics {

    /**
     * Forward position kinematics: cartesian joint poses from joint configurations.
     * 
     * Note this method uses origin frames of each joint, with the z axis rotating.
     */
    SixDofPose forward(SixDofConfig q);

    /**
     * Inverse position kinematics: joint configs from cartesian pose.
     *
     * Zero, one, two, four, or eight solutions.
     * 
     * For defaults, use the previous value, or null if you have no idea (and in
     * that case, catch the exception that may occur).
     * 
     * @param p         Tool point pose.
     * @param q1Default In case of base singularity.
     * @param q4Default In case of wrst singularity.
     */
    List<SixDofConfig> inverse(Pose3d p, Double q1Default, Double q4Default);

}