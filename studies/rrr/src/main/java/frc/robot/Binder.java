package frc.robot;

import static org.team100.lib.util.TriggerUtil.whileTrue;

import org.team100.lib.commands.MoveAndHold;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;

public class Binder {

    private final Machinery m_machinery;

    public Binder(Machinery machinery) {
        m_machinery = machinery;
        XboxController m_controller = new XboxController(0);
        // button 1, "z" in sim
        // whileTrue(m_controller::getAButton, m_machinery.m_arm.warp0());
        // button 2, "x" in sim
        whileTrue(m_controller::getBButton, m_machinery.m_arm.warp1());
        // button 3, "c" in sim
        // tool (x) pointing down
        whileTrue(m_controller::getXButton,
                m_machinery.m_arm.move(
                        new Pose2d(0.5, 0.25, new Rotation2d(0))));
        // button 4, "v" in sim
        // Make a sequence of moves with various positions and orientations.
        MoveAndHold move1 = m_machinery.m_arm.move(
                new Pose2d(0.5, 0.25, new Rotation2d(0)));
        MoveAndHold move2 = m_machinery.m_arm.move(
                new Pose2d(0.6, 0.25, new Rotation2d(0)));
        MoveAndHold move3 = m_machinery.m_arm.move(
                new Pose2d(0.6, -0.25, new Rotation2d(0)));
        MoveAndHold move4 = m_machinery.m_arm.move(
                new Pose2d(0.5, -0.25, new Rotation2d(0)));
        whileTrue(m_controller::getYButton,
                move1.until(move1::isDone)
                        .andThen(move2.until(move2::isDone))
                        .andThen(move3.until(move3::isDone))
                        .andThen(move4.until(move4::isDone)));
        // circle a target
        MoveAndHold s1 = m_machinery.m_arm.move(
                new Pose2d(0.4, 0, new Rotation2d(0)));
        MoveAndHold s2 = m_machinery.m_arm.move(
                new Pose2d(0.4, 0, new Rotation2d(Math.PI / 4)));
        MoveAndHold s3 = m_machinery.m_arm.move(
                new Pose2d(0.4, 0, new Rotation2d(Math.PI / 2)));
        MoveAndHold s4 = m_machinery.m_arm.move(
                new Pose2d(0.4, 0, new Rotation2d(3 * Math.PI / 4)));
        MoveAndHold s5 = m_machinery.m_arm.move(
                new Pose2d(0.4, 0, new Rotation2d(Math.PI)));
        MoveAndHold s6 = m_machinery.m_arm.move(
                new Pose2d(0.4, 0, new Rotation2d(-3 * Math.PI / 4)));
        MoveAndHold s7 = m_machinery.m_arm.move(
                new Pose2d(0.4, 0, new Rotation2d(-Math.PI / 2)));
        MoveAndHold s8 = m_machinery.m_arm.move(
                new Pose2d(0.4, 0, new Rotation2d(-Math.PI / 4)));
        whileTrue(m_controller::getAButton,
                s1.until(s1::isDone)
                        .andThen(s2.until(s2::isDone))
                        .andThen(s3.until(s3::isDone))
                        .andThen(s4.until(s4::isDone))
                        .andThen(s5.until(s5::isDone))
                        .andThen(s6.until(s6::isDone))
                        .andThen(s7.until(s7::isDone))
                        .andThen(s8.until(s8::isDone)));
    }

    public void close() {
    }
}
