package frc.robot;

import static org.team100.lib.util.TriggerUtil.whileTrue;

import org.team100.lib.commands.MoveAndHold;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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
                        new Pose3d(0.5, 0.25, 0.1, new Rotation3d(0, Math.PI / 2, 0))));
        // button 4, "v" in sim
        // Make a sequence of moves with various positions and orientations.
        MoveAndHold move1 = m_machinery.m_arm.move(
                new Pose3d(0.5, 0.25, 0.1, new Rotation3d(0, 0, 0)));
        MoveAndHold move2 = m_machinery.m_arm.move(
                new Pose3d(0.25, 0.25, 0.1, new Rotation3d(0, Math.PI / 2, 0)));
        MoveAndHold move3 = m_machinery.m_arm.move(
                new Pose3d(0.25, -0.25, 0.1, new Rotation3d(0, 0, 0)));
        MoveAndHold move4 = m_machinery.m_arm.move(
                new Pose3d(0.5, -0.25, 0.1, new Rotation3d(0, Math.PI / 2, 0)));
        whileTrue(m_controller::getYButton,
                move1.until(move1::isDone)
                        .andThen(move2.until(move2::isDone))
                        .andThen(move3.until(move3::isDone))
                        .andThen(move4.until(move4::isDone)));
        // circle a target
        MoveAndHold s1 = m_machinery.m_arm.move(
                new Pose3d(0.4, 0, 0.3, new Rotation3d(0, 0, Math.PI / 4)));
        MoveAndHold s2 = m_machinery.m_arm.move(
                new Pose3d(0.4, 0, 0.3, new Rotation3d(0, Math.PI / 4, 0)));
        MoveAndHold s3 = m_machinery.m_arm.move(
                new Pose3d(0.4, 0, 0.3, new Rotation3d(0, 0, -Math.PI / 4)));
        MoveAndHold s4 = m_machinery.m_arm.move(
                new Pose3d(0.4, 0, 0.3, new Rotation3d(0, -Math.PI / 4, 0)));

        whileTrue(m_controller::getAButton,
                s1.until(s1::isDone)
                        .andThen(s2.until(s2::isDone))
                        .andThen(s3.until(s3::isDone))
                        .andThen(s4.until(s4::isDone)));
    }

    public void close() {
    }
}
