package frc.robot;

import static org.team100.lib.util.TriggerUtil.whileTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.XboxController;

public class Binder {

    private final Machinery m_machinery;

    public Binder(Machinery machinery) {
        m_machinery = machinery;
        XboxController m_controller = new XboxController(0);
        // button 1, "z" in sim
        whileTrue(m_controller::getAButton, m_machinery.m_arm.warp0());
        // button 2, "x" in sim
        whileTrue(m_controller::getBButton, m_machinery.m_arm.warp1());
        // button 3, "c" in sim
        // tool pointing down
        whileTrue(m_controller::getXButton,
                m_machinery.m_arm.move(
                        new Pose3d(0.5, 0.25, 0.1, new Rotation3d(Math.PI, 0, Math.PI / 2))));
        // button 4, "v" in sim
        whileTrue(m_controller::getYButton, m_machinery.m_arm.move(
                new Pose3d(0.2, -0.2, 0.6, new Rotation3d(Math.PI/2, 0, Math.PI/2))));
    }

    public void close() {
    }
}
