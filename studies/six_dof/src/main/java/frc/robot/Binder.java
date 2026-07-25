package frc.robot;

import static org.team100.lib.util.TriggerUtil.whileTrue;

import edu.wpi.first.wpilibj.XboxController;

public class Binder {

    private final Machinery m_machinery;

    public Binder(Machinery machinery) {
        m_machinery = machinery;
        XboxController m_controller = new XboxController(0);
        whileTrue(m_controller::getAButton, m_machinery.m_arm.warp0());
        whileTrue(m_controller::getBButton, m_machinery.m_arm.warp1());
    }

    public void close() {
    }
}
