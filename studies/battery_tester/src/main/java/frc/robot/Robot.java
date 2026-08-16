
package frc.robot;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.util.Banner;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private final DriverXboxControl m_controller;
    private final CurrentSource m_subsystem;

    public Robot() {
        Banner.printBanner();
        Experiments.instance.show();
        Logging log = Logging.instance();
        LoggerFactory robotLog = log.rootLogger;
        m_controller = new DriverXboxControl(0);
        m_subsystem = new CurrentSource(robotLog);
        PWMSim s = new PWMSim(0);
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

}
