package frc.robot;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.TotalCurrentLog;
import org.team100.lib.subsystems.rrr.RRRArm;
import org.team100.lib.subsystems.rrr.RRRVisualizer;

public class Machinery {
    public final RRRArm m_arm;
    public final RRRVisualizer m_viz;

    public Machinery() {
        LoggerFactory logger = Logging.instance().rootLogger;
        TotalCurrentLog currentLog = new TotalCurrentLog(logger);
        m_arm = new RRRArm(logger, currentLog);
        m_viz = new RRRVisualizer(m_arm);
    }

    public void close() {
    }

    public void periodic() {
        m_viz.periodic();
    }
}
