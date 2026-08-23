package org.team100.battery_tester;

import org.team100.battery_tester.BatteryTester.Op;
import org.team100.lib.coherence.Takt;

import edu.wpi.first.wpilibj2.command.Command;

/** Run a full test and print the results. */
public class TestProtocol extends Command {

    private final BatteryTester m_subsystem;
    private final Summarizer m_summarizer;

    public TestProtocol(BatteryTester subsystem) {
        m_subsystem = subsystem;
        // summarize every 100 points, i.e. at 2 Hz.
        m_summarizer = new Summarizer(100);
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        // nothing to initialize.
    }

    @Override
    public void execute() {
        m_subsystem.setPower(1500);
        double t = Takt.get();
        Op op = m_subsystem.operatingPoint();
        m_summarizer.add(t, op.i(), op.v(), op.p());
    }

    @Override
    public boolean isFinished() {
        Op op = m_subsystem.operatingPoint();
        return op.v() < 9;
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.off();
        m_summarizer.header();
        m_summarizer.dump();
    }

}
