package org.team100.battery_tester;

import org.team100.battery_tester.BatteryTester.Op;
import org.team100.lib.coherence.Takt;

import edu.wpi.first.wpilibj2.command.Command;

/** Run a full test and print the results. */
public class ConstantCurrentProtocol extends Command {

    /** summarize every 100 points, i.e. at 2 Hz. */
    private static final int DECIMATION = 100;
    private final BatteryTester m_subsystem;
    private final double m_i;
    private final double m_v;
    private final Summarizer m_summarizer;

    /**
     * @param subsystem tester
     * @param i         target current, amperes
     * @param v         cutoff voltage, volts
     */
    public ConstantCurrentProtocol(
            BatteryTester subsystem, double i, double v) {
        m_subsystem = subsystem;
        m_i = i;
        m_v = v;
        m_summarizer = new Summarizer(DECIMATION);
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        // nothing to initialize.
    }

    @Override
    public void execute() {
        m_subsystem.setCurrent(m_i);
        double t = Takt.get();
        Op op = m_subsystem.operatingPoint();
        m_summarizer.add(t, op.inputI(), op.inputV(), op.p());
    }

    @Override
    public boolean isFinished() {
        Op op = m_subsystem.operatingPoint();
        return op.inputV() < m_v;
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.off();
        summary();
        m_summarizer.dump();
    }

    private void summary() {
        System.out.println("*********************************");
        System.out.println("TEST COMPLETE");
        System.out.printf("TARGET CURRENT (AMPS)  %10.3f\n", m_i);
        System.out.printf("CUTOFF VOLTAGE (VOLTS) %10.3f\n", m_v);
        m_summarizer.summary();
        System.out.println("*********************************");
    }

}
