package frc.robot;

import java.util.ArrayDeque;
import java.util.Queue;

/**
 * The test measures two quantities, current and voltage, and also computes
 * power.
 */
public class Summarizer {

    /**
     * Raw measurement at each time step.
     * 
     * @param t timestamp, seconds
     * @param i current, amperes
     * @param v voltage, volts
     * @param p power, watts
     */
    public record Measurement(double t, double i, double v, double p) {
    }

    /**
     * Summary of the buffer, written to the output.
     * 
     * @param tMax
     * @param iAvg
     * @param vAvg
     * @param pAvg
     * @param qSum
     */
    public record Summary(double tMax, double iAvg, double vAvg, double pAvg, double qSum) {
    }

    /** Requires t monotonic. :( */
    static class Op {
        private int n;
        private Double tMax;
        private double iSum;
        private double iPrev;
        private double vSum;
        private double pSum;
        private double qSum;

        public void add(Measurement m) {
            n += 1;
            iSum += m.i;
            vSum += m.v;
            pSum += m.p;
            if (tMax != null) {
                // trapezoid integration
                qSum += (iPrev + m.i) * (m.t - tMax) / 2;
            }
            iPrev = m.i;
            tMax = m.t;
        }

        public void combine(Op o) {
            n += o.n;
            iSum += o.iSum;
            vSum += o.vSum;
            pSum += o.pSum;
            qSum += o.qSum;
        }

        public Summary get() {
            return new Summary(tMax, iSum / n, vSum / n, pSum / n, qSum);
        }

    }

    // checkpoint input
    public final Queue<Measurement> m_buffer;
    // checkpoint output
    public final Queue<Summary> m_result;
    private double q;

    public Summarizer() {
        m_buffer = new ArrayDeque<>();
        m_result = new ArrayDeque<>();
        q = 0;
    }

    public void add(Measurement item) {
        m_buffer.add(item);
    }

    public void checkpoint() {
        Op op = m_buffer.stream().collect(Op::new, Op::add, Op::combine);
        m_result.add(op.get());
        m_buffer.clear();
    }

    /** Dump the contents of the buffer to stdout. */
    public void dump() {
        System.out.flush();
        m_result.stream().forEach(System.out::println);
        System.out.flush();
    }

}
