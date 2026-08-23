package frc.robot;

import java.util.ArrayDeque;
import java.util.Deque;

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
        /** So that the checkpoint period is correct for the first item. */
        static final Measurement INITIAL = new Measurement(0, 0, 0, 0);
    }

    static class Summary {
        /** First timestamp received. */
        private Double t0;
        /** Number of points in the batch */
        private int n;
        /** Time since t0 */
        private double tMax;
        private double iSum;
        private double vSum;
        private double pSum;
        private double qSum;

        /** For trapezoid integration. */
        private double iPrev;

        public Summary(Summary prev) {
            if (prev != null) {
                t0 = prev.t0;
                qSum = prev.qSum();
                tMax = prev.tMax();
            }
        }

        /** Requires t to be monotonic. */
        public void add(Measurement m) {
            if (m == Measurement.INITIAL) {
                // The initial item exists only to make the counter happy.
                return;
            }
            n += 1;
            iSum += m.i;
            vSum += m.v;
            pSum += m.p;
            if (t0 == null) {
                // First item
                t0 = m.t;
                tMax = 0.0;
            }

            double dq = getDq(m);
            qSum += dq;
        }

        /** Time since previous measurement. */
        private double getDt(Measurement m) {
            // time since the start of the run
            double t = m.t - t0;
            // time since the previous measurement
            double dt = t - tMax;
            tMax = t;
            return dt;
        }

        /** Charge since previous measurement, trapezoid integration */
        private double getDq(Measurement m) {
            double iAvg = (iPrev + m.i) / 2;
            double dt = getDt(m);
            double dq = iAvg * dt;
            iPrev = m.i;
            return dq;
        }

        public String toString() {
            return String.format("%10.3f, %10.3f, %10.3f, %10.3f, %10.3f",
                    tMax(), iAvg(), vAvg(), pAvg(), qSum());
        }

        double tMax() {
            return tMax;
        }

        double iAvg() {
            return iSum / n;
        }

        double vAvg() {
            return vSum / n;
        }

        double pAvg() {
            return pSum / n;
        }

        double qSum() {
            return qSum;
        }

    }

    /** Summarize every n */
    private final int m_n;
    /** Checkpoint input */
    public final Deque<Measurement> m_buffer;
    /** Checkpoint output */
    public final Deque<Summary> m_result;

    /** @param n summarize every n items */
    public Summarizer(int n) {
        m_n = n;
        m_buffer = new ArrayDeque<>();
        m_result = new ArrayDeque<>();
        m_buffer.add(Measurement.INITIAL);
    }

    public void add(Measurement item) {
        m_buffer.add(item);
        if (m_buffer.size() > m_n) {
            checkpoint();
        }
    }

    private void checkpoint() {
        addResult();
        clearBuffer();
    }

    /** Summarize the buffer. */
    private void addResult() {
        Summary summary = new Summary(m_result.peekLast());
        m_buffer.stream().forEach(summary::add);
        m_result.add(summary);
    }

    /** Clear all but the last item. */
    private void clearBuffer() {
        Measurement last = m_buffer.peekLast();
        m_buffer.clear();
        m_buffer.add(last);
    }

    /** Print the output table header. */
    public void header() {
        System.out.flush();
        System.out.println("t, i, v, p, q");
        System.out.flush();
    }

    /** Dump the contents of the buffer to stdout as a CSV. */
    public void dump() {
        System.out.flush();
        m_result.stream().forEach(System.out::println);
        System.out.flush();
    }

}
