package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.Summarizer.Measurement;
import frc.robot.Summarizer.Summary;

public class SummarizerTest {
    @Test
    void test0() {
        // verify one item is summarized correctly
        Summarizer r = new Summarizer(1);
        // contains just the initial value
        assertEquals(1, r.m_buffer.size());
        // checkpoints too
        r.add(new Measurement(2, 3, 4, 5));
        // contains just the above item
        assertEquals(1, r.m_buffer.size());
        assertEquals(1, r.m_result.size());
        Summary s = r.m_result.poll();
        assertEquals(0, s.tMax(), 1e-3);
        assertEquals(3, s.iAvg(), 1e-3);
        assertEquals(4, s.vAvg(), 1e-3);
        assertEquals(5, s.pAvg(), 1e-3);
        assertEquals(0, s.qSum(), 1e-3);
    }

    @Test
    void test1() {
        // verify two items are summarized correctly
        Summarizer r = new Summarizer(2);
        // contains just the initial value
        assertEquals(1, r.m_buffer.size());
        r.add(new Measurement(2, 3, 4, 5));
        // initial and the item above
        assertEquals(2, r.m_buffer.size());
        r.add(new Measurement(3, 4, 5, 6));
        // the two items above
        assertEquals(1, r.m_buffer.size());
        assertEquals(1, r.m_result.size());
        Summary s = r.m_result.poll();
        assertEquals(1, s.tMax(), 1e-3);
        assertEquals(3.5, s.iAvg(), 1e-3);
        assertEquals(4.5, s.vAvg(), 1e-3);
        assertEquals(5.5, s.pAvg(), 1e-3);
        // trapezoid integration
        assertEquals(3.5, s.qSum(), 1e-3);
    }

    @Test
    void test2() {
        // two items summarized more often
        Summarizer r = new Summarizer(1);
        // contains just the initial value
        assertEquals(1, r.m_buffer.size());
        r.add(new Measurement(2, 3, 4, 5));
        // just the item above
        assertEquals(1, r.m_buffer.size());
        r.add(new Measurement(3, 4, 5, 6));
        // just the item above
        assertEquals(1, r.m_buffer.size());
        assertEquals(2, r.m_result.size());
        // these are all just the initial values
        Summary s = r.m_result.poll();
        assertEquals(0, s.tMax(), 1e-3);
        assertEquals(3, s.iAvg(), 1e-3);
        assertEquals(4, s.vAvg(), 1e-3);
        assertEquals(5, s.pAvg(), 1e-3);
        // no dt, so no q
        assertEquals(0, s.qSum(), 1e-3);
        s = r.m_result.poll();
        assertEquals(1, s.tMax(), 1e-3);
        assertEquals(3.5, s.iAvg(), 1e-3);
        assertEquals(4.5, s.vAvg(), 1e-3);
        assertEquals(5.5, s.pAvg(), 1e-3);
        // trapezoid integration
        assertEquals(3.5, s.qSum(), 1e-3);
    }

    @Test
    void test3() {
        // verify q is the cumulative sum
        Summarizer r = new Summarizer(1);
        r.add(new Measurement(1, 1, 12, 12));
        assertEquals(1, r.m_result.size());
        Summary s = r.m_result.peekLast();
        assertEquals(0, s.tMax(), 1e-3);
        assertEquals(1, s.iAvg(), 1e-3);
        assertEquals(12, s.vAvg(), 1e-3);
        assertEquals(12, s.pAvg(), 1e-3);
        assertEquals(0, s.qSum(), 1e-3);
        r.add(new Measurement(2, 1, 12, 12));
        assertEquals(2, r.m_result.size());
        s = r.m_result.peekLast();
        assertEquals(1, s.tMax(), 1e-3);
        assertEquals(1, s.iAvg(), 1e-3);
        assertEquals(12, s.vAvg(), 1e-3);
        assertEquals(12, s.pAvg(), 1e-3);
        assertEquals(1, s.qSum(), 1e-3);
        r.add(new Measurement(3, 1, 12, 12));
        assertEquals(3, r.m_result.size());
        s = r.m_result.peekLast();
        assertEquals(2, s.tMax(), 1e-3);
        assertEquals(1, s.iAvg(), 1e-3);
        assertEquals(12, s.vAvg(), 1e-3);
        assertEquals(12, s.pAvg(), 1e-3);
        assertEquals(2, s.qSum(), 1e-3);
    }

    @Test
    void test4() {
        // summarize every 50 points
        Summarizer r = new Summarizer(50);
        double t0 = 123.456;
        // 300 sec
        for (double t = 0; t < 300; t += 0.02) {
            r.add(new Measurement(t0 + t, 100, 12, 1200));
        }
        // should summarize every 1 sec
        assertEquals(300, r.m_result.size());
        r.header();
        r.dump();
    }

    @Test
    void test5() {
        // summarize every single point
        Summarizer r = new Summarizer(1);
        double t0 = 123.456;
        for (double t = 0; t < 1; t += 0.02) {
            r.add(new Measurement(t0 + t, 100, 12, 1200));
        }
        assertEquals(50, r.m_result.size());
        r.header();
        r.dump();
    }

}
