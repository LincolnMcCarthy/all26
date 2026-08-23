package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import org.junit.jupiter.api.Test;

import frc.robot.Summarizer.Measurement;
import frc.robot.Summarizer.Summary;

public class SummarizerTest {
    @Test
    void test0() {
        Summarizer r = new Summarizer();
        r.add(new Measurement(2, 3, 4, 5));
        assertFalse(r.m_buffer.isEmpty());
        r.checkpoint();
        assertFalse(r.m_result.isEmpty());
        Summary s = r.m_result.poll();
        assertEquals(2, s.tMax(), 1e-3);
        assertEquals(3, s.iAvg(), 1e-3);
        assertEquals(4, s.vAvg(), 1e-3);
        assertEquals(5, s.pAvg(), 1e-3);
        assertEquals(0, s.qSum(), 1e-3);
    }

    @Test
    void test1() {
        Summarizer r = new Summarizer();
        r.add(new Measurement(2, 3, 4, 5));
        r.add(new Measurement(3, 4, 5, 6));
        assertFalse(r.m_buffer.isEmpty());
        r.checkpoint();
        assertFalse(r.m_result.isEmpty());
        Summary s = r.m_result.poll();
        assertEquals(3, s.tMax(), 1e-3);
        assertEquals(3.5, s.iAvg(), 1e-3);
        assertEquals(4.5, s.vAvg(), 1e-3);
        assertEquals(5.5, s.pAvg(), 1e-3);
        // trapezoid integration
        assertEquals(3.5, s.qSum(), 1e-3);
    }

}
