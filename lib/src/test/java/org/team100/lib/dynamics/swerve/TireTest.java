package org.team100.lib.dynamics.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class TireTest {
    /** Unsaturated. */
    @Test
    void test0() {
        Tire t = new Tire(200, 0.05);
        double a = t.angle(100);
        assertEquals(0.025, a, 0.001);
    }

    /** Saturated. */
    @Test
    void test1() {
        Tire t = new Tire(200, 0.05);
        double a = t.angle(1000);
        assertEquals(0.05, a, 0.001);
    }

}
