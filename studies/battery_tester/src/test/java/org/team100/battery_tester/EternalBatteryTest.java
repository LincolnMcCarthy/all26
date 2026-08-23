package org.team100.battery_tester;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class EternalBatteryTest {
    @Test
    void test0() {
        // voltage at rest
        EternalBattery b = new EternalBattery();
        double v = b.V(0);
        assertEquals(12.6, v, 1e-3);
    }

    @Test
    void test1() {
        // moderate load, 100A, 10.6v = 1060 W.
        EternalBattery b = new EternalBattery();
        double v = b.V(100);
        assertEquals(10.6, v, 1e-3);
    }

    @Test
    void test2() {
        // high load, 300A, 6.6v => 1980W
        EternalBattery b = new EternalBattery();
        double v = b.V(300);
        assertEquals(6.6, v, 1e-3);
    }
}
