package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class StatefulBatteryTest {
    @Test
    void test0() {
        StatefulBattery b = new StatefulBattery();
        assertEquals(12, b.ocv.get(0.5), 1e-3);
    }

    @Test
    void test1() {
        StatefulBattery b = new StatefulBattery();
        assertEquals(0.021, b.r.get(0.5), 1e-3);
    }

    @Test
    void test2() {
        StatefulBattery b = new StatefulBattery();
        assertEquals(18.0, b.peukert(0) * b.c0 / 3600, 0.001);
        // 20h rating
        assertEquals(18.0, b.peukert(0.9) * b.c0 / 3600, 0.001);
        // 10h rating, note this doesn't match very well
        assertEquals(17.0, b.peukert(1.7) * b.c0 / 3600, 0.79);
        // 5h rating, note this doesn't match very well at all.
        assertEquals(15.7, b.peukert(3.14) * b.c0 / 3600, 1.05);
        // 1h rating, I weighed this point the highest.
        assertEquals(11.8, b.peukert(11.8) * b.c0 / 3600, 0.001);
    }
}
