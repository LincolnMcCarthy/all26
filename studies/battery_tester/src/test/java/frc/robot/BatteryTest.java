package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class BatteryTest {
    @Test
    void test0() {
        // voltage at rest
        Battery b = new Battery();
        double v = b.VforI(0);
        assertEquals(12.6, v, 1e-3);
    }

    @Test
    void test1() {
        // moderate load
        Battery b = new Battery();
        double v = b.VforI(100);
        assertEquals(10.6, v, 1e-3);
    }

    @Test
    void test2() {
        // high load
        Battery b = new Battery();
        double v = b.VforI(300);
        assertEquals(6.6, v, 1e-3);
    }

}
