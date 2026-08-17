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
        // moderate load, 100A, 10.6v = 1060 W.
        Battery b = new Battery();
        double v = b.VforI(100);
        assertEquals(10.6, v, 1e-3);
    }

    @Test
    void test2() {
        // high load, 300A, 6.6v => 1980W
        Battery b = new Battery();
        double v = b.VforI(300);
        assertEquals(6.6, v, 1e-3);
    }

    @Test
    void test3() {
        // zero power
        Battery b = new Battery();
        double v = b.VforP(0);
        assertEquals(12.6, v, 1e-3);
    }

    @Test
    void test4() {
        // moderate load
        Battery b = new Battery();
        double v = b.VforP(1060);
        assertEquals(10.6, v, 1e-2);
    }

    @Test
    void test5() {
        // high load
        Battery b = new Battery();
        double v = b.VforP(1980);
        assertEquals(6.6, v, 1e-2);
    }

    @Test
    void test6() {
        // impossible load
        Battery b = new Battery();
        double v = b.VforP(2500);
        assertEquals(0, v, 1e-2);
    }
}
