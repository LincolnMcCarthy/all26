package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class CircuitUtilTest {
    @Test
    void test0() {
        // off
        EternalBattery b = new EternalBattery();
        LightBulb l = new LightBulb();
        CircuitUtil u = new CircuitUtil(l, b);
        assertEquals(0, u.operatingPoint(0).inputI(), 1e-3);
        assertEquals(12.6, u.operatingPoint(0).inputV(), 1e-3);
    }

    @Test
    void test1() {
        EternalBattery b = new EternalBattery();
        LightBulb l = new LightBulb();
        CircuitUtil u = new CircuitUtil(l, b);
        assertEquals(60.699, u.operatingPoint(0.5).inputI(), 1e-3);
        assertEquals(11.394, u.operatingPoint(0.5).inputV(), 1e-3);
    }

    @Test
    void test2() {
        EternalBattery b = new EternalBattery();
        LightBulb l = new LightBulb();
        CircuitUtil u = new CircuitUtil(l, b);
        assertEquals(109.516, u.operatingPoint(0.75).inputI(), 1e-3);
        assertEquals(10.418, u.operatingPoint(0.75).inputV(), 1e-3);
    }

    @Test
    void test3() {
        // Maximum power possible
        EternalBattery b = new EternalBattery();
        LightBulb l = new LightBulb();
        CircuitUtil u = new CircuitUtil(l, b);
        assertEquals(162.342, u.operatingPoint(1.0).inputI(), 1e-3);
        assertEquals(9.362, u.operatingPoint(1.0).inputV(), 1e-3);
    }

    @Test
    void test4() {
        // Low SOC, to exercise the difficult part.
        StatefulBattery b = new StatefulBattery();
        LightBulb l = new LightBulb();
        CircuitUtil u = new CircuitUtil(l, b);
        b.setC(1000);
        // Very low state of charge
        assertEquals(0.015, b.SOC(), 1e-3);
        // Very low open circuit voltage
        assertEquals(8.256, b.V0(), 1e-3);
        // Very high internal resistance
        assertEquals(2, b.R(), 1);
        // Approximately zero volts under load.
        assertEquals(3.521, u.operatingPoint(1.0).inputI(), 1e-3);
        assertEquals(0.02, u.operatingPoint(1.0).inputV(), 1e-3);
    }

}
