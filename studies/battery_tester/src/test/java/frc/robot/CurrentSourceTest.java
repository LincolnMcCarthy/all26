package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class CurrentSourceTest {
    @Test
    void test0() {
        // off
        Battery b = new Battery();
        LightBulb l = new LightBulb();
        assertEquals(12.6, CurrentSource.batteryVoltageForDutycycle(l, b, 0), 1e-3);
        assertEquals(0, CurrentSource.powerForDutyCycle(l, b, 0), 1e-3);
    }

    @Test
    void test1() {
        Battery b = new Battery();
        LightBulb l = new LightBulb();
        assertEquals(11.386, CurrentSource.batteryVoltageForDutycycle(l, b, 0.5), 1e-3);
        assertEquals(691, CurrentSource.powerForDutyCycle(l, b, 0.5), 1);
    }

    @Test
    void test2() {
        Battery b = new Battery();
        LightBulb l = new LightBulb();
        assertEquals(10.410, CurrentSource.batteryVoltageForDutycycle(l, b, 0.75), 1e-3);
        assertEquals(1140, CurrentSource.powerForDutyCycle(l, b, 0.75), 1);
    }

    @Test
    void test3() {
        // maximum power possible
        Battery b = new Battery();
        LightBulb l = new LightBulb();
        assertEquals(9.355, CurrentSource.batteryVoltageForDutycycle(l, b, 1.0), 1e-3);
        assertEquals(1518, CurrentSource.powerForDutyCycle(l, b, 1.0), 1);
    }

}
