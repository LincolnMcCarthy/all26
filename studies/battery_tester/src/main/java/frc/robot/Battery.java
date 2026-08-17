package frc.robot;

/**
 * Simple model of a battery, a fixed voltage source and a resistor.
 * 
 * This model is intended for feedforward; it doesn't need to be correct.
 */
public class Battery extends BatteryBase {
    /** Internal resistance, ohms. */
    public static double R = 0.02;
    /** Open-circuit voltage. */
    public static double V = 12.6;

    @Override
    double V() {
        return V;
    }

    @Override
    double R() {
        return R;
    }

}
