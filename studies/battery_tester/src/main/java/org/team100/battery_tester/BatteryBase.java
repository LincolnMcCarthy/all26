package org.team100.battery_tester;

/** Battery base class. */
public abstract class BatteryBase {
    private static final boolean DEBUG = false;

    /** Open-circuit voltage, a positive number */
    abstract double V0();

    /** Resistance, ohms. */
    abstract double R();

    /** State of charge, [0, 1] */
    abstract double SOC();

    /**
     * Computes "sag", the voltage at which the desired current can be delivered.
     * 
     * Applies Ohm's and Kirchoff's laws:
     * 
     * V(I) = V0 - I*R
     * 
     * Never returns a negative number; might return zero.
     */
    public double V(double i) {
        if (i < 0)
            throw new IllegalArgumentException();
        double v0 = V0();
        double r = R();
        double v = Math.max(0, v0 - i * r);
        if (DEBUG)
            System.out.printf("BatteryBase: VforI success v0 %f r %f v %f i %f soc %f\n",
                    v0, r, v, i, SOC());
        return v;
    }
}
