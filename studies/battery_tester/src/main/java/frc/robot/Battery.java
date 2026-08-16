package frc.robot;

/**
 * Simple model of a battery.
 * 
 * For now, it's just a voltage source and a resistor.
 * 
 * TODO: a more realistic model, e.g.
 * https://www.scribd.com/document/48734929/A-mathematical-model-for-lead-acid-batteries
 * https://www.mathworks.com/content/dam/mathworks/tag-team/Objects/s/40542_SAE-2007-01-0778-Battery-Modeling-Process.pdf
 * https://ut3-toulouseinp.hal.science/hal-03539078v1/document
 */
public class Battery {
    /**
     * internal resistance, ohms.
     */
    public static double R = 0.02;
    /**
     * ideal voltage source.
     * 
     * TODO: this should not be a constant, it depends on charge state.
     */
    public static double V = 12.6;

    public Battery() {

    }

    /** Voltage at which the desired current can be delivered. */
    public double VforI(double i) {
        // TODO: a more accurate battery model.
        return Math.max(0, V - i * R);
    }

    /** Voltage at which the desired power can be delivered. */
    public double VforP(double p) {
        // this iteration is really inefficient but it will work for an arbitrary
        // battery model.
        double v = V;
        for (int j = 0; j < 100; ++j) {
            double i = p / v;
            double v0 = v;
            v = VforI(i);
            if (Math.abs(v - v0) < 0.001)
                return v;
        }
        System.out.println("WARNING CONVERGENCE");
        return 0;

    }

}
