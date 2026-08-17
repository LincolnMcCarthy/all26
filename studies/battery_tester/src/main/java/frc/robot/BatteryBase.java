package frc.robot;

/** Utilities for batteries. */
public abstract class BatteryBase {

    abstract double V();

    abstract double R();

    /** Voltage at which the desired current can be delivered. */
    public double VforI(double i) {
        return Math.max(0, V() - i * R());
    }

    /** Voltage at which the desired power can be delivered. */
    public double VforP(double p) {
        // this iteration is really inefficient but it will work for an arbitrary
        // battery model.
        double v = V();
        for (int j = 0; j < 1000; ++j) {
            double i = p / v;
            double v0 = v;
            v = VforI(i);
            if (Math.abs(v - v0) < 0.001)
                return v;
        }
        System.out.printf(
                "BatteryBase: voltage failed to converge for power %f\n", p);
        return v;
    }
}
