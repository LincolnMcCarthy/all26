package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Battery model including charge state.
 * 
 * This model is intended for simulation, with the following refinements:
 * 
 * * the voltage source depends on charge state.
 * 
 * We use the Powersonic PS-12180
 * (https://www.power-sonic.com/product/ps-12180/), which
 * is rated at 18Ah for 20h discharge rate. (This is mandated by rule,
 * "20-hour discharge rate: minimum 17Ah, maximum 18.2Ah")
 * 
 * https://www.scribd.com/document/48734929/A-mathematical-model-for-lead-acid-batteries
 * https://www.mathworks.com/content/dam/mathworks/tag-team/Objects/s/40542_SAE-2007-01-0778-Battery-Modeling-Process.pdf
 * https://ut3-toulouseinp.hal.science/hal-03539078v1/document
 * https://www.athensjournals.gr/technology/2016-3-3-4-Dost.pdf
 * https://en.wikipedia.org/wiki/Peukert's_law
 * https://www.power-sonic.com/product/ps-12180/
 */
public class StatefulBattery extends BatteryBase {
    /** Open Circuit Voltage as a function of State of Charge. */
    final InterpolatingDoubleTreeMap ocv;
    /** Internal resistance as a function of State of Charge. */
    final InterpolatingDoubleTreeMap r;
    /** Rated capacity at 20h discharge rate, coulombs. */
    final double c0;
    /** 20h discharge rate in amps. */
    final double i0;
    /** Peukert's constant. */
    final double k;
    /** State of charge, [0,1]. */
    double soc;

    // TODO: implement discharging.

    public StatefulBattery() {
        ocv = makeOCV();
        r = makeR();
        c0 = 18 * 3600;
        // 18 Ah / 20 h = 0.9 A.
        i0 = 18.0 / 20;
        // Adjusted to fit the rated discharge rates.
        // The rated rates don't fit Peukert's law very well, and there are only three
        // points to fit.
        // I just used the highest rate since it's closest to the (very high) current we
        // actually use.
        // see
        // https://docs.google.com/spreadsheets/d/1gB8hojtICp1v2dbFhKkuQ7yY3v1sJFDWeo-w5ABYFcA/edit?gid=862420482#gid=862420482
        k = 1.1641;
        // starting state
        soc = 1.0;
    }

    @Override
    double V() {
        return ocv.get(soc);
    }

    @Override
    double R() {
        return r.get(soc);
    }

    /**
     * Open-circuit voltage as a function of charge state.
     * 
     * This comes from eyeballing
     * https://www.athensjournals.gr/technology/2016-3-3-4-Dost.pdf
     * transcribed here:
     * https://docs.google.com/spreadsheets/d/1D61uZiuB7fR6dDHDfRE-6lHsw0oD9cSROUWzGtkaWhY/edit?gid=850752207#gid=850752207
     */
    private static InterpolatingDoubleTreeMap makeOCV() {
        InterpolatingDoubleTreeMap ocv = new InterpolatingDoubleTreeMap();
        ocv.put(0.00, 10.70);
        ocv.put(0.05, 11.05);
        ocv.put(0.10, 11.25);
        ocv.put(0.20, 11.49);
        ocv.put(0.30, 11.67);
        ocv.put(0.40, 11.84);
        ocv.put(0.50, 12.00);
        ocv.put(0.60, 12.16);
        ocv.put(0.70, 12.32);
        ocv.put(0.80, 12.48);
        ocv.put(0.90, 12.64);
        ocv.put(1.00, 12.8);
        return ocv;
    }

    /**
     * Resistance as a function of charge state.
     * 
     * I used figures like these:
     * https://www.researchgate.net/figure/nternal-resistance-versus-SOC_fig2_312558387
     * https://www.biologic.net/documents/eis-high-frequencies-internal-resistance-battery-application-note-62/
     * transcribed here
     * https://docs.google.com/spreadsheets/d/1D61uZiuB7fR6dDHDfRE-6lHsw0oD9cSROUWzGtkaWhY/edit?gid=659397064#gid=659397064
     * I scaled the result to the label resistance of 16 mohm.
     */
    private static InterpolatingDoubleTreeMap makeR() {
        InterpolatingDoubleTreeMap r = new InterpolatingDoubleTreeMap();
        r.put(0.0, 0.07);
        r.put(0.05, 0.0532);
        r.put(0.1, 0.042);
        r.put(0.2, 0.0308);
        r.put(0.3, 0.0252);
        r.put(0.4, 0.0224);
        r.put(0.5, 0.021);
        r.put(0.6, 0.0196);
        r.put(0.7, 0.0182);
        r.put(0.8, 0.0175);
        r.put(0.9, 0.0168);
        r.put(1.0, 0.0168);
        return r;
    }

    /**
     * Applies Peukert's law, returns derating for the given discharge current.
     * 
     * The 20h capacity implies a 0.9A discharge rate.
     * 
     * https://en.wikipedia.org/wiki/Peukert's_law
     */
    double peukert(double i) {
        if (i < 0.1)
            return 1.0;
        return Math.pow(i0 / i, k - 1);
    }
}
