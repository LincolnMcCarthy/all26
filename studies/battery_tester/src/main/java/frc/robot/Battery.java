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


}
