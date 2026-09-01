package org.team100.lib.util;

/**
 * We often need to find the derivative of some signal, e.g. given a series of
 * positional setpoints, derive the velocity for motor feedforward control.
 * 
 * The raw backwards finite difference amplifies the high-frequency noise in the
 * signal, causing lots of noise and effort in the motor.
 * 
 * This class combines a causal derivative with a low-pass filter, to reduce the
 * high-frequency noise.
 */
public class LowPassDerivative {

}
