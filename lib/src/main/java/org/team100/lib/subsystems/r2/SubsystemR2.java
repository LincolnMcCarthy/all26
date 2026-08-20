package org.team100.lib.subsystems.r2;

import org.team100.lib.state.ModelR2;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** A planar subsystem for position only, not rotation. */
public interface SubsystemR2 extends Subsystem {
    /** State for the current Takt. */
    ModelR2 getState();

    /** Passthrough to motor stop. This is not "hold position", it is "disable". */
    void stop();
}
