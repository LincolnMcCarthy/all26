package org.team100.lib.reference.rn;

import java.util.List;

import org.team100.lib.state.ControlR1;
import org.team100.lib.state.ModelR1;

/**
 * Provides current and next references in R^n
 * 
 * TODO: add a parameter, N.
 */
public interface ReferenceRn {
    void init();

    List<ModelR1> current();

    List<ControlR1> next();

    boolean done();
}
