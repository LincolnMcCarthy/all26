package org.team100.lib.geometry.rn;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;

/**
 * A waypoint is a position, represented as a vector,
 * and a direction, represented as a unit vector.
 */
public record WaypointRn<N extends Num>(Vector<N> position, Vector<N> direction) {

    public WaypointRn(Vector<N> position, Vector<N> direction) {
        this.position = position;
        this.direction = direction.unit();
    }

    public int dim() {
        return position.getNumRows();
    }
}
