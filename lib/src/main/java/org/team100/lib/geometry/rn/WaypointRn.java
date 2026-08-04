package org.team100.lib.geometry.rn;

import org.team100.lib.util.StrUtil;

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

    @Override
    public String toString() {
        return String.format("WaypointRn [%s %s]",
                StrUtil.vecStr(position), StrUtil.vecStr(direction));
    }

}
