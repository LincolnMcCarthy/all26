package org.team100.lib.geometry.rn;

import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;

/**
 * @param position  vector in Rn
 * @param direction unit vector in Rn (normalized here)
 * @param scale     influence of the direction
 */
public record WaypointRn<N extends Num>(
        Vector<N> position, Vector<N> direction, double scale) {

    public WaypointRn(Vector<N> position, Vector<N> direction, double scale) {
        this.position = position;
        this.direction = direction.unit();
        this.scale = scale;
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
