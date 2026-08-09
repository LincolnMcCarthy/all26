package org.team100.lib.dynamics.six_dof;

import java.util.List;

import org.team100.lib.geometry.six_dof.SixDofAcceleration;
import org.team100.lib.geometry.six_dof.SixDofConfig;
import org.team100.lib.geometry.six_dof.SixDofVelocity;
import org.team100.lib.util.ModernRobotics;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N6;

public class SixDofDynamicsNewtonEuler {
    // TODO: make g variable, to account for drive base acceleration.
    Vector<N3> g;
    List<Matrix<N4, N4>> Mlist;
    List<Matrix<N6, N6>> Glist;
    List<Vector<N6>> Slist;

    public SixDofDynamicsNewtonEuler() {
    }

    public SixDofEffort effort(
            SixDofConfig q,
            SixDofVelocity qdot,
            SixDofAcceleration qddot) {
        Vector<N6> thetalist = q.toVector();
        Vector<N6> dthetalist = qdot.toVector();
        Vector<N6> ddthetalist = qddot.toVector();
        Vector<N6> Ftip = new Vector<>(Nat.N6());
        return SixDofEffort.fromVector(ModernRobotics.InverseDynamics(
                Nat.N6(), thetalist, dthetalist, ddthetalist, g, Ftip,
                Mlist, Glist, Slist));
    }

    public SixDofAcceleration qddot(
            SixDofConfig q,
            SixDofVelocity qdot,
            SixDofEffort effort) {
        Vector<N6> thetalist = q.toVector();
        Vector<N6> dthetalist = qdot.toVector();
        Vector<N6> taulist = effort.toVector();
        Vector<N6> Ftip = new Vector<>(Nat.N6());
        return SixDofAcceleration.fromVector(ModernRobotics.ForwardDynamics(
                Nat.N6(), thetalist, dthetalist, taulist, g, Ftip,
                Mlist, Glist, Slist));
    }
}
