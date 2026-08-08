package org.team100.lib.dynamics.six_dof;

import java.util.List;

import org.team100.lib.util.ModernRobotics;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N6;

public class SixDofDynamicsNewtonEuler {

    public SixDofDynamicsNewtonEuler() {
    }

    public Vector<N6> InverseDynamics(
            Vector<N6> thetalist,
            Vector<N6> dthetalist,
            Vector<N6> ddthetalist,
            Vector<N3> g,
            Vector<N6> Ftip,
            List<Matrix<N4, N4>> Mlist,
            List<Matrix<N6, N6>> Glist,
            List<Vector<N6>> Slist) {
        return ModernRobotics.InverseDynamics(
                Nat.N6(), thetalist, dthetalist, ddthetalist, g, Ftip, 
                Mlist, Glist, Slist);
    }

    public Vector<N6> ForwardDynamics(
            Vector<N6> thetalist,
            Vector<N6> dthetalist,
            Vector<N6> taulist,
            Vector<N3> g,
            Vector<N6> Ftip,
            List<Matrix<N4, N4>> Mlist,
            List<Matrix<N6, N6>> Glist,
            List<Vector<N6>> Slist) {
        return ModernRobotics.ForwardDynamics(
                Nat.N6(), thetalist, dthetalist, taulist, g, Ftip,
                Mlist, Glist, Slist);
    }
}
