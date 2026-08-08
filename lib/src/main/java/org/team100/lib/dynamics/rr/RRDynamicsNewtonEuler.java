package org.team100.lib.dynamics.rr;

import java.util.List;

import org.team100.lib.util.ModernRobotics;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N6;

public class RRDynamicsNewtonEuler {


    public RRDynamicsNewtonEuler() {
    }

    public Vector<N2> InverseDynamics(
            Vector<N2> thetalist,
            Vector<N2> dthetalist,
            Vector<N2> ddthetalist,
            Vector<N3> g,
            Vector<N6> Ftip,
            List<Matrix<N4, N4>> Mlist,
            List<Matrix<N6, N6>> Glist,
            List<Vector<N6>> Slist) {
        return ModernRobotics.InverseDynamics(
                Nat.N2(), thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist);
    }

    public Vector<N2> ForwardDynamics(
            Vector<N2> thetalist,
            Vector<N2> dthetalist,
            Vector<N2> taulist,
            Vector<N3> g,
            Vector<N6> Ftip,
            List<Matrix<N4, N4>> Mlist,
            List<Matrix<N6, N6>> Glist,
            List<Vector<N6>> Slist) {
        return ModernRobotics.ForwardDynamics(
                Nat.N2(), thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist);
    }

}
