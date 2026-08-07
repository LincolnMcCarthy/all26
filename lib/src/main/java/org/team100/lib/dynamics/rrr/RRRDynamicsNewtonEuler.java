package org.team100.lib.dynamics.rrr;

import org.team100.lib.dynamics.serial_chain.SerialChainDynamicsNewtonEuler;
import org.team100.lib.util.FixedList;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N6;

public class RRRDynamicsNewtonEuler {

    private final SerialChainDynamicsNewtonEuler<N3, N4> m_dyn;

    public RRRDynamicsNewtonEuler() {
        m_dyn = new SerialChainDynamicsNewtonEuler<>(Nat.N3(), Nat.N4());
    }

    public Vector<N3> InverseDynamics(
            Vector<N3> thetalist,
            Vector<N3> dthetalist,
            Vector<N3> ddthetalist,
            Vector<N3> g,
            Vector<N6> Ftip,
            FixedList<N4, Matrix<N4, N4>> Mlist,
            FixedList<N3, Matrix<N6, N6>> Glist,
            FixedList<N3, Vector<N6>> Slist) {
        return m_dyn.InverseDynamics(
                thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist);
    }

    public Vector<N3> ForwardDynamics(
            Vector<N3> thetalist,
            Vector<N3> dthetalist,
            Vector<N3> taulist,
            Vector<N3> g,
            Vector<N6> Ftip,
            FixedList<N4, Matrix<N4, N4>> Mlist,
            FixedList<N3, Matrix<N6, N6>> Glist,
            FixedList<N3, Vector<N6>> Slist) {
        return m_dyn.ForwardDynamics(
                thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist);
    }

}
