package org.team100.lib.dynamics.rr;

import org.team100.lib.dynamics.serial_chain.SerialChainDynamicsNewtonEuler;
import org.team100.lib.util.FixedList;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N6;

public class RRDynamicsNewtonEuler {

    private final SerialChainDynamicsNewtonEuler<N2, N3> m_dyn;

    public RRDynamicsNewtonEuler() {
        m_dyn = new SerialChainDynamicsNewtonEuler<>(Nat.N2(), Nat.N3());
    }

    public Vector<N2> InverseDynamics(
            Vector<N2> thetalist,
            Vector<N2> dthetalist,
            Vector<N2> ddthetalist,
            Vector<N3> g,
            Vector<N6> Ftip,
            FixedList<N3, Matrix<N4, N4>> Mlist,
            FixedList<N2, Matrix<N6, N6>> Glist,
            FixedList<N2, Vector<N6>> Slist) {
        return m_dyn.InverseDynamics(
                thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist);
    }

    public Vector<N2> ForwardDynamics(
            Vector<N2> thetalist,
            Vector<N2> dthetalist,
            Vector<N2> taulist,
            Vector<N3> g,
            Vector<N6> Ftip,
            FixedList<N3, Matrix<N4, N4>> Mlist,
            FixedList<N2, Matrix<N6, N6>> Glist,
            FixedList<N2, Vector<N6>> Slist) {
                return m_dyn.ForwardDynamics(
                    thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist);
    }

}
