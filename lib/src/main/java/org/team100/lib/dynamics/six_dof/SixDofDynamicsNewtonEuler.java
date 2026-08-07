package org.team100.lib.dynamics.six_dof;

import org.team100.lib.dynamics.serial_chain.SerialChainDynamicsNewtonEuler;
import org.team100.lib.util.FixedList;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.numbers.N7;

public class SixDofDynamicsNewtonEuler {
    private final SerialChainDynamicsNewtonEuler<N6, N7> m_dyn;

    public SixDofDynamicsNewtonEuler() {
        m_dyn = new SerialChainDynamicsNewtonEuler<>(Nat.N6(), Nat.N7());
    }

    public Vector<N6> InverseDynamics(
            Vector<N6> thetalist,
            Vector<N6> dthetalist,
            Vector<N6> ddthetalist,
            Vector<N3> g,
            Vector<N6> Ftip,
            FixedList<N7, Matrix<N4, N4>> Mlist,
            FixedList<N6, Matrix<N6, N6>> Glist,
            FixedList<N6, Vector<N6>> Slist) {
        return m_dyn.InverseDynamics(
                thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist);
    }

    public Vector<N6> ForwardDynamics(
            Vector<N6> thetalist,
            Vector<N6> dthetalist,
            Vector<N6> taulist,
            Vector<N3> g,
            Vector<N6> Ftip,
            FixedList<N7, Matrix<N4, N4>> Mlist,
            FixedList<N6, Matrix<N6, N6>> Glist,
            FixedList<N6, Vector<N6>> Slist) {
        return m_dyn.ForwardDynamics(
                thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist);
    }
}
