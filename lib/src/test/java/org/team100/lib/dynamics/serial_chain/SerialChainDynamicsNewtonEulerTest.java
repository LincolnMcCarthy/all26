package org.team100.lib.dynamics.serial_chain;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.FixedList;
import org.team100.lib.util.MatUtil;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N6;

public class SerialChainDynamicsNewtonEulerTest {
    /**
     * See modern_robotics.core.py
     * 
     * Example Input (3 Link Robot)
     */
    @Test
    void testForwardDynamics() {
        Vector<N3> thetalist = VecBuilder.fill(0.1, 0.1, 0.1);
        Vector<N3> dthetalist = VecBuilder.fill(0.1, 0.2, 0.3);
        Vector<N3> taulist = VecBuilder.fill(0.5, 0.6, 0.7);
        Vector<N3> g = VecBuilder.fill(0, 0, -9.8);
        Vector<N6> Ftip = VecBuilder.fill(1, 1, 1, 1, 1, 1);
        Matrix<N4, N4> M01 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, 0, //
                0, 1, 0, 0, //
                0, 0, 1, 0.089159, //
                0, 0, 0, 1);
        Matrix<N4, N4> M12 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                0, 0, 1, 0.28, //
                0, 1, 0, 0.13585, //
                -1, 0, 0, 0, //
                0, 0, 0, 1);
        Matrix<N4, N4> M23 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, 0, //
                0, 1, 0, -0.1197, //
                0, 0, 1, 0.395, //
                0, 0, 0, 1);
        Matrix<N4, N4> M34 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, 0, //
                0, 1, 0, 0, //
                0, 0, 1, 0.14225, //
                0, 0, 0, 1);

        Matrix<N6, N6> G1 = MatUtil.diag(Nat.N6(), VecBuilder.fill(
                0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7));
        Matrix<N6, N6> G2 = MatUtil.diag(Nat.N6(), VecBuilder.fill(
                0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393));
        Matrix<N6, N6> G3 = MatUtil.diag(Nat.N6(), VecBuilder.fill(
                0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275));
        FixedList<N3, Matrix<N6, N6>> Glist = new FixedList<>(Nat.N3(),
                List.of(G1, G2, G3));
        FixedList<N4, Matrix<N4, N4>> Mlist = new FixedList<>(Nat.N4(),
                List.of(M01, M12, M23, M34));
        FixedList<N3, Vector<N6>> Slist = new FixedList<>(Nat.N3(),
                List.of(VecBuilder.fill(1, 0, 1, 0, 1, 0),
                        VecBuilder.fill(0, 1, 0, -0.089, 0, 0),
                        VecBuilder.fill(0, 1, 0, -0.089, 0, 0.425)));

        Vector<N3> output = VecBuilder.fill(-0.97392907, 25.58466784, -32.91499212);
    }

    /**
     * See modern_robotics.core.py
     * 
     * Example Input (3 Link Robot)
     */
    @Test
    void testInverseDynamics() {
        Vector<N3> thetalist = VecBuilder.fill(0.1, 0.1, 0.1);
        Vector<N3> dthetalist = VecBuilder.fill(0.1, 0.2, 0.3);
        Vector<N3> ddthetalist = VecBuilder.fill(2, 1.5, 1);
        Vector<N3> g = VecBuilder.fill(0, 0, -9.8);
        Vector<N6> Ftip = VecBuilder.fill(1, 1, 1, 1, 1, 1);
        Matrix<N4, N4> M01 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, 0, //
                0, 1, 0, 0, //
                0, 0, 1, 0.089159, //
                0, 0, 0, 1);
        Matrix<N4, N4> M12 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                0, 0, 1, 0.28, //
                0, 1, 0, 0.13585, //
                -1, 0, 0, 0, //
                0, 0, 0, 1);
        Matrix<N4, N4> M23 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, 0, //
                0, 1, 0, -0.1197, //
                0, 0, 1, 0.395, //
                0, 0, 0, 1);
        Matrix<N4, N4> M34 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, 0, //
                0, 1, 0, 0, //
                0, 0, 1, 0.14225, //
                0, 0, 0, 1);

        Matrix<N6, N6> G1 = MatUtil.diag(Nat.N6(), VecBuilder.fill(
                0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7));
        Matrix<N6, N6> G2 = MatUtil.diag(Nat.N6(), VecBuilder.fill(
                0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393));
        Matrix<N6, N6> G3 = MatUtil.diag(Nat.N6(), VecBuilder.fill(
                0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275));
        FixedList<N3, Matrix<N6, N6>> Glist = new FixedList<>(Nat.N3(),
                List.of(G1, G2, G3));
        FixedList<N4, Matrix<N4, N4>> Mlist = new FixedList<>(Nat.N4(),
                List.of(M01, M12, M23, M34));
        FixedList<N3, Vector<N6>> Slist = new FixedList<>(Nat.N3(),
                List.of(VecBuilder.fill(1, 0, 1, 0, 1, 0),
                        VecBuilder.fill(0, 1, 0, -0.089, 0, 0),
                        VecBuilder.fill(0, 1, 0, -0.089, 0, 0.425)));

        Vector<N3> output = VecBuilder.fill(74.69616155, -33.06766016, -3.23057314);
    }

    /**
     * See modern_robotics.core.py
     * 
     * Example Input (3 Link Robot)
     */
    @Test
    void testMassMatrix() {
        Vector<N3> thetalist = VecBuilder.fill(0.1, 0.1, 0.1);

        Matrix<N4, N4> M01 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, 0, //
                0, 1, 0, 0, //
                0, 0, 1, 0.089159, //
                0, 0, 0, 1);
        Matrix<N4, N4> M12 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                0, 0, 1, 0.28, //
                0, 1, 0, 0.13585, //
                -1, 0, 0, 0, //
                0, 0, 0, 1);
        Matrix<N4, N4> M23 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, 0, //
                0, 1, 0, -0.1197, //
                0, 0, 1, 0.395, //
                0, 0, 0, 1);
        Matrix<N4, N4> M34 = MatBuilder.fill(Nat.N4(), Nat.N4(), //
                1, 0, 0, 0, //
                0, 1, 0, 0, //
                0, 0, 1, 0.14225, //
                0, 0, 0, 1);
        Matrix<N6, N6> G1 = MatUtil.diag(Nat.N6(), VecBuilder.fill(
                0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7));
        Matrix<N6, N6> G2 = MatUtil.diag(Nat.N6(), VecBuilder.fill(
                0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393));
        Matrix<N6, N6> G3 = MatUtil.diag(Nat.N6(), VecBuilder.fill(
                0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275));
        FixedList<N3, Matrix<N6, N6>> Glist = new FixedList<>(Nat.N3(),
                List.of(G1, G2, G3));
        FixedList<N4, Matrix<N4, N4>> Mlist = new FixedList<>(Nat.N4(),
                List.of(M01, M12, M23, M34));
        FixedList<N3, Vector<N6>> Slist = new FixedList<>(Nat.N3(),
                List.of(VecBuilder.fill(1, 0, 1, 0, 1, 0),
                        VecBuilder.fill(0, 1, 0, -0.089, 0, 0),
                        VecBuilder.fill(0, 1, 0, -0.089, 0, 0.425)));

        Matrix<N3, N3> output = MatBuilder.fill(Nat.N3(), Nat.N3(), //
                2.25433380e+01, -3.07146754e-01, -7.18426391e-03, //
                -3.07146754e-01, 1.96850717e+00, 4.32157368e-01, //
                -7.18426391e-03, 4.32157368e-01, 1.91630858e-01);
    }

}
