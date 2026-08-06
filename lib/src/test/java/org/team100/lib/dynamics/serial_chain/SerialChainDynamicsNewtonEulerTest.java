package org.team100.lib.dynamics.serial_chain;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public class SerialChainDynamicsNewtonEulerTest {
        /**
     * See modern_robotics.core.py
     * 
     * Example Input (3 Link Robot)
     */
    @Test
    void testForwardDynamics() {
        var thetalist = VecBuilder.fill(0.1, 0.1, 0.1);
        var dthetalist = VecBuilder.fill(0.1, 0.2, 0.3);
        var taulist = VecBuilder.fill(0.5, 0.6, 0.7);
        var g = VecBuilder.fill(0, 0, -9.8);

        /**


        g = np.array([0, 0, -9.8])
        Ftip = np.array([1, 1, 1, 1, 1, 1])
        M01 = np.array([[1, 0, 0,        0],
                        [0, 1, 0,        0],
                        [0, 0, 1, 0.089159],
                        [0, 0, 0,        1]])
        M12 = np.array([[ 0, 0, 1,    0.28],
                        [ 0, 1, 0, 0.13585],
                        [-1, 0, 0,       0],
                        [ 0, 0, 0,       1]])
        M23 = np.array([[1, 0, 0,       0],
                        [0, 1, 0, -0.1197],
                        [0, 0, 1,   0.395],
                        [0, 0, 0,       1]])
        M34 = np.array([[1, 0, 0,       0],
                        [0, 1, 0,       0],
                        [0, 0, 1, 0.14225],
                        [0, 0, 0,       1]])
        G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
        G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
        G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
        Glist = np.array([G1, G2, G3])
        Mlist = np.array([M01, M12, M23, M34])
        Slist = np.array([[1, 0, 1,      0, 1,     0],
                          [0, 1, 0, -0.089, 0,     0],
                          [0, 1, 0, -0.089, 0, 0.425]]).T
    Output:
        np.array([-0.97392907, 25.58466784, -32.91499212])

         */
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

        /**
        Ftip = np.array([1, 1, 1, 1, 1, 1])
        M01 = np.array([[1, 0, 0,        0],
                        [0, 1, 0,        0],
                        [0, 0, 1, 0.089159],
                        [0, 0, 0,        1]])
        M12 = np.array([[ 0, 0, 1,    0.28],
                        [ 0, 1, 0, 0.13585],
                        [-1, 0, 0,       0],
                        [ 0, 0, 0,       1]])
        M23 = np.array([[1, 0, 0,       0],
                        [0, 1, 0, -0.1197],
                        [0, 0, 1,   0.395],
                        [0, 0, 0,       1]])
        M34 = np.array([[1, 0, 0,       0],
                        [0, 1, 0,       0],
                        [0, 0, 1, 0.14225],
                        [0, 0, 0,       1]])
        G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
        G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
        G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
        Glist = np.array([G1, G2, G3])
        Mlist = np.array([M01, M12, M23, M34])
        Slist = np.array([[1, 0, 1,      0, 1,     0],
                          [0, 1, 0, -0.089, 0,     0],
                          [0, 1, 0, -0.089, 0, 0.425]]).T
         */

        Vector<N3> output = VecBuilder.fill(74.69616155, -33.06766016, -3.23057314);

    }
    
        /**
     * See modern_robotics.core.py
     * 
     * Example Input (3 Link Robot)
     */
    @Test
    void testMassMatrix() {
        /*

        thetalist = np.array([0.1, 0.1, 0.1])
        M01 = np.array([[1, 0, 0,        0],
                        [0, 1, 0,        0],
                        [0, 0, 1, 0.089159],
                        [0, 0, 0,        1]])
        M12 = np.array([[ 0, 0, 1,    0.28],
                        [ 0, 1, 0, 0.13585],
                        [-1, 0, 0,       0],
                        [ 0, 0, 0,       1]])
        M23 = np.array([[1, 0, 0,       0],
                        [0, 1, 0, -0.1197],
                        [0, 0, 1,   0.395],
                        [0, 0, 0,       1]])
        M34 = np.array([[1, 0, 0,       0],
                        [0, 1, 0,       0],
                        [0, 0, 1, 0.14225],
                        [0, 0, 0,       1]])
        G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
        G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
        G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
        Glist = np.array([G1, G2, G3])
        Mlist = np.array([M01, M12, M23, M34])
        Slist = np.array([[1, 0, 1,      0, 1,     0],
                          [0, 1, 0, -0.089, 0,     0],
                          [0, 1, 0, -0.089, 0, 0.425]]).T
    Output:
        np.array([[ 2.25433380e+01, -3.07146754e-01, -7.18426391e-03]
                  [-3.07146754e-01,  1.96850717e+00,  4.32157368e-01]
                  [-7.18426391e-03,  4.32157368e-01,  1.91630858e-01]])
        */
        
    }


}
