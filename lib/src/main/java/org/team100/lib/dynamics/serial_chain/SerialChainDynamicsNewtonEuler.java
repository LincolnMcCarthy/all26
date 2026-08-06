package org.team100.lib.dynamics.serial_chain;

import org.team100.lib.util.FixedList;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N6;

/**
 * Direct port of Modern Robotics dynamics python code.
 * 
 * https://github.com/NxRLab/ModernRobotics/blob/master/packages/Python/modern_robotics/core.py
 */
public class SerialChainDynamicsNewtonEuler<N extends Num, M extends Num> {
    private final Nat<N> num;
    private final Nat<M> numPlusOne;

    public SerialChainDynamicsNewtonEuler(Nat<N> num, Nat<M> numPlusOne) {
        this.num = num;
        this.numPlusOne = numPlusOne;
        if (num.getNum() + 1 != numPlusOne.getNum())
            throw new IllegalArgumentException();
    }

    /**
     * Computes forward dynamics in the space frame for an open chain robot.
     * 
     * This function computes ddthetalist by solving:
     * Mlist(thetalist) * ddthetalist = taulist - c(thetalist,dthetalist) -
     * g(thetalist) - Jtr(thetalist) * Ftip
     * 
     * 
     * @param thetalist:  A list of joint variables
     * @param dthetalist: A list of joint rates
     * @param taulist:    An n-vector of joint forces/torques
     * @param g:          Gravity vector g
     * @param Ftip:       Spatial force applied by the end-effector expressed in
     *                    frame {n+1}
     * @param Mlist:      List of link frames i relative to i-1 at the home position
     * @param Glist:      Spatial inertia matrices Gi of the links
     * @param Slist:      Screw axes Si of the joints in a space frame, in the
     *                    format of a matrix with axes as the columns
     * @return: The resulting joint accelerations
     */
    Vector<N> ForwardDynamics(
            Vector<N> thetalist,
            Vector<N> dthetalist,
            Vector<N> taulist,
            Vector<N3> g,
            Vector<N6> Ftip,
            FixedList<M, Matrix<N4, N4>> Mlist,
            FixedList<N, Matrix<N6, N6>> Glist,
            int Slist) {
        /*
         * return np.dot(np.linalg.inv(MassMatrix(thetalist, Mlist, Glist, \
         * Slist)), \
         * np.array(taulist) \
         * - VelQuadraticForces(thetalist, dthetalist, Mlist, \
         * Glist, Slist) \
         * - GravityForces(thetalist, g, Mlist, Glist, Slist) \
         * - EndEffectorForces(thetalist, Ftip, Mlist, Glist, \
         * Slist))
         */

        return null;
    }

    /**
     * Computes the mass matrix of an open chain robot based on the given
     * configuration
     * 
     * This function calls InverseDynamics n times, each time passing a
     * ddthetalist vector with a single element equal to one and all other
     * inputs set to zero.
     * 
     * Each call of InverseDynamics generates a single column, and these columns
     * are assembled to create the inertia matrix.
     * 
     * @param thetalist A list of joint variables
     * @param Mlist     List of link frames i relative to i-1 at the home position
     * @param Glist     Spatial inertia matrices Gi of the links
     * @param Slist     Screw axes Si of the joints in a space frame, in the format
     *                  of a matrix with axes as the columns
     * @return The numerical inertia matrix M(thetalist) of an n-joint serial
     *         chain at the given configuration thetalist
     */
    Matrix<N, N> MassMatrix(
            Vector<N> thetalist,
            FixedList<M, Matrix<N4, N4>> Mlist,
            FixedList<N, Matrix<N6, N6>> Glist,
            int Slist) {
        int n = num.getNum();
        Matrix<N, N> M = new Matrix<>(num, num);
        for (int i = 0; i < n; ++i) {
            Vector<N> ddthetalist = new Vector<>(num);
            ddthetalist.set(i, 0, 1);
            M.setColumn(i, InverseDynamics(
                    thetalist,
                    new Vector<>(num),
                    ddthetalist,
                    VecBuilder.fill(0, 0, 0),
                    new Vector<>(Nat.N6()),
                    Mlist,
                    Glist,
                    Slist));
        }
        return M;

    }

    /**
     * Computes inverse dynamics in the space frame for an open chain robot.
     * 
     * This function uses forward-backward Newton-Euler iterations to solve the
     * equation:
     * 
     * taulist = Mlist(thetalist)ddthetalist + c(thetalist,dthetalist) +
     * g(thetalist) + Jtr(thetalist)Ftip
     * 
     * @param thetalist   n-vector of joint variables
     * @param dthetalist  n-vector of joint rates
     * @param ddthetalist n-vector of joint accelerations
     * @param g           Gravity vector g
     * @param Ftip        Spatial force applied by the end-effector expressed in
     *                    frame {n+1}
     * @param Mlist       List of link frames {i} relative to {i-1} at the home
     *                    position
     * @param Glist       Spatial inertia matrices Gi of the links
     * @param Slist       Screw axes Si of the joints in a space frame, in the
     *                    format of a matrix with axes as the columns
     * @return The n-vector of required joint forces/torques
     */
    Vector<N> InverseDynamics(
            Vector<N> thetalist,
            Vector<N> dthetalist,
            Vector<N> ddthetalist,
            Vector<N3> g,
            Vector<N6> Ftip,
            FixedList<M, Matrix<N4, N4>> Mlist,
            FixedList<N, Matrix<N6, N6>> Glist,
            int Slist) {
        int n = num.getNum();
        Matrix<N4, N4> Mi = Matrix.eye(Nat.N4());
        Matrix<N6, N> Ai = new Matrix<>(Nat.N6(), num);
        Matrix<N6, M> AdTi = new Matrix<>(Nat.N6(), numPlusOne);
        Matrix<N6, M> Vi = new Matrix<>(Nat.N6(), numPlusOne);
        Matrix<N6, M> Vdi = new Matrix<>(Nat.N6(), numPlusOne);

        /*
         * Vdi[:, 0] = np.r_[[0, 0, 0], -np.array(g)]
         * AdTi[n] = Adjoint(TransInv(Mlist[n]))
         * Fi = np.array(Ftip).copy()
         */

        Vector<N> taulist = new Vector<>(num);
        for (int i = 0; i < n; ++i) {
            /*
             * Mi = np.dot(Mi,Mlist[i])
             * Ai[:, i] = np.dot(Adjoint(TransInv(Mi)), np.array(Slist)[:, i])
             * AdTi[i] = Adjoint(np.dot(MatrixExp6(VecTose3(Ai[:, i] * \
             * -thetalist[i])), \
             * TransInv(Mlist[i])))
             * Vi[:, i + 1] = np.dot(AdTi[i], Vi[:,i]) + Ai[:, i] * dthetalist[i]
             * Vdi[:, i + 1] = np.dot(AdTi[i], Vdi[:, i]) \
             * + Ai[:, i] * ddthetalist[i] \
             * + np.dot(ad(Vi[:, i + 1]), Ai[:, i]) * dthetalist[i]
             */
        }
        for (int i = n - 1; i > -1; --i) {

            /*
             * Fi = np.dot(np.array(AdTi[i + 1]).T, Fi) \
             * + np.dot(np.array(Glist[i]), Vdi[:, i + 1]) \
             * - np.dot(np.array(ad(Vi[:, i + 1])).T, \
             * np.dot(np.array(Glist[i]), Vi[:, i + 1]))
             * taulist[i] = np.dot(np.array(Fi).T, Ai[:, i])
             */
        }
        return taulist;
    }
}
