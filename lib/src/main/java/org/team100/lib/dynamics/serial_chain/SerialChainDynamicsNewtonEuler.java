package org.team100.lib.dynamics.serial_chain;

import org.team100.lib.util.FixedList;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
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
     * Calculate the 6x6 matrix [adV] of the given 6-vector
     * 
     * Used to calculate the Lie bracket [V1, V2] = [adV1]V2
     * 
     * | w 0 |
     * | [w] w |
     * 
     * @param V A 6-vector spatial velocity
     * @return The corresponding 6x6 matrix [adV]
     */
    static Matrix<N6, N6> ad(Matrix<N6, N1> V) {
        Matrix<N3, N3> omgmat = VecToso3(V.block(Nat.N3(), Nat.N1(), 0, 0));
        Matrix<N6, N6> ad = new Matrix<>(Nat.N6(), Nat.N6());
        ad.assignBlock(0, 0, omgmat);
        ad.assignBlock(3, 0, VecToso3(V.block(Nat.N3(), Nat.N1(), 3, 0)));
        ad.assignBlock(3, 3, omgmat);
        return ad;
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
     * @param Slist       Screw axes Si of the joints in a space frame
     * @return The n-vector of required joint forces/torques
     */
    public Vector<N> InverseDynamics(
            Vector<N> thetalist,
            Vector<N> dthetalist,
            Vector<N> ddthetalist,
            Vector<N3> g,
            Vector<N6> Ftip,
            FixedList<M, Matrix<N4, N4>> Mlist,
            FixedList<N, Matrix<N6, N6>> Glist,
            FixedList<N, Vector<N6>> Slist) {
        int n = num.getNum();
        Matrix<N4, N4> Mi = Matrix.eye(Nat.N4());
        Matrix<N6, N> Ai = new Matrix<>(Nat.N6(), num);
        // a list of adjoint matrices
        FixedList<M, Matrix<N6, N6>> AdTi = new FixedList<>(numPlusOne);
        Matrix<N6, M> Vi = new Matrix<>(Nat.N6(), numPlusOne);
        // this means Vdot
        Matrix<N6, M> Vdi = new Matrix<>(Nat.N6(), numPlusOne);
        // Vdot for the base
        Vector<N6> Vd0 = new Vector<>(Nat.N6());
        Vd0.assignBlock(3, 0, g.times(-1));
        // so this is [0,0,0,-g] where g is a 3 vector
        Vdi.assignBlock(0, 0, Vd0);
        // tool point adjoint
        AdTi.set(n, Adjoint(TransInv(Mlist.get(n))));
        // tool point force
        Matrix<N6, N1> Fi = Ftip.copy();

        Vector<N> taulist = new Vector<>(num);
        // Walk from the base to the tool point, computing velocity and accel.
        for (int i = 0; i < n; ++i) {
            Mi = Mi.times(Mlist.get(i));
            // A_i = Ad_{m_i^-1} S_i
            Ai.assignBlock(0, i, Adjoint(TransInv(Mi)).times(Slist.get(i)));
            AdTi.set(i, Adjoint(
                    MatrixExp6(
                            VecTose3(col6(Ai, i)
                                    .times(-thetalist.get(i))))
                            .times(TransInv(Mlist.get(i)))));
            Vi.assignBlock(0, i + 1,
                    AdTi.get(i).times(col6(Vi, i))
                            .plus(col6(Ai, i).times(dthetalist.get(i))));
            Vdi.assignBlock(0, i + 1,
                    AdTi.get(i).times(col6(Vdi, i))
                            .plus(col6(Ai, i).times(ddthetalist.get(i)))
                            .plus(ad(col6(Vi, i + 1))
                                    .times(col6(Ai, i))
                                    .times(dthetalist.get(i))));
        }
        // Walk from the tool point to the base, computing force and torque
        for (int i = n - 1; i > -1; --i) {
            Fi = AdTi.get(i + 1).transpose().times(Fi)
                    .plus(Glist.get(i).times(col6(Vdi, i + 1)))
                    .minus(ad(col6(Vi, i + 1)).transpose().times(
                            Glist.get(i)).times(col6(Vi, i + 1)));
            taulist.set(i, 0, Fi.transpose().times(col6(Ai, i)).get(0, 0));
        }
        return taulist;
    }

    /**
     * Computes the mass matrix of an open chain robot based on the given
     * configuration.
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
     * @param Slist     Screw axes Si of the joints in a space frame
     * @return The numerical inertia matrix M
     */
    Matrix<N, N> MassMatrix(
            Vector<N> thetalist,
            FixedList<M, Matrix<N4, N4>> Mlist,
            FixedList<N, Matrix<N6, N6>> Glist,
            FixedList<N, Vector<N6>> Slist) {
        int n = num.getNum();
        Matrix<N, N> M = new Matrix<>(num, num);
        for (int i = 0; i < n; ++i) {
            Vector<N> ddthetalist = new Vector<>(num);
            ddthetalist.set(i, 0, 1);
            M.setColumn(i, InverseDynamics(
                    thetalist,
                    new Vector<>(num),
                    ddthetalist,
                    new Vector<>(Nat.N3()),
                    new Vector<>(Nat.N6()),
                    Mlist, Glist, Slist));
        }
        return M;
    }

    /**
     * Computes the Coriolis and centripetal terms in the inverse dynamics of
     * an open chain robot
     * 
     * This function calls InverseDynamics with g = 0, Ftip = 0, and
     * ddthetalist = 0.
     * 
     * @param thetalist  A list of joint variables
     * @param dthetalist A list of joint rates
     * @param Mlist      List of link frames i relative to i-1 at the home position
     * @param Glist      Spatial inertia matrices Gi of the links
     * @param Slist      Screw axes Si of the joints in a space frame
     * @return The vector of Coriolis and centripetal terms
     */
    Vector<N> VelQuadraticForces(
            Vector<N> thetalist,
            Vector<N> dthetalist,
            FixedList<M, Matrix<N4, N4>> Mlist,
            FixedList<N, Matrix<N6, N6>> Glist,
            FixedList<N, Vector<N6>> Slist) {
        return InverseDynamics(
                thetalist,
                dthetalist,
                new Vector<>(num),
                new Vector<>(Nat.N3()),
                new Vector<>(Nat.N6()),
                Mlist, Glist, Slist);
    }

    /**
     * 
     * Computes the joint forces/torques an open chain robot requires to
     * overcome gravity at its configuration
     * 
     * This function calls InverseDynamics with Ftip = 0, dthetalist = 0, and
     * ddthetalist = 0.
     * 
     * @param thetalist A list of joint variables
     * @param g         3-vector for gravitational acceleration
     * @param Mlist     List of link frames i relative to i-1 at the home position
     * @param Glist     Spatial inertia matrices Gi of the links
     * @param Slist     Screw axes Si of the joints in a space frame
     * @return The joint forces/torques required to overcome gravity
     */
    Vector<N> GravityForces(
            Vector<N> thetalist,
            Vector<N3> g,
            FixedList<M, Matrix<N4, N4>> Mlist,
            FixedList<N, Matrix<N6, N6>> Glist,
            FixedList<N, Vector<N6>> Slist) {
        return InverseDynamics(
                thetalist,
                new Vector<>(num),
                new Vector<>(num),
                g,
                new Vector<>(Nat.N6()),
                Mlist, Glist, Slist);
    }

    /**
     * Computes the joint forces/torques an open chain robot requires only to
     * create the end-effector force Ftip
     * 
     * This function calls InverseDynamics with g = 0, dthetalist = 0, and
     * ddthetalist = 0.
     * 
     * @param thetalist A list of joint variables
     * @param Ftip      Spatial force applied by the end-effector expressed in frame
     *                  {n+1}
     * @param Mlist     List of link frames i relative to i-1 at the home position
     * @param Glist     Spatial inertia matrices Gi of the links
     * @param Slist     Screw axes Si of the joints in a space frame
     * @return The joint forces and torques required only to create the
     *         end-effector force Ftip
     */
    Vector<N> EndEffectorForces(
            Vector<N> thetalist,
            Vector<N6> Ftip,
            FixedList<M, Matrix<N4, N4>> Mlist,
            FixedList<N, Matrix<N6, N6>> Glist,
            FixedList<N, Vector<N6>> Slist) {
        return InverseDynamics(
                thetalist,
                new Vector<>(num),
                new Vector<>(num),
                new Vector<>(Nat.N3()),
                Ftip,
                Mlist, Glist, Slist);
    }

    /**
     * Computes forward dynamics in the space frame for an open chain robot.
     * 
     * This function computes ddthetalist by solving:
     * Mlist(thetalist) * ddthetalist = taulist - c(thetalist,dthetalist) -
     * g(thetalist) - Jtr(thetalist) * Ftip
     * 
     * @param thetalist  A list of joint variables
     * @param dthetalist A list of joint rates
     * @param taulist    An n-vector of joint forces/torques
     * @param g          Gravity vector g
     * @param Ftip       Spatial force applied by the end-effector expressed in
     *                   frame {n+1}
     * @param Mlist      List of link frames i relative to i-1 at the home position
     * @param Glist      Spatial inertia matrices Gi of the links
     * @param Slist      Screw axes Si of the joints in a space frame
     * @return The resulting joint accelerations
     */
    public Vector<N> ForwardDynamics(
            Vector<N> thetalist,
            Vector<N> dthetalist,
            Vector<N> taulist,
            Vector<N3> g,
            Vector<N6> Ftip,
            FixedList<M, Matrix<N4, N4>> Mlist,
            FixedList<N, Matrix<N6, N6>> Glist,
            FixedList<N, Vector<N6>> Slist) {
        return new Vector<>(MassMatrix(thetalist, Mlist, Glist, Slist).inv().times(taulist
                .minus(VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist))
                .minus(GravityForces(thetalist, g, Mlist, Glist, Slist))
                .minus(EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist))));
    }

    /** Return column i from 6xn matrix m. */
    private static Matrix<N6, N1> col6(Matrix<N6, ? extends Num> m, int i) {
        return m.block(Nat.N6(), Nat.N1(), 0, i);
    }

    /**
     * Inverts a homogeneous transformation matrix
     * Uses the structure of transformation matrices to avoid taking a matrix
     * inverse, for efficiency.
     * 
     * i.e. it's the inverse (i.e. transpose) rotation and also the inverse rotation
     * rotating the inverse (i.e. negative) translation.
     * 
     * | Rt -Rt*p |
     * | 0 1 |
     * 
     * @param T A homogeneous transformation matrix
     * @return The inverse of T
     */
    static Matrix<N4, N4> TransInv(Matrix<N4, N4> T) {
        Pair<Matrix<N3, N3>, Matrix<N3, N1>> pair = TransToRp(T);
        Matrix<N3, N3> R = pair.getFirst();
        Matrix<N3, N1> p = pair.getSecond();
        Matrix<N3, N3> Rt = R.transpose();
        Matrix<N4, N4> Tinv = new Matrix<>(Nat.N4(), Nat.N4());
        Tinv.assignBlock(0, 0, Rt);
        Tinv.assignBlock(0, 3, Rt.times(p).times(-1));
        Tinv.set(3, 3, 1);
        return Tinv;
    }

    /**
     * Converts a homogeneous transformation matrix into a rotation matrix
     * and position vector
     * 
     * @param T A homogeneous transformation matrix
     * @return Pair of (rotation matrix, position vector)
     */
    static Pair<Matrix<N3, N3>, Matrix<N3, N1>> TransToRp(Matrix<N4, N4> T) {
        return new Pair<>(
                T.block(Nat.N3(), Nat.N3(), 0, 0),
                T.block(Nat.N3(), Nat.N1(), 0, 3));
    }

    /**
     * Computes the adjoint representation of a homogeneous transformation
     * matrix.
     * 
     * | R 0 |
     * | [p]R R |
     * 
     * @param T A homogeneous transformation matrix
     * @return The 6x6 adjoint representation [AdT] of T
     */
    static Matrix<N6, N6> Adjoint(Matrix<N4, N4> T) {
        Pair<Matrix<N3, N3>, Matrix<N3, N1>> pair = TransToRp(T);
        Matrix<N3, N3> R = pair.getFirst();
        Matrix<N3, N1> p = pair.getSecond();
        Matrix<N6, N6> AdT = new Matrix<>(Nat.N6(), Nat.N6());
        AdT.assignBlock(0, 0, R);
        AdT.assignBlock(3, 0, VecToso3(p).times(R));
        AdT.assignBlock(3, 3, R);
        return AdT;
    }

    /**
     * Converts a 3-vector to an so(3) representation
     * 
     * @param omg A 3-vector omega
     * @return The skew-symmetric representation of omg
     */
    static Matrix<N3, N3> VecToso3(Matrix<N3, N1> omg) {
        return MatBuilder.fill(Nat.N3(), Nat.N3(), //
                0, -omg.get(2, 0), omg.get(1, 0), //
                omg.get(2, 0), 0, -omg.get(0, 0), //
                -omg.get(1, 0), omg.get(0, 0), 0);
    }

    /**
     * Computes the matrix exponential of an se3 representation of
     * exponential coordinates
     * 
     * @param se3mat A matrix in se3
     * @return The matrix exponential of se3mat
     */
    static Matrix<N4, N4> MatrixExp6(Matrix<N4, N4> se3mat) {
        Matrix<N3, N1> omgtheta = so3ToVec(se3mat.block(Nat.N3(), Nat.N3(), 0, 0));
        if (omgtheta.normF() < 1e-6) {
            Matrix<N4, N4> exp = new Matrix<>(Nat.N4(), Nat.N4());
            exp.assignBlock(0, 0, Matrix.eye(Nat.N3()));
            exp.assignBlock(0, 3, se3mat.block(Nat.N3(), Nat.N1(), 0, 3));
            exp.set(3, 3, 1);
            return exp;
        }
        double theta = AxisAng3(omgtheta).getSecond();
        Matrix<N3, N3> omgmat = se3mat.block(Nat.N3(), Nat.N3(), 0, 0).div(theta);
        Matrix<N4, N4> exp = new Matrix<>(Nat.N4(), Nat.N4());
        exp.assignBlock(0, 0, MatrixExp3(se3mat.block(Nat.N3(), Nat.N3(), 0, 0)));
        exp.assignBlock(0, 3, Matrix.eye(Nat.N3()).times(theta)
                .plus(omgmat.times(1 - Math.cos(theta)))
                .plus(omgmat.times(omgmat).times(theta - Math.sin(theta)))
                .times(se3mat.block(Nat.N3(), Nat.N1(), 0, 3).div(theta)));
        exp.set(3, 3, 1);
        return exp;
    }

    /**
     * Computes the matrix exponential of a matrix in so(3)
     * 
     * @param so3mat A 3x3 skew-symmetric matrix
     * @return The matrix exponential of so3mat
     */
    static Matrix<N3, N3> MatrixExp3(Matrix<N3, N3> so3mat) {
        Matrix<N3, N1> omgtheta = so3ToVec(so3mat);
        if (omgtheta.normF() < 1e-6) {
            return Matrix.eye(Nat.N3());
        }
        double theta = AxisAng3(omgtheta).getSecond();
        Matrix<N3, N3> omgmat = so3mat.div(theta);
        return Matrix.eye(Nat.N3())
                .plus(omgmat.times(Math.sin(theta)))
                .plus(omgmat.times(omgmat).times((1 - Math.cos(theta))));
    }

    /**
     * Converts an so(3) representation to a 3-vector
     * 
     * @param so3mat A 3x3 skew-symmetric matrix
     * @return The 3-vector corresponding to so3mat
     */
    static Matrix<N3, N1> so3ToVec(Matrix<N3, N3> so3mat) {
        return MatBuilder.fill(Nat.N3(), Nat.N1(), //
                so3mat.get(2, 1), //
                so3mat.get(0, 2), //
                so3mat.get(1, 0));
    }

    /**
     * Converts a 3-vector of exponential coordinates for rotation into
     * axis-angle form
     * 
     * @param expc3 A 3-vector of exponential coordinates for rotation
     * @return A pair of (unit rotation axis, corresponding rotation angle)
     */
    static Pair<Matrix<N3, N1>, Double> AxisAng3(Matrix<N3, N1> expc3) {
        return new Pair<>(expc3.div(expc3.normF()), expc3.normF());
    }

    /**
     * Converts a spatial velocity vector into a 4x4 matrix in se3
     * 
     * @param V A 6-vector representing a spatial velocity
     * @return The 4x4 se3 representation of V
     */
    static Matrix<N4, N4> VecTose3(Matrix<N6, N1> V) {
        Matrix<N4, N4> se3 = new Matrix<>(Nat.N4(), Nat.N4());
        se3.assignBlock(0, 0, VecToso3(V.block(Nat.N3(), Nat.N1(), 0, 0)));
        se3.assignBlock(0, 3, V.block(Nat.N3(), Nat.N1(), 3, 0));
        return se3;
    }
}
