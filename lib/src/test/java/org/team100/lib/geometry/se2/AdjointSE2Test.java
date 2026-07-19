package org.team100.lib.geometry.se2;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class AdjointSE2Test {
    @Test
    void test0() {
        Pose2d p = Pose2d.kZero;
        Matrix<N3, N3> ad = AdjointSE2.ad(p);
        Twist2d t = new Twist2d(1, 0, 0);
        Vector<N3> v = GeometryUtil.toVec(t);
        Matrix<N3, N1> v1 = ad.times(v);
        verify(new Twist2d(1, 0, 0), v1);
        Matrix<N3, N3> adInv = AdjointSE2.adInv(p);
        Matrix<N3, N1> v2 = adInv.times(v1);
        verify(t, v2);
    }

    @Test
    void test1() {
        Pose2d p = new Pose2d(1, 0, new Rotation2d());
        Matrix<N3, N3> ad = AdjointSE2.ad(p);
        Twist2d t = new Twist2d(1, 0, 0);
        Vector<N3> v = GeometryUtil.toVec(t);
        Matrix<N3, N1> v1 = ad.times(v);
        verify(new Twist2d(1, 0, 0), v1);
        Matrix<N3, N3> adInv = AdjointSE2.adInv(p);
        Matrix<N3, N1> v2 = adInv.times(v1);
        verify(t, v2);
    }

    @Test
    void test2() {
        Pose2d p = new Pose2d(0, 1, new Rotation2d());
        Matrix<N3, N3> ad = AdjointSE2.ad(p);
        Twist2d t = new Twist2d(1, 0, 0);
        Vector<N3> v = GeometryUtil.toVec(t);
        Matrix<N3, N1> v1 = ad.times(v);
        verify(new Twist2d(1, 0, 0), v1);
        Matrix<N3, N3> adInv = AdjointSE2.adInv(p);
        Matrix<N3, N1> v2 = adInv.times(v1);
        verify(t, v2);
    }

    @Test
    void test3() {
        Pose2d p = new Pose2d(0, 0, new Rotation2d(Math.PI / 2));
        Matrix<N3, N3> ad = AdjointSE2.ad(p);
        Twist2d t = new Twist2d(1, 0, 0);
        Vector<N3> v = GeometryUtil.toVec(t);
        Matrix<N3, N1> v1 = ad.times(v);
        verify(new Twist2d(0, 1, 0), v1);
        Matrix<N3, N3> adInv = AdjointSE2.adInv(p);
        Matrix<N3, N1> v2 = adInv.times(v1);
        verify(t, v2);
    }

    @Test
    void test4() {
        Pose2d p = new Pose2d(1, 0, new Rotation2d(Math.PI / 2));
        Matrix<N3, N3> ad = AdjointSE2.ad(p);
        Twist2d t = new Twist2d(1, 0, 0);
        Vector<N3> v = GeometryUtil.toVec(t);
        Matrix<N3, N1> v1 = ad.times(v);
        verify(new Twist2d(0, 1, 0), v1);
        Matrix<N3, N3> adInv = AdjointSE2.adInv(p);
        Matrix<N3, N1> v2 = adInv.times(v1);
        verify(t, v2);
    }

    @Test
    void test5() {
        Pose2d p = new Pose2d(1, 0, new Rotation2d());
        Matrix<N3, N3> ad = AdjointSE2.ad(p);
        System.out.println(ad);
        Twist2d t = new Twist2d(0, 0, 1);
        Vector<N3> v = GeometryUtil.toVec(t);
        Matrix<N3, N1> v1 = ad.times(v);
        // origin has to move -y to keep rotational center still
        verify(new Twist2d(0, -1, 1), v1);
        Matrix<N3, N3> adInv = AdjointSE2.adInv(p);
        Matrix<N3, N1> v2 = adInv.times(v1);
        verify(t, v2);
    }

    @Test
    void test6() {
        Pose2d p = new Pose2d(1, 0, new Rotation2d(Math.PI / 2));
        Matrix<N3, N3> ad = AdjointSE2.ad(p);
        Twist2d t = new Twist2d(0, 0, 1);
        Vector<N3> v = GeometryUtil.toVec(t);
        Matrix<N3, N1> v1 = ad.times(v);
        // origin has to move -y to keep rotational center still
        verify(new Twist2d(0, -1, 1), v1);
        Matrix<N3, N3> adInv = AdjointSE2.adInv(p);
        Matrix<N3, N1> v2 = adInv.times(v1);
        verify(t, v2);
    }

    private void verify(Twist2d t, Matrix<N3, N1> v) {
        assertEquals(t.dx, v.get(0, 0), 1e-3, "x");
        assertEquals(t.dy, v.get(1, 0), 1e-3, "y");
        assertEquals(t.dtheta, v.get(2, 0), 1e-3, "r");
    }
}
