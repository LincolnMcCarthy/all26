package org.team100.lib.dynamics.rr;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.rr.RRAcceleration;
import org.team100.lib.geometry.rr.RRConfig;
import org.team100.lib.geometry.rr.RRVelocity;

public class RRDynamicsNewtonEulerTest {
    private static final double DELTA = 1e-3;

    // Same test cases as the other RR Dynamics, but with
    // different implementation.
    // Since the new implementation also does forward dynamics,
    // add the inverse of each case.

    @Test
    void test0() {
        RRDynamicsNewtonEuler d = new RRDynamicsNewtonEuler(
                1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // straight up
        RREffort t = d.effort(
                new RRConfig(0, 0),
                new RRVelocity(0, 0),
                new RRAcceleration(0, 0));
        // no torques
        assertEquals(0, t.t1(), DELTA);
        assertEquals(0, t.t2(), DELTA);
    }

    @Test
    void test0a() {
        RRDynamicsNewtonEuler d = new RRDynamicsNewtonEuler(
                1, 1, 1, 1, 0.5, 0.5, 1, 1);
        RRAcceleration t = d.qddot(
                new RRConfig(0, 0),
                new RRVelocity(0, 0),
                new RREffort(0, 0));
        assertEquals(0, t.q1ddot(), DELTA);
        assertEquals(0, t.q2ddot(), DELTA);
    }

    @Test
    void test1() {
        RRDynamicsNewtonEuler d = new RRDynamicsNewtonEuler(
                1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // to the side
        RREffort t = d.effort(
                new RRConfig(Math.PI / 2, 0),
                new RRVelocity(0, 0),
                new RRAcceleration(0, 0));
        // 1 kg is 0.5 m away, so 5Nm, 1 kg 1.5 m away so 15Nm
        assertEquals(-19.6, t.t1(), DELTA);
        // 1 kg 0.5 m away
        assertEquals(-4.9, t.t2(), DELTA);
    }

    @Test
    void test1a() {
        RRDynamicsNewtonEuler d = new RRDynamicsNewtonEuler(
                1, 1, 1, 1, 0.5, 0.5, 1, 1);
        RRAcceleration t = d.qddot(
                new RRConfig(Math.PI / 2, 0),
                new RRVelocity(0, 0),
                new RREffort(-19.6, -4.9));
        assertEquals(0, t.q1ddot(), DELTA);
        assertEquals(0, t.q2ddot(), DELTA);
    }

    @Test
    void test2() {
        RRDynamicsNewtonEuler d = new RRDynamicsNewtonEuler(
                1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // wrist only to the side (bent arm)
        RREffort t = d.effort(
                new RRConfig(0, Math.PI / 2),
                new RRVelocity(0, 0),
                new RRAcceleration(0, 0));
        // 1 kg 0.5 m away so 5Nm
        assertEquals(-4.9, t.t1(), DELTA);
        // 1 kg 0.5 m away (same as above)
        assertEquals(-4.9, t.t2(), DELTA);
    }

    @Test
    void test3() {
        RRDynamicsNewtonEuler d = new RRDynamicsNewtonEuler(
                1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // bent arm moving at the root
        RREffort t = d.effort(
                new RRConfig(0, Math.PI / 2),
                new RRVelocity(1, 0),
                new RRAcceleration(0, 0));
        // 1 kg 0.5 m away so 5Nm
        assertEquals(-4.9, t.t1(), DELTA);
        // 1 kg 0.5 m away (same as above), minus centrifugal force
        assertEquals(-4.4, t.t2(), DELTA);
    }

    @Test
    void test4() {
        RRDynamicsNewtonEuler d = new RRDynamicsNewtonEuler(
                1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // bent arm accelerating at the root
        RREffort t = d.effort(
                new RRConfig(0, Math.PI / 2),
                new RRVelocity(0, 0),
                new RRAcceleration(1, 0));
        // 1 kg 0.5 m away so 5Nm, minus centrifugal force
        assertEquals(-1.4, t.t1(), DELTA);
        // 1 kg 0.5 m away (same as above), minus centrifugal force
        assertEquals(-3.65, t.t2(), DELTA);
    }

    @Test
    void test5() {
        RRDynamicsNewtonEuler d = new RRDynamicsNewtonEuler(
                1, 1, 1, 1, 0.5, 0.5, 1, 1);
        // like a whip: extended, moving, slowing down at the root
        RREffort t = d.effort(
                new RRConfig(0, 0),
                new RRVelocity(1, 0),
                new RRAcceleration(-1, 0));
        // elbow tries to keep going, so push back
        assertEquals(-4.5, t.t1(), DELTA);
        // trying to slow down
        assertEquals(-1.75, t.t2(), DELTA);
    }

    @Test
    void test5a() {
        // inverse of 5
        RRDynamicsNewtonEuler d = new RRDynamicsNewtonEuler(
                1, 1, 1, 1, 0.5, 0.5, 1, 1);
        RRAcceleration t = d.qddot(
                new RRConfig(0, 0),
                new RRVelocity(1, 0),
                new RREffort(-4.5, -1.75));
        assertEquals(-1, t.q1ddot(), DELTA);
        assertEquals(0, t.q2ddot(), DELTA);
    }
}
