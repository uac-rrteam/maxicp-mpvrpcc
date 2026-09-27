/*
 * MaxiCP is under MIT License
 * Copyright (c)  2023 UCLouvain
 */

package org.maxicp.cp.engine.constraints.seqvar;

import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.maxicp.cp.CPSolverTest;
import org.maxicp.cp.engine.core.CPIntVar;
import org.maxicp.cp.engine.core.CPSeqVar;
import org.maxicp.cp.engine.core.CPSolver;
import org.maxicp.state.StateManager;
import org.maxicp.util.exception.InconsistencyException;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrowsExactly;
import static org.maxicp.cp.CPFactory.makeIntVar;
import static org.maxicp.cp.CPFactory.makeSeqVar;

public class TransitionCostTest extends CPSolverTest {

    private static final int PENALTY_INF = Integer.MAX_VALUE;

    @ParameterizedTest
    @MethodSource("getSolver")
    public void testCostOfInitiallyFixedSequence(CPSolver cp) {
        CPSeqVar seqVar = makeSeqVar(cp, 2, 0, 1);
        CPIntVar total = makeIntVar(cp, -20, 20);
        int[][] costs = {
                {0, -7},
                {3, 0}
        };

        cp.post(new TransitionCost(seqVar, costs, total));

        assertEquals(-7, total.min());
        assertEquals(-7, total.max());
    }

    @ParameterizedTest
    @MethodSource("getSolver")
    public void testTotalIsFixedOnlyWhenSequenceIsFixed(CPSolver cp) {
        CPSeqVar seqVar = makeSeqVar(cp, 4, 2, 3);
        CPIntVar total = makeIntVar(cp, -50, 50);
        int[][] costs = {
                {0, 0, 0, -4},
                {0, 0, 0, 0},
                {6, 0, 0, 8},
                {0, 0, 0, 0}
        };
        cp.post(new TransitionCost(seqVar, costs, total));

        assertEquals(-50, total.min());
        assertEquals(50, total.max());

        seqVar.insert(seqVar.start(), 0);
        cp.fixPoint();
        assertFalse(seqVar.isFixed());
        assertEquals(-50, total.min());
        assertEquals(50, total.max());

        seqVar.exclude(1);
        cp.fixPoint();
        assertEquals(2, total.min());
        assertEquals(2, total.max());
    }

    @ParameterizedTest
    @MethodSource("getSolver")
    public void testOrderedAsymmetricCostsAndStateRestoration(CPSolver cp) {
        CPSeqVar seqVar = makeSeqVar(cp, 5, 3, 4);
        CPIntVar total = makeIntVar(cp, -100, 100);
        int[][] costs = {
                {0, -5, 0, 0, -8},
                {6, 0, 0, 0, 9},
                {0, 0, 0, 0, 0},
                {7, 4, 0, 0, 0},
                {0, 0, 0, 0, 0}
        };
        cp.post(new TransitionCost(seqVar, costs, total));
        StateManager sm = cp.getStateManager();
        sm.saveState();

        seqVar.insert(3, 0);
        seqVar.insert(0, 1);
        seqVar.exclude(2);
        cp.fixPoint();
        assertEquals(11, total.min()); // 3 -> 0 -> 1 -> 4: 7 - 5 + 9

        sm.restoreState();
        assertEquals(-100, total.min());
        assertEquals(100, total.max());

        seqVar.insert(3, 1);
        seqVar.insert(1, 0);
        seqVar.exclude(2);
        cp.fixPoint();
        assertEquals(2, total.min()); // 3 -> 1 -> 0 -> 4: 4 + 6 - 8
        assertEquals(2, total.max());
    }

    @ParameterizedTest
    @MethodSource("getSolver")
    public void testPenaltyOnUsedTransitionIsInconsistent(CPSolver cp) {
        CPSeqVar seqVar = makeSeqVar(cp, 2, 0, 1);
        CPIntVar total = makeIntVar(cp, 0, PENALTY_INF - 1);
        int[][] costs = {
                {0, PENALTY_INF},
                {0, 0}
        };

        assertThrowsExactly(InconsistencyException.class,
                () -> cp.post(new TransitionCost(seqVar, costs, total)));
    }

    @ParameterizedTest
    @MethodSource("getSolver")
    public void testPenaltyOnUnusedTransitionIsIgnored(CPSolver cp) {
        CPSeqVar seqVar = makeSeqVar(cp, 3, 1, 2);
        seqVar.exclude(0);
        CPIntVar total = makeIntVar(cp, 0, 20);
        int[][] costs = {
                {0, 0, PENALTY_INF},
                {PENALTY_INF, 0, 12},
                {0, 0, 0}
        };

        cp.post(new TransitionCost(seqVar, costs, total));

        assertEquals(12, total.min());
        assertEquals(12, total.max());
    }

    @ParameterizedTest
    @MethodSource("getSolver")
    public void testMillionIsAValidFiniteCost(CPSolver cp) {
        CPSeqVar seqVar = makeSeqVar(cp, 2, 0, 1);
        CPIntVar total = makeIntVar(cp, 0, 1_000_000);
        int[][] costs = {
                {0, 1_000_000},
                {0, 0}
        };

        cp.post(new TransitionCost(seqVar, costs, total));

        assertEquals(1_000_000, total.min());
        assertEquals(1_000_000, total.max());
    }

    @ParameterizedTest
    @MethodSource("getSolver")
    public void testCostOutsideTotalDomainIsInconsistent(CPSolver cp) {
        CPSeqVar seqVar = makeSeqVar(cp, 2, 0, 1);
        CPIntVar total = makeIntVar(cp, 0, 11);
        int[][] costs = {
                {0, 12},
                {0, 0}
        };

        assertThrowsExactly(InconsistencyException.class,
                () -> cp.post(new TransitionCost(seqVar, costs, total)));
    }
}
