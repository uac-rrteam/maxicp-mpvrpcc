/*
 * MaxiCP is under MIT License
 * Copyright (c)  2023 UCLouvain
 */

package org.maxicp.cp.engine.constraints.seqvar;

import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.maxicp.cp.CPSolverTest;
import org.maxicp.cp.engine.core.CPSeqVar;
import org.maxicp.cp.engine.core.CPSolver;
import org.maxicp.state.StateManager;
import org.maxicp.util.exception.InconsistencyException;

import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrowsExactly;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.maxicp.cp.CPFactory.makeSeqVar;
import static org.maxicp.modeling.algebra.sequence.SeqStatus.EXCLUDED;
import static org.maxicp.modeling.algebra.sequence.SeqStatus.POSSIBLE;

public class FirstLastTest extends CPSolverTest {

    @ParameterizedTest
    @MethodSource("getSolver")
    public void testSingleFirstCandidate(CPSolver cp) {
        CPSeqVar sequence = makeSeqVar(cp, 5, 3, 4);

        cp.post(new First(sequence, Set.of(1)));

        assertEquals(1, sequence.memberAfter(sequence.start()));
        for (int node : new int[] {0, 2}) {
            assertTrue(sequence.isNode(node, POSSIBLE));
            assertFalse(sequence.hasInsert(sequence.start(), node));
            assertTrue(sequence.hasInsert(1, node));
        }
    }

    @ParameterizedTest
    @MethodSource("getSolver")
    public void testSingleLastCandidate(CPSolver cp) {
        CPSeqVar sequence = makeSeqVar(cp, 5, 3, 4);

        cp.post(new Last(sequence, 1));

        assertEquals(1, sequence.memberBefore(sequence.end()));
        for (int node : new int[] {0, 2}) {
            assertTrue(sequence.isNode(node, POSSIBLE));
            assertFalse(sequence.hasInsert(1, node));
            assertTrue(sequence.hasInsert(sequence.start(), node));
        }
    }

    @ParameterizedTest
    @MethodSource("getSolver")
    public void testFirstCandidatesCanOnlyBeInsertedFirst(CPSolver cp) {
        CPSeqVar sequence = makeSeqVar(cp, 6, 4, 5);
        sequence.insert(sequence.start(), 2);
        sequence.insert(2, 3);

        cp.post(new First(sequence, Set.of(0, 1)));

        for (int candidate : new int[] {0, 1}) {
            assertEquals(1, sequence.nInsert(candidate));
            assertTrue(sequence.hasInsert(sequence.start(), candidate));
        }
        assertFalse(sequence.hasInsert(sequence.start(), 2));
    }

    @ParameterizedTest
    @MethodSource("getSolver")
    public void testLastCandidatesCanOnlyBeInsertedLast(CPSolver cp) {
        CPSeqVar sequence = makeSeqVar(cp, 6, 4, 5);
        sequence.insert(sequence.start(), 2);
        sequence.insert(2, 3);

        cp.post(new Last(sequence, Set.of(0, 1)));

        for (int candidate : new int[] {0, 1}) {
            assertEquals(1, sequence.nInsert(candidate));
            assertTrue(sequence.hasInsert(3, candidate));
        }
        assertFalse(sequence.hasInsert(3, 2));
    }

    @ParameterizedTest
    @MethodSource("getSolver")
    public void testOnlyRemainingFirstCandidateIsSelected(CPSolver cp) {
        CPSeqVar sequence = makeSeqVar(cp, 5, 3, 4);
        cp.post(new First(sequence, Set.of(0, 1)));

        sequence.exclude(0);
        cp.fixPoint();

        assertEquals(1, sequence.memberAfter(sequence.start()));
        assertTrue(sequence.isNode(0, EXCLUDED));
    }

    @ParameterizedTest
    @MethodSource("getSolver")
    public void testOnlyRemainingLastCandidateIsSelected(CPSolver cp) {
        CPSeqVar sequence = makeSeqVar(cp, 5, 3, 4);
        cp.post(new Last(sequence, Set.of(0, 1)));

        sequence.exclude(0);
        cp.fixPoint();

        assertEquals(1, sequence.memberBefore(sequence.end()));
        assertTrue(sequence.isNode(0, EXCLUDED));
    }

    @ParameterizedTest
    @MethodSource("getSolver")
    public void testSelectedCandidateExcludesOtherCandidates(CPSolver cp) {
        CPSeqVar sequence = makeSeqVar(cp, 6, 4, 5);
        cp.post(new First(sequence, Set.of(0, 1, 2)));

        sequence.insert(sequence.start(), 1);
        cp.fixPoint();

        assertTrue(sequence.isNode(0, EXCLUDED));
        assertTrue(sequence.isNode(2, EXCLUDED));
        assertEquals(1, sequence.memberAfter(sequence.start()));
    }

    @ParameterizedTest
    @MethodSource("getSolver")
    public void testConstraintIsRestoredAfterBacktracking(CPSolver cp) {
        CPSeqVar sequence = makeSeqVar(cp, 5, 3, 4);
        cp.post(new First(sequence, Set.of(0, 1)));
        StateManager stateManager = cp.getStateManager();
        stateManager.saveState();

        sequence.insert(sequence.start(), 0);
        cp.fixPoint();
        assertTrue(sequence.isNode(1, EXCLUDED));

        stateManager.restoreState();
        assertTrue(sequence.isNode(0, POSSIBLE));
        assertTrue(sequence.isNode(1, POSSIBLE));

        sequence.insert(sequence.start(), 1);
        cp.fixPoint();
        assertTrue(sequence.isNode(0, EXCLUDED));
    }

    @ParameterizedTest
    @MethodSource("getSolver")
    public void testCandidateAlreadyAtWrongPositionIsInconsistent(CPSolver cp) {
        CPSeqVar sequence = makeSeqVar(cp, 6, 4, 5);
        sequence.insert(sequence.start(), 2);
        sequence.insert(2, 0);

        assertThrowsExactly(InconsistencyException.class,
                () -> cp.post(new First(sequence, Set.of(0, 1))));
    }

    @ParameterizedTest
    @MethodSource("getSolver")
    public void testCandidateAlreadyBeforeAnotherNodeCannotBeLast(CPSolver cp) {
        CPSeqVar sequence = makeSeqVar(cp, 6, 4, 5);
        sequence.insert(sequence.start(), 0);
        sequence.insert(0, 2);

        assertThrowsExactly(InconsistencyException.class,
                () -> cp.post(new Last(sequence, Set.of(0, 1))));
    }

    @ParameterizedTest
    @MethodSource("getSolver")
    public void testEmptyCandidateSetIsInconsistent(CPSolver cp) {
        CPSeqVar firstSequence = makeSeqVar(cp, 4, 2, 3);
        CPSeqVar lastSequence = makeSeqVar(cp, 4, 2, 3);

        assertThrowsExactly(InconsistencyException.class,
                () -> cp.post(new First(firstSequence, Set.of())));
        assertThrowsExactly(InconsistencyException.class,
                () -> cp.post(new Last(lastSequence, Set.of())));
    }
}
