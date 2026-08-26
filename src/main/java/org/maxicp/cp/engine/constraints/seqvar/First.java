/*
 * MaxiCP is under MIT License
 * Copyright (c)  2023 UCLouvain
 */

package org.maxicp.cp.engine.constraints.seqvar;

import org.maxicp.cp.engine.core.AbstractCPConstraint;
import org.maxicp.cp.engine.core.CPSeqVar;

import java.util.Arrays;
import java.util.Set;
import java.util.stream.Collectors;

import static org.maxicp.modeling.algebra.sequence.SeqStatus.INSERTABLE;
import static org.maxicp.modeling.algebra.sequence.SeqStatus.MEMBER;
import static org.maxicp.util.exception.InconsistencyException.INCONSISTENCY;

/**
 * Ensures that exactly one node among a set of candidates is visited first.
 * The selected candidate is inserted immediately after the start node and all
 * other candidates are excluded.
 */
public class First extends AbstractCPConstraint {

    private final CPSeqVar seqVar;
    private final Set<Integer> candidates;
    private final int[] nodes;

    /**
     * Creates a first-node constraint.
     *
     * @param seqVar sequence whose first visit is constrained
     * @param candidates candidates for the first visited node
     */
    public First(CPSeqVar seqVar, Set<Integer> candidates) {
        super(seqVar.getSolver());
        this.seqVar = seqVar;
        this.candidates = Set.copyOf(candidates);
        this.nodes = new int[seqVar.nNode()];
    }

    /**
     * Creates a first-node constraint.
     *
     * @param seqVar sequence whose first visit is constrained
     * @param candidates candidates for the first visited node
     */
    public First(CPSeqVar seqVar, int... candidates) {
        this(seqVar, Arrays.stream(candidates).boxed().collect(Collectors.toSet()));
    }

    @Override
    public void post() {
        propagate();
        if (isActive()) {
            seqVar.propagateOnInsert(this);
            seqVar.propagateOnExclude(this);
            seqVar.propagateOnInsertRemoved(this);
        }
    }

    @Override
    public void propagate() {
        if (candidates.isEmpty()) {
            throw INCONSISTENCY;
        }

        removeNonFirstInsertionsFromCandidates();

        int firstNode = seqVar.memberAfter(seqVar.start());
        Integer selectedCandidate = null;
        int availableCandidates = 0;
        int lastAvailableCandidate = -1;

        for (int candidate : candidates) {
            if (seqVar.isNode(candidate, MEMBER)) {
                if (candidate != firstNode || selectedCandidate != null) {
                    throw INCONSISTENCY;
                }
                selectedCandidate = candidate;
            } else if (seqVar.hasInsert(seqVar.start(), candidate)) {
                availableCandidates++;
                lastAvailableCandidate = candidate;
            }
        }

        if (selectedCandidate != null) {
            select(selectedCandidate);
        } else if (availableCandidates == 1) {
            seqVar.insert(seqVar.start(), lastAvailableCandidate);
            select(lastAvailableCandidate);
        } else if (availableCandidates == 0) {
            throw INCONSISTENCY;
        }
    }

    private void removeNonFirstInsertionsFromCandidates() {
        int start = seqVar.start();
        for (int candidate : candidates) {
            if (!seqVar.isNode(candidate, INSERTABLE)) {
                continue;
            }
            int insertionCount = seqVar.fillInsert(candidate, nodes);
            for (int position = 0;
                 position < insertionCount && seqVar.isNode(candidate, INSERTABLE);
                 position++) {
                int predecessor = nodes[position];
                if (predecessor != start && seqVar.hasInsert(predecessor, candidate)) {
                    seqVar.notBetween(predecessor, candidate, seqVar.memberAfter(predecessor));
                }
            }
        }
    }

    private void select(int selectedCandidate) {
        for (int candidate : candidates) {
            if (candidate != selectedCandidate) {
                seqVar.exclude(candidate);
            }
        }

        int start = seqVar.start();
        int insertableCount = seqVar.fillNode(nodes, INSERTABLE);
        for (int position = 0; position < insertableCount; position++) {
            int node = nodes[position];
            if (seqVar.hasInsert(start, node)) {
                seqVar.notBetween(start, node, selectedCandidate);
            }
        }
        setActive(false);
    }
}
