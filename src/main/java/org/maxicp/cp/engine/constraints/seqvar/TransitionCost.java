package org.maxicp.cp.engine.constraints.seqvar;

import org.maxicp.cp.engine.core.AbstractCPConstraint;
import org.maxicp.cp.engine.core.CPIntVar;
import org.maxicp.cp.engine.core.CPSeqVar;
import org.maxicp.modeling.algebra.sequence.SeqStatus;
import org.maxicp.util.exception.InconsistencyException;

import java.util.Arrays;

import static org.maxicp.modeling.algebra.sequence.SeqStatus.*;

/** Computes the transition cost of a sequence and propagates safe cost bounds. */
public class TransitionCost extends AbstractCPConstraint {

    private static final int PENALTY_INF = 1_000_000;

    private final CPSeqVar seqVar;
    private final int[][] costs;
    private final CPIntVar total;
    private final int[] nodes;
    private final int[] candidates;
    private final int[] insertions;
    private final int[] minOutgoing;
    private final int[] minIncoming;
    private final int[][] shortestPath;
    private final boolean nonNegativeCosts;

    /**
     * @param seqVar sequence whose arcs are costed
     * @param costs directed transition-cost matrix; values at least
     *              {@value #PENALTY_INF} represent forbidden arcs
     * @param total total cost of the completed sequence
     */
    public TransitionCost(CPSeqVar seqVar, int[][] costs, CPIntVar total) {
        super(seqVar.getSolver());
        int n = seqVar.nNode();
        if (costs.length != n || Arrays.stream(costs).anyMatch(row -> row.length != n)) {
            throw new IllegalArgumentException("Transition-cost matrix must match the sequence size");
        }
        this.seqVar = seqVar;
        this.costs = costs;
        this.total = total;
        this.nodes = new int[n];
        this.candidates = new int[n];
        this.insertions = new int[n];
        this.minOutgoing = new int[n];
        this.minIncoming = new int[n];
        this.nonNegativeCosts = Arrays.stream(costs).flatMapToInt(Arrays::stream).allMatch(cost -> cost >= 0);
        this.shortestPath = nonNegativeCosts ? computeShortestPaths(costs) : null;
    }

    @Override
    public void post() {
        propagate();
        if (isActive()) {
            seqVar.propagateOnInsert(this);
            seqVar.propagateOnInsertRemoved(this);
            seqVar.propagateOnExclude(this);
            seqVar.propagateOnRequire(this);
            seqVar.propagateOnFix(this);
            total.propagateOnBoundChange(this);
        }
    }

    @Override
    public void propagate() {
        if (seqVar.isFixed()) {
            int n = seqVar.fillNode(nodes, SeqStatus.MEMBER_ORDERED);
            int value = 0;
            for (int i = 0; i < n - 1; i++) {
                int moveCost = costs[nodes[i]][nodes[i + 1]];
                if (moveCost >= PENALTY_INF) {
                    throw InconsistencyException.INCONSISTENCY;
                }
                value = Math.addExact(value, moveCost);
            }
            total.fix(value);
            setActive(false);
        } else if (nonNegativeCosts) {
            propagateNonNegativeCosts();
        }
    }

    /**
     * Combines minimum outgoing arcs, minimum incoming arcs, and shortest paths
     * between consecutive members. Their maximum remains a valid lower bound
     * even when the raw cost matrix violates the triangle inequality.
     */
    private void propagateNonNegativeCosts() {
        int nCandidates = seqVar.fillNode(candidates, NOT_EXCLUDED);
        Arrays.fill(minOutgoing, PENALTY_INF);
        Arrays.fill(minIncoming, PENALTY_INF);
        for (int i = 0; i < nCandidates; i++) {
            int from = candidates[i];
            for (int j = 0; j < nCandidates; j++) {
                int to = candidates[j];
                if (from == to || from == seqVar.end() || to == seqVar.start()
                        || costs[from][to] >= PENALTY_INF) {
                    continue;
                }
                minOutgoing[from] = Math.min(minOutgoing[from], costs[from][to]);
                minIncoming[to] = Math.min(minIncoming[to], costs[from][to]);
            }
        }

        long outgoingBound = 0;
        long incomingBound = 0;
        int nRequired = seqVar.fillNode(nodes, REQUIRED);
        for (int i = 0; i < nRequired; i++) {
            int node = nodes[i];
            if (node != seqVar.end()) {
                outgoingBound = addFinite(outgoingBound, minOutgoing[node]);
            }
            if (node != seqVar.start()) {
                incomingBound = addFinite(incomingBound, minIncoming[node]);
            }
        }

        long orderedBound = orderedMemberBound();
        enforceLowerBound(Math.max(orderedBound, Math.max(outgoingBound, incomingBound)));

        // MiniCP-inspired insertion filtering. Shortest paths are used instead
        // of raw member arcs because later insertions may replace those arcs.
        int nInsertable = seqVar.fillNode(nodes, INSERTABLE);
        for (int i = 0; i < nInsertable; i++) {
            int node = nodes[i];
            long conditionalOutgoing = outgoingBound;
            long conditionalIncoming = incomingBound;
            if (seqVar.isNode(node, POSSIBLE)) {
                conditionalOutgoing = addConditional(conditionalOutgoing, minOutgoing[node]);
                conditionalIncoming = addConditional(conditionalIncoming, minIncoming[node]);
            }
            int nInsertions = seqVar.fillInsert(node, insertions);
            for (int j = 0; j < nInsertions; j++) {
                int pred = insertions[j];
                int succ = seqVar.memberAfter(pred);
                long conditionalOrder = orderedBound - shortestPath[pred][succ];
                conditionalOrder = addConditional(conditionalOrder, shortestPath[pred][node]);
                conditionalOrder = addConditional(conditionalOrder, shortestPath[node][succ]);
                long conditionalBound = Math.max(conditionalOrder,
                        Math.max(conditionalOutgoing, conditionalIncoming));
                if (conditionalBound > total.max()) {
                    seqVar.notBetween(pred, node, succ);
                }
            }
        }
    }

    private long orderedMemberBound() {
        int nMembers = seqVar.fillNode(nodes, MEMBER_ORDERED);
        long bound = 0;
        for (int i = 0; i < nMembers - 1; i++) {
            bound = addFinite(bound, shortestPath[nodes[i]][nodes[i + 1]]);
        }
        return bound;
    }

    private void enforceLowerBound(long lowerBound) {
        if (lowerBound > total.max() || lowerBound > Integer.MAX_VALUE) {
            throw InconsistencyException.INCONSISTENCY;
        }
        total.removeBelow((int) lowerBound);
    }

    private static long addFinite(long sum, int value) {
        if (value >= PENALTY_INF) {
            throw InconsistencyException.INCONSISTENCY;
        }
        return sum + value;
    }

    private static long addConditional(long sum, int value) {
        return value >= PENALTY_INF ? Long.MAX_VALUE / 4 : sum + value;
    }

    private static int[][] computeShortestPaths(int[][] costs) {
        int n = costs.length;
        int[][] shortest = new int[n][n];
        for (int from = 0; from < n; from++) {
            for (int to = 0; to < n; to++) {
                shortest[from][to] = costs[from][to] >= PENALTY_INF ? PENALTY_INF : costs[from][to];
            }
            shortest[from][from] = 0;
        }
        for (int intermediate = 0; intermediate < n; intermediate++) {
            for (int from = 0; from < n; from++) {
                if (shortest[from][intermediate] >= PENALTY_INF) {
                    continue;
                }
                for (int to = 0; to < n; to++) {
                    long through = (long) shortest[from][intermediate] + shortest[intermediate][to];
                    if (through < shortest[from][to]) {
                        shortest[from][to] = (int) through;
                    }
                }
            }
        }
        return shortest;
    }
}
