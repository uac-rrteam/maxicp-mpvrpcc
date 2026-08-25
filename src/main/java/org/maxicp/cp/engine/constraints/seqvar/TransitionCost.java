package org.maxicp.cp.engine.constraints.seqvar;

import org.maxicp.cp.engine.core.AbstractCPConstraint;
import org.maxicp.cp.engine.core.CPIntVar;
import org.maxicp.cp.engine.core.CPSeqVar;
import org.maxicp.modeling.algebra.sequence.SeqStatus;
import org.maxicp.util.exception.InconsistencyException;

/**
 * compute the total cost of a sequence given a cost matrix
 */
public class TransitionCost extends AbstractCPConstraint {

    private final CPSeqVar seqVar;
    private final int[] nodes;
    private final int[][] costs;
    private final CPIntVar total;
    private static final int PENALTY_INF = 1_000_000;

    /**
     * compute the total cost of a sequence of nodes given a cost matrix
     * @param seqVar sequence on which the constraint is applied
     * @param costs cost change from one node to another. Can be negative or positive
     * @param total total cost of the sequence
     */
    public TransitionCost(CPSeqVar seqVar, int[][] costs, CPIntVar total) {
        super(seqVar.getSolver());
        this.seqVar = seqVar;
        this.costs = costs;
        this.total = total;
        this.nodes = new int[seqVar.nNode()];
    }

    @Override
    public void post() {
        propagate();
        if (isActive()) {
            seqVar.propagateOnInsert(this);
            seqVar.propagateOnFix(this);
        }
    }

    @Override
    public void propagate() {
        if (!seqVar.isFixed()) {
            return;
        }
        // compute the current total cost of the sequence
        int n = seqVar.fillNode(nodes, SeqStatus.MEMBER_ORDERED);
        int value = 0;
        for (int i = 0; i < n -1; i++){
            int moveCost = costs[nodes[i]][nodes[i+1]];
            if (moveCost >= PENALTY_INF){
                throw InconsistencyException.INCONSISTENCY;
            }
            value += moveCost;
        }
        total.fix(value);
        setActive(false);
    }
}
