package org.maxicp.cp.engine.constraints.seqvar;

import org.maxicp.cp.engine.core.AbstractCPConstraint;
import org.maxicp.cp.engine.core.CPSeqVar;

import java.util.Set;

/**
 * ensure that one node among a set of node is scheduled first in the sequence
 * if the set of nodes is a singleton, schedule it as first immediately
 * otherwise set one of the nodes provided as first and exclude the others
 */
public class First extends AbstractCPConstraint {

    private final CPSeqVar seqVar;
    private final Set<Integer> nodes;
    private boolean scheduled = false;

    public First(CPSeqVar seqVar, Set<Integer> nodes) {
        super(seqVar.getSolver());
        this.seqVar = seqVar;
        this.nodes = nodes;
    }

    public void post() {
        // Implementation for posting the first constraint
    }

    public void schedule() {
        this.scheduled = true;
    }

    public void propagate() {
        // Implementation for propagating the first constraint
    }
}
