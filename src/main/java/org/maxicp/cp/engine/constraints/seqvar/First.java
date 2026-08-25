package org.maxicp.cp.engine.constraints.seqvar;

import org.maxicp.cp.engine.core.AbstractCPConstraint;
import org.maxicp.cp.engine.core.CPSeqVar;

public class First extends AbstractCPConstraint {

    private final CPSeqVar seqVar;
    private final int node;
    private boolean scheduled = false;

    public First(CPSeqVar seqVar, int node) {
        super(seqVar.getSolver());
        this.seqVar = seqVar;
        this.node = node;
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
