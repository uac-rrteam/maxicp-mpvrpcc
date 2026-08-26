/*
 * MaxiCP is under MIT License
 * Copyright (c)  2023 UCLouvain
 */

package org.maxicp.cp.engine.constraints.seqvar;

import org.maxicp.cp.engine.core.CPSeqVar;

import java.util.Set;

import static org.maxicp.cp.CPFactory.flip;

/**
 * Ensures that exactly one node among a set of candidates is visited last.
 * The selected candidate is inserted immediately before the end node and all
 * other candidates are excluded.
 */
public class Last extends First {

    /**
     * Creates a last-node constraint.
     *
     * @param seqVar sequence whose last visit is constrained
     * @param candidates candidates for the last visited node
     */
    public Last(CPSeqVar seqVar, int... candidates) {
        super(flip(seqVar), candidates);
    }

    /**
     * Creates a last-node constraint.
     *
     * @param seqVar sequence whose last visit is constrained
     * @param candidates candidates for the last visited node
     */
    public Last(CPSeqVar seqVar, Set<Integer> candidates) {
        super(flip(seqVar), candidates);
    }
}
