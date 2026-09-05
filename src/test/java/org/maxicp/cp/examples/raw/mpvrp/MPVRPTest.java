/*
 * MaxiCP is under MIT License
 * Copyright (c) 2024 UCLouvain
 */

package org.maxicp.cp.examples.raw.mpvrp;

import org.junit.jupiter.api.Test;
import org.maxicp.cp.examples.utils.MPVRPInstance;
import org.maxicp.cp.examples.utils.MPVRPInstance.DepotProductNode;
import org.maxicp.cp.examples.utils.MPVRPInstance.RequestNode;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

public class MPVRPTest {

    @Test
    public void testDepotVisitCopiesUseRequestAndStockBounds() {
        MPVRPInstance instance = new MPVRPInstance();
        instance.nProds = 2;
        instance.requests = new RequestNode[] {
                new RequestNode(1, 3, 0, 0, 0, 4),
                new RequestNode(2, 4, 0, 0, 0, 5),
                new RequestNode(3, 5, 0, 0, 0, 6),
                new RequestNode(4, 6, 0, 0, 1, 7)
        };
        instance.nRequests = instance.requests.length;
        instance.depotProducts = new DepotProductNode[] {
                new DepotProductNode(1, 0, 0, 0, 0, 10),
                new DepotProductNode(2, 1, 0, 0, 0, 2),
                new DepotProductNode(3, 2, 0, 0, 1, 10)
        };
        instance.nDepotProducts = instance.depotProducts.length;

        assertArrayEquals(new int[] {3, 2, 1}, MPVRP.depotVisitCopyCounts(instance));
    }
}
