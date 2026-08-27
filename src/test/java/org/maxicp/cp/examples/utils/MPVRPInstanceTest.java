/*
 * MaxiCP is under MIT License
 * Copyright (c)  2023 UCLouvain
 */

package org.maxicp.cp.examples.utils;

import org.junit.jupiter.api.Test;
import org.maxicp.cp.examples.utils.MPVRPInstance.DepotProductNode;
import org.maxicp.cp.examples.utils.MPVRPInstance.GarageNode;
import org.maxicp.cp.examples.utils.MPVRPInstance.RequestNode;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.maxicp.cp.examples.utils.MPVRPInstance.isAllowedTransition;

public class MPVRPInstanceTest {

    private final GarageNode homeGarage = new GarageNode(1, 0, 0, 0);
    private final GarageNode otherGarage = new GarageNode(2, 1, 10, 10);
    private final DepotProductNode productZeroDepot = new DepotProductNode(1, 2, 1, 1, 0, 100);
    private final DepotProductNode productOneDepot = new DepotProductNode(1, 3, 1, 1, 1, 100);
    private final RequestNode productZeroStation = new RequestNode(1, 4, 2, 2, 0, 10);
    private final RequestNode otherProductZeroStation = new RequestNode(2, 5, 3, 3, 0, 10);
    private final RequestNode productOneStation = new RequestNode(3, 6, 4, 4, 1, 10);

    @Test
    public void testTransitionTypesAndProducts() {
        assertTrue(isAllowedTransition(homeGarage, productZeroDepot));
        assertFalse(isAllowedTransition(homeGarage, productZeroStation));
        assertFalse(isAllowedTransition(homeGarage, otherGarage));

        assertTrue(isAllowedTransition(productZeroDepot, productZeroStation));
        assertFalse(isAllowedTransition(productZeroDepot, productOneStation));
        assertFalse(isAllowedTransition(productZeroDepot, productOneDepot));

        assertTrue(isAllowedTransition(productZeroStation, productOneDepot));
        assertTrue(isAllowedTransition(productZeroStation, otherProductZeroStation));
        assertFalse(isAllowedTransition(productZeroStation, productOneStation));
        assertTrue(isAllowedTransition(productZeroStation, homeGarage));
    }

    @Test
    public void testOnlyHomeGarageIsAllowedForVehicleTransitions() {
        assertTrue(isAllowedTransition(homeGarage, productZeroDepot, homeGarage));
        assertFalse(isAllowedTransition(otherGarage, productZeroDepot, homeGarage));

        assertTrue(isAllowedTransition(productZeroStation, homeGarage, homeGarage));
        assertFalse(isAllowedTransition(productZeroStation, otherGarage, homeGarage));
    }
}
