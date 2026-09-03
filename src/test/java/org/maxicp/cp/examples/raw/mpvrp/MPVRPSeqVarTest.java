/*
 * MaxiCP is under MIT License
 * Copyright (c)  2023 UCLouvain
 */

package org.maxicp.cp.examples.raw.mpvrp;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.maxicp.cp.CPSolverTest;
import org.maxicp.cp.engine.core.CPSeqVar;
import org.maxicp.cp.engine.core.CPSolver;
import org.maxicp.cp.examples.utils.MPVRPInstance;
import org.maxicp.cp.examples.utils.MPVRPInstance.DepotProductNode;
import org.maxicp.cp.examples.utils.MPVRPInstance.GarageNode;
import org.maxicp.cp.examples.utils.MPVRPInstance.Request;
import org.maxicp.cp.examples.utils.MPVRPInstance.RequestNode;
import org.maxicp.cp.examples.utils.MPVRPInstance.SolutionData;
import org.maxicp.cp.examples.utils.MPVRPInstance.Trip;
import org.maxicp.cp.examples.utils.MPVRPInstance.Vehicule;

import java.io.IOException;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.maxicp.cp.CPFactory.makeSeqVar;
import static org.maxicp.cp.examples.utils.MPVRPInstance.captureSolution;

public class MPVRPSeqVarTest extends CPSolverTest {

    @Test
    public void testDemandCanBeSplitAcrossSeveralDepots() {
        MPVRPInstance instance = instance(
                new int[] {6, 6},
                new RequestNode[] {new RequestNode(1, 3, 5, 0, 0, 10)});

        List<Trip> trips = MPVRPSeqVar.buildTrips(instance, new int[] {6, 6});
        List<Request> deliveries = trips.stream().flatMap(trip -> trip.deliveries().stream()).toList();

        assertEquals(2, deliveries.size());
        assertEquals(10, deliveries.stream().mapToInt(Request::quantity).sum());
        assertEquals(2, deliveries.stream().map(Request::depot).distinct().count());
        assertTrue(deliveries.stream().allMatch(delivery -> delivery.quantity() <= 6));
    }

    @Test
    public void testCompatibleDeliveriesArePackedInOneTrip() {
        MPVRPInstance instance = instance(
                new int[] {20},
                new RequestNode[] {
                        new RequestNode(1, 2, 2, 0, 0, 4),
                        new RequestNode(2, 3, 3, 0, 0, 3)
                });

        List<Trip> trips = MPVRPSeqVar.buildTrips(instance, new int[] {10, 10});

        assertEquals(1, trips.size());
        assertEquals(2, trips.getFirst().deliveries().size());
        assertEquals(7, trips.getFirst().quantity());
    }

    @Test
    public void testHeterogeneousFleetUsesLargestCapacityWhenPackingTrips() {
        MPVRPInstance instance = instance(
                new int[] {20},
                new RequestNode[] {
                        new RequestNode(1, 2, 2, 0, 0, 6),
                        new RequestNode(2, 3, 3, 0, 0, 4)
                });

        List<Trip> trips = MPVRPSeqVar.buildTrips(instance, new int[] {5, 10});

        assertEquals(1, trips.size());
        assertEquals(10, trips.getFirst().quantity());
    }

    @Test
    public void testStockPreservingFallbackAvoidsGreedyFalseInfeasibility() {
        MPVRPInstance instance = instance(
                new int[] {1, 1, 4},
                new RequestNode[] {
                        new RequestNode(1, 4, 2, 0, 0, 3),
                        new RequestNode(2, 5, 3, 0, 0, 3)
                });

        List<Trip> trips = MPVRPSeqVar.buildTrips(instance, new int[] {3, 3});
        int[] delivered = new int[2];
        for (Trip trip : trips) {
            for (Request delivery : trip.deliveries()) {
                delivered[delivery.logicalRequest()] += delivery.quantity();
            }
        }

        assertArrayEquals(new int[] {3, 3}, delivered);
    }

    @Test
    public void testOneVehicleCannotVisitSameRequestTwice() {
        MPVRPInstance instance = instance(
                new int[] {20},
                new RequestNode[] {new RequestNode(1, 2, 2, 0, 0, 9)});

        List<Trip> trips = MPVRPSeqVar.buildTrips(instance, new int[] {5, 5});

        assertEquals(2, trips.size());
        assertTrue(trips.stream().allMatch(trip -> trip.deliveries().size() == 1));
    }

    @Test
    public void testTwoOptImprovesEndpointAwareTripScore() {
        int[][] coordinates = {
                {0, 0},  // depot
                {0, 0},  // garage
                {8, 2},
                {0, 5},
                {-7, 5},
                {-2, 3},
                {7, 4}
        };
        RequestNode[] requests = new RequestNode[5];
        for (int request = 0; request < requests.length; request++) {
            int globalId = request + 2;
            requests[request] = new RequestNode(request + 1, globalId,
                    coordinates[globalId][0], coordinates[globalId][1], 0, 1);
        }
        MPVRPInstance instance = instance(new int[] {10}, requests);
        for (int from = 0; from < coordinates.length; from++) {
            for (int to = 0; to < coordinates.length; to++) {
                instance.distMatrix[from][to] = (int) Math.round(Math.hypot(
                        coordinates[from][0] - coordinates[to][0],
                        coordinates[from][1] - coordinates[to][1]));
            }
        }

        DepotProductNode depot = instance.depotProducts[0];
        List<Request> deliveries = java.util.stream.IntStream.range(0, requests.length)
                .mapToObj(i -> new Request(i, requests[i], depot, 1))
                .toList();
        List<Request> nearest = MPVRPSeqVar.orderByNearestStation(instance, depot, deliveries);
        List<Request> twoOpt = MPVRPSeqVar.orderBy2Opt(instance, depot, deliveries);

        assertEquals(new HashSet<>(deliveries), new HashSet<>(twoOpt));
        assertTrue(MPVRPSeqVar.tripConstructionScore(instance, depot, twoOpt)
                < MPVRPSeqVar.tripConstructionScore(instance, depot, nearest));
    }

    @Test
    public void testInsufficientFleetOrStockIsRejected() {
        MPVRPInstance fleetLimited = instance(
                new int[] {20},
                new RequestNode[] {new RequestNode(1, 2, 2, 0, 0, 11)});
        assertThrows(IllegalArgumentException.class,
                () -> MPVRPSeqVar.buildTrips(fleetLimited, new int[] {5, 5}));

        MPVRPInstance stockLimited = instance(
                new int[] {4, 4},
                new RequestNode[] {new RequestNode(1, 3, 3, 0, 0, 9)});
        assertThrows(IllegalArgumentException.class,
                () -> MPVRPSeqVar.buildTrips(stockLimited, new int[] {10, 10}));
    }

    @Test
    public void testInstance011TripsRespectDemandStockProductAndCapacity() throws IOException {
        MPVRPInstance instance = MPVRPInstance.read("data/MPVRP_CC/MPVRP_011_s47_d3_p6.dat");
        int[] capacities = Arrays.stream(instance.vehicules)
                .mapToInt(Vehicule::capacity).toArray();
        List<Trip> trips = MPVRPSeqVar.buildTrips(instance, capacities);
        int[] delivered = new int[instance.nRequests];
        int[] usedStock = new int[instance.nDepotProducts];
        int largestCapacity = Arrays.stream(capacities).max().orElseThrow();

        for (Trip trip : trips) {
            assertTrue(trip.quantity() <= largestCapacity);
            for (Request delivery : trip.deliveries()) {
                assertEquals(trip.depot().product(), delivery.station().product());
                delivered[delivery.logicalRequest()] += delivery.quantity();
                for (int depot = 0; depot < instance.nDepotProducts; depot++) {
                    if (instance.depotProducts[depot].equals(delivery.depot())) {
                        usedStock[depot] += delivery.quantity();
                        break;
                    }
                }
            }
        }

        for (int request = 0; request < instance.nRequests; request++) {
            assertEquals(instance.requests[request].demand(), delivered[request]);
        }
        for (int depot = 0; depot < instance.nDepotProducts; depot++) {
            assertTrue(usedStock[depot] <= instance.depotProducts[depot].stock());
        }
    }

    @ParameterizedTest
    @MethodSource("getSolver")
    public void testCaptureSolutionKeepsSeveralStationsInSameTrip(CPSolver cp) {
        MPVRPInstance instance = instance(
                new int[] {20},
                new RequestNode[] {
                        new RequestNode(1, 2, 2, 0, 0, 4),
                        new RequestNode(2, 3, 3, 0, 0, 3)
                });
        DepotProductNode depot = instance.depotProducts[0];
        Trip trip = new Trip(depot, List.of(
                new Request(0, instance.requests[0], depot, 4),
                new Request(1, instance.requests[1], depot, 3)));
        CPSeqVar route = makeSeqVar(cp, 3, 1, 2);
        route.insert(1, 0);

        SolutionData solution = captureSolution(instance, List.of(trip), new CPSeqVar[] {route},
                new int[] {0}, 3);

        assertEquals(1, solution.routes().size());
        assertEquals("1: 1 - 1 [7] - 1 (4) - 2 (3) - 1", solution.routes().getFirst().visitLine());
        assertEquals(7, trip.quantity());
        assertEquals(5.0, solution.transitionCost());
        assertEquals(6.0, solution.distance());

        int[][] cost = MPVRPSeqVar.buildTripCosts(instance, List.of(trip), new int[] {0}, 1);
        assertEquals(9, cost[1][0]);
        assertEquals(2, cost[0][2]);
        assertEquals(solution.transitionCost() + solution.distance(), cost[1][0] + cost[0][2]);
    }

    private static MPVRPInstance instance(int[] stocks, RequestNode[] requests) {
        MPVRPInstance instance = new MPVRPInstance();
        instance.nProds = 1;
        instance.nRequests = requests.length;
        instance.nVehicules = 2;
        instance.depotProducts = new DepotProductNode[stocks.length];
        for (int depot = 0; depot < stocks.length; depot++) {
            instance.depotProducts[depot] = new DepotProductNode(depot + 1, depot, depot, 0, 0, stocks[depot]);
        }
        instance.nDepotProducts = stocks.length;
        instance.requests = requests;

        GarageNode garage = new GarageNode(1, stocks.length, 0, 0);
        instance.garages = new GarageNode[] {garage};
        instance.vehicules = new Vehicule[] {
                new Vehicule(1, 10, garage, 0),
                new Vehicule(2, 10, garage, 0)
        };
        instance.changeoverCost = new int[][] {{5}};

        int nodeCount = stocks.length + 1 + requests.length;
        instance.distMatrix = new int[nodeCount][nodeCount];
        for (int from = 0; from < nodeCount; from++) {
            for (int to = 0; to < nodeCount; to++) {
                instance.distMatrix[from][to] = Math.abs(from - to);
            }
        }
        return instance;
    }

}
