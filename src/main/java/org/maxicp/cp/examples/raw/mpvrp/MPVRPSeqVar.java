/*
 * MaxiCP is under MIT License
 * Copyright (c) 2024 UCLouvain
 *
 */

package org.maxicp.cp.examples.raw.mpvrp;

import org.maxicp.cp.engine.constraints.Sum;
import org.maxicp.cp.engine.constraints.seqvar.TransitionCost;
import org.maxicp.cp.engine.core.CPIntVar;
import org.maxicp.cp.engine.core.CPSeqVar;
import org.maxicp.cp.engine.core.CPSolver;
import org.maxicp.cp.examples.utils.MPVRPInstance;
import org.maxicp.cp.examples.utils.MPVRPInstance.Request;
import org.maxicp.cp.examples.utils.MPVRPInstance.SolutionData;
import org.maxicp.cp.examples.utils.MPVRPInstance.Trip;
import org.maxicp.search.DFSearch;
import org.maxicp.search.Objective;
import org.maxicp.search.SearchStatistics;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import static org.maxicp.cp.CPFactory.*;
import static org.maxicp.cp.examples.utils.MPVRPInstance.buildSolutionText;
import static org.maxicp.cp.examples.utils.MPVRPInstance.captureSolution;
import static org.maxicp.cp.examples.utils.MPVRPInstance.defaultOutputPath;
import static org.maxicp.cp.examples.utils.MPVRPInstance.writeSolution;
import static org.maxicp.modeling.algebra.sequence.SeqStatus.INSERTABLE;
import static org.maxicp.search.Searches.*;

/**
 * Multi-Product Vehicle Routing Problem with Changeover Cost
 * (<a href="https://ifri-ai-classes.github.io/MPVRP-CC">MPVRP-CC</a>).
 *
 * @author Rosas Behoundja
 */
public class MPVRPSeqVar {

    /** Cost assigned to transitions that are not allowed. */
    private static final int PENALTY_INF = 1_000_000;

    private enum AllocationStrategy {
        MAX_QUANTITY,
        PRESERVE_LARGE_STOCKS
    }

    /** Delivery together with the vehicle slot used to construct a feasible packing witness. */
    private record AllocatedDelivery(Request request, int vehicle) {}

    /** Heuristic used to order the deliveries inside each trip. */
    public enum TripOrdering {
        NEAREST_NEIGHBOR,
        TWO_OPT;

        static TripOrdering parse(String value) {
            return switch (value.toLowerCase(Locale.ROOT).replace("_", "").replace("-", "")) {
                case "nearest", "nearestneighbor", "nn" -> NEAREST_NEIGHBOR;
                case "2opt", "twoopt" -> TWO_OPT;
                default -> throw new IllegalArgumentException("Unknown trip ordering '" + value
                        + "'. Expected nearest-neighbor or 2-opt.");
            };
        }
    }

    public static void main(String[] args) throws Exception {
        String inputFile = args.length > 0 ? args[0] : "data/MPVRP_CC/MPVRP_002_s3_d6_p3.dat";
        String outputFile = args.length > 1 ? args[1] : null;
        TripOrdering tripOrdering = args.length > 2
                ? TripOrdering.parse(args[2])
                : TripOrdering.TWO_OPT;

        MPVRPInstance instance = MPVRPInstance.read(inputFile);
        int vehicleCount = instance.nVehicules;
        int[] initialProducts = new int[vehicleCount];
        int[] capacities = new int[vehicleCount];
        for (int vehicle = 0; vehicle < vehicleCount; vehicle++) {
            initialProducts[vehicle] = instance.vehicules[vehicle].initProd();
            capacities[vehicle] = instance.vehicules[vehicle].capacity();
        }

        // A trip starts at a depot and contains compatible deliveries that fit in one vehicle.
        List<Trip> trips = buildTrips(instance, capacities, tripOrdering);
        if (trips.isEmpty()) {
            System.out.println("No positive station demand in the instance.");
            return;
        }
        int tripCount = trips.size();

        // Each route contains one node per trip, plus a start and an end node per vehicle.
        int nodeCount = tripCount + 2 * vehicleCount;
        int[][] cost = buildTripCosts(instance, trips, initialProducts, vehicleCount);
        int maxArc = Arrays.stream(cost).flatMapToInt(Arrays::stream)
                .filter(value -> value != PENALTY_INF).max().orElse(0);
        int maxRouteCost = (int) Math.min(Integer.MAX_VALUE, (long) maxArc * nodeCount);

        // ===================== variables =====================

        CPSolver cp = makeSolver();
        CPSeqVar[] routes = new CPSeqVar[vehicleCount];
        CPIntVar[] routeCosts = new CPIntVar[vehicleCount];

        for (int vehicle = 0; vehicle < vehicleCount; vehicle++) {
            int start = tripCount + vehicle;
            int end = tripCount + vehicleCount + vehicle;
            routes[vehicle] = makeSeqVar(cp, nodeCount, start, end);

            // A route may only contain the start and end nodes of its own vehicle.
            for (int other = 0; other < vehicleCount; other++) {
                if (other != vehicle) {
                    cp.post(exclude(routes[vehicle], tripCount + other));
                    cp.post(exclude(routes[vehicle], tripCount + vehicleCount + other));
                }
            }
            routeCosts[vehicle] = makeIntVar(cp, 0, maxRouteCost);
        }
        CPIntVar totalCost = sum(routeCosts);

        // ===================== constraints =====================

        for (int vehicle = 0; vehicle < vehicleCount; vehicle++) {
            // Compute the cost of every transition along the vehicle route.
            cp.post(new TransitionCost(routes[vehicle], cost, routeCosts[vehicle]));
        }
        // Every trip is assigned to exactly one vehicle with sufficient capacity.
        for (int trip = 0; trip < tripCount; trip++) {
            CPIntVar[] tripVisits = new CPIntVar[vehicleCount];
            for (int vehicle = 0; vehicle < vehicleCount; vehicle++) {
                tripVisits[vehicle] = routes[vehicle].getNodeVar(trip).isRequired();
                if (trips.get(trip).quantity() > capacities[vehicle]) {
                    cp.post(exclude(routes[vehicle], trip));
                }
            }
            cp.post(new Sum(tripVisits, 1));
        }

        // Two parts of the same station-product request cannot use the same vehicle.
        for (int logicalRequest = 0; logicalRequest < instance.nRequests; logicalRequest++) {
            int requestId = logicalRequest;
            for (int vehicle = 0; vehicle < vehicleCount; vehicle++) {
                List<CPIntVar> visits = new ArrayList<>();
                for (int trip = 0; trip < tripCount; trip++) {
                    if (trips.get(trip).deliveries().stream()
                            .anyMatch(delivery -> delivery.logicalRequest() == requestId)) {
                        visits.add(routes[vehicle].getNodeVar(trip).isRequired());
                    }
                }
                if (visits.size() > 1) {
                    cp.post(le(sum(visits.toArray(new CPIntVar[0])), 1));
                }
            }
        }

        // ===================== search =====================

        Integer[] tripIds = IntStream.range(0, tripCount).boxed().toArray(Integer[]::new);
        int[] insertions = new int[nodeCount];
        CPIntVar impossible = makeIntVar(cp, 0, 0);

        Supplier<Runnable[]> branching = () -> {
            if (Arrays.stream(routes).allMatch(CPSeqVar::isFixed)) {
                return EMPTY; // All routes are fixed: a solution has been found.
            }

            // Select an unplanned trip with the fewest insertion positions across all routes.
            Integer trip = selectMin(tripIds,
                    candidate -> Arrays.stream(routes).anyMatch(route -> route.isNode(candidate, INSERTABLE)),
                    candidate -> Arrays.stream(routes).mapToInt(route -> route.nInsert(candidate)).sum());

            if (trip == null) {
                return branch(() -> cp.post(eq(impossible, 1)));
            }

            // Among the remaining positions, prefer the insertion with the smallest detour.
            int bestDetour = Integer.MAX_VALUE;
            int bestPred = -1;
            int bestSucc = -1;
            CPSeqVar bestRoute = null;

            for (CPSeqVar currentRoute : routes) {
                int nInsert = currentRoute.fillInsert(trip, insertions);
                for (int i = 0; i < nInsert; i++) {
                    int pred = insertions[i];
                    int succ = currentRoute.memberAfter(pred);
                    int detour = cost[pred][trip] + cost[trip][succ] - cost[pred][succ];
                    if (detour < bestDetour) {
                        bestDetour = detour;
                        bestPred = pred;
                        bestSucc = succ;
                        bestRoute = currentRoute;
                    }
                }
            }

            if (bestRoute == null) {
                return branch(() -> cp.post(eq(impossible, 1)));
            }

            // Either use the best insertion or remove it from the current search branch.
            CPSeqVar route = bestRoute;
            int pred = bestPred;
            int succ = bestSucc;
            int tripId = trip;
            return branch(
                    () -> cp.post(insert(route, pred, tripId)),
                    () -> cp.post(notBetween(route, pred, tripId, succ)));
        };

        // ===================== solve =====================

        DFSearch search = makeDfs(cp, branching);
        Objective objective = cp.minimize(totalCost);
        SolutionData[] bestSolution = new SolutionData[1];
        int[] bestCost = {Integer.MAX_VALUE};
        long startTime = System.currentTimeMillis();

        search.onSolution(() -> {
            int solutionCost = totalCost.min();
            if (solutionCost < bestCost[0]) {
                bestCost[0] = solutionCost;
                bestSolution[0] = captureSolution(instance, trips, routes, initialProducts, nodeCount);
                System.out.println("Solution found! Objective: " + solutionCost);
                long elapsedMs = System.currentTimeMillis() - startTime;
                if (bestSolution[0] != null) {
                    String text = buildSolutionText(bestSolution[0], elapsedMs / 1000.0);
                    Path outputPath = Path.of(outputFile != null ? outputFile : defaultOutputPath(inputFile));
                    try {
                        writeSolution(outputPath, text);
                    } catch (IOException e) {
                        throw new RuntimeException(e);
                    }
                    System.out.println("Solution written to " + outputPath.toAbsolutePath());
                }
            }
        });

        System.out.println("Starting search...");
        System.out.println("Trip ordering: " + tripOrdering);
        SearchStatistics statistics = search.optimize(objective);

        System.out.println(statistics);
    }

    /**
     * Splits station demands into deliveries and groups compatible deliveries into trips.
     *
     * @param instance problem instance containing stocks and station demands
     * @param capacities capacity of each available vehicle
     * @return trips to assign to the vehicle routes
     */
    static List<Trip> buildTrips(MPVRPInstance instance, int[] capacities) {
        return buildTrips(instance, capacities, TripOrdering.TWO_OPT);
    }

    /**
     * Splits station demands into deliveries and groups compatible deliveries into trips.
     *
     * @param instance problem instance containing stocks and station demands
     * @param capacities capacity of each available vehicle
     * @param tripOrdering heuristic used to order deliveries within each trip
     * @return trips to assign to the vehicle routes
     */
    static List<Trip> buildTrips(MPVRPInstance instance, int[] capacities, TripOrdering tripOrdering) {
        if (capacities.length == 0 || Arrays.stream(capacities).anyMatch(capacity -> capacity <= 0)) {
            throw new IllegalArgumentException("Vehicle capacities must be positive");
        }

        Integer[] vehicleOrder = IntStream.range(0, capacities.length).boxed()
                .sorted(Comparator.comparingInt(vehicle -> capacities[vehicle]))
                .toArray(Integer[]::new);

        Integer[] requestOrder = IntStream.range(0, instance.nRequests).boxed()
                .sorted(Comparator.comparingInt((Integer index) -> instance.requests[index].demand()).reversed())
                .toArray(Integer[]::new);
        List<AllocatedDelivery> deliveries = allocateDeliveries(instance, capacities, vehicleOrder,
                requestOrder, AllocationStrategy.MAX_QUANTITY);
        if (deliveries == null) {
            // The quantity-maximizing rule can fragment the remaining depot stocks
            // even when a feasible allocation exists. Retry while preserving large
            // stocks for later requests.
            deliveries = allocateDeliveries(instance, capacities, vehicleOrder,
                    requestOrder, AllocationStrategy.PRESERVE_LARGE_STOCKS);
        }
        if (deliveries == null) {
            throw new IllegalArgumentException("The trip-construction heuristic could not allocate all demands; "
                    + "this does not prove that the MPVRP-CC instance is infeasible");
        }

        deliveries.sort(Comparator.comparingInt((AllocatedDelivery delivery) -> delivery.request().quantity())
                .reversed());
        List<TripBin> bins = new ArrayList<>();

        // Pack only deliveries sharing the same construction vehicle. This retains
        // heterogeneous capacities and provides a feasible assignment witness for
        // all split-delivery distinct-vehicle constraints.
        for (AllocatedDelivery allocated : deliveries) {
            Request delivery = allocated.request();
            int constructionVehicle = allocated.vehicle();
            int tripCapacity = capacities[constructionVehicle];
            TripBin selectedBin = null;
            int smallestRemainingCapacity = Integer.MAX_VALUE;
            for (TripBin bin : bins) {
                int remainingCapacity = tripCapacity - bin.quantity - delivery.quantity();
                if (bin.canPack(delivery, constructionVehicle, tripCapacity)
                        && remainingCapacity < smallestRemainingCapacity) {
                    selectedBin = bin;
                    smallestRemainingCapacity = remainingCapacity;
                }
            }
            if (selectedBin == null) {
                selectedBin = new TripBin(delivery.depot(), constructionVehicle);
                bins.add(selectedBin);
            }
            selectedBin.add(delivery);
        }

        // Order the deliveries inside each trip with the selected construction heuristic.
        List<Trip> trips = new ArrayList<>(bins.size());
        for (TripBin bin : bins) {
            List<Request> orderedDeliveries = switch (tripOrdering) {
                case NEAREST_NEIGHBOR -> orderByNearestStation(instance, bin.depot, bin.deliveries);
                case TWO_OPT -> orderBy2Opt(instance, bin.depot, bin.deliveries);
            };
            trips.add(new Trip(bin.depot, orderedDeliveries));
        }
        return trips;
    }

    private static List<AllocatedDelivery> allocateDeliveries(MPVRPInstance instance, int[] capacities,
                                                              Integer[] vehicleOrder, Integer[] requestOrder,
                                                              AllocationStrategy strategy) {
        int[] remainingStock = Arrays.stream(instance.depotProducts)
                .mapToInt(MPVRPInstance.DepotProductNode::stock)
                .toArray();
        List<AllocatedDelivery> deliveries = new ArrayList<>();

        for (int requestIndex : requestOrder) {
            MPVRPInstance.RequestNode request = instance.requests[requestIndex];
            int remainingDemand = request.demand();
            for (int position = vehicleOrder.length - 1; position >= 0 && remainingDemand > 0; position--) {
                int vehicle = vehicleOrder[position];
                int vehicleCapacity = capacities[vehicle];
                long remainingVehicleCapacity = 0;
                for (int other = 0; other < position; other++) {
                    remainingVehicleCapacity += capacities[vehicleOrder[other]];
                }
                int minimumQuantity = (int) Math.max(1L, remainingDemand - remainingVehicleCapacity);
                int selectedDepot = -1;
                int selectedQuantity = 0;
                int selectedDistance = Integer.MAX_VALUE;
                int selectedStock = Integer.MAX_VALUE;
                for (int depotIndex = 0; depotIndex < instance.nDepotProducts; depotIndex++) {
                    MPVRPInstance.DepotProductNode depot = instance.depotProducts[depotIndex];
                    if (depot.product() != request.product() || remainingStock[depotIndex] == 0) {
                        continue;
                    }
                    int quantity = Math.min(remainingDemand, Math.min(vehicleCapacity, remainingStock[depotIndex]));
                    if (quantity < minimumQuantity) {
                        continue;
                    }
                    int distance = instance.distMatrix[depot.globalId()][request.globalId()];
                    boolean better = strategy == AllocationStrategy.MAX_QUANTITY
                            ? quantity > selectedQuantity || quantity == selectedQuantity && distance < selectedDistance
                            : remainingStock[depotIndex] < selectedStock
                                    || remainingStock[depotIndex] == selectedStock && distance < selectedDistance;
                    if (better) {
                        selectedDepot = depotIndex;
                        selectedQuantity = quantity;
                        selectedDistance = distance;
                        selectedStock = remainingStock[depotIndex];
                    }
                }

                if (selectedDepot == -1) {
                    break;
                }
                MPVRPInstance.DepotProductNode depot = instance.depotProducts[selectedDepot];
                remainingStock[selectedDepot] -= selectedQuantity;
                remainingDemand -= selectedQuantity;
                deliveries.add(new AllocatedDelivery(
                        new Request(requestIndex, request, depot, selectedQuantity), vehicle));
            }

            if (remainingDemand > 0) {
                return null;
            }
        }
        return deliveries;
    }

    /**
     * Orders a trip's deliveries with a nearest-neighbor heuristic.
     *
     * @param instance problem instance providing the distance matrix
     * @param depot starting depot of the trip
     * @param deliveries deliveries to order
     * @return a new list ordered from the depot to the nearest remaining station
     */
    static List<Request> orderByNearestStation(MPVRPInstance instance,
                                               MPVRPInstance.DepotProductNode depot,
                                               List<Request> deliveries) {
        List<Request> remaining = new ArrayList<>(deliveries);
        List<Request> ordered = new ArrayList<>(deliveries.size());
        int currentGlobalId = depot.globalId();
        while (!remaining.isEmpty()) {
            int nearest = 0;
            int nearestDistance = Integer.MAX_VALUE;
            for (int i = 0; i < remaining.size(); i++) {
                int distance = instance.distMatrix[currentGlobalId][remaining.get(i).station().globalId()];
                if (distance < nearestDistance) {
                    nearest = i;
                    nearestDistance = distance;
                }
            }
            Request delivery = remaining.remove(nearest);
            ordered.add(delivery);
            currentGlobalId = delivery.station().globalId();
        }
        return ordered;
    }

    /**
     * Improves the nearest-neighbor ordering by reversing station subsequences.
     * The depot remains the fixed start. The construction score also includes the
     * cheapest possible continuation from the last station to any depot or garage,
     * avoiding an order that is locally short but leaves a very poor trip endpoint.
     * Full path lengths are evaluated, so this also works for asymmetric matrices.
     *
     * @param instance problem instance providing the distance matrix
     * @param depot starting depot of the trip
     * @param deliveries deliveries to order
     * @return a nearest-neighbor ordering improved to a 2-opt local optimum
     */
    static List<Request> orderBy2Opt(MPVRPInstance instance,
                                     MPVRPInstance.DepotProductNode depot,
                                     List<Request> deliveries) {
        List<Request> ordered = orderByNearestStation(instance, depot, deliveries);
        long currentDistance = tripConstructionScore(instance, depot, ordered);

        boolean improved;
        do {
            improved = false;
            long bestDistance = currentDistance;
            int bestFrom = -1;
            int bestTo = -1;

            for (int from = 0; from < ordered.size() - 1; from++) {
                for (int to = from + 1; to < ordered.size(); to++) {
                    Collections.reverse(ordered.subList(from, to + 1));
                    long candidateDistance = tripConstructionScore(instance, depot, ordered);
                    Collections.reverse(ordered.subList(from, to + 1));

                    if (candidateDistance < bestDistance) {
                        bestDistance = candidateDistance;
                        bestFrom = from;
                        bestTo = to;
                    }
                }
            }

            if (bestFrom >= 0) {
                Collections.reverse(ordered.subList(bestFrom, bestTo + 1));
                currentDistance = bestDistance;
                improved = true;
            }
        } while (improved);

        return ordered;
    }

    /**
     * Scores a trip by its internal distance and a route-independent lower bound
     * on the connection to its next depot or final garage.
     */
    static long tripConstructionScore(MPVRPInstance instance,
                                      MPVRPInstance.DepotProductNode depot,
                                      List<Request> deliveries) {
        long distance = 0;
        int previous = depot.globalId();
        for (Request delivery : deliveries) {
            int station = delivery.station().globalId();
            distance += instance.distMatrix[previous][station];
            previous = station;
        }
        if (!deliveries.isEmpty()) {
            int bestContinuation = Integer.MAX_VALUE;
            for (MPVRPInstance.DepotProductNode candidate : instance.depotProducts) {
                bestContinuation = Math.min(bestContinuation,
                        instance.distMatrix[previous][candidate.globalId()]);
            }
            for (MPVRPInstance.GarageNode garage : instance.garages) {
                bestContinuation = Math.min(bestContinuation,
                        instance.distMatrix[previous][garage.globalId()]);
            }
            if (bestContinuation != Integer.MAX_VALUE) {
                distance += bestContinuation;
            }
        }
        return distance;
    }

    /**
     * Builds the transition-cost matrix used by the sequence variables.
     * A transition to a trip includes travel to its depot, product changeover,
     * and travel between the deliveries contained in that trip.
     *
     * @param instance problem instance providing distances and changeover costs
     * @param trips trips represented by the regular sequence nodes
     * @param initialProducts product initially carried by each vehicle
     * @param vehicleCount number of vehicle routes
     * @return transition-cost matrix for trip, start, and end nodes
     */
    static int[][] buildTripCosts(MPVRPInstance instance, List<Trip> trips,
                                  int[] initialProducts, int vehicleCount) {
        int tripCount = trips.size();
        int nodeCount = tripCount + 2 * vehicleCount;
        int[][] cost = new int[nodeCount][nodeCount];
        for (int[] row : cost) {
            Arrays.fill(row, PENALTY_INF);
        }

        int[] internalDistance = new int[tripCount];
        int[] lastStation = new int[tripCount];

        // Precompute the distance traveled from each depot through all deliveries of its trip.
        for (int trip = 0; trip < tripCount; trip++) {
            Trip currentTrip = trips.get(trip);
            int previous = currentTrip.depot().globalId();
            for (Request delivery : currentTrip.deliveries()) {
                int station = delivery.station().globalId();
                internalDistance[trip] += instance.distMatrix[previous][station];
                previous = station;
            }
            lastStation[trip] = previous;
        }

        // Cost of finishing one trip and performing the next one.
        for (int from = 0; from < tripCount; from++) {
            Trip fromTrip = trips.get(from);
            for (int to = 0; to < tripCount; to++) {
                if (from == to) {
                    continue;
                }
                Trip toTrip = trips.get(to);
                cost[from][to] = instance.distMatrix[lastStation[from]][toTrip.depot().globalId()]
                        + instance.changeoverCost[fromTrip.depot().product()][toTrip.depot().product()]
                        + internalDistance[to];
            }
        }

        // Vehicle-specific costs from and back to the home garage.
        for (int vehicle = 0; vehicle < vehicleCount; vehicle++) {
            int start = tripCount + vehicle;
            int end = tripCount + vehicleCount + vehicle;
            int garage = instance.vehicules[vehicle].startG().globalId();
            cost[start][end] = 0;
            for (int trip = 0; trip < tripCount; trip++) {
                Trip currentTrip = trips.get(trip);
                cost[start][trip] = instance.distMatrix[garage][currentTrip.depot().globalId()]
                        + instance.changeoverCost[initialProducts[vehicle]][currentTrip.depot().product()]
                        + internalDistance[trip];
                cost[trip][end] = instance.distMatrix[lastStation[trip]][garage];
            }
        }
        return cost;
    }

    /** Collects deliveries that can be performed during the same loading operation. */
    private static final class TripBin {
        private final MPVRPInstance.DepotProductNode depot;
        private final int constructionVehicle;
        private final List<Request> deliveries = new ArrayList<>();
        private final Set<Integer> logicalRequests = new HashSet<>();
        private int quantity;

        private TripBin(MPVRPInstance.DepotProductNode depot, int constructionVehicle) {
            this.depot = depot;
            this.constructionVehicle = constructionVehicle;
        }

        private boolean canPack(Request delivery, int vehicle, int capacity) {
            return depot.equals(delivery.depot())
                    && constructionVehicle == vehicle
                    && !logicalRequests.contains(delivery.logicalRequest())
                    && quantity + delivery.quantity() <= capacity;
        }

        private void add(Request delivery) {
            deliveries.add(delivery);
            logicalRequests.add(delivery.logicalRequest());
            quantity += delivery.quantity();
        }
    }
}
