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
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
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
 *
 * @author Rosas Behoundja
 */
public class MPVRPSeqVar {

    /** Large penalty for invalid node transitions. */
    private static final int PENALTY_INF = 1_000_000;

    public static void main(String[] args) throws Exception {
        String inputFile = args.length > 0 ? args[0] : "data/MPVRP_CC/MPVRP_073_s5_d6_p3.dat";
        String outputFile = args.length > 1 ? args[1] : null;

        MPVRPInstance instance = MPVRPInstance.read(inputFile);
        CPSolver cp = makeSolver();

        int vehicleCount = instance.nVehicules;
        int[] initialProducts = new int[vehicleCount];
        int[] capacities = new int[vehicleCount];
        for (int vehicle = 0; vehicle < vehicleCount; vehicle++) {
            initialProducts[vehicle] = instance.vehicules[vehicle].initProd();
            capacities[vehicle] = instance.vehicules[vehicle].capacity();
        }

        // Split demands across depots and pack compatible deliveries into trips.
        List<Trip> trips = buildTrips(instance, capacities);
        if (trips.isEmpty()) {
            System.out.println("No positive station demand in the instance.");
            return;
        }
        int tripCount = trips.size();

        // A trip is atomic in the sequence: loading and all its deliveries cannot
        // be separated or interleaved with another loading operation.
        int nNodes = tripCount + 2 * vehicleCount;
        int[][] cost = buildTripCosts(instance, trips, initialProducts, vehicleCount);
        int maxArc = Arrays.stream(cost).flatMapToInt(Arrays::stream)
                .filter(value -> value != PENALTY_INF).max().orElse(0);
        int maxRouteCost = (int) Math.min(Integer.MAX_VALUE, (long) maxArc * nNodes);

        // decisions variables
        CPSeqVar[] routes = new CPSeqVar[vehicleCount];
        CPIntVar[] routeCosts = new CPIntVar[vehicleCount];

        for (int vehicle = 0; vehicle < vehicleCount; vehicle++) {
            int start = tripCount + vehicle;
            int end = tripCount + vehicleCount + vehicle;
            routes[vehicle] = makeSeqVar(cp, nNodes, start, end);
            for (int other = 0; other < vehicleCount; other++) {
                if (other != vehicle) {
                    cp.post(exclude(routes[vehicle], tripCount + other));
                    cp.post(exclude(routes[vehicle], tripCount + vehicleCount + other));
                }
            }
            routeCosts[vehicle] = makeIntVar(cp, 0, maxRouteCost);
        }

        CPIntVar totalCost = sum(routeCosts);

        // constraints
        for (int vehicle = 0; vehicle < vehicleCount; vehicle++) {
            cp.post(new TransitionCost(routes[vehicle], cost, routeCosts[vehicle]));
        }

        // Every trip is assigned to exactly one vehicle that can carry its load.
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

        // Chunks of a split station-product demand must use distinct vehicles.
        // This enforces the benchmark's at-most-once rule for each vehicle and
        // station-product pair.
        for (int logicalRequest = 0; logicalRequest < instance.nRequests; logicalRequest++) {
            int request = logicalRequest;
            for (int vehicle = 0; vehicle < vehicleCount; vehicle++) {
                List<CPIntVar> visits = new ArrayList<>();
                for (int trip = 0; trip < tripCount; trip++) {
                    if (trips.get(trip).deliveries().stream()
                            .anyMatch(delivery -> delivery.logicalRequest() == request)) {
                        visits.add(routes[vehicle].getNodeVar(trip).isRequired());
                    }
                }
                if (visits.size() > 1) {
                    cp.post(le(sum(visits.toArray(new CPIntVar[0])), 1));
                }
            }
        }

        // search strategy
        Integer[] routingTrips = IntStream.range(0, tripCount).boxed().toArray(Integer[]::new);
        int[] insertions = new int[nNodes];
        CPIntVar zero = makeIntVar(cp, 0, 0);

        Supplier<Runnable[]> branching = () -> {
            if (Arrays.stream(routes).allMatch(CPSeqVar::isFixed)) {
                return EMPTY;
            }

            Integer trip = selectMin(routingTrips,
                    candidate -> Arrays.stream(routes).anyMatch(r -> r.isNode(candidate, INSERTABLE)),
                    candidate -> Arrays.stream(routes).mapToInt(r -> r.nInsert(candidate)).sum());

            if (trip == null) {
                return branch(() -> cp.post(eq(zero, 1)));
            }

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
                return branch(() -> cp.post(eq(zero, 1)));
            }

            CPSeqVar chosen = bestRoute;
            int chosenPred = bestPred;
            int chosenSucc = bestSucc;
            int chosenTrip = trip;
            return branch(
                    () -> cp.post(insert(chosen, chosenPred, chosenTrip)),
                    () -> cp.post(notBetween(chosen, chosenPred, chosenTrip, chosenSucc)));
        };

        // objective function
        DFSearch search = makeDfs(cp, branching);
        Objective objective = cp.minimize(totalCost);
        final SolutionData[] bestSolution = new SolutionData[1];
        final int[] bestObjective = {Integer.MAX_VALUE};
        long init = System.currentTimeMillis();

        search.onSolution(() -> {
            int objectiveValue = totalCost.min();
            if (objectiveValue < bestObjective[0]) {
                bestObjective[0] = objectiveValue;
                bestSolution[0] = captureSolution(instance, trips, routes, initialProducts, nNodes);
                System.out.println("Solution found! Objective: " + objectiveValue);
                long elapsedMs = System.currentTimeMillis() - init;
                if (bestSolution[0] != null) {
                    String text = buildSolutionText(bestSolution[0], elapsedMs / 1000.0);
                    Path out = Path.of(outputFile != null ? outputFile : defaultOutputPath(inputFile));
                    try {
                        writeSolution(out, text);
                    } catch (IOException e) {
                        throw new RuntimeException(e);
                    }
                    System.out.println("Solution written to " + out.toAbsolutePath());
                }
            }
        });

        System.out.println("Starting search...");
        SearchStatistics stats = search.optimize(objective);

        System.out.println(stats);
    }

    static List<Trip> buildTrips(MPVRPInstance instance, int[] capacities) {
        if (capacities.length == 0 || Arrays.stream(capacities).anyMatch(capacity -> capacity <= 0)) {
            throw new IllegalArgumentException("Vehicle capacities must be positive");
        }

        int[] remainingStock = Arrays.stream(instance.depotProducts)
                .mapToInt(MPVRPInstance.DepotProductNode::stock)
                .toArray();
        int[] sortedCapacities = capacities.clone();
        Arrays.sort(sortedCapacities);

        Integer[] requestOrder = IntStream.range(0, instance.nRequests).boxed()
                .sorted(Comparator.comparingInt((Integer index) -> instance.requests[index].demand()).reversed())
                .toArray(Integer[]::new);
        List<Request> deliveries = new ArrayList<>();

        for (int requestIndex : requestOrder) {
            MPVRPInstance.RequestNode request = instance.requests[requestIndex];
            int remainingDemand = request.demand();
            for (int vehicle = sortedCapacities.length - 1; vehicle >= 0 && remainingDemand > 0; vehicle--) {
                int vehicleCapacity = sortedCapacities[vehicle];
                int selectedDepot = -1;
                int selectedQuantity = 0;
                int selectedDistance = Integer.MAX_VALUE;
                for (int depotIndex = 0; depotIndex < instance.nDepotProducts; depotIndex++) {
                    MPVRPInstance.DepotProductNode depot = instance.depotProducts[depotIndex];
                    if (depot.product() != request.product() || remainingStock[depotIndex] == 0) {
                        continue;
                    }
                    int quantity = Math.min(remainingDemand, Math.min(vehicleCapacity, remainingStock[depotIndex]));
                    int distance = instance.distMatrix[depot.globalId()][request.globalId()];
                    if (quantity > selectedQuantity || quantity == selectedQuantity && distance < selectedDistance) {
                        selectedDepot = depotIndex;
                        selectedQuantity = quantity;
                        selectedDistance = distance;
                    }
                }

                if (selectedDepot == -1) {
                    break;
                }
                MPVRPInstance.DepotProductNode depot = instance.depotProducts[selectedDepot];
                remainingStock[selectedDepot] -= selectedQuantity;
                remainingDemand -= selectedQuantity;
                deliveries.add(new Request(requestIndex, request, depot, selectedQuantity));
            }

            if (remainingDemand > 0) {
                throw new IllegalArgumentException("Demand for station " + request.id() + " and product "
                        + request.product() + " cannot be split among the available vehicles and depot stocks"
                        + " (remaining quantity: " + remainingDemand + ")");
            }
        }

        int sharedTripCapacity = sortedCapacities[0];
        deliveries.sort(Comparator.comparingInt(Request::quantity).reversed());
        List<TripBin> bins = new ArrayList<>();
        for (Request delivery : deliveries) {
            TripBin selectedBin = null;
            int smallestRemainingCapacity = Integer.MAX_VALUE;
            for (TripBin bin : bins) {
                int remainingCapacity = sharedTripCapacity - bin.quantity - delivery.quantity();
                if (bin.canPack(delivery, sharedTripCapacity) && remainingCapacity < smallestRemainingCapacity) {
                    selectedBin = bin;
                    smallestRemainingCapacity = remainingCapacity;
                }
            }
            if (selectedBin == null) {
                selectedBin = new TripBin(delivery.depot());
                bins.add(selectedBin);
            }
            selectedBin.add(delivery);
        }

        List<Trip> trips = new ArrayList<>(bins.size());
        for (TripBin bin : bins) {
            trips.add(new Trip(bin.depot, orderByNearestStation(instance, bin.depot, bin.deliveries)));
        }
        return trips;
    }

    private static List<Request> orderByNearestStation(MPVRPInstance instance,
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

    private static final class TripBin {
        private final MPVRPInstance.DepotProductNode depot;
        private final List<Request> deliveries = new ArrayList<>();
        private final Set<Integer> logicalRequests = new HashSet<>();
        private int quantity;

        private TripBin(MPVRPInstance.DepotProductNode depot) {
            this.depot = depot;
        }

        private boolean canPack(Request delivery, int capacity) {
            return depot.equals(delivery.depot())
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
