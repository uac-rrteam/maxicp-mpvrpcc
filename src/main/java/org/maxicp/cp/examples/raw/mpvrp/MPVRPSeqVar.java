package org.maxicp.cp.examples.raw.mpvrp;

import org.maxicp.cp.engine.constraints.Sum;
import org.maxicp.cp.engine.constraints.seqvar.Cumulative;
import org.maxicp.cp.engine.core.AbstractCPConstraint;
import org.maxicp.cp.engine.core.CPIntVar;
import org.maxicp.cp.engine.core.CPSeqVar;
import org.maxicp.cp.engine.core.CPSolver;
import org.maxicp.cp.examples.utils.MPVRPInstance;
import org.maxicp.search.DFSearch;
import org.maxicp.search.Objective;
import org.maxicp.search.SearchStatistics;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import static org.maxicp.cp.CPFactory.*;
import static org.maxicp.modeling.algebra.sequence.SeqStatus.INSERTABLE;
import static org.maxicp.modeling.algebra.sequence.SeqStatus.MEMBER_ORDERED;
import static org.maxicp.search.Searches.*;
import static org.maxicp.util.exception.InconsistencyException.INCONSISTENCY;

/**
 * Multi-Product Vehicle Routing Problem with Changeover Cost
 *
 * @author Rosas Behoundja
 */
public class MPVRPSeqVar {

    // Node Kind Constants
    private static final int NODE_PICKUP = 0; // Depot loading visit
    private static final int NODE_DROP = 1;   // Station delivery visit
    private static final int NODE_GARAGE = 2; // Garage start/end node

    /** Large penalty for invalid node transitions. */
    private static final int PENALTY_INF = 1_000_000;

    // Helper data structures for sequence solution export
    private record Request(int logicalRequest, int stationId, int stationGlobal, int depotId,
                           int depotGlobal, int product, int quantity) {}
    private record RouteSolution(String visitLine, String productLine, int productChanges, double transitionCost, double distance) {}
    private record SolutionData(List<RouteSolution> routes, int usedVehicles, int productChanges, double transitionCost, double distance) {}

    /** Exact route cost for arbitrary (not necessarily metric) transition matrices. */
    private static final class RouteCost extends AbstractCPConstraint {
        private final CPSeqVar route;
        private final int[][] costs;
        private final CPIntVar total;
        private final int[] nodes;

        private RouteCost(CPSeqVar route, int[][] costs, CPIntVar total) {
            super(route.getSolver());
            this.route = route;
            this.costs = costs;
            this.total = total;
            this.nodes = new int[route.nNode()];
        }

        @Override
        public void post() {
            propagate();
            route.propagateOnInsert(this);
            route.propagateOnFix(this);
        }

        @Override
        public void propagate() {
            if (!route.isFixed()) {
                return;
            }
            int n = route.fillNode(nodes, MEMBER_ORDERED);
            int value = 0;
            for (int i = 0; i < n - 1; i++) {
                int arcCost = costs[nodes[i]][nodes[i + 1]];
                if (arcCost >= PENALTY_INF) {
                    throw INCONSISTENCY;
                }
                value += arcCost;
            }
            total.fix(value);
            setActive(false);
        }
    }

    public static void main(String[] args) throws Exception {
        String inputFile = args.length > 0 ? args[0] : "data/MPVRP_CC/MPVRP_013_s7_d2_p2.dat";
        String outputFile = args.length > 1 ? args[1] : null;

        MPVRPInstance inst = MPVRPInstance.read(inputFile);
        CPSolver cp = makeSolver();

        int V = inst.nVehicules;
        int P = inst.nProds;

        int[] departure = new int[V];
        int[] initProd = new int[V];
        int[] cap = new int[V];
        for (int v = 0; v < V; v++) {
            departure[v] = inst.vehicules[v].startG().globalId();
            initProd[v] = inst.vehicules[v].initProd(); // 0-indexed product ID
            cap[v] = inst.vehicules[v].capacity();
        }

        // Build station requests supplied by feasible depots
        List<Request> requests = buildRequests(inst, cap);
        int R = requests.size();
        if (R == 0) {
            System.out.println("No positive station demand in the instance.");
            return;
        }

        // Total sequence nodes N = 2*R + 2*V (R pickups + R drops + V garage starts + V garage ends)
        int nNodes = 2 * R + 2 * V;
        int[] starts = new int[R];
        int[] ends = new int[R];
        int[] load = new int[R];

        int[] nodeKind = new int[nNodes];
        int[] nodeProduct = new int[nNodes];
        int[] nodeGlobalId = new int[nNodes];
        int[] startVehicle = new int[nNodes];
        Arrays.fill(nodeKind, NODE_GARAGE);
        Arrays.fill(nodeProduct, -1);
        Arrays.fill(startVehicle, -1);

        for (int i = 0; i < R; i++) {
            Request req = requests.get(i);
            starts[i] = i;              // Pickup node index
            ends[i] = R + i;            // Drop node index
            load[i] = req.quantity();   // Demand load

            nodeKind[i] = NODE_PICKUP;
            nodeKind[R + i] = NODE_DROP;
            nodeProduct[i] = req.product();
            nodeProduct[R + i] = req.product();
            nodeGlobalId[i] = req.depotGlobal();
            nodeGlobalId[R + i] = req.stationGlobal();
        }

        int[][] dist = buildArcCosts(inst, nodeKind, nodeProduct, nodeGlobalId, startVehicle, initProd, departure, R, V);
        int maxArc = Arrays.stream(dist).flatMapToInt(Arrays::stream).max().orElse(10000);
        int totalDemand = Arrays.stream(load).sum();

        // decisions variables
        CPSeqVar[] route = new CPSeqVar[V];
        CPIntVar[] distV = new CPIntVar[V];
        CPIntVar[] loadV = new CPIntVar[V];

        for (int v = 0; v < V; v++) {
            int start = 2 * R + v;
            int end = 2 * R + V + v;
            route[v] = makeSeqVar(cp, nNodes, start, end);
            for (int other = 0; other < V; other++) {
                if (other != v) {
                    cp.post(exclude(route[v], 2 * R + other));
                    cp.post(exclude(route[v], 2 * R + V + other));
                }
            }
            distV[v] = makeIntVar(cp, 0, maxArc * nNodes);
            loadV[v] = makeIntVar(cp, 0, totalDemand);

            nodeKind[start] = NODE_GARAGE;
            nodeKind[end] = NODE_GARAGE;
            nodeGlobalId[start] = departure[v];
            nodeGlobalId[end] = departure[v];
            startVehicle[start] = v;
        }

        CPIntVar sumDistance = sum(distV);

        // constraints
        for (int v = 0; v < V; v++) {
            // Traverse sequence distance and changeover cost propagation
            cp.post(new RouteCost(route[v], dist, distV[v]));

            // Capacity constraint over pickup/drop activities
            cp.post(new Cumulative(route[v], starts, ends, load, cap[v]));

            // Vehicle total carried load aggregation
            CPIntVar[] servedLoad = new CPIntVar[R];
            for (int i = 0; i < R; i++) {
                servedLoad[i] = mul(route[v].getNodeVar(ends[i]).isRequired(), load[i]);
            }
            cp.post(new Sum(servedLoad, loadV[v]));
        }

        // Request Coverage: each pickup & drop pair must be served by exactly 1 vehicle
        for (int i = 0; i < R; i++) {
            CPIntVar[] dropVisits = new CPIntVar[V];
            CPIntVar[] pickupVisits = new CPIntVar[V];
            for (int v = 0; v < V; v++) {
                dropVisits[v] = route[v].getNodeVar(ends[i]).isRequired();
                pickupVisits[v] = route[v].getNodeVar(starts[i]).isRequired();
                // Pickup and drop must belong to the same vehicle route
                cp.post(eq(dropVisits[v], pickupVisits[v]));
            }
            cp.post(new Sum(dropVisits, 1));
        }

        // Chunks of a split station-product demand must use distinct vehicles.
        // This enforces the benchmark's at-most-once rule for each vehicle and
        // station-product pair.
        for (int logicalRequest = 0; logicalRequest < inst.nRequests; logicalRequest++) {
            for (int v = 0; v < V; v++) {
                List<CPIntVar> visits = new ArrayList<>();
                for (int i = 0; i < R; i++) {
                    if (requests.get(i).logicalRequest() == logicalRequest) {
                        visits.add(route[v].getNodeVar(ends[i]).isRequired());
                    }
                }
                if (visits.size() > 1) {
                    cp.post(le(sum(visits.toArray(new CPIntVar[0])), 1));
                }
            }
        }

        // Physical Depot Stock Constraint check
        for (int d = 0; d < inst.nDepotsRead; d++) {
            int depotId = inst.depots[d].id();
            for (int p = 0; p < P; p++) {
                int stock = inst.depots[d].stocks()[p];
                if (stock <= 0) continue;

                List<Integer> reqIndices = new ArrayList<>();
                for (int i = 0; i < R; i++) {
                    if (requests.get(i).depotId() == depotId && requests.get(i).product() == p) {
                        reqIndices.add(i);
                    }
                }
                if (!reqIndices.isEmpty()) {
                    int totalReqStock = reqIndices.stream().mapToInt(idx -> requests.get(idx).quantity()).sum();
                    if (totalReqStock > stock) {
                        throw new IllegalArgumentException("Depot " + depotId + " stock for product " + p + " is insufficient!");
                    }
                }
            }
        }

        // search strategy
        Integer[] routingNodes = IntStream.range(0, R).boxed().toArray(Integer[]::new);
        int[] insertions = new int[nNodes];
        CPIntVar zero = makeIntVar(cp, 0, 0);

        Supplier<Runnable[]> branching = () -> {
            if (Arrays.stream(route).allMatch(CPSeqVar::isFixed)) {
                return EMPTY;
            }

            Integer node = selectMin(routingNodes,
                    n -> Arrays.stream(route).anyMatch(r -> r.isNode(ends[n], INSERTABLE)),
                    n -> Arrays.stream(route).mapToInt(r -> r.nInsert(ends[n])).sum());

            if (node == null) {
                // Propagation can require one half of a pickup/drop pair before it
                // gets positioned. Finish any such pending insertion so that the
                // route, and therefore the distance objective, is bound at leaves.
                for (int pending = 0; pending < 2 * R; pending++) {
                    int pendingNode = pending;
                    List<Runnable> branches = new ArrayList<>();
                    for (CPSeqVar r : route) {
                        int nInsert = r.fillInsert(pendingNode, insertions);
                        for (int i = 0; i < nInsert; i++) {
                            int predecessor = insertions[i];
                            int successor = r.memberAfter(predecessor);
                            if (!isFeasibleInsertion(pendingNode, predecessor, successor, nodeKind, nodeProduct)) {
                                continue;
                            }
                            branches.add(() -> cp.post(insert(r, predecessor, pendingNode)));
                        }
                    }
                    if (!branches.isEmpty()) {
                        return branches.toArray(new Runnable[0]);
                    }
                }
                return branch(() -> cp.post(eq(zero, 1)));
            }

            int bestDetour = Integer.MAX_VALUE;
            int bestPred = -1;
            int bestSucc = -1;
            CPSeqVar bestRoute = null;

            int dropNode = ends[node];
            int pickupNode = starts[node];

            for (CPSeqVar r : route) {
                int nInsert = r.fillInsert(dropNode, insertions);
                for (int i = 0; i < nInsert; i++) {
                    int pred = insertions[i];
                    int succ = r.memberAfter(pred);
                    if (!isFeasiblePairInsertion(dropNode, pred, succ, nodeKind, nodeProduct)) {
                        continue;
                    }
                    int detour = dist[pred][dropNode] + dist[dropNode][succ] - dist[pred][succ];
                    if (detour < bestDetour) {
                        bestDetour = detour;
                        bestPred = pred;
                        bestSucc = succ;
                        bestRoute = r;
                    }
                }
            }

            if (bestRoute == null) {
                CPSeqVar fallbackRoute = selectMin(route, r -> r.isNode(dropNode, INSERTABLE), CPSeqVar::nNode);
                if (fallbackRoute == null) {
                    return EMPTY;
                }
                int nInsert = fallbackRoute.fillInsert(dropNode, insertions);
                if (nInsert == 0) {
                    return EMPTY;
                }
                int pred = -1;
                int succ = -1;
                for (int i = 0; i < nInsert; i++) {
                    int candidatePred = insertions[i];
                    int candidateSucc = fallbackRoute.memberAfter(candidatePred);
                    if (isFeasiblePairInsertion(dropNode, candidatePred, candidateSucc, nodeKind, nodeProduct)) {
                        pred = candidatePred;
                        succ = candidateSucc;
                        break;
                    }
                }
                if (pred == -1) {
                    return branch(() -> cp.post(eq(zero, 1)));
                }
                int selectedPred = pred;
                int selectedSucc = succ;
                return branch(() -> {
                    cp.post(insert(fallbackRoute, selectedPred, pickupNode));
                    cp.post(insert(fallbackRoute, pickupNode, dropNode));
                }, () -> cp.post(notBetween(fallbackRoute, selectedPred, dropNode, selectedSucc)));
            }

            CPSeqVar chosen = bestRoute;
            int chosenPred = bestPred;
            int chosenSucc = bestSucc;
            return branch(() -> {
                cp.post(insert(chosen, chosenPred, pickupNode));
                cp.post(insert(chosen, pickupNode, dropNode));
            }, () -> cp.post(notBetween(chosen, chosenPred, dropNode, chosenSucc)));
        };

        // objective function
        DFSearch search = makeDfs(cp, branching);
        Objective objective = cp.minimize(sumDistance);
        final SolutionData[] bestSolution = new SolutionData[1];
        final int[] bestObjective = {Integer.MAX_VALUE};
        long init = System.currentTimeMillis();

        search.onSolution(() -> {
            int objectiveValue = sumDistance.min();
            if (objectiveValue < bestObjective[0]) {
                bestObjective[0] = objectiveValue;
                bestSolution[0] = captureSolution(inst, requests, route, initProd, nNodes, R);
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

    // help method

    private static boolean isFeasibleInsertion(int node, int pred, int succ, int[] nodeKind, int[] nodeProduct) {
        if (nodeKind[node] == NODE_PICKUP) {
            return nodeKind[pred] != NODE_PICKUP && nodeKind[succ] != NODE_PICKUP;
        }
        if (nodeKind[node] == NODE_DROP) {
            boolean leftOk = nodeKind[pred] != NODE_DROP || nodeProduct[pred] == nodeProduct[node];
            boolean rightOk = nodeKind[succ] != NODE_DROP || nodeProduct[succ] == nodeProduct[node];
            return leftOk && rightOk;
        }
        return true;
    }

    private static boolean isFeasiblePairInsertion(int drop, int pred, int succ,
                                                    int[] nodeKind, int[] nodeProduct) {
        return nodeKind[pred] != NODE_PICKUP
                && (nodeKind[succ] != NODE_DROP || nodeProduct[succ] == nodeProduct[drop]);
    }

    private static List<Request> buildRequests(MPVRPInstance inst, int[] capacities) {
        int[] remainingStock = Arrays.stream(inst.depotProducts)
                .mapToInt(MPVRPInstance.DepotProductNode::stock)
                .toArray();
        int[] sortedCapacities = capacities.clone();
        Arrays.sort(sortedCapacities);

        List<Request> requests = new ArrayList<>();
        for (int requestIndex = 0; requestIndex < inst.nRequests; requestIndex++) {
            MPVRPInstance.RequestNode request = inst.requests[requestIndex];
            int nVehicles = 0;
            int availableCapacity = 0;
            while (nVehicles < sortedCapacities.length && availableCapacity < request.demand()) {
                availableCapacity += sortedCapacities[sortedCapacities.length - 1 - nVehicles];
                nVehicles++;
            }
            if (availableCapacity < request.demand()) {
                throw new IllegalArgumentException("Fleet capacity is insufficient for station " + request.id()
                        + " and product " + request.product());
            }

            int remainingDemand = request.demand();
            int firstCapacity = sortedCapacities.length - nVehicles;
            for (int c = firstCapacity; c < sortedCapacities.length; c++) {
                int quantity = Math.min(sortedCapacities[c], remainingDemand);
                remainingDemand -= quantity;

                int bestDepot = -1;
                int bestDistance = Integer.MAX_VALUE;
                for (int d = 0; d < inst.nDepotProducts; d++) {
                    MPVRPInstance.DepotProductNode depot = inst.depotProducts[d];
                    if (depot.product() != request.product() || remainingStock[d] < quantity) {
                        continue;
                    }
                    int distance = inst.distMatrix[depot.globalId()][request.globalId()];
                    if (distance < bestDistance) {
                        bestDistance = distance;
                        bestDepot = d;
                    }
                }
                if (bestDepot == -1) {
                    throw new IllegalArgumentException("No single depot can supply a " + quantity
                            + "-unit split for station " + request.id() + " and product " + request.product());
                }
                MPVRPInstance.DepotProductNode depot = inst.depotProducts[bestDepot];
                remainingStock[bestDepot] -= quantity;
                requests.add(new Request(requestIndex, request.id(), request.globalId(), depot.id(),
                        depot.globalId(), request.product(), quantity));
            }
        }
        return requests;
    }

    private static SolutionData captureSolution(MPVRPInstance inst, List<Request> requests, CPSeqVar[] route,
                                                int[] initProd, int nNodes, int nRequest) {
        int[] nodes = new int[nNodes];
        List<RouteSolution> routes = new ArrayList<>();
        int usedVehicles = 0;
        int totalProductChanges = 0;
        double totalTransitionCost = 0.0;
        double totalDistance = 0.0;

        for (int v = 0; v < route.length; v++) {
            int nMember = route[v].fillNode(nodes, MEMBER_ORDERED);
            if (nMember <= 2) {
                continue;
            }

            usedVehicles++;
            List<String> visitParts = new ArrayList<>();
            List<String> productParts = new ArrayList<>();

            int currentProduct = initProd[v];
            double cumulativeChangeCost = 0.0;
            int productChanges = 0;
            double routeDistance = 0.0;

            for (int i = 0; i < nMember; i++) {
                int node = nodes[i];
                int globalId = globalIdOf(node, inst, requests, v, nRequest);

                if (i > 0) {
                    int prev = nodes[i - 1];
                    int prevGlobalId = globalIdOf(prev, inst, requests, v, nRequest);
                    routeDistance += inst.distMatrix[prevGlobalId][globalId];
                }

                if (node < nRequest) {
                    Request req = requests.get(node);
                    cumulativeChangeCost += inst.changeoverCost[currentProduct][req.product()];
                    if (req.product() != currentProduct) {
                        productChanges++;
                    }
                    currentProduct = req.product();
                    visitParts.add(formatDepotLabel(req.depotId(), req.quantity()));
                    productParts.add(formatProductLabel(currentProduct, cumulativeChangeCost));
                } else if (node < 2 * nRequest) {
                    Request req = requests.get(node - nRequest);
                    visitParts.add(formatStationLabel(req.stationId(), req.quantity()));
                    productParts.add(formatProductLabel(currentProduct, cumulativeChangeCost));
                } else {
                    int garageId = inst.vehicules[v].startG().id();
                    visitParts.add(String.valueOf(garageId));
                    if (i == 0) {
                        productParts.add(formatProductLabel(currentProduct, cumulativeChangeCost));
                    }
                }
            }

            totalProductChanges += productChanges;
            totalTransitionCost += cumulativeChangeCost;
            totalDistance += routeDistance;

            routes.add(new RouteSolution(
                    inst.vehicules[v].id() + ": " + String.join(" - ", visitParts),
                    inst.vehicules[v].id() + ": " + String.join(" - ", productParts),
                    productChanges,
                    cumulativeChangeCost,
                    routeDistance));
        }

        return new SolutionData(routes, usedVehicles, totalProductChanges, totalTransitionCost, totalDistance);
    }

    private static String buildSolutionText(SolutionData solution, double elapsedSeconds) {
        StringBuilder sb = new StringBuilder();
        for (RouteSolution route : solution.routes()) {
            sb.append(route.visitLine()).append('\n');
            sb.append(route.productLine()).append('\n').append('\n');
        }
        sb.append(solution.usedVehicles()).append('\n');
        sb.append(solution.productChanges()).append('\n');
        sb.append(formatCost(solution.transitionCost())).append('\n');
        sb.append(formatCost(solution.distance())).append('\n');
        sb.append(readProcessorModel()).append('\n');
        sb.append(String.format(Locale.US, "%.3f", elapsedSeconds)).append('\n');
        return sb.toString();
    }

    private static void writeSolution(Path file, String text) throws IOException {
        Path parent = file.toAbsolutePath().getParent();
        if (parent != null) {
            Files.createDirectories(parent);
        }
        Files.writeString(file, text);
    }

    private static String defaultOutputPath(String instancePath) {
        String name = Path.of(instancePath).getFileName().toString();
        return name.startsWith("MPVRP_") ? "Sol_" + name.substring("MPVRP_".length()) : "Sol_" + name;
    }

    private static String formatDepotLabel(int depotId, double qty) {
        return depotId + " [" + formatQuantity(qty) + "]";
    }

    private static String formatStationLabel(int stationId, double qty) {
        return stationId + " (" + formatQuantity(qty) + ")";
    }

    private static String formatProductLabel(int product, double cumulativeCost) {
        return product + "(" + formatCost(cumulativeCost) + ")";
    }

    private static String formatQuantity(double value) {
        return formatNumber(value);
    }

    private static String formatCost(double value) {
        return String.format(Locale.US, "%.2f", value);
    }

    private static String formatNumber(double value) {
        double rounded = Math.rint(value);
        if (Math.abs(value - rounded) < 1e-9) {
            return Long.toString(Math.round(rounded));
        }
        return String.format(Locale.US, "%.2f", value);
    }

    private static String readProcessorModel() {
        try {
            List<String> lines = Files.readAllLines(Path.of("/proc/cpuinfo"));
            for (String line : lines) {
                if (line.startsWith("model name")) {
                    int idx = line.indexOf(':');
                    if (idx >= 0) {
                        return line.substring(idx + 1).trim();
                    }
                }
            }
        } catch (IOException ignored) {
        }
        return System.getProperty("os.name") + " " + System.getProperty("os.arch");
    }

    private static int globalIdOf(int node, MPVRPInstance inst, List<Request> requests, int vehicle, int nRequest) {
        if (node < nRequest) {
            return requests.get(node).depotGlobal();
        }
        if (node < 2 * nRequest) {
            return requests.get(node - nRequest).stationGlobal();
        }
        return inst.vehicules[vehicle].startG().globalId();
    }

    private static int[][] buildArcCosts(MPVRPInstance inst, int[] nodeKind, int[] nodeProduct,
                                         int[] nodeGlobalId, int[] startVehicle, int[] initProd,
                                         int[] departure, int nRequest, int nVehicle) {
        for (int v = 0; v < nVehicle; v++) {
            int start = 2 * nRequest + v;
            int end = 2 * nRequest + nVehicle + v;
            nodeKind[start] = NODE_GARAGE;
            nodeKind[end] = NODE_GARAGE;
            nodeGlobalId[start] = departure[v];
            nodeGlobalId[end] = departure[v];
            startVehicle[start] = v;
        }

        int nNodes = nodeKind.length;
        int[][] arcCost = new int[nNodes][nNodes];
        for (int i = 0; i < nNodes; i++) {
            for (int j = 0; j < nNodes; j++) {
                int base = inst.distMatrix[nodeGlobalId[i]][nodeGlobalId[j]];
                int setup = 0;

                // A pickup represents one loading operation and must start a
                // trip. Its paired delivery is therefore the next route node;
                // this sequence example deliberately models one station per
                // trip while the successor model can group several stations.
                if (nodeKind[i] == NODE_PICKUP && j != nRequest + i) {
                    setup += PENALTY_INF;
                }

                if (nodeKind[i] == NODE_DROP && nodeKind[j] == NODE_PICKUP) {
                    int p1 = nodeProduct[i];
                    int p2 = nodeProduct[j];
                    setup += inst.changeoverCost[p1][p2];
                } else if (nodeKind[i] == NODE_GARAGE && startVehicle[i] >= 0 && nodeKind[j] == NODE_PICKUP) {
                    int v = startVehicle[i];
                    setup += inst.changeoverCost[initProd[v]][nodeProduct[j]];
                } else if (nodeKind[i] == NODE_DROP && nodeKind[j] == NODE_DROP && nodeProduct[i] != nodeProduct[j]) {
                    setup += PENALTY_INF; // Cannot change product between drops without visiting a depot
                }

                arcCost[i][j] = base + setup;
            }
        }
        return arcCost;
    }
}
