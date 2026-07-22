package org.maxicp.cp.examples.raw.mpvrp;

import org.maxicp.cp.engine.constraints.Sum;
import org.maxicp.cp.engine.constraints.seqvar.Cumulative;
import org.maxicp.cp.engine.constraints.seqvar.Distance;
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

/**
 * Multi-Product VRP with Changeover Costs using Sequence Variables (CPSeqVar)
 *
 * @author Rosas Behoundja
 */
public class MPVRPSeqVar {

    private static final int SCALE = 100;
    private static final int NODE_PICKUP = 0; // depot visit
    private static final int NODE_DROP = 1;   // station visit
    private static final int NODE_GARAGE = 2;
    private static final int PENALTY_INF = 1_000_000;

    private record Request(int stationId, int stationGlobal, int depotId, int depotGlobal, int product, int quantity, double rawQuantity) {
    }

    private record RouteSolution(String visitLine, String productLine, int productChanges, double transitionCost, double distance) {
    }

    private record SolutionData(List<RouteSolution> routes, int usedVehicles, int productChanges, double transitionCost, double distance) {
    }

    public static void main(String[] args) throws Exception {
        String inputFile = args.length > 0 ? args[0] : "data/MPVRPCC/inst/MPVRP_007_s36_d8_p2.dat";
        String outputFile = args.length > 1 ? args[1] : null;

        MPVRPInstance inst = MPVRPInstance.read(inputFile);
        CPSolver cp = makeSolver();

        // ===================== data =====================
        int V = inst.nVehicules;
        int S = inst.nStations;
        int P = inst.nProds;

        int[] departure = new int[V];
        int[] initProd = new int[V];
        int[] cap = new int[V];
        for (int v = 0; v < V; v++) {
            departure[v] = inst.vehicules[v].startG().globalId();
            initProd[v] = inst.vehicules[v].initProd(); // Already 0-indexed from parser
            cap[v] = (int) Math.round(inst.vehicules[v].capacity() * SCALE);
        }

        List<Request> requests = buildRequests(inst);
        int R = requests.size();
        if (R == 0) {
            System.out.println("No positive station demand in the instance.");
            return;
        }

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
            starts[i] = i;
            ends[i] = R + i;
            load[i] = req.quantity();

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

        // ===================== decision variables =====================
        CPSeqVar[] route = new CPSeqVar[V];
        CPIntVar[] distV = new CPIntVar[V];
        CPIntVar[] loadV = new CPIntVar[V];
        CPIntVar[] prodV = new CPIntVar[V];

        for (int v = 0; v < V; v++) {
            int start = 2 * R + v;
            int end = 2 * R + V + v;
            route[v] = makeSeqVar(cp, nNodes, start, end);
            distV[v] = makeIntVar(cp, 0, maxArc * nNodes);
            loadV[v] = makeIntVar(cp, 0, totalDemand);
            prodV[v] = makeIntVar(cp, 0, P - 1);
            cp.post(eq(prodV[v], initProd[v]));

            nodeKind[start] = NODE_GARAGE;
            nodeKind[end] = NODE_GARAGE;
            nodeGlobalId[start] = departure[v];
            nodeGlobalId[end] = departure[v];
            startVehicle[start] = v;
        }

        CPIntVar sumDistance = sum(distV);

        // ===================== constraints =====================

        for (int v = 0; v < V; v++) {
            cp.post(new Distance(route[v], dist, distV[v]));
            // Capacity constraint over pickup/drop activities
            cp.post(new Cumulative(route[v], starts, ends, load, cap[v]));

            // Carried load by vehicle
            CPIntVar[] servedLoad = new CPIntVar[R];
            for (int i = 0; i < R; i++) {
                servedLoad[i] = mul(route[v].getNodeVar(ends[i]).isRequired(), load[i]);
            }
            cp.post(new Sum(servedLoad, loadV[v]));
        }

        // Each drop & pickup node pair belongs to exactly one vehicle route (or 0 if unserved)
        for (int i = 0; i < R; i++) {
            CPIntVar[] dropVisits = new CPIntVar[V];
            CPIntVar[] pickupVisits = new CPIntVar[V];
            for (int v = 0; v < V; v++) {
                dropVisits[v] = route[v].getNodeVar(ends[i]).isRequired();
                pickupVisits[v] = route[v].getNodeVar(starts[i]).isRequired();
                // Pickup and drop must be served by the same vehicle
                cp.post(eq(dropVisits[v], pickupVisits[v]));
            }
            // Exactly 1 vehicle serves each request
            cp.post(new Sum(dropVisits, 1));
        }

        // Depot Stock Constraint across all requests
        for (int d = 0; d < inst.nDepotsRead; d++) {
            int depotId = inst.depots[d].id();
            for (int p = 0; p < P; p++) {
                double stockVal = inst.depots[d].stocks()[p];
                if (stockVal <= 0) continue;
                int scaledStock = (int) Math.round(stockVal * SCALE);

                List<Integer> reqIndices = new ArrayList<>();
                for (int i = 0; i < R; i++) {
                    if (requests.get(i).depotId() == depotId && requests.get(i).product() == p) {
                        reqIndices.add(i);
                    }
                }
                if (!reqIndices.isEmpty()) {
                    int totalReqStock = reqIndices.stream().mapToInt(idx -> requests.get(idx).quantity()).sum();
                    if (totalReqStock > scaledStock) {
                        throw new IllegalArgumentException("Depot " + depotId + " stock for product " + p + " is insufficient!");
                    }
                }
            }
        }

        // ===================== insertion-based search =====================

        Integer[] routingNodes = IntStream.range(0, R).boxed().toArray(Integer[]::new);
        int[] insertions = new int[nNodes];

        Supplier<Runnable[]> branching = () -> {
            if (Arrays.stream(route).allMatch(CPSeqVar::isFixed)) {
                return EMPTY;
            }

            Integer node = selectMin(routingNodes,
                    n -> Arrays.stream(route).anyMatch(r -> r.isNode(ends[n], INSERTABLE)),
                    n -> Arrays.stream(route).mapToInt(r -> r.nInsert(ends[n])).sum());

            if (node == null) {
                return EMPTY;
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
                    if (!isFeasibleInsertion(dropNode, pred, succ, nodeKind, nodeProduct)) {
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
                int pred = insertions[0];
                int succ = fallbackRoute.memberAfter(pred);
                return branch(() -> {
                    cp.post(insert(fallbackRoute, pred, pickupNode));
                    cp.post(insert(fallbackRoute, pickupNode, dropNode));
                }, () -> cp.post(notBetween(fallbackRoute, pred, dropNode, succ)));
            }

            CPSeqVar chosen = bestRoute;
            int chosenPred = bestPred;
            int chosenSucc = bestSucc;
            return branch(() -> {
                cp.post(insert(chosen, chosenPred, pickupNode));
                cp.post(insert(chosen, pickupNode, dropNode));
            }, () -> cp.post(notBetween(chosen, chosenPred, dropNode, chosenSucc)));
        };

        // ===================== solve =====================

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
                System.out.println("Solution found! Objective: " + (objectiveValue / (double) SCALE));
                long elapsedMs = System.currentTimeMillis() - init;
                if (bestSolution[0] != null) {
                    String text = buildSolutionText(bestSolution[0], elapsedMs / 1000.0);
                    Path out = Path.of(outputFile != null ? outputFile : defaultOutputPath(inst));
                    try {
                        writeSolution(out, text);
                    } catch (IOException e) {
                        throw new RuntimeException(e);
                    }
                    System.out.println("Solution written to " + out.toAbsolutePath());
                }
            }
        });

        System.out.println("Starting search (CPSeqVar)...");
        SearchStatistics stats = search.optimize(objective);

        System.out.println(stats);
    }

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

    private static List<Request> buildRequests(MPVRPInstance inst) {
        int S = inst.nStations;
        int D = inst.nDepots;
        int P = inst.nProds;

        int[][] stock = new int[D][P];
        for (int d = 0; d < D; d++) {
            for (int p = 0; p < P; p++) {
                stock[d][p] = (int) Math.round(inst.depots[d].stocks()[p] * SCALE);
            }
        }

        List<Request> requests = new ArrayList<>();
        for (int s = 0; s < S; s++) {
            int stationGlobal = inst.stations[s].id();
            for (int p = 0; p < P; p++) {
                int qty = (int) Math.round(inst.stations[s].demand()[p] * SCALE);
                if (qty <= 0) {
                    continue;
                }
                int bestDepot = -1;
                double bestDist = Double.POSITIVE_INFINITY;
                for (int d = 0; d < D; d++) {
                    if (stock[d][p] < qty) {
                        continue;
                    }
                    int depotGlobal = inst.depots[d].id();
                    double dsd = inst.distMatrix[depotGlobal][stationGlobal];
                    if (dsd < bestDist) {
                        bestDist = dsd;
                        bestDepot = d;
                    }
                }
                if (bestDepot == -1) {
                    throw new IllegalArgumentException("No depot can supply station " + s + " for product " + p);
                }
                stock[bestDepot][p] -= qty;
                requests.add(new Request(inst.stations[s].id(), stationGlobal, inst.depots[bestDepot].id(), inst.depots[bestDepot].id(), p, qty, inst.stations[s].demand()[p]));
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
                    if (req.product() != currentProduct) {
                        cumulativeChangeCost += inst.changeoverCost[currentProduct][req.product()];
                        currentProduct = req.product();
                        productChanges++;
                    }
                    visitParts.add(formatDepotLabel(req.depotId(), req.rawQuantity()));
                    productParts.add(formatProductLabel(currentProduct, cumulativeChangeCost));
                } else if (node < 2 * nRequest) {
                    Request req = requests.get(node - nRequest);
                    visitParts.add(formatStationLabel(req.stationId(), req.rawQuantity()));
                    productParts.add(formatProductLabel(currentProduct, cumulativeChangeCost));
                } else {
                    int garageId = inst.vehicules[v].startG().id();
                    visitParts.add(String.valueOf(garageId));
                    productParts.add(formatProductLabel(currentProduct, cumulativeChangeCost));
                }
            }

            totalProductChanges += productChanges;
            totalTransitionCost += cumulativeChangeCost;
            totalDistance += routeDistance;

            routes.add(new RouteSolution(
                    (v + 1) + ": " + String.join(" - ", visitParts),
                    (v + 1) + ": " + String.join(" - ", productParts),
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

    private static String defaultOutputPath(MPVRPInstance inst) {
        return "Sol_MPVRP_" + sanitize(inst.uuid) + ".dat";
    }

    private static String sanitize(String s) {
        return s.replaceAll("[^a-zA-Z0-9._-]+", "_");
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

    private static int[][] buildArcCosts(MPVRPInstance inst,
                                         int[] nodeKind,
                                         int[] nodeProduct,
                                         int[] nodeGlobalId,
                                         int[] startVehicle,
                                         int[] initProd,
                                         int[] departure,
                                         int nRequest,
                                         int nVehicle) {
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
                int base = (int) Math.round(inst.distMatrix[nodeGlobalId[i]][nodeGlobalId[j]] * SCALE);
                int setup = 0;

                if (nodeKind[i] == NODE_DROP && nodeKind[j] == NODE_PICKUP) {
                    int p1 = nodeProduct[i];
                    int p2 = nodeProduct[j];
                    setup += (int) Math.round(inst.changeoverCost[p1][p2] * SCALE);
                } else if (nodeKind[i] == NODE_GARAGE && startVehicle[i] >= 0 && nodeKind[j] == NODE_PICKUP) {
                    int v = startVehicle[i];
                    setup += (int) Math.round(inst.changeoverCost[initProd[v]][nodeProduct[j]] * SCALE);
                } else if (nodeKind[i] == NODE_DROP && nodeKind[j] == NODE_DROP && nodeProduct[i] != nodeProduct[j]) {
                    setup += PENALTY_INF; // Cannot change product between drops without visiting a depot
                }

                arcCost[i][j] = base + setup;
            }
        }
        return arcCost;
    }
}
