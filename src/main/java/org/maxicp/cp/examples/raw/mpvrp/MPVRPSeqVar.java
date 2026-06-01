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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import static org.maxicp.cp.CPFactory.*;
import static org.maxicp.modeling.algebra.sequence.SeqStatus.INSERTABLE;
import static org.maxicp.modeling.algebra.sequence.SeqStatus.MEMBER;
import static org.maxicp.search.Searches.*;

public class MPVRPSeqVar {

    private static final int SCALE = 100;
    private static final int NODE_PICKUP = 0; // depot visit
    private static final int NODE_DROP = 1;   // station visit
    private static final int NODE_GARAGE = 2;

    private record Request(int station, int stationGlobal, int depotGlobal, int product, int quantity) {
    }

    public static void main(String[] args) throws Exception {
        MPVRPInstance inst = MPVRPInstance.read("data/MPVRPCC/one.dat");
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
            initProd[v] = normalizeProductId(inst.vehicules[v].initProd(), P);
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
        int maxArc = Arrays.stream(dist).flatMapToInt(Arrays::stream).max().orElse(0);
        int totalDemand = Arrays.stream(load).sum();

        // ===================== decision variables =====================
        CPSeqVar[] route = new CPSeqVar[V];
        CPIntVar[] distV = new CPIntVar[V];
        CPIntVar[] loadV = new CPIntVar[V];
        CPIntVar[] prodV = new CPIntVar[V];
        CPIntVar[][] visitedNV = new CPIntVar[V][S];

        for (int v = 0; v < V; v++) {
            int start = 2 * R + v;
            int end = 2 * R + V + v;
            route[v] = makeSeqVar(cp, nNodes, start, end);
            distV[v] = makeIntVar(cp, 0, maxArc * nNodes);
            loadV[v] = makeIntVar(cp, 0, totalDemand);
            // vehicle product domain: explicit variable kept to match the model interface.
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
            // Capacity while traversing pickup/drop activities.
            cp.post(new Cumulative(route[v], starts, ends, load, cap[v]));

            // Total carried quantity by vehicle (aggregated over served requests).
            CPIntVar[] servedLoad = new CPIntVar[R];
            for (int i = 0; i < R; i++) {
                servedLoad[i] = mul(route[v].getNodeVar(ends[i]).isRequired(), load[i]);
            }
            cp.post(new Sum(servedLoad, loadV[v]));

            // Bound the number of visits per station by |P| for each vehicle.
            for (int s = 0; s < S; s++) {
                final int station = s;
                int[] stationDrops = IntStream.range(0, R).filter(i -> requests.get(i).station() == station).map(i -> ends[i]).toArray();
                if (stationDrops.length == 0) {
                    visitedNV[v][s] = makeIntVar(cp, 0, 0);
                    continue;
                }
                CPIntVar[] stationVisits = new CPIntVar[stationDrops.length];
                for (int i = 0; i < stationDrops.length; i++) {
                    stationVisits[i] = route[v].getNodeVar(stationDrops[i]).isRequired();
                }
                visitedNV[v][s] = makeIntVar(cp, 0, P);
                cp.post(new Sum(stationVisits, visitedNV[v][s]));
            }
        }

        // Each pickup/drop/garage node belongs to exactly one route.
        for (int node = 0; node < nNodes; node++) {
            CPIntVar[] visits = new CPIntVar[V];
            for (int v = 0; v < V; v++) {
                visits[v] = route[v].getNodeVar(node).isRequired();
            }
            cp.post(new Sum(visits, 1));
        }

        // ===================== insertion-based search =====================

        Integer[] routingNodes = IntStream.range(0, 2 * R).boxed().toArray(Integer[]::new);
        int[] insertions = new int[nNodes];

        Supplier<Runnable[]> branching = () -> {
            if (Arrays.stream(route).allMatch(CPSeqVar::isFixed)) {
                return EMPTY;
            }

            Integer node = selectMin(routingNodes,
                    n -> Arrays.stream(route).anyMatch(r -> r.isNode(n, INSERTABLE)),
                    n -> Arrays.stream(route).mapToInt(r -> r.nInsert(n)).sum());

            if (node == null) {
                return EMPTY;
            }

            int bestDetour = Integer.MAX_VALUE;
            int bestPred = -1;
            int bestSucc = -1;
            CPSeqVar bestRoute = null;

            for (CPSeqVar r : route) {
                int nInsert = r.fillInsert(node, insertions);
                for (int i = 0; i < nInsert; i++) {
                    int pred = insertions[i];
                    int succ = r.memberAfter(pred);
                    if (!isFeasibleInsertion(node, pred, succ, nodeKind, nodeProduct)) {
                        continue;
                    }
                    int detour = dist[pred][node] + dist[node][succ] - dist[pred][succ];
                    if (detour < bestDetour) {
                        bestDetour = detour;
                        bestPred = pred;
                        bestSucc = succ;
                        bestRoute = r;
                    }
                }
            }

            if (bestRoute == null) {
                // Fallback: keep search complete even if the local insertion filter removes all preferred moves.
                CPSeqVar fallbackRoute = selectMin(route, r -> r.isNode(node, INSERTABLE), CPSeqVar::nNode);
                if (fallbackRoute == null) {
                    return EMPTY;
                }
                int nInsert = fallbackRoute.fillInsert(node, insertions);
                if (nInsert == 0) {
                    return EMPTY;
                }
                int pred = insertions[0];
                int succ = fallbackRoute.memberAfter(pred);
                return branch(() -> cp.post(insert(fallbackRoute, pred, node)),
                        () -> cp.post(notBetween(fallbackRoute, pred, node, succ)));
            }

            CPSeqVar chosen = bestRoute;
            int chosenPred = bestPred;
            int chosenSucc = bestSucc;
            return branch(() -> cp.post(insert(chosen, chosenPred, node)),
                    () -> cp.post(notBetween(chosen, chosenPred, node, chosenSucc)));
        };

        // ===================== solve =====================

        DFSearch search = makeDfs(cp, branching);
        Objective objective = cp.minimize(sumDistance);

        search.onSolution(() -> {
            System.out.printf("distance = %.2f%n", sumDistance.min() / (double) SCALE);
            for (int v = 0; v < V; v++) {
                CPSeqVar r = route[v];
                if (r.nNode(MEMBER) > 2) {
                    String path = r.membersOrdered(" -> ", n -> n != r.start() && n != r.end());
                    System.out.printf("v%d | prod=%d | load=%d | %s%n", v, prodV[v].min(), loadV[v].min(), path);
                }
            }
            System.out.println("-----------------");
        });

        SearchStatistics stats = search.optimize(objective);
        System.out.println(stats);
    }

    private static boolean isFeasibleInsertion(int node, int pred, int succ, int[] nodeKind, int[] nodeProduct) {
        if (nodeKind[node] == NODE_PICKUP) {
            // Two depot pickups cannot be adjacent.
            return nodeKind[pred] != NODE_PICKUP && nodeKind[succ] != NODE_PICKUP;
        }
        if (nodeKind[node] == NODE_DROP) {
            // Consecutive station drops must keep the same product.
            boolean leftOk = nodeKind[pred] != NODE_DROP || nodeProduct[pred] == nodeProduct[node];
            boolean rightOk = nodeKind[succ] != NODE_DROP || nodeProduct[succ] == nodeProduct[node];
            return leftOk && rightOk;
        }
        return true;
    }

    private static int normalizeProductId(int rawProd, int nProducts) {
        if (rawProd >= 0 && rawProd < nProducts) {
            return rawProd;
        }
        if (rawProd >= 1 && rawProd <= nProducts) {
            return rawProd - 1;
        }
        throw new IllegalArgumentException("Unexpected product id: " + rawProd + " (nProducts=" + nProducts + ")");
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
            int stationGlobal = inst.stations[s].globalId();
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
                    int depotGlobal = inst.depots[d].globalId();
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
                requests.add(new Request(s, stationGlobal, inst.depots[bestDepot].globalId(), p, qty));
            }
        }
        return requests;
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
                }

                if (nodeKind[i] == NODE_GARAGE && startVehicle[i] >= 0 && nodeKind[j] == NODE_PICKUP) {
                    int v = startVehicle[i];
                    setup += (int) Math.round(inst.changeoverCost[initProd[v]][nodeProduct[j]] * SCALE);
                }

                arcCost[i][j] = base + setup;
            }
        }
        return arcCost;
    }
}
