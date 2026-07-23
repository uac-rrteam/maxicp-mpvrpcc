package org.maxicp.cp.examples.raw.mpvrp;

import org.maxicp.cp.engine.constraints.*;
import org.maxicp.cp.engine.core.CPBoolVar;
import org.maxicp.cp.engine.core.CPIntVar;
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

import static org.maxicp.cp.CPFactory.*;
import static org.maxicp.search.Searches.branch;
import static org.maxicp.search.Searches.selectMin;
import static org.maxicp.search.Searches.EMPTY;

/**
 * Multi-Product Vehicle Routing Problem with Changeover Cost (MPVRP-CC)
 *
 * @author Rosas Behoundja
 */
public class MPVRP {

    // Node Type Constants
    private static final int GARAGE = 1;
    private static final int DEPOT = 2;
    private static final int REQUEST = 3;

    /** Scaling factor to convert floating-point demands/costs into integer CP domains. */
    private static final int SCALE = 100;

    public static void main(String[] args) throws IOException {
        String instancePath = args.length > 0 ? args[0] : "data/MPVRPCC/inst/MPVRP_036_s5_d6_p3.dat";
        MPVRPInstance inst = MPVRPInstance.read(instancePath);
        CPSolver cp = makeSolver();

        int V = inst.nVehicules;
        int P = inst.nProds;

        int nDepots = inst.nDepotProducts;
        int nGarages = inst.nGarages;
        int nRequests = inst.nRequests;
        int N = nDepots + nGarages + nRequests; // Total node count in decision space

        // Node metadata arrays mapped by globalId
        int[] nodeType = new int[N];
        int[] nodeProduct = new int[N];
        int[] nodeDepotId = new int[N];
        Arrays.fill(nodeDepotId, -1);

        // Indices [0 .. nDepots - 1]: Depot-Product nodes
        for (int i = 0; i < nDepots; i++) {
            nodeType[i] = DEPOT;
            nodeProduct[i] = inst.depotProducts[i].product();
            nodeDepotId[i] = inst.depotProducts[i].id();
        }
        // Indices [nDepots .. nDepots + nGarages - 1]: Garage nodes
        for (int i = 0; i < nGarages; i++) {
            nodeType[nDepots + i] = GARAGE;
            nodeProduct[nDepots + i] = -1;
        }
        // Indices [nDepots + nGarages .. N - 1]: Station Request nodes
        for (int i = 0; i < nRequests; i++) {
            nodeType[nDepots + nGarages + i] = REQUEST;
            nodeProduct[nDepots + nGarages + i] = inst.requests[i].product();
        }

        // Indicator array: 1 if node is a Request, 0 otherwise
        int[] isRequestNode = new int[N];
        for (int i = 0; i < N; i++) {
            isRequestNode[i] = (nodeType[i] == REQUEST) ? 1 : 0;
        }

        // Vehicle fleet metadata
        int[] departure = new int[V];
        int[] initProd = new int[V];
        int[] cap = new int[V];
        for (int v = 0; v < V; v++) {
            departure[v] = inst.vehicules[v].startG().globalId();
            initProd[v] = inst.vehicules[v].initProd(); // 0-indexed product ID
            cap[v] = (int) Math.round(inst.vehicules[v].capacity() * SCALE);
        }

        // decisions variables
        CPIntVar[][] succ = new CPIntVar[V][N];      // Successor node of node i on vehicle v's route
        CPIntVar[][] pred = new CPIntVar[V][N];      // Predecessor node of node i on vehicle v's route
        CPIntVar[][] load = new CPIntVar[V][N];      // Remaining load after departing node i
        CPIntVar[][] delivered = new CPIntVar[V][N]; // Quantity delivered at node i by vehicle v

        CPIntVar[][] distStep = new CPIntVar[V][N];  // Distance of transition (i -> succ[v][i])
        CPIntVar[][] rinseStep = new CPIntVar[V][N]; // Changeover cost of transition (i -> succ[v][i])

        for (int v = 0; v < V; v++) {
            succ[v] = makeIntVarArray(cp, N, N);
            pred[v] = makeIntVarArray(cp, N, N);
            load[v] = makeIntVarArray(cp, N, cap[v] + 1);
            delivered[v] = makeIntVarArray(cp, N, cap[v] + 1);
            distStep[v] = makeIntVarArray(cp, N, 10_000_000);
            rinseStep[v] = makeIntVarArray(cp, N, 10_000_000);
        }

        // constraints
        for (int v = 0; v < V; v++) {
            int G_v = departure[v];

            // SubCircuit and InversePerm to enforce single valid cycle per vehicle
            cp.post(new SubCircuit(succ[v]));
            cp.post(new InversePerm(succ[v], pred[v]));

            // Prevent vehicle v from visiting garages belonging to other vehicles
            for (int g = nDepots; g < nDepots + nGarages; g++) {
                if (g != G_v) {
                    cp.post(eq(succ[v][g], g));
                }
            }

            int[][] edgeDist = new int[N][N];
            int[][] edgeRinse = new int[N][N];

            for (int i = 0; i < N; i++) {
                for (int j = 0; j < N; j++) {
                    if (i == j) {
                        edgeDist[i][j] = 0;
                        edgeRinse[i][j] = 0;
                        continue;
                    }

                    boolean allowed = false;
                    int typeI = nodeType[i];
                    int typeJ = nodeType[j];

                    // Transition Domain Rules:
                    // Garage -> Depot
                    // Depot -> Request (same product only)
                    // Request -> Request (same product) | Depot | Home Garage
                    if (typeI == GARAGE && i == G_v && typeJ == DEPOT) {
                        allowed = true;
                    } else if (typeI == DEPOT && typeJ == REQUEST && nodeProduct[i] == nodeProduct[j]) {
                        allowed = true;
                    } else if (typeI == REQUEST) {
                        if (typeJ == GARAGE && j == G_v) allowed = true;
                        if (typeJ == DEPOT) allowed = true;
                        if (typeJ == REQUEST && nodeProduct[i] == nodeProduct[j]) allowed = true;
                    }

                    if (!allowed) {
                        succ[v][i].remove(j);
                    } else {
                        edgeDist[i][j] = (int) Math.round(inst.distMatrix[i][j] * SCALE);

                        // Tank cleaning changeover cost when transitioning into a depot node
                        if (typeI == GARAGE && typeJ == DEPOT) {
                            edgeRinse[i][j] = (int) Math.round(inst.changeoverCost[initProd[v]][nodeProduct[j]] * SCALE);
                        } else if (typeI == REQUEST && typeJ == DEPOT) {
                            edgeRinse[i][j] = (int) Math.round(inst.changeoverCost[nodeProduct[i]][nodeProduct[j]] * SCALE);
                        } else {
                            edgeRinse[i][j] = 0;
                        }
                    }
                }

                // Channel transition distance & rinse cost to variables
                cp.post(new Element1D(edgeDist[i], succ[v][i], distStep[v][i]));
                cp.post(new Element1D(edgeRinse[i], succ[v][i], rinseStep[v][i]));

                // Reification: isVisited is 1 iff succ[v][i] != i
                CPBoolVar isVisited = isNeq(succ[v][i], i);

                if (nodeType[i] == GARAGE) {
                    cp.post(eq(delivered[v][i], 0));
                    cp.post(eq(load[v][i], 0));
                } else if (nodeType[i] == DEPOT) {
                    cp.post(eq(delivered[v][i], 0));
                    // Depot load is 0 if unvisited, and >= 1 if visited
                    cp.post(le(load[v][i], mul(isVisited, cap[v])));
                    cp.post(ge(load[v][i], isVisited));
                } else if (nodeType[i] == REQUEST) {
                    // Delivery bounds: 0 if unvisited, >= 1 if visited (no empty visits)
                    cp.post(le(delivered[v][i], mul(isVisited, cap[v])));
                    cp.post(ge(delivered[v][i], isVisited));

                    // Load conservation along request sequence: load[v][i] + delivered[v][i] == load[v][pred[v][i]]
                    CPIntVar loadBefore = makeIntVar(cp, 0, cap[v]);
                    cp.post(new Element1DVar(load[v], pred[v][i], loadBefore));
                    cp.post(sum(new CPIntVar[]{load[v][i], delivered[v][i]}, loadBefore));

                    // Mini-route end reset: remaining load must be 0 if next node is a DEPOT or GARAGE
                    CPBoolVar isNextRequest = makeBoolVar(cp);
                    cp.post(new Element1D(isRequestNode, succ[v][i], isNextRequest));
                    cp.post(le(load[v][i], mul(isNextRequest, cap[v])));
                }
            }
        }

        // Demand satisfaction constraint across fleet
        for (int r = 0; r < nRequests; r++) {
            int idx = nDepots + nGarages + r;
            int scaledDemand = (int) Math.round(inst.requests[r].demand() * SCALE);
            CPIntVar[] delivAllV = new CPIntVar[V];
            for (int v = 0; v < V; v++) {
                delivAllV[v] = delivered[v][idx];
            }
            cp.post(sum(delivAllV, scaledDemand));
        }

        // Total Depot Stock Constraint across all vehicles and depot node copies
        for (int d = 0; d < inst.nDepotsRead; d++) {
            int depotId = inst.depots[d].id();
            for (int p = 0; p < P; p++) {
                double stockVal = inst.depots[d].stocks()[p];
                if (stockVal <= 0) continue;
                int scaledStock = (int) Math.round(stockVal * SCALE);

                List<CPIntVar> depotLoads = new ArrayList<>();
                for (int i = 0; i < nDepots; i++) {
                    if (nodeDepotId[i] == depotId && nodeProduct[i] == p) {
                        for (int v = 0; v < V; v++) {
                            depotLoads.add(load[v][i]);
                        }
                    }
                }
                if (!depotLoads.isEmpty()) {
                    CPIntVar totalDepotLoad = sum(depotLoads.toArray(new CPIntVar[0]));
                    cp.post(le(totalDepotLoad, scaledStock));
                }
            }
        }

        // objective function + search strategy
        CPIntVar[] allDist = Arrays.stream(distStep).flatMap(Arrays::stream).toArray(CPIntVar[]::new);
        CPIntVar[] allRinse = Arrays.stream(rinseStep).flatMap(Arrays::stream).toArray(CPIntVar[]::new);
        CPIntVar totalDist = sum(allDist);
        CPIntVar totalRinse = sum(allRinse);
        CPIntVar objectiveVar = sum(totalDist, totalRinse);

        CPIntVar[] allSuccVars = Arrays.stream(succ).flatMap(Arrays::stream).toArray(CPIntVar[]::new);

        // First-fail branching on successor variables
        Supplier<Runnable[]> branching = () -> {
            CPIntVar xs = selectMin(allSuccVars,
                    x -> x.size() > 1,
                    x -> x.size());
            if (xs == null) return EMPTY;
            int val = xs.min();
            return branch(() -> cp.post(eq(xs, val)), () -> cp.post(neq(xs, val)));
        };

        DFSearch search = makeDfs(cp, branching);
        Objective obj = cp.minimize(objectiveVar);

        int[][] bestSucc = new int[V][N];
        int[][] bestDeliv = new int[V][N];
        double[] bestDist = {0};
        double[] bestRinse = {0};
        int[] bestChanges = {0};
        long t0 = System.currentTimeMillis();

        search.onSolution(() -> {
            bestDist[0] = totalDist.min() / (double) SCALE;
            bestRinse[0] = totalRinse.min() / (double) SCALE;

            int changes = 0;
            for (int v = 0; v < V; v++) {
                for (int i = 0; i < N; i++) {
                    bestSucc[v][i] = succ[v][i].min();
                    bestDeliv[v][i] = delivered[v][i].min();
                }

                int G_v = departure[v];
                if (bestSucc[v][G_v] == G_v) continue;
                int curr = G_v;
                int currentProd = initProd[v];
                curr = bestSucc[v][curr];
                while (curr != G_v) {
                    if (nodeType[curr] == DEPOT) {
                        int prod = nodeProduct[curr];
                        if (prod != currentProd) {
                            changes++;
                            currentProd = prod;
                        }
                    }
                    curr = bestSucc[v][curr];
                }
            }
            bestChanges[0] = changes;
            System.out.println("Solution found! Objective: " + (objectiveVar.min() / (double) SCALE));

            double elapsed = (System.currentTimeMillis() - t0) / 1000.0;
            String outFileName = "Sol_MPVRP_" + inst.uuid.replace("-", "_") + ".dat";
            writeSolution(outFileName, inst, bestSucc, bestDeliv, bestRinse[0], bestDist[0], bestChanges[0], elapsed, nodeType, nodeProduct, departure, initProd);
        });

        System.out.println("Starting search (Successor Formulation)...");
        SearchStatistics stats = search.optimize(obj);
        System.out.println(stats);
    }

    // help functions

    private static void writeSolution(String filename, MPVRPInstance inst, int[][] bestSucc, int[][] bestDelivered,
                                      double totalRinse, double totalDist, int nChanges, double elapsed,
                                      int[] nodeType, int[] nodeProduct, int[] departure, int[] initProd) {
        try {
            StringBuilder sb = new StringBuilder();
            int usedVehicles = 0;

            for (int v = 0; v < bestSucc.length; v++) {
                int G_v = departure[v];
                if (bestSucc[v][G_v] == G_v) continue;

                usedVehicles++;
                List<String> visitSeq = new ArrayList<>();
                List<String> prodSeq = new ArrayList<>();

                int curr = G_v;
                int currentProd = initProd[v];
                double cumulativeRinse = 0.0;

                visitSeq.add(String.valueOf(inst.garages[G_v - inst.nDepotProducts].id()));
                prodSeq.add(currentProd + "(" + formatCost(0.0) + ")");

                curr = bestSucc[v][curr];
                while (curr != G_v) {
                    int type = nodeType[curr];
                    if (type == DEPOT) {
                        int prod = nodeProduct[curr];
                        if (prod != currentProd) {
                            cumulativeRinse += inst.changeoverCost[currentProd][prod];
                            currentProd = prod;
                        }

                        double load = 0;
                        int look = bestSucc[v][curr];
                        while (look != G_v && nodeType[look] != DEPOT) {
                            if (nodeType[look] == REQUEST) {
                                load += bestDelivered[v][look] / (double) SCALE;
                            }
                            look = bestSucc[v][look];
                        }
                        visitSeq.add(inst.depotProducts[curr].id() + " [" + formatNumber(load) + "]");
                        prodSeq.add(currentProd + "(" + formatCost(cumulativeRinse) + ")");
                    } else if (type == REQUEST) {
                        int reqIdx = curr - inst.nDepotProducts - inst.nGarages;
                        double deliv = bestDelivered[v][curr] / (double) SCALE;
                        visitSeq.add(inst.requests[reqIdx].id() + " (" + formatNumber(deliv) + ")");
                        prodSeq.add(currentProd + "(" + formatCost(cumulativeRinse) + ")");
                    }
                    curr = bestSucc[v][curr];
                }

                visitSeq.add(String.valueOf(inst.garages[G_v - inst.nDepotProducts].id()));
                prodSeq.add(currentProd + "(" + formatCost(cumulativeRinse) + ")");

                sb.append(v + 1).append(": ").append(String.join(" - ", visitSeq)).append("\n");
                sb.append(v + 1).append(": ").append(String.join(" - ", prodSeq)).append("\n\n");
            }

            sb.append(usedVehicles).append("\n");
            sb.append(nChanges).append("\n");
            sb.append(formatCost(totalRinse)).append("\n");
            sb.append(formatCost(totalDist)).append("\n");
            sb.append(getProcessorModel()).append("\n");
            sb.append(String.format(Locale.US, "%.3f", elapsed)).append("\n");

            Files.writeString(Path.of(filename), sb.toString());
            System.out.println("Solution written to " + filename);
        } catch (Exception e) {
            e.printStackTrace();
        }
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

    private static String getProcessorModel() {
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
}
