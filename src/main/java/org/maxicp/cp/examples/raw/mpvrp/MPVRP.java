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
import java.util.Arrays;
import java.util.function.Supplier;

import static org.maxicp.cp.CPFactory.*;
import static org.maxicp.search.Searches.branch;
import static org.maxicp.search.Searches.selectMin;
import static org.maxicp.search.Searches.EMPTY;

public class MPVRP {

    private static final int GARAGE = 1;
    private static final int DEPOT = 2;
    private static final int REQUEST = 3;
    private static final int SCALE = 100;

    public static void main(String[] args) throws IOException {
        String instancePath = args.length > 0 ? args[0] : "data/MPVRPCC/one.dat";
        MPVRPInstance inst = MPVRPInstance.read(instancePath);
        CPSolver cp = makeSolver();

        // ===================== data =====================
        int V = inst.nVehicules;
        int P = inst.nProds;
        
        int nDepots = inst.nDepotProducts;
        int nGarages = inst.nGarages;
        int nRequests = inst.nRequests;
        int N = nDepots + nGarages + nRequests; // Total nodes

        // Node type and product arrays mapped by globalId
        int[] nodeType = new int[N];
        int[] nodeProduct = new int[N];
        for (int i = 0; i < nDepots; i++) {
            nodeType[i] = DEPOT;
            nodeProduct[i] = inst.depotProducts[i].product();
        }
        for (int i = 0; i < nGarages; i++) {
            nodeType[nDepots + i] = GARAGE;
            nodeProduct[nDepots + i] = -1;
        }
        for (int i = 0; i < nRequests; i++) {
            nodeType[nDepots + nGarages + i] = REQUEST;
            nodeProduct[nDepots + nGarages + i] = inst.requests[i].product();
        }

        int[] departure = new int[V]; 
        int[] initProd = new int[V]; 
        int[] cap = new int[V];
        for (int v = 0; v < V; v++) {
            departure[v] = inst.vehicules[v].startG().globalId();
            initProd[v] = inst.vehicules[v].initProd();
            if (initProd[v] >= 1 && initProd[v] <= P) initProd[v]--; // Normalize if 1-indexed
            cap[v] = (int) Math.round(inst.vehicules[v].capacity() * SCALE);
        }

        // ===================== decisions variables =====================
        CPIntVar[][] succ = new CPIntVar[V][N]; 
        CPIntVar[][] pred = new CPIntVar[V][N]; 
        CPIntVar[][] load = new CPIntVar[V][N]; 
        CPIntVar[][] delivered = new CPIntVar[V][N];
        
        CPIntVar[][] distStep = new CPIntVar[V][N];
        CPIntVar[][] rinseStep = new CPIntVar[V][N];

        for (int v = 0; v < V; v++) {
            succ[v] = makeIntVarArray(cp, N, N);
            pred[v] = makeIntVarArray(cp, N, N);
            load[v] = makeIntVarArray(cp, N, cap[v] + 1);
            delivered[v] = makeIntVarArray(cp, N, cap[v] + 1);
            distStep[v] = makeIntVarArray(cp, N, 10000000); // suitably large
            rinseStep[v] = makeIntVarArray(cp, N, 10000000);
        }

        // ===================== constraints =====================
        for (int v = 0; v < V; v++) {
            int G_v = departure[v];
            
            // SubCircuit and InversePerm
            cp.post(new SubCircuit(succ[v]));
            cp.post(new InversePerm(succ[v], pred[v]));

            // Garage constraints
            for (int g = nDepots; g < nDepots + nGarages; g++) {
                if (g != G_v) {
                    cp.post(eq(succ[v][g], g)); // Do not visit other garages
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
                    
                    if (typeI == GARAGE && i == G_v && typeJ == DEPOT) allowed = true;
                    if (typeI == DEPOT && typeJ == REQUEST && nodeProduct[i] == nodeProduct[j]) allowed = true;
                    if (typeI == REQUEST) {
                        if (typeJ == GARAGE && j == G_v) allowed = true;
                        if (typeJ == DEPOT) allowed = true;
                        if (typeJ == REQUEST && nodeProduct[i] == nodeProduct[j]) allowed = true;
                    }

                    if (!allowed) {
                        succ[v][i].remove(j);
                    } else {
                        edgeDist[i][j] = (int) Math.round(inst.distMatrix[i][j] * SCALE);
                        
                        // Changeover cost logic
                        if (typeI == GARAGE && typeJ == DEPOT) {
                            edgeRinse[i][j] = (int) Math.round(inst.changeoverCost[initProd[v]][nodeProduct[j]] * SCALE);
                        } else if (typeI == REQUEST && typeJ == DEPOT) {
                            edgeRinse[i][j] = (int) Math.round(inst.changeoverCost[nodeProduct[i]][nodeProduct[j]] * SCALE);
                        } else {
                            edgeRinse[i][j] = 0;
                        }
                    }
                }
                
                cp.post(new Element1D(edgeDist[i], succ[v][i], distStep[v][i]));
                cp.post(new Element1D(edgeRinse[i], succ[v][i], rinseStep[v][i]));
                
                if (nodeType[i] != REQUEST) {
                    cp.post(eq(delivered[v][i], 0));
                } else {
                    // Capacity / split deliveries logic
                    CPBoolVar isVisited = isNeq(succ[v][i], i);
                    CPIntVar maxDeliv = mul(isVisited, cap[v]);
                    cp.post(le(delivered[v][i], maxDeliv));
                    
                    // load[v][i] + delivered[v][i] == load[v][pred[v][i]]
                    CPIntVar loadBefore = makeIntVar(cp, 0, cap[v]);
                    cp.post(new Element1DVar(load[v], pred[v][i], loadBefore));
                    cp.post(sum(new CPIntVar[]{load[v][i], delivered[v][i]}, loadBefore));
                }
            }
        }

        // Demand satisfaction
        for (int r = 0; r < nRequests; r++) {
            int idx = nDepots + nGarages + r;
            int scaledDemand = (int) Math.round(inst.requests[r].demand() * SCALE);
            CPIntVar[] delivAllV = new CPIntVar[V];
            for (int v = 0; v < V; v++) {
                delivAllV[v] = delivered[v][idx];
            }
            cp.post(sum(delivAllV, scaledDemand));
        }

        // Objectives
        CPIntVar[] allDist = Arrays.stream(distStep).flatMap(Arrays::stream).toArray(CPIntVar[]::new);
        CPIntVar[] allRinse = Arrays.stream(rinseStep).flatMap(Arrays::stream).toArray(CPIntVar[]::new);
        CPIntVar totalDist = sum(allDist);
        CPIntVar totalRinse = sum(allRinse);
        CPIntVar objectiveVar = sum(totalDist, totalRinse);

        // ===================== search =====================
        CPIntVar[] allSuccVars = Arrays.stream(succ).flatMap(Arrays::stream).toArray(CPIntVar[]::new);
        
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
        
        search.onSolution(() -> {
            System.out.println("Solution found! Objective: " + (objectiveVar.min() / (double)SCALE));
        });

        System.out.println("Starting search...");
        SearchStatistics stats = search.optimize(obj);
        System.out.println(stats);
    }
}
