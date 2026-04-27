package org.maxicp.cp.examples.raw.mpvrp;

import org.maxicp.cp.engine.constraints.*;
import org.maxicp.cp.engine.core.CPIntVar;
import org.maxicp.cp.engine.core.CPSolver;
import org.maxicp.cp.examples.utils.MPVRPInstance;

import java.io.IOException;
import java.util.Arrays;
import java.util.Set;
import java.util.stream.IntStream;


import static java.util.stream.Collectors.toSet;
import static org.maxicp.cp.CPFactory.*;


/**
 * Multi-Product VRP with Changeover Costs
 *
 * @author Rosas Behoundja
 */
public class MPVRP {

    public static void main(String[] args) throws IOException {
        MPVRPInstance inst = MPVRPInstance.read("data/MPVRPCC/one.dat");
        CPSolver cp = makeSolver();

        // ===================== data =====================
        int V = inst.nVehicules;
        int S = inst.nStations;
        int G = inst.nGarages;
        int D = inst.nDepots;
        int P = inst.nProds;

        int N = S * P + D * P + G; // number of nodes per vehicules
        double[][] distMatrix = inst.distMatrix;
        double[][] changeCost = inst.changeoverCost; // changeover cost between products p and p' (0 if p=p')

        int[] departure = new int[V]; // starting garage for vehicule v
        int[] initProd = new int[V]; // initial prod assigned to vehicule v
        double[] cap = new double[V]; // capacity for each vehicule v
        for (int v = 0; v < V; v++) {
            departure[v] = inst.vehicules[v].startG().globalId();
            initProd[v] = inst.vehicules[v].initProd();
            cap[v] = inst.vehicules[v].capacity();
        }

        double[][] demand = new double[S][P]; // demand of station S for product P
        for (int s = 0; s < S; s++) {
            System.arraycopy(inst.stations[s].demand(), 0, demand[s], 0, P);
        }

        double[][] stocks = new double[D][P]; // stock of depot D for product P
        for (int d = 0; d < D; d++) {
            System.arraycopy(inst.depots[d].stocks(), 0, stocks[d], 0, P);
        }

        Set<Integer> stationIds = IntStream.range(0, S).map(s -> inst.stations[s].globalId()).boxed().collect(toSet());
        Set<Integer> depotIds = IntStream.range(0, D).map(d -> inst.depots[d].globalId()).boxed().collect(toSet());
        Set<Integer> garageIds = IntStream.range(0, G).map(g -> inst.garages[g].globalId()).boxed().collect(toSet());

        // ===================== decisions variables =====================
        CPIntVar[][] succ = new CPIntVar[V][N]; // succesor of node n on vehicule's v route
        CPIntVar[][] pred = new CPIntVar[V][N]; // predecessor of node n on vehicule's v route
        CPIntVar[][] prod = new CPIntVar[V][N]; // product transported by vehicule v after visiting node n
        CPIntVar[][] load = new CPIntVar[V][N]; // load of vehicule v after visiting node n
        CPIntVar[][] rinse = new CPIntVar[V][N]; // total change-over cost of vehicule v after visiting node n

        for (int v = 0; v < V; v++) {
            for (int n = 0; n < N; n++) {
                succ[v][n] = makeIntVar(cp, 0, N - 1);
                pred[v][n] = makeIntVar(cp, 0, N - 1);
                prod[v][n] = makeIntVar(cp, 0, P - 1);
                load[v][n] = makeIntVar(cp, 0, (int) cap[v]);
                rinse[v][n] = makeIntVar(cp, 0, (int) Arrays.stream(changeCost).flatMapToDouble(Arrays::stream).max().orElse(0) * N);
            }
        }

         // ===================== constraints =====================
        for (int v = 0; v < V; v++) {
            // each car start at its departure garage
            cp.post(eq(pred[v][0], departure[v]));
            // each car start with initProd
            cp.post(eq(prod[v][0], initProd[v]));
            // initial load of each car = 0
            cp.post(eq(load[v][0], 0));
            // initial changeover cost of each car = 0
            cp.post(eq(rinse[v][0], 0));
            // final node of each car must be a garage (not necessarily the same as the departure garage)
            // cp.post(new In(pred[v][N - 1], garageIds));
            cp.post(among(succ[v], garageIds, 1));



            // the second node must be a depot

            for  (int n = 0; n < N; n++) {
                // link pred and succ nodes
                cp.post(new InversePerm(succ[v], pred[v]));
            }

        }
    }
}
