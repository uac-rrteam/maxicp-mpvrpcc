package org.maxicp.cp.examples.modeling;

import org.maxicp.cp.engine.core.CPSolver;
import org.maxicp.cp.engine.core.CPIntVar;
import org.maxicp.search.DFSearch;
import org.maxicp.search.Objective;
import org.maxicp.search.SearchStatistics;
import org.maxicp.cp.CPFactory;
import org.maxicp.search.Searches;

import java.io.*;
import java.nio.file.*;
import java.util.*;

import static org.maxicp.cp.CPFactory.*;

/**
 * Multi-Product Vehicle Routing Problem with Changeover Cost <a href="https://ifri-ai-classes.github.io/MPVRP-CC/">(MPVRP-CC)</a>
 */
public class MPVRPCC {

    static class Instance {
        String uuid;
        int nbProducts, nbDepots, nbGarages, nbStations, nbVehicles;
        double[][] changeover;
        int[] vehicleCapacity, vehicleHomeGarage, vehicleInitProduct;
        double[] depotX, depotY;
        double[][] depotStock;
        double[] garageX, garageY;
        double[] stationX, stationY;
        double[][] stationDemand;
    }

    static final int DIST_SCALE = 100;

    // Structure interne pour unifier les positions du routage
    static class RoutingNode {
        int id, type; // 0=Demande, 1=Garage Départ, 2=Garage Arrivée, 3=Dépôt
        double x, y;
        int stationId, productId, quantity, depotId, vehicleId;

        RoutingNode(int id, int type, double x, double y) {
            this.id = id; this.type = type; this.x = x; this.y = y;
        }
    }

    public static void main(String[] args) throws IOException {

        Instance inst = parse("data/MPVRPCC/small/MPVRP_S_005_s7_d1_p3.dat");

        final int K = inst.nbVehicles;
        final int P = inst.nbProducts;
        final int D = inst.nbDepots;

        // ===================== 1. CREATION DES NOEUDS =====================
        List<RoutingNode> nodes = new ArrayList<>();
        int idCounter = 0;

        // 1a. Noeuds de demandes (Stations)
        int demandsStart = idCounter;
        for (int s = 0; s < inst.nbStations; s++) {
            for (int p = 0; p < P; p++) {
                if (inst.stationDemand[s][p] > 0) {
                    RoutingNode n = new RoutingNode(idCounter++, 0, inst.stationX[s], inst.stationY[s]);
                    n.stationId = s; n.productId = p; n.quantity = (int) inst.stationDemand[s][p];
                    nodes.add(n);
                }
            }
        }
        int demandsEnd = idCounter;
        int nDemands = demandsEnd - demandsStart;

        // 1b. Garages de Départ
        int startG = idCounter;
        for (int k = 0; k < K; k++) {
            int g = inst.vehicleHomeGarage[k];
            RoutingNode n = new RoutingNode(idCounter++, 1, inst.garageX[g], inst.garageY[g]);
            n.vehicleId = k;
            nodes.add(n);
        }

        // 1c. Garages d'Arrivée
        int endG = idCounter;
        for (int k = 0; k < K; k++) {
            int g = inst.vehicleHomeGarage[k];
            RoutingNode n = new RoutingNode(idCounter++, 2, inst.garageX[g], inst.garageY[g]);
            n.vehicleId = k;
            nodes.add(n);
        }

        // 1d. Noeuds de passages aux Dépôts (Optionnels)
        // On autorise par exemple un maximum de 3 recharges par camion pour tous les dépôts confondus
        int maxReloadsPerVehicle = 3;
        int depotOffset = idCounter;
        for (int k = 0; k < K; k++) {
            for (int r = 0; r < maxReloadsPerVehicle; r++) {
                for (int d = 0; d < D; d++) {
                    RoutingNode n = new RoutingNode(idCounter++, 3, inst.depotX[d], inst.depotY[d]);
                    n.depotId = d;
                    nodes.add(n);
                }
            }
        }
        int totalNodes = nodes.size();

        // ===================== 2. MATRICE DE DISTANCES ET COUTS =====================
        int[][] distance = new int[totalNodes][totalNodes];
        for (int i = 0; i < totalNodes; i++) {
            for (int j = 0; j < totalNodes; j++) {
                distance[i][j] = dist(nodes.get(i).x, nodes.get(i).y, nodes.get(j).x, nodes.get(j).y);
            }
        }

        int[][] changeoverCost = new int[P][P];
        for (int i = 0; i < P; i++) {
            for (int j = 0; j < P; j++) {
                changeoverCost[i][j] = (int) (inst.changeover[i][j] * DIST_SCALE);
            }
        }

        // ===================== 3. MODELE MAXICP =====================
        CPSolver cp = makeSolver();

        // Variables : next[i], vehicle[i], prod[i]
        // K désigne le "véhicule poubelle" pour les nœuds de dépôts non visités
        CPIntVar[] next = makeIntVarArray(cp, totalNodes, totalNodes);
        CPIntVar[] vehicle = makeIntVarArray(cp, totalNodes, K + 1);
        CPIntVar[] prod = makeIntVarArray(cp, totalNodes, P);
        CPIntVar[] isVisited = makeIntVarArray(cp, totalNodes, 2);

        // --- Contraintes sur les Noeuds de Demande ---
        for (int i = demandsStart; i < demandsEnd; i++) {
            cp.post(CPFactory.eq(isVisited[i], 1)); // Toujours visité
            cp.post(CPFactory.neq(next[i], i)); // Pas de boucle sur soi
            cp.post(CPFactory.lt(vehicle[i], K)); // Doit appartenir à un vrai véhicule
            cp.post(CPFactory.eq(prod[i], nodes.get(i).productId)); // Fixer le produit requis
        }

        // --- Contraintes de structure des véhicules ---
        for (int k = 0; k < K; k++) {
            int startNode = startG + k;
            int endNode = endG + k;

            cp.post(CPFactory.eq(isVisited[startNode], 1));
            cp.post(CPFactory.eq(isVisited[endNode], 1));

            cp.post(CPFactory.eq(vehicle[startNode], k));
            cp.post(CPFactory.eq(vehicle[endNode], k));

            cp.post(CPFactory.eq(prod[startNode], inst.vehicleInitProduct[k]));

            // Le garage de fin boucle sur lui-même (fin de route)
            cp.post(CPFactory.eq(next[endNode], endNode));
        }

        // --- Contraintes de transition (Flux) ---
        for (int i = 0; i < totalNodes; i++) {
            CPIntVar j = next[i];

            // Si le noeud est visité et que ce n'est pas le garage d'arrivée
            // il transmet son véhicule à son successeur
            if (nodes.get(i).type != 2) {
                cp.post(CPFactory.eq(vehicle[i], element(vehicle, j)));
            }

            // Si "i" n'est pas un dépôt, il ne peut pas changer le produit
            if (nodes.get(i).type != 3 && nodes.get(i).type != 2) {
                cp.post(CPFactory.eq(prod[i], element(prod, j)));
            }

            // Nœuds de dépôts non visités
            if (nodes.get(i).type == 3) {
                // Si successeur == i, alors le noeud n'est pas visité
                cp.post(CPFactory.eq(isVisited[i], neq(next[i], i).priority()));

                // Si non visité, on l'assigne au "véhicule poubelle" K
                // (Optionnel selon l'implémentation MaxiCP, mais évite la fragmentation)
            }
        }

        // ===================== 4. OBJECTIF =====================
        CPIntVar[] distArray = new CPIntVar[totalNodes];
        CPIntVar[] costArray = new CPIntVar[totalNodes];

        for (int i = 0; i < totalNodes; i++) {
            // Distance vers le prochain noeud
            distArray[i] = element(distance[i], next[i]);

            // Calcul du coût de nettoyage si i est un dépôt et est visité
            if (nodes.get(i).type == 3) {
                CPIntVar prodEntrant = prod[i];
                CPIntVar prodSortant = element(prod, next[i]); // Produit après recharge
                CPIntVar cleaningCost = element(changeoverCost, prodEntrant, prodSortant);
                costArray[i] = mul(cleaningCost, isVisited[i]);
            } else {
                costArray[i] = makeIntVar(cp, 0, 0);
            }
        }

        CPIntVar totalDist = sum(distArray);
        CPIntVar totalCost = sum(costArray);
        Objective objective = cp.minimize(sum(totalDist, totalCost));

        // ===================== 5. RECHERCHE =====================
        System.out.println("Lancement de la résolution...");

        long t0 = System.currentTimeMillis();
        // Stratégie de recherche basique (First Fail)
        DFSearch dfs = makeDfs(cp, Searches.firstFailBinary(next));

        dfs.onSolution(() -> {
            System.out.println("Nouvelle solution trouvée ! Coût = " + sum(totalDist, totalCost));
        });

        // Limite et Optimisation
        SearchStatistics stats = dfs.optimize(objective);
        System.out.println("Recherche terminée. Noeuds explorés: " + stats.numberOfNodes());
        // TODO : Afficher les sequences par vehicules
        System.out.println("Time: " + (System.currentTimeMillis() - t0) + "ms");
    }

    // =========================================================================
    // Méthodes utilitaires (Parse et Dist) fournies dans l'énoncé
    // =========================================================================

    public static Instance parse(String filename) throws IOException {
        Instance inst = new Instance();
        List<String> lines = new ArrayList<>();

        for (String raw : Files.readAllLines(Path.of(filename))) {
            String trimmed = raw.trim();
            if (!trimmed.isEmpty()) lines.add(trimmed);
        }

        int cursor = 0;
        inst.uuid = lines.get(cursor++).replace("#", "").trim();

        String[] dims = lines.get(cursor++).split("\\s+");
        inst.nbProducts  = Integer.parseInt(dims[0]);
        inst.nbDepots    = Integer.parseInt(dims[1]);
        inst.nbGarages   = Integer.parseInt(dims[2]);
        inst.nbStations  = Integer.parseInt(dims[3]);
        inst.nbVehicles  = Integer.parseInt(dims[4]);

        inst.changeover = new double[inst.nbProducts][inst.nbProducts];
        for (int i = 0; i < inst.nbProducts; i++) {
            String[] row = lines.get(cursor++).split("\\s+");
            for (int j = 0; j < inst.nbProducts; j++) {
                inst.changeover[i][j] = Double.parseDouble(row[j]);
            }
        }

        inst.vehicleCapacity   = new int[inst.nbVehicles];
        inst.vehicleHomeGarage = new int[inst.nbVehicles];
        inst.vehicleInitProduct = new int[inst.nbVehicles];

        int[][] vehicleRaw = new int[inst.nbVehicles][4];
        for (int k = 0; k < inst.nbVehicles; k++) {
            String[] tok = lines.get(cursor++).split("\\s+");
            vehicleRaw[k][0] = Integer.parseInt(tok[0]);
            vehicleRaw[k][1] = Integer.parseInt(tok[1]);
            vehicleRaw[k][2] = Integer.parseInt(tok[2]);
            vehicleRaw[k][3] = Integer.parseInt(tok[3]);
        }

        inst.depotX     = new double[inst.nbDepots];
        inst.depotY     = new double[inst.nbDepots];
        inst.depotStock = new double[inst.nbDepots][inst.nbProducts];
        int[] depotIds  = new int[inst.nbDepots];

        for (int d = 0; d < inst.nbDepots; d++) {
            String[] tok = lines.get(cursor++).split("\\s+");
            depotIds[d]    = Integer.parseInt(tok[0]);
            inst.depotX[d] = Double.parseDouble(tok[1]);
            inst.depotY[d] = Double.parseDouble(tok[2]);
            for (int p = 0; p < inst.nbProducts; p++) {
                inst.depotStock[d][p] = Double.parseDouble(tok[3 + p]);
            }
        }

        inst.garageX   = new double[inst.nbGarages];
        inst.garageY   = new double[inst.nbGarages];
        int[] garageIds = new int[inst.nbGarages];

        for (int g = 0; g < inst.nbGarages; g++) {
            String[] tok = lines.get(cursor++).split("\\s+");
            garageIds[g]    = Integer.parseInt(tok[0]);
            inst.garageX[g] = Double.parseDouble(tok[1]);
            inst.garageY[g] = Double.parseDouble(tok[2]);
        }

        inst.stationX      = new double[inst.nbStations];
        inst.stationY      = new double[inst.nbStations];
        inst.stationDemand = new double[inst.nbStations][inst.nbProducts];

        for (int s = 0; s < inst.nbStations; s++) {
            String[] tok = lines.get(cursor++).split("\\s+");
            inst.stationX[s] = Double.parseDouble(tok[1]);
            inst.stationY[s] = Double.parseDouble(tok[2]);
            for (int p = 0; p < inst.nbProducts; p++) {
                inst.stationDemand[s][p] = Double.parseDouble(tok[3 + p]);
            }
        }

        Map<Integer, Integer> garageIdToIndex = new HashMap<>();
        for (int g = 0; g < inst.nbGarages; g++) {
            garageIdToIndex.put(garageIds[g], g);
        }

        for (int k = 0; k < inst.nbVehicles; k++) {
            inst.vehicleCapacity[k]    = vehicleRaw[k][1];
            int rawGarageId = vehicleRaw[k][2];
            inst.vehicleHomeGarage[k]  = garageIdToIndex.getOrDefault(rawGarageId, 0);
            inst.vehicleInitProduct[k] = vehicleRaw[k][3] - 1;
        }

        return inst;
    }

    static int dist(double x1, double y1, double x2, double y2) {
        double dx = x1 - x2;
        double dy = y1 - y2;
        return (int) Math.round(Math.sqrt(dx * dx + dy * dy) * DIST_SCALE);
    }
}