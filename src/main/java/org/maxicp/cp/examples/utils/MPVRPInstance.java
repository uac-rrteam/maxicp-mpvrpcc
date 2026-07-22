package org.maxicp.cp.examples.utils;

import org.maxicp.util.io.InputReader;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Data parser and container for the Multi-Product Vehicle Routing Problem
 * with Changeover Costs (MPVRP-CC).
 *
 * <p>Reads instances from benchmark .dat files and builds node graphs.</p>
 *
 * @author Rosas Behoundja
 */
public class MPVRPInstance {

    // =========================================================================
    // Instance Metadata & Dimensions
    // =========================================================================
    public String uuid;
    public int nProds;           // Number of product types
    public int nDepotsRead;      // Number of physical depots in instance file
    public int nDepots;          // Number of physical depots
    public int nDepotProducts;  // Total depot-product nodes in graph (including copies)
    public int nGarages;        // Number of garages
    public int nStationsRead;   // Number of physical stations in instance file
    public int nStations;       // Number of physical stations
    public int nRequests;       // Total positive station demand requests
    public int nVehicules;      // Number of available vehicles

    /** Changeover cost matrix: changeoverCost[p1][p2] = cost of cleaning tank from product p1 to p2 */
    public double[][] changeoverCost;

    // Entity collections
    public Vehicule[] vehicules;
    public RequestNode[] requests;
    public DepotProductNode[] depotProducts;
    public GarageNode[] garages;
    public Depot[] depots;
    public Station[] stations;

    /** Global pairwise distance matrix across all graph nodes */
    public double[][] distMatrix;

    // =========================================================================
    // Data Record Definitions
    // =========================================================================

    /** Represents a vehicle with capacity, initial home garage, and starting product. */
    public record Vehicule(int id, double capacity, GarageNode startG, int initProd) {}

    /** Represents a physical depot with stocks for all products. */
    public record Depot(int id, double x, double y, double[] stocks) {}

    /** Represents a physical station with product demands. */
    public record Station(int id, double x, double y, double[] demand) {}

    /** Base interface for nodes in the execution graph. */
    public sealed interface MPVRPNode permits RequestNode, GarageNode, DepotProductNode {
        int id();         // 1-based entity ID from instance file
        int globalId();   // 0-based unique index in the distance matrix
        double x();
        double y();
        int type();       // 0 = Request, 1 = Garage, 2 = Depot-Product

        default double distance(MPVRPNode o) {
            double dx = this.x() - o.x();
            double dy = this.y() - o.y();
            return Math.hypot(dx, dy);
        }
    }

    /** Garage node representing vehicle departure and arrival location. */
    public record GarageNode(int id, int globalId, double x, double y) implements MPVRPNode {
        @Override public int type() { return 1; }
    }

    /** Request node representing a specific station's demand for a single product. */
    public record RequestNode(int id, int globalId, double x, double y, int product, double demand) implements MPVRPNode {
        @Override public int type() { return 0; }
    }

    /** Depot-product node representing loading capability for a specific product at a depot. */
    public record DepotProductNode(int id, int globalId, double x, double y, int product, double stock) implements MPVRPNode {
        @Override public int type() { return 2; }
    }

    // =========================================================================
    // Factory Reader Methods
    // =========================================================================

    /**
     * Reads an MPVRP-CC instance file with default 3 node copies per depot-product pair
     * to support multiple visits per depot.
     */
    public static MPVRPInstance read(String filename) throws IOException {
        return read(filename, 3);
    }

    /**
     * Reads an MPVRP-CC instance file.
     *
     * @param filename Path to the .dat instance file.
     * @param depotCopies Number of node copies per depot-product pair to allow multiple mini-route visits.
     * @return Parsed MPVRPInstance object.
     */
    public static MPVRPInstance read(String filename, int depotCopies) throws IOException {
        MPVRPInstance inst = new MPVRPInstance();
        InputReader reader = new InputReader(filename);

        // Header: UUID line
        reader.getString(); // "#"
        inst.uuid = reader.getString();

        // Instance dimensions
        inst.nProds = reader.getInt();
        inst.nDepotsRead = reader.getInt();
        inst.nGarages = reader.getInt();
        inst.nStationsRead = reader.getInt();
        inst.nVehicules = reader.getInt();
        inst.nDepots = inst.nDepotsRead;
        inst.nStations = inst.nStationsRead;

        // Changeover cost matrix (nProds x nProds)
        inst.changeoverCost = reader.getDoubleMatrix(inst.nProds, inst.nProds);

        // Vehicle fleet initialization
        int[] homeGarageIds = new int[inst.nVehicules];
        inst.vehicules = new Vehicule[inst.nVehicules];

        for (int i = 0; i < inst.nVehicules; i++) {
            int id = reader.getInt();
            double capacity = reader.getDouble();
            homeGarageIds[i] = reader.getInt();
            int initProd = reader.getInt() - 1; // Normalize to 0-based index [0..nProds-1]
            inst.vehicules[i] = new Vehicule(id, capacity, null, initProd);
        }

        int currentGlobalId = 0;

        // Physical Depots and Depot-Product Nodes (with copies)
        inst.depots = new Depot[inst.nDepotsRead];
        List<DepotProductNode> depotList = new ArrayList<>();
        for (int i = 0; i < inst.nDepotsRead; i++) {
            int id = reader.getInt();
            double x = reader.getDouble();
            double y = reader.getDouble();
            double[] stocks = new double[inst.nProds];

            for (int p = 0; p < inst.nProds; p++) {
                double stock = reader.getDouble();
                stocks[p] = stock;
                if (stock > 0) {
                    for (int c = 0; c < depotCopies; c++) {
                        depotList.add(new DepotProductNode(id, currentGlobalId++, x, y, p, stock));
                    }
                }
            }
            inst.depots[i] = new Depot(id, x, y, stocks);
        }

        inst.depotProducts = depotList.toArray(new DepotProductNode[0]);
        inst.nDepotProducts = inst.depotProducts.length;

        // Garages
        inst.garages = new GarageNode[inst.nGarages];
        for (int i = 0; i < inst.nGarages; i++) {
            inst.garages[i] = new GarageNode(reader.getInt(), currentGlobalId++, reader.getDouble(), reader.getDouble());
        }

        // Physical Stations and Station Request Nodes
        inst.stations = new Station[inst.nStationsRead];
        List<RequestNode> requestList = new ArrayList<>();
        for (int i = 0; i < inst.nStationsRead; i++) {
            int id = reader.getInt();
            double x = reader.getDouble();
            double y = reader.getDouble();
            double[] demands = new double[inst.nProds];

            for (int p = 0; p < inst.nProds; p++) {
                double demand = reader.getDouble();
                demands[p] = demand;
                if (demand > 0) {
                    requestList.add(new RequestNode(id, currentGlobalId++, x, y, p, demand));
                }
            }
            inst.stations[i] = new Station(id, x, y, demands);
        }

        inst.requests = requestList.toArray(new RequestNode[0]);
        inst.nRequests = inst.requests.length;

        // Link vehicles to home garage nodes and compute pairwise distances
        inst.linkVehicles(homeGarageIds);
        inst.buildDistanceMatrix();

        return inst;
    }

    private void linkVehicles(int[] garageIds) {
        for (int i = 0; i < nVehicules; i++) {
            for (GarageNode g : garages) {
                if (g.id() == garageIds[i]) {
                    Vehicule v = vehicules[i];
                    vehicules[i] = new Vehicule(v.id(), v.capacity(), g, v.initProd());
                    break;
                }
            }
        }
    }

    private void buildDistanceMatrix() {
        List<MPVRPNode> allNodes = new ArrayList<>();
        allNodes.addAll(Arrays.asList(depotProducts));
        allNodes.addAll(Arrays.asList(garages));
        allNodes.addAll(Arrays.asList(requests));

        int size = allNodes.size();
        distMatrix = new double[size][size];

        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                distMatrix[i][j] = allNodes.get(i).distance(allNodes.get(j));
            }
        }
    }

    /** Display summary of instance metrics. */
    public void display() {
        System.out.println("UUID: " + uuid);
        System.out.printf("Prods:%d DepotLines:%d DepotProducts:%d Garages:%d Stations:%d Requests:%d Vehicules:%d%n",
                nProds, nDepotsRead, nDepotProducts, nGarages, nStationsRead, nRequests, nVehicules);
    }
}