package org.maxicp.cp.examples.utils;

import org.maxicp.util.io.InputReader;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Parser for MPVRP-CC problem instances from
 * <a href="https://github.com/uac-rrteam/instances-mpvrp-cc">github</a>.
 *
 * <h3>File Format:</h3>
 * <pre>
 * # &lt;uuid&gt;
 * NbProducts NbDepots NbGarages NbStations NbVehicles
 * &lt;product transition matrix&gt;
 * &lt;vehicles&gt;
 * &lt;depots&gt;
 * &lt;garages&gt;
 * &lt;service stations&gt;
 * </pre>
 *
 * @author Rosas Behoundja
 */
public class MPVRPInstance {

    public String uuid;
    public int nProds;           // Number of product types
    public int nDepotsRead;      // Number of physical depots in instance file
    public int nDepots;          // Number of physical depots
    public int nDepotProducts;  // Total positive-stock depot-product nodes in graph
    public int nGarages;        // Number of garages
    public int nStationsRead;   // Number of physical stations in instance file
    public int nStations;       // Number of physical stations
    public int nRequests;       // Total positive station demand requests
    public int nVehicules;      // Number of available vehicles

    /** Changeover cost matrix: changeoverCost[p1][p2] = cost of cleaning tank from product p1 to p2 */
    public int[][] changeoverCost;

    // Entity collections
    public Vehicule[] vehicules;
    public RequestNode[] requests;
    public DepotProductNode[] depotProducts;
    public GarageNode[] garages;
    public Depot[] depots;
    public Station[] stations;

    /** Global pairwise distance matrix across all graph nodes */
    public int[][] distMatrix;

    public enum NodeType {
        STATION,
        GARAGE,
        DEPOT
    }

    /** Represents a vehicle with capacity, initial home garage, and starting product. */
    public record Vehicule(int id, int capacity, GarageNode startG, int initProd) {}

    /** Represents a physical depot with stocks for all products. */
    public record Depot(int id, int x, int y, int[] stocks) {}

    /** Represents a physical station with product demands. */
    public record Station(int id, int x, int y, int[] demand) {}

    /** Base interface for nodes in the execution graph. */
    public sealed interface MPVRPNode permits RequestNode, GarageNode, DepotProductNode {
        int id();         // 1-based entity ID from instance file
        int globalId();   // 0-based unique index in the distance matrix
        int x();
        int y();
        NodeType type();

        default int distance(MPVRPNode o) {
            long dx = (long) this.x() - o.x();
            long dy = (long) this.y() - o.y();
            return (int) Math.round(Math.hypot(dx, dy));
        }
    }

    /** Garage node representing vehicle departure and arrival location. */
    public record GarageNode(int id, int globalId, int x, int y) implements MPVRPNode {
        @Override public NodeType type() { return NodeType.GARAGE; }
    }

    /** Request node representing a specific station's demand for a single product. */
    public record RequestNode(int id, int globalId, int x, int y, int product, int demand) implements MPVRPNode {
        @Override public NodeType type() { return NodeType.STATION; }
    }

    /** Depot-product node representing loading capability for a specific product at a depot. */
    public record DepotProductNode(int id, int globalId, int x, int y, int product, int stock) implements MPVRPNode {
        @Override public NodeType type() { return NodeType.DEPOT; }
    }

    /** Reads an MPVRP-CC instance file. */
    public static MPVRPInstance read(String filename) throws IOException {
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
        inst.changeoverCost = reader.getIntMatrix(inst.nProds, inst.nProds);

        // Vehicle fleet initialization
        int[] homeGarageIds = new int[inst.nVehicules];
        inst.vehicules = new Vehicule[inst.nVehicules];

        for (int i = 0; i < inst.nVehicules; i++) {
            int id = reader.getInt();
            int capacity = reader.getInt();
            homeGarageIds[i] = reader.getInt();
            int initProd = reader.getInt() - 1; // Normalize to 0-based index [0..nProds-1]
            inst.vehicules[i] = new Vehicule(id, capacity, null, initProd);
        }

        int currentGlobalId = 0;

        // Physical Depots and Depot-Product Nodes
        inst.depots = new Depot[inst.nDepotsRead];
        List<DepotProductNode> depotList = new ArrayList<>();
        for (int i = 0; i < inst.nDepotsRead; i++) {
            int id = reader.getInt();
            int x = reader.getInt();
            int y = reader.getInt();
            int[] stocks = new int[inst.nProds];

            for (int p = 0; p < inst.nProds; p++) {
                int stock = reader.getInt();
                stocks[p] = stock;
                if (stock > 0) {
                    depotList.add(new DepotProductNode(id, currentGlobalId++, x, y, p, stock));
                }
            }
            inst.depots[i] = new Depot(id, x, y, stocks);
        }

        inst.depotProducts = depotList.toArray(new DepotProductNode[0]);
        inst.nDepotProducts = inst.depotProducts.length;

        // Garages
        inst.garages = new GarageNode[inst.nGarages];
        for (int i = 0; i < inst.nGarages; i++) {
            inst.garages[i] = new GarageNode(reader.getInt(), currentGlobalId++, reader.getInt(), reader.getInt());
        }

        // Physical Stations and Station Request Nodes
        inst.stations = new Station[inst.nStationsRead];
        List<RequestNode> requestList = new ArrayList<>();
        for (int i = 0; i < inst.nStationsRead; i++) {
            int id = reader.getInt();
            int x = reader.getInt();
            int y = reader.getInt();
            int[] demands = new int[inst.nProds];

            for (int p = 0; p < inst.nProds; p++) {
                int demand = reader.getInt();
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
        distMatrix = new int[size][size];

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
