package org.maxicp.cp.examples.utils;

import org.maxicp.cp.engine.core.CPSeqVar;
import org.maxicp.util.io.InputReader;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

import static org.maxicp.modeling.algebra.sequence.SeqStatus.MEMBER_ORDERED;

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

    /** Represents one station-product demand assigned to a supplying depot. */
    public record Request(int logicalRequest, int stationId, int stationGlobal, int depotId,
                          int depotGlobal, int product, int quantity) {}

    /** Text and metrics captured for one vehicle route. */
    public record RouteSolution(String visitLine, String productLine, int productChanges,
                                double transitionCost, double distance) {}

    /** Complete solution data ready to be written to a file. */
    public record SolutionData(List<RouteSolution> routes, int usedVehicles, int productChanges,
                               double transitionCost, double distance) {}

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

    public static SolutionData captureSolution(MPVRPInstance instance, List<Request> requests,
                                               CPSeqVar[] routes, int[] initialProducts,
                                               int nodeCount, int requestCount) {
        int[] nodes = new int[nodeCount];
        List<RouteSolution> routeSolutions = new ArrayList<>();
        int usedVehicles = 0;
        int totalProductChanges = 0;
        double totalTransitionCost = 0.0;
        double totalDistance = 0.0;

        for (int vehicle = 0; vehicle < routes.length; vehicle++) {
            int memberCount = routes[vehicle].fillNode(nodes, MEMBER_ORDERED);
            if (memberCount <= 2) {
                continue;
            }

            usedVehicles++;
            List<String> visits = new ArrayList<>();
            List<String> products = new ArrayList<>();

            int currentProduct = initialProducts[vehicle];
            double transitionCost = 0.0;
            int productChanges = 0;
            double distance = 0.0;

            for (int position = 0; position < memberCount; position++) {
                int node = nodes[position];
                int globalId = globalIdOf(node, instance, requests, vehicle, requestCount);

                if (position > 0) {
                    int previousNode = nodes[position - 1];
                    int previousGlobalId = globalIdOf(previousNode, instance, requests, vehicle, requestCount);
                    distance += instance.distMatrix[previousGlobalId][globalId];
                }

                if (node < requestCount) {
                    Request request = requests.get(node);
                    transitionCost += instance.changeoverCost[currentProduct][request.product()];
                    if (request.product() != currentProduct) {
                        productChanges++;
                    }
                    currentProduct = request.product();
                    visits.add(formatDepotLabel(request.depotId(), request.quantity()));
                    products.add(formatProductLabel(currentProduct, transitionCost));
                } else if (node < 2 * requestCount) {
                    Request request = requests.get(node - requestCount);
                    visits.add(formatStationLabel(request.stationId(), request.quantity()));
                    products.add(formatProductLabel(currentProduct, transitionCost));
                } else {
                    int garageId = instance.vehicules[vehicle].startG().id();
                    visits.add(String.valueOf(garageId));
                    if (position == 0) {
                        products.add(formatProductLabel(currentProduct, transitionCost));
                    }
                }
            }

            totalProductChanges += productChanges;
            totalTransitionCost += transitionCost;
            totalDistance += distance;

            int vehicleId = instance.vehicules[vehicle].id();
            routeSolutions.add(new RouteSolution(
                    vehicleId + ": " + String.join(" - ", visits),
                    vehicleId + ": " + String.join(" - ", products),
                    productChanges,
                    transitionCost,
                    distance));
        }

        return new SolutionData(routeSolutions, usedVehicles, totalProductChanges,
                totalTransitionCost, totalDistance);
    }

    public static String buildSolutionText(SolutionData solution, double elapsedSeconds) {
        StringBuilder text = new StringBuilder();
        for (RouteSolution route : solution.routes()) {
            text.append(route.visitLine()).append('\n');
            text.append(route.productLine()).append('\n').append('\n');
        }
        text.append(solution.usedVehicles()).append('\n');
        text.append(solution.productChanges()).append('\n');
        text.append(formatCost(solution.transitionCost())).append('\n');
        text.append(formatCost(solution.distance())).append('\n');
        text.append(readProcessorModel()).append('\n');
        text.append(String.format(Locale.US, "%.3f", elapsedSeconds)).append('\n');
        return text.toString();
    }

    public static void writeSolution(Path outputFile, String solutionText) throws IOException {
        Path outputDirectory = outputFile.toAbsolutePath().getParent();
        if (outputDirectory != null) {
            Files.createDirectories(outputDirectory);
        }
        Files.writeString(outputFile, solutionText);
    }

    public static String defaultOutputPath(String instancePath) {
        String instanceName = Path.of(instancePath).getFileName().toString();
        return instanceName.startsWith("MPVRP_")
                ? "Sol_" + instanceName.substring("MPVRP_".length())
                : "Sol_" + instanceName;
    }

    public static String formatDepotLabel(int depotId, double quantity) {
        return depotId + " [" + formatQuantity(quantity) + "]";
    }

    public static String formatStationLabel(int stationId, double quantity) {
        return stationId + " (" + formatQuantity(quantity) + ")";
    }

    public static String formatProductLabel(int product, double cumulativeCost) {
        return product + "(" + formatCost(cumulativeCost) + ")";
    }

    public static String formatQuantity(double value) {
        return formatNumber(value);
    }

    public static String formatCost(double value) {
        return String.format(Locale.US, "%.2f", value);
    }

    public static String formatNumber(double value) {
        double roundedValue = Math.rint(value);
        if (Math.abs(value - roundedValue) < 1e-9) {
            return Long.toString(Math.round(roundedValue));
        }
        return String.format(Locale.US, "%.2f", value);
    }

    public static String readProcessorModel() {
        try {
            List<String> cpuInfo = Files.readAllLines(Path.of("/proc/cpuinfo"));
            for (String line : cpuInfo) {
                if (line.startsWith("model name")) {
                    int separator = line.indexOf(':');
                    if (separator >= 0) {
                        return line.substring(separator + 1).trim();
                    }
                }
            }
        } catch (IOException ignored) {
            // Fall back to the JVM system properties on platforms without /proc/cpuinfo.
        }
        return System.getProperty("os.name") + " " + System.getProperty("os.arch");
    }

    public static int globalIdOf(int node, MPVRPInstance instance, List<Request> requests,
                                 int vehicle, int requestCount) {
        if (node < requestCount) {
            return requests.get(node).depotGlobal();
        }
        if (node < 2 * requestCount) {
            return requests.get(node - requestCount).stationGlobal();
        }
        return instance.vehicules[vehicle].startG().globalId();
    }

    /** Display summary of instance metrics. */
    public void display() {
        System.out.println("UUID: " + uuid);
        System.out.printf("Prods:%d DepotLines:%d DepotProducts:%d Garages:%d Stations:%d Requests:%d Vehicules:%d%n",
                nProds, nDepotsRead, nDepotProducts, nGarages, nStationsRead, nRequests, nVehicules);
    }
}
