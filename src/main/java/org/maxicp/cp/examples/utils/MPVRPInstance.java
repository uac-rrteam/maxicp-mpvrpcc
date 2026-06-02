package org.maxicp.cp.examples.utils;

import org.maxicp.util.io.InputReader;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class MPVRPInstance {

    public String uuid;
    public int nProds, nDepotsRead, nDepotProducts, nGarages, nStationsRead, nRequests, nVehicules;
    public double[][] changeoverCost;

    public Vehicule[] vehicules;
    public RequestNode[] requests;
    public DepotProductNode[] depotProducts;
    public GarageNode[] garages;

    public double[][] distMatrix;

    public record Vehicule(int id, double capacity, GarageNode startG, int initProd) {}

    public sealed interface MPVRPNode permits RequestNode, GarageNode, DepotProductNode {
        int id();
        int globalId(); // for the distance matrix mapping
        double x();
        double y();
        int type(); // 0=request, 1=garage, 2=depot-product

        default double distance(MPVRPNode o) {
            double dx = this.x() - o.x();
            double dy = this.y() - o.y();
            return Math.hypot(dx, dy);
        }
    }

    public record GarageNode(int id, int globalId, double x, double y) implements MPVRPNode {
        @Override public int type() { return 1;}
    }

    public record RequestNode(int id, int globalId, double x, double y, int product, double demand) implements MPVRPNode {
        @Override public int type() { return 0;}
    }

    public record DepotProductNode(int id, int globalId, double x, double y, int product, double stock) implements MPVRPNode {
        @Override public int type() { return 2;}
    }

    public static MPVRPInstance read(String filename) throws IOException {
        MPVRPInstance inst = new MPVRPInstance();
        InputReader reader = new InputReader(filename);

        // inst.uuid = reader.getString().replace("#", "");
        reader.getString();
        inst.uuid = reader.getString();

        inst.nProds = reader.getInt();
        inst.nDepotsRead = reader.getInt();
        inst.nGarages = reader.getInt();
        inst.nStationsRead = reader.getInt();
        inst.nVehicules = reader.getInt();

        inst.changeoverCost = reader.getDoubleMatrix(inst.nProds, inst.nProds);

        // vehicules fleet
        int[] homeGarageIds = new int[inst.nVehicules];
        inst.vehicules = new Vehicule[inst.nVehicules];

        for (int i = 0; i < inst.nVehicules; i++) {
            int id = reader.getInt();
            double capacity = reader.getDouble();
            homeGarageIds[i] = reader.getInt();
            int initProd = reader.getInt();
            inst.vehicules[i] = new Vehicule(id, capacity, null, initProd);
        }

        int currentGlobalId = 0;

        // Depot-product nodes
        List<DepotProductNode> depotList = new ArrayList<>();
        for (int i = 0; i < inst.nDepotsRead; i++) {
            int id = reader.getInt();
            double x = reader.getDouble();
            double y = reader.getDouble();

            for (int p = 0; p < inst.nProds; p++) {
                double stock = reader.getDouble();
                if (stock > 0) {
                    depotList.add(new  DepotProductNode(id, currentGlobalId++, x, y, p, stock));
                }
            }
        }

        inst.depotProducts = depotList.toArray(new DepotProductNode[0]);
        inst.nDepotProducts =  inst.depotProducts.length;

        // Garages = nodes where vehicules start and end their routes
        inst.garages = new GarageNode[inst.nGarages];
        for (int i = 0; i < inst.nGarages; i++) {
            inst.garages[i] = new GarageNode(reader.getInt(), currentGlobalId++, reader.getDouble(), reader.getDouble());
        }

        // Requests
        List<RequestNode> requestList = new ArrayList<>();
        for (int i = 0; i < inst.nStationsRead; i++) {
            int id = reader.getInt();
            double x = reader.getDouble();
            double y = reader.getDouble();

            for (int p = 0; p < inst.nProds; p++) {
                double demand = reader.getDouble();
                if (demand > 0) {
                    requestList.add(new  RequestNode(id, currentGlobalId++, x, y, p, demand));
                }
            }
        }

        inst.requests = requestList.toArray(new RequestNode[0]);
        inst.nRequests = inst.requests.length;

        // Post-processing
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

    public void display() {
        System.out.println("UUID: " + uuid);

        System.out.printf("Prods:%d DepotLines:%d DepotProducts:%d Garages:%d Stations:%d Requests:%d Vehicules:%d%n",
                nProds, nDepotsRead, nDepotProducts, nGarages, nStationsRead, nRequests, nVehicules);

        System.out.println("\nChangeover Costs:");
        for (int i = 0; i < nProds; i++) {
            for (int j = 0; j < nProds; j++) {
                System.out.print(changeoverCost[i][j] + "\t");
            }
            System.out.println();
        }

        System.out.println("\nVehicules:");
        for (Vehicule v : vehicules) {
            System.out.printf("ID:%d | Cap:%.1f | Garage:%d | InitProd:%d%n",
                    v.id(), v.capacity(), v.startG().id(), v.initProd());
        }

        System.out.println("\nDepot Products:");
        for (DepotProductNode d : depotProducts) {
            System.out.printf("Depot:%d | GlobalID:%d | Product:%d | Stock:%.1f | X:%.1f Y:%.1f%n",
                    d.id(), d.globalId(), d.product(), d.stock(), d.x(), d.y());
        }

        System.out.println("\nGarages:");
        for (GarageNode g : garages) {
            System.out.printf("ID:%d | GlobalID:%d | X:%.1f Y:%.1f%n",
                    g.id(), g.globalId(), g.x(), g.y());
        }

        System.out.println("\nRequests:");
        for (RequestNode r : requests) {
            System.out.printf(
                    "Station:%d | GlobalID:%d | Product:%d | Demand:%.1f | X:%.1f Y:%.1f%n",
                    r.id(), r.globalId(), r.product(), r.demand(), r.x(), r.y());
        }
    }

    public static void main(String[] args) {
        try {
            MPVRPInstance inst = MPVRPInstance.read("data/MPVRPCC/small/MPVRP_S_011_s8_d1_p2.dat");
            inst.display();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}