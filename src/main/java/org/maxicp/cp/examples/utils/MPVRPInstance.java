package org.maxicp.cp.examples.utils;

import org.maxicp.util.io.InputReader;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class MPVRPInstance {

    public String uuid;
    public int nProds, nDepots, nGarages, nStations, nVehicules;
    public double[][] changeoverCost;
    public Vehicule[] vehicules;
    public StationNode[] stations;
    public DepotNode[] depots;
    public GarageNode[] garages;
    public double[][] distMatrix;

    public record Vehicule(int id, double capacity, GarageNode startG, int initProd) {}

    public sealed interface MPVRPNode permits StationNode, GarageNode, DepotNode {
        int id();
        int globalId(); // For the distanceMatrix Mapping
        double x();
        double y();
        int type(); // 0=station, 1=garage, 2=depot

        // Compute distance
        default double distance(MPVRPNode o) {
            double dx = this.x() - o.x();
            double dy = this.y() - o.y();
            return Math.hypot(dx, dy);
        }
    }

    public record StationNode(int id, int globalId, double x, double y, double[] demand) implements MPVRPNode {
        @Override public int type() { return 0; }
    }

    public record GarageNode(int id, int globalId, double x, double y) implements MPVRPNode {
        @Override public int type() { return 1; }
    }

    public record DepotNode(int id, int globalId, double x, double y, double[] stocks) implements MPVRPNode {
        @Override public int type() { return 2; }
    }

    public static MPVRPInstance read(String filename) throws IOException {
        MPVRPInstance inst = new MPVRPInstance();
        InputReader reader = new InputReader(filename);

        // inst.uuid = reader.getString().replace("#", "");
        reader.getString();
        inst.uuid = reader.getString();
        inst.nProds = reader.getInt();
        inst.nDepots = reader.getInt();
        inst.nGarages = reader.getInt();
        inst.nStations = reader.getInt();
        inst.nVehicules = reader.getInt();

        inst.changeoverCost = reader.getDoubleMatrix(inst.nProds, inst.nProds);

        // Get the fleet
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
        // Get the depots
        inst.depots = new DepotNode[inst.nDepots];
        for (int i = 0; i < inst.nDepots; i++) {
            int id = reader.getInt();
            double x = reader.getDouble();
            double y = reader.getDouble();
            double[] stocks = new double[inst.nProds];
            for (int p = 0; p < inst.nProds; p++) {
                stocks[p] = reader.getDouble();
            }
            inst.depots[i] = new DepotNode(id, currentGlobalId++, x, y, stocks);
        }

        // Get the garages
        inst.garages = new GarageNode[inst.nGarages];
        for (int i = 0; i < inst.nGarages; i++) {
            inst.garages[i] = new GarageNode(reader.getInt(), currentGlobalId++, reader.getDouble(), reader.getDouble());
        }

        // Get the stations
        inst.stations = new StationNode[inst.nStations];
        for (int i = 0; i < inst.nStations; i++) {
            int id = reader.getInt();
            double x = reader.getDouble();
            double y = reader.getDouble();
            double[] demand = new double[inst.nProds];
            for (int p = 0; p < inst.nProds; p++) {
                demand[p] = reader.getDouble();
            }
            inst.stations[i] = new StationNode(id, currentGlobalId++, x, y, demand);
        }

        // post treatment
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
        allNodes.addAll(Arrays.asList(depots));
        allNodes.addAll(Arrays.asList(garages));
        allNodes.addAll(Arrays.asList(stations));

        int size = allNodes.size();
        this.distMatrix = new double[size][size];
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                distMatrix[i][j] = allNodes.get(i).distance(allNodes.get(j));
            }
        }
    }

    public void display() {
        System.out.println("UUID: " + uuid);

        System.out.printf("Prods:%d Depots:%d Garages:%d Stations:%d Vehicules:%d%n",
                nProds, nDepots, nGarages, nStations, nVehicules);

        for (int i = 0; i < nProds; i++) {
            for (int j = 0; j < nProds; j++) {
                System.out.print(changeoverCost[i][j] + "\t");
            }
            System.out.println();
        }

        for (Vehicule v : vehicules) {
            System.out.printf("ID:%d | Cap:%s | GarageID:%d | InitProd:%d%n",
                    v.id(), v.capacity(), v.startG().id(), v.initProd());
        }

        for (DepotNode d : depots) {
            System.out.printf("ID:%d | X:%.1f Y:%.1f | Stocks: ", d.id(), d.x(), d.y());
            for (double s : d.stocks()) System.out.print(s + " ");
            System.out.println();
        }

        for (GarageNode g : garages) {
            System.out.printf("ID:%d | X:%.1f Y:%.1f%n", g.id(), g.x(), g.y());
        }

        for (StationNode s : stations) {
            System.out.printf("ID:%d | X:%.1f Y:%.1f | Requests: ", s.id(), s.x(), s.y());
            for (double d : s.demand()) System.out.print(d + " ");
            System.out.println();
        }
    }

    public static void main(String[] args) {
        try {
            MPVRPInstance inst = MPVRPInstance.read("data/MPVRPCC/large/MPVRP_L_012_s176_d7_p11.dat");
            inst.display();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
