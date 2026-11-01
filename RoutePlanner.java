import java.io.FileDescriptor;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.charset.Charset;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;

public class RoutePlanner {

    private static final double WALK_SPEED_KMH = 5.0; // walking speed: 5 km/h
    private static final double EARTH_RADIUS_KM = 6371.0088; // mean earth radius for distance calculation
    private static final double EPSILON = 1e-9; // tolerance for floating point comparisons

    // Default search parameters
    // p_max: the longest optimal route in the network has 47 actions
    public static final int DEFAULT_P_MAX = 50;
    public static final double DEFAULT_TIMEOUT_S = 10.0;
    public static final double MAX_WALK_TO_STATION_MIN = 15.0; // max. walk start->station and station->destination
    // max. walk start->destination without the U-Bahn (2 x 15 min)
    public static final double MAX_DIRECT_WALK_MIN = 2 * MAX_WALK_TO_STATION_MIN;
    public static final int FALLBACK_STATIONS = 2; // nearest stations used when none is within walking range

    private static final String DATA_FILE = "berlin_ubahn_network.csv";

    public static class Coordinate { // geographic position of a place or station
        final double lat;
        final double lon;

        public Coordinate(double lat, double lon) {
            this.lat = lat;
            this.lon = lon;
        }
    }

    public static class Connection { // an edge in the graph
        final String target;
        final double duration;
        final String actionType;

        public Connection(String target, double duration, String actionType) {
            this.target = target;
            this.duration = duration;
            this.actionType = actionType;
        }
    }
    // A node in the A* search tree
    public static class SearchNode implements Comparable<SearchNode> {
        final String state;
        final double gCost;
        final double fCost;
        final SearchNode parent;
        final String actionTaken;
        final double stepCost;
        final int depth;

        public SearchNode(String state, double gCost, double fCost, SearchNode parent, String actionTaken, double stepCost, int depth) {
            this.state = state;
            this.gCost = gCost;
            this.fCost = fCost;
            this.parent = parent;
            this.actionTaken = actionTaken;
            this.stepCost = stepCost;
            this.depth = depth;
        }
        // Order by lowest f = g + h, ties broken by lower g
        @Override
        public int compareTo(SearchNode other) {
            if (Math.abs(this.fCost - other.fCost) > EPSILON) {
                return Double.compare(this.fCost, other.fCost);
            }
            return Double.compare(this.gCost, other.gCost);
        }
    }
    // Result and statistics of a search
    public static class SearchResult {
        public List<String> pathDetails = new ArrayList<>();
        public List<String> pathStates = new ArrayList<>(); // states of the solution, START..GOAL
        public List<String> pathActions = new ArrayList<>(); // action leading into each state
        public Set<String> expandedStates = new HashSet<>(); // for the SVG visualization
        public double totalTimeMinutes = 0.0;
        public int exploredStatesCount = 0;
        public int maxQueueSize = 0;
        public boolean hitDepthLimit = false;
        public boolean hitTimeLimit = false;
        public boolean startFallback = false; // no station within walking range of the start
        public boolean targetFallback = false; // no station within walking range of the destination
        public double maxWalkToStationMin;
        public String statusMessage = "";
        public Coordinate startCoord;
        public Coordinate targetCoord;
    }
    // The U-Bahn graph and station coordinates
    private final Map<String, List<Connection>> networkGraph = new HashMap<>();
    private final Map<String, Coordinate> nodeLocations = new HashMap<>();
    private double maximumMetroSpeed = WALK_SPEED_KMH;
    private boolean verbose = false; // -v: print every expanded path
    private boolean useHeuristic = true; // false: h = 0, i.e. uniform-cost search
    // Creates the planner and loads the network from the CSV file
    public RoutePlanner(Path csvFilePath) throws IOException {
        loadNetwork(csvFilePath);
    }
    // Reads all connections and stations from the CSV file
    private void loadNetwork(Path csvFilePath) throws IOException {
        List<String> records = Files.readAllLines(csvFilePath, StandardCharsets.UTF_8);

        for (int i = 1; i < records.size(); i++) {
            String currentLine = records.get(i).trim();
            if (currentLine.isEmpty()) continue;
            // Parse one CSV row
            String[] tokens = currentLine.split(";", -1);
            if (tokens.length < 10) continue;

            String origin = tokens[0].trim();
            String destination = tokens[1].trim();
            String action = tokens[2].trim();
            double duration = parseDecimal(tokens[3]);
            Coordinate originCoord = new Coordinate(parseDecimal(tokens[6]), parseDecimal(tokens[7]));
            Coordinate destCoord = new Coordinate(parseDecimal(tokens[8]), parseDecimal(tokens[9]));
            // Remember station coordinates
            registerLocation(origin, originCoord);
            registerLocation(destination, destCoord);
            // The CSV lists both directions as separate rows
            addGraphEdge(origin, destination, duration, action);
            // Fastest speed in the network, used by the heuristic
            if (duration > 0) {
                double distanceKm = calculateDistanceKm(originCoord, destCoord);
                double speedKmH = (distanceKm / duration) * 60.0;
                if (speedKmH > maximumMetroSpeed) {
                    maximumMetroSpeed = speedKmH;
                }
            }
        }
    }
    // Adds a directed edge to the graph
    private void addGraphEdge(String from, String to, double cost, String action) {
        networkGraph.computeIfAbsent(from, k -> new ArrayList<>()).add(new Connection(to, cost, action));
    }
    // Stores the coordinates of a node
    private void registerLocation(String key, Coordinate coord) {
        if (!key.isEmpty()) {
            nodeLocations.putIfAbsent(key, coord);
        }
    }
    // Accepts both decimal comma and decimal point
    private static double parseDecimal(String rawValue) {
        return Double.parseDouble(rawValue.replace(',', '.').trim());
    }
    // Great-circle distance in km (haversine formula)
    public static double calculateDistanceKm(Coordinate p1, Coordinate p2) {
        double dLat = Math.toRadians(p2.lat - p1.lat);
        double dLon = Math.toRadians(p2.lon - p1.lon);
        double lat1 = Math.toRadians(p1.lat);
        double lat2 = Math.toRadians(p2.lat);

        double a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
                Math.sin(dLon / 2) * Math.sin(dLon / 2) * Math.cos(lat1) * Math.cos(lat2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        return EARTH_RADIUS_KM * c;
    }
    // Walking time in minutes at 5 km/h
    public static double calculateWalkMinutes(Coordinate p1, Coordinate p2) {
        return (calculateDistanceKm(p1, p2) / WALK_SPEED_KMH) * 60.0;
    }
    // Admissible heuristic: straight-line distance / fastest speed in the network
    private double calculateHeuristic(Coordinate current, Coordinate destination) {
        if (!useHeuristic || current == null || destination == null) return 0.0;
        double distKm = calculateDistanceKm(current, destination);
        return (distKm / maximumMetroSpeed) * 60.0;
    }
    // Station name without line prefix (U2:Alexanderplatz -> Alexanderplatz)
    private static String stationName(String node) {
        return node.substring(node.indexOf(':') + 1);
    }
    // Line nodes within walking range
    private Set<String> nodesInWalkingRange(Coordinate pos, double maxWalkMin) {
        Set<String> nodes = new HashSet<>();
        for (Map.Entry<String, Coordinate> entry : nodeLocations.entrySet()) {
            if (calculateWalkMinutes(pos, entry.getValue()) <= maxWalkMin) {
                nodes.add(entry.getKey());
            }
        }
        return nodes;
    }
    // Line nodes of the nearest stations (fallback)
    private Set<String> nodesOfNearestStations(Coordinate pos, int count) {
        Map<String, Double> stationDistances = new HashMap<>();
        for (Map.Entry<String, Coordinate> entry : nodeLocations.entrySet()) {
            stationDistances.merge(stationName(entry.getKey()), calculateDistanceKm(pos, entry.getValue()), Math::min);
        }
        List<String> stations = new ArrayList<>(stationDistances.keySet());
        stations.sort(Comparator.comparingDouble(stationDistances::get));
        Set<String> nearest = new HashSet<>(stations.subList(0, Math.min(count, stations.size())));

        Set<String> nodes = new HashSet<>();
        for (String node : nodeLocations.keySet()) {
            if (nearest.contains(stationName(node))) {
                nodes.add(node);
            }
        }
        return nodes;
    }
    // Checks whether a state is already on the path (cycle check)
    private static boolean isOnPath(SearchNode node, String state) {
        for (SearchNode trace = node; trace != null; trace = trace.parent) {
            if (trace.state.equals(state)) return true;
        }
        return false;
    }
    // Checks whether the path contains at least one U-Bahn ride
    private static boolean hasTraveled(SearchNode node) {
        for (SearchNode trace = node; trace != null; trace = trace.parent) {
            if (trace.actionTaken.equals("travel")) return true;
        }
        return false;
    }
    // A* search from a start coordinate to a destination coordinate
    public SearchResult findRoute(Coordinate startPos, Coordinate targetPos, int maxDepth, double timeoutSeconds, double maxWalkToStationMin, double maxDirectWalkMin) {
        SearchResult result = new SearchResult();
        result.startCoord = startPos;
        result.targetCoord = targetPos;
        result.maxWalkToStationMin = maxWalkToStationMin;

        // Stations for the first and last walk (otherwise the nearest ones)
        Set<String> startNodes = nodesInWalkingRange(startPos, maxWalkToStationMin);
        if (startNodes.isEmpty()) {
            result.startFallback = true;
            startNodes = nodesOfNearestStations(startPos, FALLBACK_STATIONS);
        }
        Set<String> targetNodes = nodesInWalkingRange(targetPos, maxWalkToStationMin);
        if (targetNodes.isEmpty()) {
            result.targetFallback = true;
            targetNodes = nodesOfNearestStations(targetPos, FALLBACK_STATIONS);
        }

        PriorityQueue<SearchNode> openList = new PriorityQueue<>();
        Map<String, Double> visitedBestCosts = new HashMap<>();

        long startTimeNano = System.nanoTime();

        // Initial state
        openList.add(new SearchNode("START", 0.0, calculateHeuristic(startPos, targetPos), null, "START", 0.0, 0));
        visitedBestCosts.put("START|0", 0.0);
        result.maxQueueSize = 1;

        SearchNode goalNode = null;
        SearchNode bestGoalInOpen = null; // best generated solution, used if the time limit is hit

        while (!openList.isEmpty()) {
            double elapsedSeconds = (System.nanoTime() - startTimeNano) / 1e9;
            if (elapsedSeconds >= timeoutSeconds) { // time limit check
                result.hitTimeLimit = true;
                break;
            }

            SearchNode current = openList.poll();

            // Skip stale duplicates
            String stateDepthKey = current.state + "|" + current.depth;
            if (current.gCost > visitedBestCosts.getOrDefault(stateDepthKey, Double.POSITIVE_INFINITY) + EPSILON) {
                continue;
            }

            // Count expanded states (including the goal node)
            result.exploredStatesCount++;
            result.expandedStates.add(current.state);
            if (verbose) {
                System.out.printf(Locale.US, "  [%d] f=%.2f g=%.2f depth=%d  %s%n",
                        result.exploredStatesCount, current.fCost, current.gCost, current.depth, formatPath(current));
            }

            // Goal test on expansion guarantees an optimal solution
            if (current.state.equals("GOAL")) {
                goalNode = current;
                break;
            }

            List<Connection> candidateActions = new ArrayList<>();

            if (current.state.equals("START")) {
                // 1. Walk directly to the destination (no limit in fallback mode)
                double directWalk = calculateWalkMinutes(startPos, targetPos);
                if (directWalk <= maxDirectWalkMin || result.startFallback || result.targetFallback) {
                    candidateActions.add(new Connection("GOAL", directWalk, "walk_direct"));
                }
                // 2. Walk to a nearby station
                for (String node : startNodes) {
                    double walkDuration = calculateWalkMinutes(startPos, nodeLocations.get(node));
                    candidateActions.add(new Connection(node, walkDuration, "walk_start"));
                }
            } else {
                // Ride to the next station or transfer to another line
                List<Connection> connections = networkGraph.getOrDefault(current.state, Collections.emptyList());
                candidateActions.addAll(connections);

                // Walk to the destination, only after at least one ride
                if (targetNodes.contains(current.state) && hasTraveled(current)) {
                    double walkToEnd = calculateWalkMinutes(nodeLocations.get(current.state), targetPos);
                    candidateActions.add(new Connection("GOAL", walkToEnd, "walk_end"));
                }
            }

            for (Connection edge : candidateActions) {
                if (isOnPath(current, edge.target)) {
                    continue;
                }
                double newG = current.gCost + edge.duration;
                int newDepth = current.depth + 1;

                String nextKey = edge.target + "|" + newDepth;
                if (visitedBestCosts.getOrDefault(nextKey, Double.POSITIVE_INFINITY) <= newG + EPSILON) {
                    continue;
                }
                // Path length limit (p_max)
                if (newDepth > maxDepth) {
                    result.hitDepthLimit = true;
                    continue;
                }
                visitedBestCosts.put(nextKey, newG);
                // Heuristic and f-cost (g + h) of the successor
                Coordinate nextCoord = edge.target.equals("GOAL") ? targetPos : nodeLocations.get(edge.target);
                double hVal = calculateHeuristic(nextCoord, targetPos);
                double newF = newG + hVal;
                // Create the successor and push it onto OPEN
                SearchNode nextNode = new SearchNode(edge.target, newG, newF, current, edge.actionType, edge.duration, newDepth);
                openList.add(nextNode);
                if (nextNode.state.equals("GOAL") && (bestGoalInOpen == null || newG < bestGoalInOpen.gCost)) {
                    bestGoalInOpen = nextNode;
                }
            }

            result.maxQueueSize = Math.max(result.maxQueueSize, openList.size());
        }
        // On timeout, return the best solution generated so far
        if (goalNode == null && result.hitTimeLimit) {
            goalNode = bestGoalInOpen;
        }
        // Reconstruct the path if the goal was reached
        if (goalNode != null) {
            result.totalTimeMinutes = goalNode.gCost;
            reconstructPath(goalNode, result);
            // Status depending on which limit was hit
            if (result.hitTimeLimit) {
                result.statusMessage = "time limit reached - best solution found so far (not guaranteed optimal)";
            } else if (result.hitDepthLimit) {
                result.statusMessage = "potentially non-optimal solution found";
            } else {
                result.statusMessage = "optimal solution found";
            }
        } else {
            if (result.hitTimeLimit) {
                result.statusMessage = "search did not succeed within the time limit";
            } else if (result.hitDepthLimit) {
                result.statusMessage = "no solution can be generated with limited memory";
            } else {
                result.statusMessage = "no route found: destination is not reachable from the available stations";
            }
        }

        return result;
    }

    // Path as text for the -v output
    private static String formatPath(SearchNode node) {
        LinkedList<String> states = new LinkedList<>();
        for (SearchNode trace = node; trace != null; trace = trace.parent) {
            states.addFirst(trace.state);
        }
        return String.join(" > ", states);
    }

    private void reconstructPath(SearchNode goalNode, SearchResult result) {
        LinkedList<String> steps = new LinkedList<>();
        SearchNode trace = goalNode;
        int stepNumber = 1;

        List<SearchNode> chain = new ArrayList<>();
        while (trace != null) {
            chain.add(0, trace);
            trace = trace.parent;
        }
        for (SearchNode node : chain) {
            result.pathStates.add(node.state);
            result.pathActions.add(node.actionTaken);
        }

        for (int i = 1; i < chain.size(); i++) {
            SearchNode fromNode = chain.get(i - 1);
            SearchNode toNode = chain.get(i);
            String formatted = String.format(Locale.US, "%02d. %-12s %-25s -> %-25s %6.2f min",
                    stepNumber++, toNode.actionTaken, fromNode.state, toNode.state, toNode.stepCost);
            steps.add(formatted);
        }
        result.pathDetails = steps;
    }

    public static void displayReport(String title, SearchResult report) {
        System.out.println("\n=======================================================");
        System.out.println("SCENARIO: " + title);
        System.out.printf(Locale.US, "Start: (%.6f, %.6f) | Destination: (%.6f, %.6f)%n",
                report.startCoord.lat, report.startCoord.lon, report.targetCoord.lat, report.targetCoord.lon);
        if (report.startFallback) {
            System.out.printf(Locale.US, "Note: no U-Bahn station within %.0f min walk of the start, "
                    + "walking to the %d nearest stations allowed%n", report.maxWalkToStationMin, FALLBACK_STATIONS);
        }
        if (report.targetFallback) {
            System.out.printf(Locale.US, "Note: no U-Bahn station within %.0f min walk of the destination, "
                    + "walking from the %d nearest stations allowed%n", report.maxWalkToStationMin, FALLBACK_STATIONS);
        }
        System.out.println("Status: " + report.statusMessage);

        if (report.pathDetails.isEmpty()) {
            System.out.println("-> No route found.");
            if (report.hitDepthLimit) {
                System.out.println("-> no solution can be generated within the maximum path length");
            }
        } else {
            for (String step : report.pathDetails) {
                System.out.println("  " + step);
            }
            System.out.printf(Locale.US, "Total time: %.2f min | Path length: %d%n",
                    report.totalTimeMinutes, report.pathDetails.size());
        }
        System.out.println("Expanded states   : " + report.exploredStatesCount);
        System.out.println("Max. OPEN size    : " + report.maxQueueSize);
        System.out.println("p_max limit hit   : " + report.hitDepthLimit);
        System.out.println("Time limit hit    : " + report.hitTimeLimit);
        System.out.println("=======================================================");
    }

    private static void printUsage() {
        System.out.println("Usage: java RoutePlanner [-v] [--compare] [--svg file] [startLat startLon destLat destLon [p_max] [timeoutSec]]");
        System.out.println("Without coordinates the demo scenarios A-F are run.");
        System.out.println("-v:         print every expanded path with f, g and depth");
        System.out.println("--compare:  compare A* against uniform-cost search (h = 0)");
        System.out.println("--svg file: draw the states expanded by A* and UCS on a map");
    }

    // Runs one scenario and prints the result
    private void runScenario(String title, Coordinate start, Coordinate target, int pMax, double timeoutSeconds) {
        if (verbose) {
            System.out.println("\n--- Expanded paths: " + title + " ---");
        }
        displayReport(title, findRoute(start, target, pMax, timeoutSeconds, MAX_WALK_TO_STATION_MIN, MAX_DIRECT_WALK_MIN));
    }

    // Runs the same query with and without the heuristic and prints one table row
    private void compareRow(String title, Coordinate start, Coordinate target) {
        useHeuristic = true;
        SearchResult aStar = findRoute(start, target, DEFAULT_P_MAX, DEFAULT_TIMEOUT_S, MAX_WALK_TO_STATION_MIN, MAX_DIRECT_WALK_MIN);
        useHeuristic = false;
        SearchResult ucs = findRoute(start, target, DEFAULT_P_MAX, DEFAULT_TIMEOUT_S, MAX_WALK_TO_STATION_MIN, MAX_DIRECT_WALK_MIN);
        useHeuristic = true;
        System.out.printf(Locale.US, "%-38s %8.2f %8.2f %9d %9d %7.0f%%%n", title,
                aStar.totalTimeMinutes, ucs.totalTimeMinutes, aStar.exploredStatesCount, ucs.exploredStatesCount,
                100.0 * (1.0 - (double) aStar.exploredStatesCount / ucs.exploredStatesCount));
    }

    private void runComparison() {
        System.out.println("\nA* (straight-line heuristic) vs. uniform-cost search (h = 0)");
        System.out.printf(Locale.US, "%-38s %8s %8s %9s %9s %8s%n",
                "Route", "A* min", "UCS min", "A* exp.", "UCS exp.", "saved");
        compareRow("Alexanderplatz -> Prenzlauer Berg",
                new Coordinate(52.5213, 13.4114), new Coordinate(52.5270, 13.4196));
        compareRow("Möckernbrücke -> Friedrichshain",
                new Coordinate(52.499167, 13.382778), new Coordinate(52.510000, 13.445000));
        compareRow("Alexanderplatz -> Rathaus Neukölln",
                new Coordinate(52.521389, 13.411944), new Coordinate(52.481667, 13.434167));
        compareRow("Siemensdamm -> Hermannplatz",
                new Coordinate(52.536667, 13.273056), new Coordinate(52.486389, 13.424444));
        compareRow("Rathaus Spandau -> Rudow",
                new Coordinate(52.5350, 13.2003), new Coordinate(52.4158, 13.4961));
        compareRow("Hönow -> Rathaus Spandau",
                new Coordinate(52.5383, 13.6339), new Coordinate(52.535, 13.200278));
    }

    // Official BVG line colors
    private static final Map<String, String> LINE_COLORS = new HashMap<>();
    static {
        LINE_COLORS.put("U1", "#7DAD4C");
        LINE_COLORS.put("U2", "#DA421E");
        LINE_COLORS.put("U3", "#16683D");
        LINE_COLORS.put("U4", "#F0D722");
        LINE_COLORS.put("U5", "#7E5330");
        LINE_COLORS.put("U6", "#8C6DAB");
        LINE_COLORS.put("U7", "#528DBA");
        LINE_COLORS.put("U8", "#224F86");
        LINE_COLORS.put("U9", "#F3791D");
    }

    private static final double PANEL_WIDTH = 480;
    private static final double PANEL_GAP = 20;
    private static final double PANEL_TOP = 64;
    private static final String EXPANDED_COLOR = "#E8590C";

    // Maps geo coordinates onto one SVG panel (equirectangular, longitude scaled by cos(lat))
    private static class Projection {
        final double minLon, maxLat, lonFactor, scale, offsetX, offsetY;

        Projection(double minLon, double maxLat, double lonFactor, double scale, double offsetX, double offsetY) {
            this.minLon = minLon;
            this.maxLat = maxLat;
            this.lonFactor = lonFactor;
            this.scale = scale;
            this.offsetX = offsetX;
            this.offsetY = offsetY;
        }

        double x(Coordinate c) { return offsetX + (c.lon - minLon) * lonFactor * scale; }
        double y(Coordinate c) { return offsetY + (maxLat - c.lat) * scale; }
    }

    private Coordinate locate(String state, Coordinate start, Coordinate target) {
        if (state.equals("START")) return start;
        if (state.equals("GOAL")) return target;
        return nodeLocations.get(state);
    }

    // Writes an SVG that shows the states expanded by A* next to those expanded by uniform-cost search
    private void writeComparisonSvg(Path outFile, Coordinate start, Coordinate target, String routeName) throws IOException {
        useHeuristic = true;
        SearchResult aStar = findRoute(start, target, DEFAULT_P_MAX, DEFAULT_TIMEOUT_S, MAX_WALK_TO_STATION_MIN, MAX_DIRECT_WALK_MIN);
        useHeuristic = false;
        SearchResult ucs = findRoute(start, target, DEFAULT_P_MAX, DEFAULT_TIMEOUT_S, MAX_WALK_TO_STATION_MIN, MAX_DIRECT_WALK_MIN);
        useHeuristic = true;

        // Bounding box of the network plus start and destination
        List<Coordinate> all = new ArrayList<>(nodeLocations.values());
        all.add(start);
        all.add(target);
        double minLat = Double.POSITIVE_INFINITY, maxLat = Double.NEGATIVE_INFINITY;
        double minLon = Double.POSITIVE_INFINITY, maxLon = Double.NEGATIVE_INFINITY;
        for (Coordinate c : all) {
            minLat = Math.min(minLat, c.lat);
            maxLat = Math.max(maxLat, c.lat);
            minLon = Math.min(minLon, c.lon);
            maxLon = Math.max(maxLon, c.lon);
        }
        double lonFactor = Math.cos(Math.toRadians((minLat + maxLat) / 2));
        double inner = PANEL_WIDTH - 2 * PANEL_GAP;
        double scale = inner / ((maxLon - minLon) * lonFactor);
        double panelHeight = (maxLat - minLat) * scale + 2 * PANEL_GAP;
        double width = 2 * PANEL_WIDTH + 3 * PANEL_GAP;
        double height = PANEL_TOP + panelHeight + 56;

        StringBuilder svg = new StringBuilder();
        svg.append(String.format(Locale.US,
                "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%.0f\" height=\"%.0f\" viewBox=\"0 0 %.0f %.0f\" "
                        + "font-family=\"Helvetica, Arial, sans-serif\">%n", width, height, width, height));
        svg.append(String.format(Locale.US, "<rect width=\"%.0f\" height=\"%.0f\" fill=\"#ffffff\"/>%n", width, height));
        svg.append(String.format(Locale.US,
                "<text x=\"%.0f\" y=\"26\" font-size=\"17\" font-weight=\"bold\" fill=\"#222\">%s</text>%n",
                PANEL_GAP, escapeXml(routeName)));

        String[] titles = {
                String.format(Locale.US, "A* with straight-line heuristic: %d states expanded", aStar.exploredStatesCount),
                String.format(Locale.US, "Uniform-cost search (h = 0): %d states expanded", ucs.exploredStatesCount)
        };
        SearchResult[] results = {aStar, ucs};
        for (int i = 0; i < 2; i++) {
            double panelX = PANEL_GAP + i * (PANEL_WIDTH + PANEL_GAP);
            Projection proj = new Projection(minLon, maxLat, lonFactor, scale, panelX + PANEL_GAP, PANEL_TOP + PANEL_GAP);
            svg.append(String.format(Locale.US,
                    "<text x=\"%.0f\" y=\"%.0f\" font-size=\"13\" fill=\"#444\">%s</text>%n",
                    panelX, PANEL_TOP - 10, escapeXml(titles[i])));
            svg.append(String.format(Locale.US,
                    "<rect x=\"%.0f\" y=\"%.0f\" width=\"%.0f\" height=\"%.0f\" rx=\"8\" fill=\"#f7f7f5\" stroke=\"#e2e2de\"/>%n",
                    panelX, PANEL_TOP, PANEL_WIDTH, panelHeight));
            appendPanel(svg, proj, results[i], start, target);
        }

        // Legend
        double ly = PANEL_TOP + panelHeight + 30;
        double lx = PANEL_GAP;
        svg.append(String.format(Locale.US,
                "<line x1=\"%.0f\" y1=\"%.0f\" x2=\"%.0f\" y2=\"%.0f\" stroke=\"#111\" stroke-width=\"4\"/>%n", lx, ly, lx + 26, ly));
        svg.append(legendText(lx + 34, ly, String.format(Locale.US, "optimal route (%.1f min)", aStar.totalTimeMinutes)));
        lx += 210;
        svg.append(String.format(Locale.US,
                "<line x1=\"%.0f\" y1=\"%.0f\" x2=\"%.0f\" y2=\"%.0f\" stroke=\"#111\" stroke-width=\"2.5\" stroke-dasharray=\"4 3\"/>%n",
                lx, ly, lx + 26, ly));
        svg.append(legendText(lx + 34, ly, "walking"));
        lx += 110;
        svg.append(String.format(Locale.US, "<circle cx=\"%.0f\" cy=\"%.0f\" r=\"4.5\" fill=\"%s\"/>%n", lx + 5, ly, EXPANDED_COLOR));
        svg.append(legendText(lx + 16, ly, "expanded by the search"));
        lx += 190;
        svg.append(String.format(Locale.US, "<circle cx=\"%.0f\" cy=\"%.0f\" r=\"2.5\" fill=\"#b9b9b4\"/>%n", lx + 5, ly));
        svg.append(legendText(lx + 16, ly, "never touched"));
        lx += 130;
        svg.append(String.format(Locale.US,
                "<circle cx=\"%.0f\" cy=\"%.0f\" r=\"5\" fill=\"#fff\" stroke=\"#111\" stroke-width=\"2.5\"/>%n", lx + 5, ly));
        svg.append(legendText(lx + 16, ly, "start / destination"));
        svg.append("</svg>\n");

        Files.write(outFile, svg.toString().getBytes(StandardCharsets.UTF_8));
        System.out.printf(Locale.US, "SVG written to %s (A*: %d, UCS: %d expanded states)%n",
                outFile, aStar.exploredStatesCount, ucs.exploredStatesCount);
    }

    private static String legendText(double x, double y, String text) {
        return String.format(Locale.US, "<text x=\"%.0f\" y=\"%.0f\" font-size=\"12\" fill=\"#444\">%s</text>%n",
                x, y + 4, escapeXml(text));
    }

    private static String escapeXml(String text) {
        return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;");
    }

    // Draws the network, the expanded stations and the solution of one search into a panel
    private void appendPanel(StringBuilder svg, Projection proj, SearchResult result, Coordinate start, Coordinate target) {
        // U-Bahn lines in their colors (each edge appears twice in the data, draw it once)
        for (Map.Entry<String, List<Connection>> entry : networkGraph.entrySet()) {
            String from = entry.getKey();
            for (Connection edge : entry.getValue()) {
                if (!edge.actionType.equals("travel") || from.compareTo(edge.target) > 0) continue;
                Coordinate a = nodeLocations.get(from);
                Coordinate b = nodeLocations.get(edge.target);
                String color = LINE_COLORS.getOrDefault(from.substring(0, from.indexOf(':')), "#999999");
                svg.append(String.format(Locale.US,
                        "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" stroke=\"%s\" stroke-width=\"2.5\" "
                                + "stroke-opacity=\"0.45\" stroke-linecap=\"round\"/>%n",
                        proj.x(a), proj.y(a), proj.x(b), proj.y(b), color));
            }
        }
        // Stations: small grey dots, expanded ones highlighted
        for (Map.Entry<String, Coordinate> entry : nodeLocations.entrySet()) {
            if (result.expandedStates.contains(entry.getKey())) continue;
            svg.append(String.format(Locale.US, "<circle cx=\"%.1f\" cy=\"%.1f\" r=\"2\" fill=\"#b9b9b4\"/>%n",
                    proj.x(entry.getValue()), proj.y(entry.getValue())));
        }
        for (String state : result.expandedStates) {
            Coordinate c = nodeLocations.get(state);
            if (c == null) continue;
            svg.append(String.format(Locale.US, "<circle cx=\"%.1f\" cy=\"%.1f\" r=\"4\" fill=\"%s\" fill-opacity=\"0.85\"/>%n",
                    proj.x(c), proj.y(c), EXPANDED_COLOR));
        }
        // Solution path: white casing below, walking dashed
        for (int pass = 0; pass < 2; pass++) {
            for (int i = 1; i < result.pathStates.size(); i++) {
                Coordinate a = locate(result.pathStates.get(i - 1), start, target);
                Coordinate b = locate(result.pathStates.get(i), start, target);
                boolean walk = result.pathActions.get(i).startsWith("walk");
                String style = pass == 0
                        ? "stroke=\"#ffffff\" stroke-width=\"7\""
                        : "stroke=\"#111111\" stroke-width=\"" + (walk ? "2.5\" stroke-dasharray=\"4 3\"" : "4\"");
                svg.append(String.format(Locale.US,
                        "<line x1=\"%.1f\" y1=\"%.1f\" x2=\"%.1f\" y2=\"%.1f\" %s stroke-linecap=\"round\"/>%n",
                        proj.x(a), proj.y(a), proj.x(b), proj.y(b), style));
            }
        }
        for (Coordinate c : new Coordinate[]{start, target}) {
            svg.append(String.format(Locale.US,
                    "<circle cx=\"%.1f\" cy=\"%.1f\" r=\"6\" fill=\"#ffffff\" stroke=\"#111111\" stroke-width=\"2.5\"/>%n",
                    proj.x(c), proj.y(c)));
        }
    }

    public static void main(String[] args) {
        try {
            // Umlauts: use the console code page on Windows, UTF-8 otherwise
            String consoleEncoding = System.getProperty("sun.stdout.encoding");
            String outputEncoding = "UTF-8";
            if (consoleEncoding != null && !consoleEncoding.equalsIgnoreCase("cp65001") && Charset.isSupported(consoleEncoding)) {
                outputEncoding = consoleEncoding;
            }
            System.setOut(new PrintStream(new FileOutputStream(FileDescriptor.out), true, outputEncoding));

            RoutePlanner planner = new RoutePlanner(Paths.get(DATA_FILE));
            System.out.printf(Locale.US, "Network loaded. Fastest U-Bahn speed: %.2f km/h%n", planner.maximumMetroSpeed);

            // Strip flags from the arguments
            List<String> argList = new ArrayList<>(Arrays.asList(args));
            planner.verbose = argList.remove("-v");
            boolean compare = argList.remove("--compare");
            String svgFile = null;
            int svgIndex = argList.indexOf("--svg");
            if (svgIndex >= 0 && svgIndex + 1 < argList.size()) {
                svgFile = argList.remove(svgIndex + 1);
                argList.remove(svgIndex);
            }
            args = argList.toArray(new String[0]);

            if (compare) {
                planner.runComparison();
                return;
            }

            // SVG of A* vs. UCS, for the given coordinates or Rathaus Spandau -> Rudow
            if (svgFile != null) {
                if (args.length == 4) {
                    planner.writeComparisonSvg(Paths.get(svgFile),
                            new Coordinate(parseDecimal(args[0]), parseDecimal(args[1])),
                            new Coordinate(parseDecimal(args[2]), parseDecimal(args[3])),
                            "A* vs. uniform-cost search");
                } else {
                    planner.writeComparisonSvg(Paths.get(svgFile),
                            new Coordinate(52.5350, 13.2003), new Coordinate(52.4158, 13.4961),
                            "Rathaus Spandau → Rudow: A* vs. uniform-cost search");
                }
                return;
            }

            // Custom coordinates from the command line
            if (args.length > 0) {
                if (args.length != 4 && args.length != 5 && args.length != 6) {
                    printUsage();
                    return;
                }
                Coordinate start;
                Coordinate target;
                int pMax = DEFAULT_P_MAX;
                double timeout = DEFAULT_TIMEOUT_S;
                try {
                    start = new Coordinate(parseDecimal(args[0]), parseDecimal(args[1]));
                    target = new Coordinate(parseDecimal(args[2]), parseDecimal(args[3]));
                    if (args.length >= 5) pMax = Integer.parseInt(args[4].trim());
                    if (args.length == 6) timeout = parseDecimal(args[5]);
                } catch (NumberFormatException ex) {
                    System.out.println("Invalid number: " + ex.getMessage());
                    printUsage();
                    return;
                }
                planner.runScenario(String.format(Locale.US, "Custom route (p_max = %d)", pMax),
                        start, target, pMax, timeout);
                return;
            }

            // A: walking is fastest, but A* still explores some U-Bahn paths
            planner.runScenario("A: Alexanderplatz -> Prenzlauer Berg (walking is fastest)",
                    new Coordinate(52.5213, 13.4114), new Coordinate(52.5270, 13.4196), DEFAULT_P_MAX, DEFAULT_TIMEOUT_S);

            // B: single ride without transfer
            planner.runScenario("B: Möckernbrücke -> Friedrichshain (one line, no transfer)",
                    new Coordinate(52.499167, 13.382778), new Coordinate(52.510000, 13.445000), DEFAULT_P_MAX, DEFAULT_TIMEOUT_S);

            // C: route with one transfer
            planner.runScenario("C: Alexanderplatz -> Rathaus Neukölln (one transfer)",
                    new Coordinate(52.521389, 13.411944), new Coordinate(52.481667, 13.434167), DEFAULT_P_MAX, DEFAULT_TIMEOUT_S);

            // D: depth limit cuts off paths, solution may not be optimal
            planner.runScenario("D: Rathaus Spandau -> Rudow (p_max = 40, potentially non-optimal)",
                    new Coordinate(52.5350, 13.2003), new Coordinate(52.4158, 13.4961), 40, DEFAULT_TIMEOUT_S);

            // E: depth limit too small for any solution
            planner.runScenario("E: Hönow -> Rathaus Spandau (p_max = 40, no solution)",
                    new Coordinate(52.5383, 13.6339), new Coordinate(52.535, 13.200278), 40, DEFAULT_TIMEOUT_S);

            // F: time limit hit before a solution is found
            planner.runScenario("F: Siemensdamm -> Hermannplatz (time limit 0.0001 s, no result)",
                    new Coordinate(52.536667, 13.273056), new Coordinate(52.486389, 13.424444), DEFAULT_P_MAX, 0.0001);

        } catch (Exception ex) {
            ex.printStackTrace();
        }
    }
}
