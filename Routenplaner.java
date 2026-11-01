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

public class Routenplaner {

    private static final double WALK_SPEED_KMH = 5.0; // Das Tempo zu Fuss betraegt: 5 km/h
    private static final double EARTH_RADIUS_KM = 6371.0088; //Erdradius für die Abstand berechnung in km
    private static final double EPSILON = 1e-9; //verhindert Rundungsfehler

    // Standardwerte der Suche
    // p_max: laengste optimale Route im Netz hat 47 Aktionen
    public static final int DEFAULT_P_MAX = 50;
    public static final double DEFAULT_TIMEOUT_S = 10.0;
    public static final double MAX_WALK_TO_STATION_MIN = 15.0; //max. Fussweg Start->Station bzw. Station->Ziel
    // max. Fussweg Start->Ziel ohne U-Bahn (2 x 15 Min.)
    public static final double MAX_DIRECT_WALK_MIN = 2 * MAX_WALK_TO_STATION_MIN;
    public static final int FALLBACK_STATIONS = 2; //naechste Stationen, falls keine im Umkreis liegt

    public static class Coordinate { //speichert die Geokoordinaten von Orten und Stationen
        final double lat;
        final double lon;

        public Coordinate(double lat, double lon) {
            this.lat = lat;
            this.lon = lon;
        }
    }

    public static class Connection { //Eine Kante im Graphen
        final String target;
        final double duration;
        final String actionType;

        public Connection(String target, double duration, String actionType) {
            this.target = target;
            this.duration = duration;
            this.actionType = actionType;
        }
    }
    //Ein Schritt für die A*-Suche
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
        //Sortiert nach den geringsten Kosten(f=g+h)
        @Override
        public int compareTo(SearchNode other) {
            if (Math.abs(this.fCost - other.fCost) > EPSILON) {
                return Double.compare(this.fCost, other.fCost);
            }
            return Double.compare(this.gCost, other.gCost);
        }
    }
    //Speichert das Ergebnis und die Statistik der Suche
    public static class SearchResult {
        public List<String> pathDetails = new ArrayList<>();
        public double totalTimeMinutes = 0.0;
        public int exploredStatesCount = 0;
        public int maxQueueSize = 0;
        public boolean hitDepthLimit = false;
        public boolean hitTimeLimit = false;
        public boolean startFallback = false; //keine Station im Umkreis des Starts
        public boolean targetFallback = false; //keine Station im Umkreis des Ziels
        public double maxWalkToStationMin;
        public String statusMessage = "";
        public Coordinate startCoord;
        public Coordinate targetCoord;
    }
    //Der U-Bahn-Graph und die Koordinaten der Stationen
    private final Map<String, List<Connection>> networkGraph = new HashMap<>();
    private final Map<String, Coordinate> nodeLocations = new HashMap<>();
    private double maximumMetroSpeed = WALK_SPEED_KMH;
    private boolean verbose = false; //-v: jeden untersuchten Pfad ausgeben
    //erstellt den Planer und laedt die CSV Daten
    public Routenplaner(Path csvFilePath) throws IOException {
        loadNetwork(csvFilePath);
    }
    //Liest alle Verbindungen und Stationen aus der CSv Datei
    private void loadNetwork(Path csvFilePath) throws IOException {
        List<String> records = Files.readAllLines(csvFilePath, StandardCharsets.UTF_8);

        for (int i = 1; i < records.size(); i++) {
            String currentLine = records.get(i).trim();
            if (currentLine.isEmpty()) continue;
            //Daten aus der CSV-zeile parsen
            String[] tokens = currentLine.split(";", -1);
            if (tokens.length < 10) continue;

            String origin = tokens[0].trim();
            String destination = tokens[1].trim();
            String action = tokens[2].trim();
            double duration = parseDecimal(tokens[3]);
            Coordinate originCoord = new Coordinate(parseDecimal(tokens[6]), parseDecimal(tokens[7]));
            Coordinate destCoord = new Coordinate(parseDecimal(tokens[8]), parseDecimal(tokens[9]));
            //Koordinaten der Haltestellen speichern
            registerLocation(origin, originCoord);
            registerLocation(destination, destCoord);
            //Die CSV enthaelt beide Fahrtrichtungen als eigene Zeilen
            addGraphEdge(origin, destination, duration, action);
            //maximale Geschwindigkeit fuer die Heuristik
            if (duration > 0) {
                double distanceKm = calculateDistanceKm(originCoord, destCoord);
                double speedKmH = (distanceKm / duration) * 60.0;
                if (speedKmH > maximumMetroSpeed) {
                    maximumMetroSpeed = speedKmH;
                }
            }
        }
    }
    //Fuegt eine Kante zum Graphen hin zu
    private void addGraphEdge(String from, String to, double cost, String action) {
        networkGraph.computeIfAbsent(from, k -> new ArrayList<>()).add(new Connection(to, cost, action));
    }
    //Speichert die Koordinaten für einen Namen
    private void registerLocation(String key, Coordinate coord) {
        if (!key.isEmpty()) {
            nodeLocations.putIfAbsent(key, coord);
        }
    }
    //Ersetzt Komma durch Punkt
    private static double parseDecimal(String rawValue) {
        return Double.parseDouble(rawValue.replace(',', '.').trim());
    }
    //Berechnet die Distanz in km mit der Haversine-Formel
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
    //Berechnet die Gehzeit in Minuten bei 5 km/h
    public static double calculateWalkMinutes(Coordinate p1, Coordinate p2) {
        return (calculateDistanceKm(p1, p2) / WALK_SPEED_KMH) * 60.0;
    }
    //zulaessige Heuristik: Luftlinie / maximale Geschwindigkeit
    private double calculateHeuristic(Coordinate current, Coordinate destination) {
        if (current == null || destination == null) return 0.0;
        double distKm = calculateDistanceKm(current, destination);
        return (distKm / maximumMetroSpeed) * 60.0;
    }
    //Stationsname ohne Linie (U2:Alexanderplatz -> Alexanderplatz)
    private static String stationName(String node) {
        return node.substring(node.indexOf(':') + 1);
    }
    //Linienknoten in Gehweite
    private Set<String> nodesInWalkingRange(Coordinate pos, double maxWalkMin) {
        Set<String> nodes = new HashSet<>();
        for (Map.Entry<String, Coordinate> entry : nodeLocations.entrySet()) {
            if (calculateWalkMinutes(pos, entry.getValue()) <= maxWalkMin) {
                nodes.add(entry.getKey());
            }
        }
        return nodes;
    }
    //Linienknoten der naechsten Stationen (Fallback)
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
    //Prueft, ob ein Zustand schon auf dem Pfad liegt (Zyklen)
    private static boolean isOnPath(SearchNode node, String state) {
        for (SearchNode trace = node; trace != null; trace = trace.parent) {
            if (trace.state.equals(state)) return true;
        }
        return false;
    }
    //Prueft, ob auf dem Pfad U-Bahn gefahren wurde
    private static boolean hasTraveled(SearchNode node) {
        for (SearchNode trace = node; trace != null; trace = trace.parent) {
            if (trace.actionTaken.equals("travel")) return true;
        }
        return false;
    }
    //A* Suche von Start Koordinate zu Zielkoordinate
    public SearchResult findRoute(Coordinate startPos, Coordinate targetPos, int maxDepth, double timeoutSeconds, double maxWalkToStationMin, double maxDirectWalkMin) {
        SearchResult result = new SearchResult();
        result.startCoord = startPos;
        result.targetCoord = targetPos;
        result.maxWalkToStationMin = maxWalkToStationMin;

        // Stationen fuer den ersten und letzten Fussweg (sonst die naechsten)
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

        // Startzustand initialisieren
        openList.add(new SearchNode("START", 0.0, calculateHeuristic(startPos, targetPos), null, "START", 0.0, 0));
        visitedBestCosts.put("START|0", 0.0);
        result.maxQueueSize = 1;

        SearchNode goalNode = null;
        SearchNode bestGoalInOpen = null; //beste erzeugte Loesung, falls das Zeitlimit greift

        while (!openList.isEmpty()) {
            double elapsedSeconds = (System.nanoTime() - startTimeNano) / 1e9;
            if (elapsedSeconds >= timeoutSeconds) { //Prüft das Zeitlimit
                result.hitTimeLimit = true;
                break;
            }

            SearchNode current = openList.poll();

            // Veraltete Kopie ueberspringen
            String stateDepthKey = current.state + "|" + current.depth;
            if (current.gCost > visitedBestCosts.getOrDefault(stateDepthKey, Double.POSITIVE_INFINITY) + EPSILON) {
                continue;
            }

            // Untersuchte Zustaende zaehlen (auch der Zielknoten)
            result.exploredStatesCount++;
            if (verbose) {
                System.out.printf(Locale.US, "  [%d] f=%.2f g=%.2f depth=%d  %s%n",
                        result.exploredStatesCount, current.fCost, current.gCost, current.depth, formatPath(current));
            }

            // Ziel erreicht Abbruch mit optimaler Lösung
            if (current.state.equals("ZIEL")) {
                goalNode = current;
                break;
            }

            List<Connection> candidateActions = new ArrayList<>();

            if (current.state.equals("START")) {
                // 1. Direkt zu Fuss zum Ziel gehen (im Fallback ohne Limit)
                double directWalk = calculateWalkMinutes(startPos, targetPos);
                if (directWalk <= maxDirectWalkMin || result.startFallback || result.targetFallback) {
                    candidateActions.add(new Connection("ZIEL", directWalk, "walk_direct"));
                }
                // 2. Zu Fuss zu Stationen in der Naehe gehen
                for (String node : startNodes) {
                    double walkDuration = calculateWalkMinutes(startPos, nodeLocations.get(node));
                    candidateActions.add(new Connection(node, walkDuration, "walk_start"));
                }
            } else {
                // U-Bahn-Fahrt und Umstieg
                List<Connection> connections = networkGraph.getOrDefault(current.state, Collections.emptyList());
                candidateActions.addAll(connections);

                // Zu Fuss zum Zielort, erst nach mindestens einer Fahrt
                if (targetNodes.contains(current.state) && hasTraveled(current)) {
                    double walkToEnd = calculateWalkMinutes(nodeLocations.get(current.state), targetPos);
                    candidateActions.add(new Connection("ZIEL", walkToEnd, "walk_end"));
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
                // Pfadlaengenlimit (p_max)
                if (newDepth > maxDepth) {
                    result.hitDepthLimit = true;
                    continue;
                }
                visitedBestCosts.put(nextKey, newG);
                //Heuristik und f-Kosten(g+h) für den Nachfolger berechnen
                Coordinate nextCoord = edge.target.equals("ZIEL") ? targetPos : nodeLocations.get(edge.target);
                double hVal = calculateHeuristic(nextCoord, targetPos);
                double newF = newG + hVal;
                //Neuen Knoten erstellen und in OPEN einfügen
                SearchNode nextNode = new SearchNode(edge.target, newG, newF, current, edge.actionType, edge.duration, newDepth);
                openList.add(nextNode);
                if (nextNode.state.equals("ZIEL") && (bestGoalInOpen == null || newG < bestGoalInOpen.gCost)) {
                    bestGoalInOpen = nextNode;
                }
            }

            result.maxQueueSize = Math.max(result.maxQueueSize, openList.size());
        }
        //Bei Zeitlimit die beste bisher erzeugte Loesung liefern
        if (goalNode == null && result.hitTimeLimit) {
            goalNode = bestGoalInOpen;
        }
        //Pfad rekonstruieren, falls Ziel erreicht wurde
        if (goalNode != null) {
            result.totalTimeMinutes = goalNode.gCost;
            reconstructPath(goalNode, result);
            //Statusmeldung gemaess Aufgabenstellung setzen
            if (result.hitTimeLimit) {
                result.statusMessage = "Zeitlimit erreicht – beste bisher gefundene Lösung (nicht garantiert optimal)";
            } else if (result.hitDepthLimit) {
                result.statusMessage = "potenziell nicht optimale Lösung gefunden";
            } else {
                result.statusMessage = "optimale Lösung gefunden";
            }
        } else {
            if (result.hitTimeLimit) {
                result.statusMessage = "Suche mit Zeitlimit ohne Erfolg";
            } else if (result.hitDepthLimit) {
                result.statusMessage = "Lösung mit begrenztem Speicher nicht generierbar";
            } else {
                result.statusMessage = "Keine Route gefunden: Ziel ist von den erreichbaren Stationen aus nicht erreichbar";
            }
        }

        return result;
    }

    //Pfad als Text fuer die -v Ausgabe
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

        for (int i = 1; i < chain.size(); i++) {
            SearchNode fromNode = chain.get(i - 1);
            SearchNode toNode = chain.get(i);
            String formatted = String.format(Locale.US, "%02d. %-12s %-25s -> %-25s %6.2f Min.",
                    stepNumber++, toNode.actionTaken, fromNode.state, toNode.state, toNode.stepCost);
            steps.add(formatted);
        }
        result.pathDetails = steps;
    }

    public static void displayReport(String title, SearchResult report) {
        System.out.println("\n=======================================================");
        System.out.println("EXPERIMENT: " + title);
        System.out.printf(Locale.US, "Start: (%.6f, %.6f) | Ziel: (%.6f, %.6f)%n",
                report.startCoord.lat, report.startCoord.lon, report.targetCoord.lat, report.targetCoord.lon);
        if (report.startFallback) {
            System.out.printf(Locale.US, "Hinweis: Keine U-Bahn-Station innerhalb von %.0f Min. Fussweg vom Start, "
                    + "Fussweg zu den %d naechsten Stationen erlaubt%n", report.maxWalkToStationMin, FALLBACK_STATIONS);
        }
        if (report.targetFallback) {
            System.out.printf(Locale.US, "Hinweis: Keine U-Bahn-Station innerhalb von %.0f Min. Fussweg vom Ziel, "
                    + "Fussweg von den %d naechsten Stationen erlaubt%n", report.maxWalkToStationMin, FALLBACK_STATIONS);
        }
        System.out.println("Status: " + report.statusMessage);

        if (report.pathDetails.isEmpty()) {
            System.out.println("-> Keine Route gefunden.");
            if (report.hitDepthLimit) {
                System.out.println("-> mit der maximalen Pfadlänge kann eine Lösung nicht generiert werden");
            }
        } else {
            for (String step : report.pathDetails) {
                System.out.println("  " + step);
            }
            System.out.printf(Locale.US, "Gesamtdauer: %.2f minuten | Pfadlaenge: %d%n",
                    report.totalTimeMinutes, report.pathDetails.size());
        }
        System.out.println("Untersuchte Zustaende: " + report.exploredStatesCount);
        System.out.println("Maximale OPEN-Grösse: " + report.maxQueueSize);
        System.out.println("p_max-Limit erreicht: " + report.hitDepthLimit);
        System.out.println("Zeitlimit erreicht : " + report.hitTimeLimit);
        System.out.println("=======================================================");
    }

    private static void printUsage() {
        System.out.println("Aufruf: java Routenplaner [-v] [startLat startLon zielLat zielLon [p_max] [zeitlimitSek]]");
        System.out.println("Ohne Koordinaten werden die Experimente A-F ausgefuehrt.");
        System.out.println("-v: jeden untersuchten Pfad mit f, g und depth ausgeben");
    }

    //Fuehrt ein Experiment aus und gibt das Ergebnis aus
    private void runExperiment(String title, Coordinate start, Coordinate target, int pMax, double timeoutSeconds) {
        if (verbose) {
            System.out.println("\n--- Untersuchte Pfade: " + title + " ---");
        }
        displayReport(title, findRoute(start, target, pMax, timeoutSeconds, MAX_WALK_TO_STATION_MIN, MAX_DIRECT_WALK_MIN));
    }

    public static void main(String[] args) {
        try {
            //Umlaute: unter Windows Codepage der Konsole, sonst UTF-8
            String consoleEncoding = System.getProperty("sun.stdout.encoding");
            String outputEncoding = "UTF-8";
            if (consoleEncoding != null && !consoleEncoding.equalsIgnoreCase("cp65001") && Charset.isSupported(consoleEncoding)) {
                outputEncoding = consoleEncoding;
            }
            System.setOut(new PrintStream(new FileOutputStream(FileDescriptor.out), true, outputEncoding));

            Path csvPath = Paths.get("src/A1-GKI-WS26-berliner_ubahn_geokoordinaten.csv");
            if (!Files.exists(csvPath)) {
                csvPath = Paths.get("A1-GKI-WS26-berliner_ubahn_geokoordinaten.csv");
            }

            Routenplaner planner = new Routenplaner(csvPath);
            System.out.printf(Locale.US, "System gestartet. Maximale U-Bahn Geschwindigkeit: %.2f km/h%n", planner.maximumMetroSpeed);

            // Option -v aus den Argumenten entfernen
            List<String> argList = new ArrayList<>(Arrays.asList(args));
            planner.verbose = argList.remove("-v");
            args = argList.toArray(new String[0]);

            // Eigene Koordinaten ueber die Kommandozeile
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
                    System.out.println("Ungueltige Zahl: " + ex.getMessage());
                    printUsage();
                    return;
                }
                planner.runExperiment(String.format(Locale.US, "Eigene Route (p_max = %d)", pMax),
                        start, target, pMax, timeout);
                return;
            }

            // EXP A
            planner.runExperiment("Exp A: Alexanderplatz -> Mendelssohnstraße (Prenzlauer Berg) (am schnellsten zu Fuss)",
                    new Coordinate(52.5213, 13.4114), new Coordinate(52.5270, 13.4196), DEFAULT_P_MAX, DEFAULT_TIMEOUT_S);

            // EXP B
            planner.runExperiment("Exp B: Möckernbrücke -> Schnitzery Friedrichshain (U-Bahn ohne Umsteigen + zu Fuss Gehen)",
                    new Coordinate(52.499167, 13.382778), new Coordinate(52.510000, 13.445000), DEFAULT_P_MAX, DEFAULT_TIMEOUT_S);

            // EXP C
            planner.runExperiment("Exp C: Alexanderplatz -> Rathaus Neukölln (Reise mit Umsteigen)",
                    new Coordinate(52.521389, 13.411944), new Coordinate(52.481667, 13.434167), DEFAULT_P_MAX, DEFAULT_TIMEOUT_S);

            // EXP D
            planner.runExperiment("Exp D: Rathaus Spandau -> Rudow (p_max = 40, potenziell nicht optimal)",
                    new Coordinate(52.5350, 13.2003), new Coordinate(52.4158, 13.4961), 40, DEFAULT_TIMEOUT_S);

            // EXP E
            planner.runExperiment("Exp E: Hönow -> Rathaus Spandau (p_max = 40, keine Lösung)",
                    new Coordinate(52.5383, 13.6339), new Coordinate(52.535, 13.200278), 40, DEFAULT_TIMEOUT_S);

            // EXP F
            planner.runExperiment("Exp F: Siemensdamm -> Hermannplatz (Zeitlimit 0.0001 s, ohne Erfolg)",
                    new Coordinate(52.536667, 13.273056), new Coordinate(52.486389, 13.424444), DEFAULT_P_MAX, 0.0001);

        } catch (Exception ex) {
            ex.printStackTrace();
        }
    }
}