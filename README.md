# Aufgabenblatt 1: Multimodale Routenplanung

## Voraussetzungen
- Java 8 oder neuer (JDK, `javac` und `java` im PATH; getestet mit OpenJDK 1.8.0_492)

## Kompilieren
`Routenplaner.java` und `A1-GKI-WS26-berliner_ubahn_geokoordinaten.csv` müssen im selben Verzeichnis liegen.
```bash
javac -encoding UTF-8 Routenplaner.java
```

## Ausführen
```bash
java Routenplaner [-v] [startLat startLon zielLat zielLon [p_max] [zeitlimitSek]]
```

| Argument | Bedeutung |
|---|---|
| *(keine Koordinaten)* | führt die Experimente A–F aus dem Report aus |
| `startLat startLon zielLat zielLon` | eigene Route für beliebige Geokoordinaten (Dezimalgrad, Punkt oder Komma) |
| `p_max` | optional, maximale Pfadlänge (Standard: 50) |
| `zeitlimitSek` | optional, Zeitlimit der Suche in Sekunden (Standard: 10) |
| `-v` | gibt jeden untersuchten Pfad mit f, g und depth aus (an beliebiger Stelle) |

Beispiele:
```bash
java Routenplaner
java Routenplaner -v
java Routenplaner 52.521389 13.411944 52.481667 13.434167
java Routenplaner 52.5350 13.2003 52.4158 13.4961 40
```

## Feste Parameter
| Parameter | Wert |
|---|---|
| Gehgeschwindigkeit | 5 km/h |
| Umsteigezeit | 5 min (aus der CSV) |
| max. Fußweg Start → Station bzw. Station → Ziel | 15 min; liegt keine Station im Umkreis, werden die 2 nächsten Stationen verwendet |
| max. Fußweg Start → Ziel ohne U-Bahn | 30 min (2 × 15 min); unbegrenzt, wenn Start oder Ziel keine Station im Umkreis hat |


