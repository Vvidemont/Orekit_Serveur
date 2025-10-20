package org.example;
// Package de l’application (doit correspondre à l’arborescence src/...).

import org.hipparchus.analysis.function.Abs;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.maneuvers.Hohmann;
import org.maneuvers.InclinationChange;
import org.maneuvers.PosHohmann;
import org.maneuvers.PosKep;
import org.orekit.data.DataContext;
import org.orekit.data.DirectoryCrawler;
import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.orbits.KeplerianOrbit;
import org.orekit.orbits.PositionAngleType;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.PVCoordinates;
import org.springframework.web.bind.annotation.CrossOrigin;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RestController;
// Imports Orekit + Spring pour la logique orbitale et l’API REST.

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.time.Instant;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

@CrossOrigin(origins = "*")
// Autorise les requêtes depuis n’importe quelle origine (CORS ouvert).
@RestController
// Déclare un contrôleur REST (expose des endpoints HTTP).
public class OrekitApplication {

    private static boolean orekitReady = false;
    // Flag pour ne charger orekit-data qu’une seule fois.

    // Charge orekit-data une seule fois
    private static void ensureOrekit() {
        if (!orekitReady) {
            File orekitDir = new File(
                    "C:/Users/valentin.videmont/OneDrive - Ecole de l'air/Bureau/Stage/orekit/orekit-data" // à modifier à l'emplacement actuel du fichier
            );
            DataContext.getDefault()
                    .getDataProvidersManager()
                    .addProvider(new DirectoryCrawler(orekitDir));
            // Ajoute un provider qui lit les fichiers d’éphémérides/constantes.
            orekitReady = true;
            System.out.println("[OK] orekit-data chargé depuis " + orekitDir.getAbsolutePath());
        }
    }

    @PostMapping("/orekit")
    // Endpoint principal : reçoit un JSON {action, params}.
    public Map<String, Object> orekit(@RequestBody Map<String, Object> req) {
        try {
            ensureOrekit();
            String action = Objects.toString(req.get("action"), "");
            Map<String, Object> p = (Map<String, Object>) req.get("params");

            if ("compute_orbit".equals(action)) {
                return ok(computeOrbit(p));
            }
            if ("compute_hohmann".equals(action)){
                return ok(computeHohmann(p));
            }
            if("compute_inclination".equals(action)){
                return ok(computeInclination(p));
            }
            else {
                return err("Unknown action: " + action);
            }
        } catch (Exception e) {
            // Renvoie une erreur JSON propre si une exception survient.
            return err(e.getMessage());
        }
    }

    private Map<String, Object> computeOrbit(Map<String, Object> p) {
        // Lit les 6 paramètres + type (0 = képlérien, 1 = cartésien), frame, dates, dt.
        double p1    = toD(p.get("p1"));
        double p2    = toD(p.get("p2"));
        double p3    = toD(p.get("p3"));
        double p4    = toD(p.get("p4"));
        double p5    = toD(p.get("p5"));
        double p6    = toD(p.get("p6"));
        double type  = toD(p.get("type"));
        String frameName = Objects.toString(p.getOrDefault("frame","EME2000"));
        String epochend =   Objects.toString(p.get("epoch_end"));
        String epochIso  = Objects.toString(p.getOrDefault("epoch_start","2025-01-01T00:00:00Z"));
        double dt        = toD(p.get("dt_seconds"));

        Frame frame = switch (frameName) {
            case "GCRF" -> FramesFactory.getGCRF();
            default     -> FramesFactory.getEME2000();
        };
        // Sélectionne le repère inertiel (EME2000 par défaut).

        AbsoluteDate dateend = new AbsoluteDate(Instant.parse(epochend), TimeScalesFactory.getUTC());
        AbsoluteDate date0 = new AbsoluteDate(Instant.parse(epochIso), TimeScalesFactory.getUTC());
        double deltaSecond = dateend.durationFrom(date0);
        // Durée de propagation = fin - début (en secondes).

        KeplerianOrbit orbit = null;
        // Construit l’orbite selon le type d’entrée.

        if (type == 0 ) {
            // Entrée képlérienne : angles en radians, a en mètres, μ Terre WGS84.
            p3 = Math.toRadians(p3);
            p4 = Math.toRadians(p4);
            p5 = Math.toRadians(p5);
            p6 = Math.toRadians(p6);

            orbit = new KeplerianOrbit(
                    p1, p2, p3, p4, p5, p6,
                    PositionAngleType.TRUE, frame, date0, Constants.WGS84_EARTH_MU
            );
        } else if (type == 1) {
            // Entrée cartésienne : position (m) et vitesse (m/s) → PV → Orbite.
            Vector3D position = new Vector3D(p1, p2, p3);
            Vector3D velocity = new Vector3D(p4, p5, p6);
            PVCoordinates pv = new PVCoordinates(position, velocity);
            orbit = new KeplerianOrbit(pv, frame, date0, Constants.WGS84_EARTH_MU);
        }

        PosKep tableau = new PosKep(orbit, deltaSecond, dt);
        double[][] data = tableau.positionsTimeArray();
        // Échantillonne la position temporelle [t, x, y, z] selon dt.

        Map<String,Object> out = new HashMap<>();
        out.put("time_length", deltaSecond);
        out.put("n", data.length);
        out.put("data", data);
        // Réponse : durée, nombre d’échantillons, tableau des positions.
        return out;
    }

    private Map<String, Object> computeHohmann(Map<String, Object> p) {
        // Paramètres Hohmann : a_init/final (m), dates, dt, méthode/valeur de manœuvre.
        double a_init = toD(p.get("a_init"));
        double a_final = toD(p.get("a_final"));
        String epochstart = Objects.toString(p.get("epoch_start"));
        String epochend   = Objects.toString(p.get("epoch_end"));
        double dt         = toD(p.get("dt"));
        double man_method = toD((p.get("man_method")));
        String start_man  = Objects.toString(p.get("man_value"));

        AbsoluteDate datestart = new AbsoluteDate(Instant.parse(epochstart), TimeScalesFactory.getUTC());
        AbsoluteDate dateend   = new AbsoluteDate(Instant.parse(epochend),   TimeScalesFactory.getUTC());
        double deltasecond = dateend.durationFrom(datestart);

        KeplerianOrbit orbit = new KeplerianOrbit(
                a_init, 0, 0, 0, 0, 0,
                PositionAngleType.TRUE, FramesFactory.getEME2000(), datestart, Constants.WGS84_EARTH_MU
        );
        // Orbite circulaire équatoriale de départ (excentricité et angles nuls).

        AbsoluteDate date_man = datestart;
        // Calcule la date d’allumage selon la méthode choisie.
        if (man_method == 0 ) {
            date_man = new AbsoluteDate(Instant.parse(start_man), TimeScalesFactory.getUTC());
        } else if (man_method == 1) {
            date_man = datestart.shiftedBy(Integer.parseInt(start_man));
        } else if (man_method == 2) {
            date_man = datestart.shiftedBy(Integer.parseInt(start_man) * orbit.getKeplerianPeriod());
        }

        Hohmann hohmann = new Hohmann(a_final, orbit, date_man);
        // Calcule la manœuvre Hohmann (ΔV1/ΔV2, dates, transfert).

        PosHohmann pashohmann = new PosHohmann(hohmann, orbit, dt, deltasecond);
        double[][] data = pashohmann.positionsTimeArray();
        // Échantillonne la trajectoire (avant/pendant/après transfert).
        saveToCSV(data, "C:\\Users\\valentin.videmont\\OneDrive - Ecole de l'air\\Bureau\\Test\\orbit.csv");
        Map<String,Object> out = new HashMap<>();
        out.put("data", data);
        // Réponse : tableau des positions [t, x, y, z].
        return out;
    }

    private Map<String, Object> computeInclination(Map<String, Object> p) {

        double a = toD(p.get("a"));
        double e = toD(p.get("e"));
        double i = Math.toRadians(toD(p.get("i")));
        double raan = Math.toRadians(toD(p.get("raan")));
        double argp = Math.toRadians(toD(p.get("argp")));
        double ta = Math.toRadians(toD(p.get("ta")));

        String epochstart = Objects.toString(p.get("epoch_start"));
        String epochend   = Objects.toString(p.get("epoch_end"));
        double dt = toD(p.get("dt"));

        double itarget = Math.toRadians(toD(p.get("itarget")));

        String strnode = Objects.toString(p.get("node"));

        boolean node = Objects.equals(strnode, "true");

        double man_method = toD((p.get("man_method")));
        String start_man  = Objects.toString(p.get("man_value"));



        AbsoluteDate dateend = new AbsoluteDate(Instant.parse(epochend), TimeScalesFactory.getUTC());
        AbsoluteDate date0 = new AbsoluteDate(Instant.parse(epochstart), TimeScalesFactory.getUTC());
        double deltaSecond = dateend.durationFrom(date0);
        KeplerianOrbit orbit = new KeplerianOrbit(a,e,i,raan,argp,ta,PositionAngleType.TRUE,FramesFactory.getEME2000(),date0,Constants.WGS84_EARTH_MU);

        AbsoluteDate date_man = date0;
        // Calcule la date d’allumage selon la méthode choisie.
        if (man_method == 0 ) {
            date_man = new AbsoluteDate(Instant.parse(start_man), TimeScalesFactory.getUTC());
        } else if (man_method == 1) {
            date_man = date0.shiftedBy(Integer.parseInt(start_man));
        } else if (man_method == 2) {
            date_man = date0.shiftedBy(Integer.parseInt(start_man) * orbit.getKeplerianPeriod());
        }

        double man = date_man.durationFrom(date0);

        InclinationChange ichange = new InclinationChange(orbit,itarget,node,date_man);

        ImpulseManeuver maneuver = ichange.computeManeuver();

        PosKep tableau = new PosKep(orbit,deltaSecond,dt,maneuver,man);

        double[][] data = tableau.positionsTimeArray();

        Map<String,Object> out = new HashMap<>();
        out.put("data", data);
        return out;
    }

    private static double toD(Object o){ return (o instanceof Number n) ? n.doubleValue() : Double.parseDouble(o.toString()); }
    // Parse robuste des nombres (Number → double, sinon parse String).

    private static Map<String,Object> ok(Object result){ return Map.of("ok", true, "result", result); }
    // Enveloppe succès standard { ok: true, result: ... }.

    private static Map<String,Object> err(String msg){ return Map.of("ok", false, "error", msg); }
    // Enveloppe erreur standard { ok: false, error: "..." }.

    private void saveToCSV(double[][] data, String filename) {
        try (FileWriter writer = new FileWriter(filename)) {
            writer.write("t(s);x(Mm);y(Mm);z(Mm)\n"); // entête
            for (double[] row : data) {
                writer.write(String.format("%.3f;%.6f;%.6f;%.6f\n", row[0], row[1], row[2], row[3]));
            }
            System.out.println("[OK] Fichier sauvegardé : " + filename);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}