package org.maneuvers;

import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.orbits.KeplerianOrbit;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.KeplerianPropagator;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.PVCoordinates;

public class PosKep {
    private final KeplerianOrbit initialOrbit;
    private final double tmax;
    private final double pas;
    private final ImpulseManeuver maneuver;
    private final double startdate;


    public PosKep(KeplerianOrbit initialOrbit, double tmax, double pas) {
        if (pas <= 0) {
            throw new IllegalArgumentException("Le pas doit être > 0");
        }
        if (tmax < 0) {
            throw new IllegalArgumentException("tmax doit être >= 0");
        }
        this.initialOrbit = initialOrbit;
        this.tmax = tmax;
        this.pas = pas;
        this.maneuver = null;
        this.startdate = 0;
    }

    public PosKep(KeplerianOrbit initialOrbit, double tmax, double pas, ImpulseManeuver maneuver, double startdate) {
        if (pas <= 0) {
            throw new IllegalArgumentException("Le pas doit être > 0");
        }
        if (tmax < 0) {
            throw new IllegalArgumentException("tmax doit être >= 0");
        }
        this.initialOrbit = initialOrbit;
        this.tmax = tmax;
        this.pas = pas;
        this.maneuver = maneuver;
        this.startdate = startdate;
    }


    /** Renvoie un tableau [t, x, y, z] en secondes et mètres */
    public double[][] positionsTimeArray() {
        double i_init = initialOrbit.getI();
        KeplerianPropagator propagator = new KeplerianPropagator(initialOrbit);
        AbsoluteDate start = initialOrbit.getDate();
        double date;
        boolean ok = true;
        int n = (int) Math.floor(tmax / pas) + 1;
        double[][] data = new double[n][4];

        if (maneuver != null && i_init < 1e-5){
            propagator.addEventDetector(maneuver);
        }

        for (int k = 0; k < n; k++) {
            double t = k * pas;

            SpacecraftState s = propagator.propagate(start.shiftedBy(t));
            PVCoordinates pv = s.getPVCoordinates();
            date = s.getDate().durationFrom(start);

            if (maneuver != null && date>startdate && ok) {
                propagator.addEventDetector(maneuver);
                ok = false;

            }

            data[k][0] = t; // temps écoulé en secondes
            data[k][1] = pv.getPosition().getX()*1E-6;
            data[k][2] = pv.getPosition().getY()*1E-6;
            data[k][3] = pv.getPosition().getZ()*1E-6;

            ;

            if (Math.abs(i_init - s.getOrbit().getI()) > 1e-6){
                propagator.clearEventsDetectors();

            }
        }
        return data;
    }
}