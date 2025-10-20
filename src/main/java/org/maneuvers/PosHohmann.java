package org.maneuvers;

import org.hipparchus.ode.nonstiff.DormandPrince853Integrator;
import org.orekit.attitudes.AttitudeProvider;
import org.orekit.attitudes.LofOffset;
import org.orekit.forces.gravity.NewtonianAttraction;
import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.frames.LOFType;
import org.orekit.orbits.KeplerianOrbit;
import org.orekit.orbits.OrbitType;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.KeplerianPropagator;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.numerical.NumericalPropagator;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.PVCoordinates;

import java.util.List;
public class PosHohmann {

    private final Hohmann hohmann;
    private final KeplerianOrbit initorbit;
    private final double dt;
    private final double tmax;

    public PosHohmann(Hohmann hohmann, KeplerianOrbit initorbit, double dt, double tmax){
        if (dt <= 0) throw new IllegalArgumentException("dt must be > 0");
        if (tmax < 0) throw new IllegalArgumentException("tmax must be >= 0");
        this.hohmann = hohmann;
        this.initorbit = initorbit;
        this.dt = dt;
        this.tmax = tmax;
    }

    // <<< type de retour ajouté
    private NumericalPropagator getpropagator() {
        // Attitude en LOF TNW (tes Δv sont définis en repère sat)
        final AttitudeProvider attitudeProvider = new LofOffset(initorbit.getFrame(), LOFType.TNW);

        // Intégrateur num.
        DormandPrince853Integrator integrator =
                new DormandPrince853Integrator(0.01, 500.0, 1e-6, 1e-6);

        NumericalPropagator propagator = new NumericalPropagator(integrator);
        propagator.setOrbitType(OrbitType.KEPLERIAN);

        // Masse optionnelle si tu veux modéliser la conso (ta classe Hohmann le calcule, mais ici la masse par défaut = 1 kg)
        propagator.setInitialState(new SpacecraftState(initorbit));

        propagator.setAttitudeProvider(attitudeProvider);

        // Gravité 2-corps indispensable
        propagator.addForceModel(new NewtonianAttraction(initorbit.getMu()));

        // Récupère tes ImpulseManeuver (déjà construits avec isp, dv, etc.)
        List<ImpulseManeuver> maneuvers = hohmann.computeManeuvers();

        // IMPORTANT : on ajoute le DETECTOR câblé avec le handler (maneuver)
        for (ImpulseManeuver maneuver : maneuvers) {
            propagator.addEventDetector(maneuver);
        }

        return propagator;
    }

    /** Renvoie [t, x, y, z] (t en s, positions en km) */
    public double[][] positionsTimeArray() {
        NumericalPropagator propagator = getpropagator();
        AbsoluteDate start = initorbit.getDate();

        int n = (int) Math.floor(tmax / dt) + 1;
        double[][] data = new double[n][4];

        for (int k = 0; k < n; k++) {
            double t = k * dt;
            SpacecraftState s = propagator.propagate(start.shiftedBy(t));
            PVCoordinates pv = s.getPVCoordinates();
            data[k][0] = t;
            data[k][1] = pv.getPosition().getX() * 1e-6; // km
            data[k][2] = pv.getPosition().getY() * 1e-6; // km
            data[k][3] = pv.getPosition().getZ() * 1e-6; // km
        }
        return data;
    }

    public double[][] positionsArray() {
        NumericalPropagator propagator = getpropagator();
        AbsoluteDate start = initorbit.getDate();

        int n = (int) Math.floor(tmax / dt) + 1;
        double[][] data = new double[n][4];

        for (int k = 0; k < n; k++) {
            double t = k * dt;
            SpacecraftState s = propagator.propagate(start.shiftedBy(t));
            PVCoordinates pv = s.getPVCoordinates();
            data[k][0] = t; // temps écoulé en secondes
            data[k][1] = pv.getPosition().getX()*1E-6;
            data[k][2] = pv.getPosition().getY()*1E-6;
            data[k][3] = pv.getPosition().getZ()*1E-6;
        }
        return data;
    }
}
