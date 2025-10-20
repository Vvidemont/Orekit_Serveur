package org.maneuvers;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.frames.Frame;
import org.orekit.orbits.KeplerianOrbit;
import org.orekit.propagation.events.*;
import org.orekit.time.AbsoluteDate;


public class InclinationChange {

    public KeplerianOrbit initialOrbit;
    private final double iFinal;
    private boolean ascendingnode;
    private AbsoluteDate date_man;

    public InclinationChange(KeplerianOrbit initialOrbit, double iFinal){
        this.initialOrbit = initialOrbit;
        this.iFinal = iFinal;
        this.ascendingnode = true;
        this.date_man = null;
    }

    public InclinationChange(KeplerianOrbit initialOrbit, double iFinal, boolean ascendingnode,AbsoluteDate date_man){
        this.initialOrbit = initialOrbit;
        this.iFinal = iFinal;
        this.ascendingnode = ascendingnode;
        this.date_man = date_man;
    }

    public double getDeltaVInit() {
        return initialOrbit.getPVCoordinates().getVelocity().getNorm();
    }

    public double getInclinationChange() {
        return iFinal - initialOrbit.getI();
    }

    public double getDeltaVManeuver() {
        if (ascendingnode){
            return 2 * getDeltaVInit() * FastMath.sin(getInclinationChange() / 2);
        }
        else
            return -2 * getDeltaVInit() * FastMath.sin(getInclinationChange() / 2);
    }

    private EventDetector getEventDetector() {
        final Frame frame = initialOrbit.getFrame();
        final double i0 = initialOrbit.getI();

        if (FastMath.abs(i0) < 1e-5) {
            if (date_man == null) {
                AbsoluteDate date = initialOrbit.getDate().shiftedBy(0);
                return new DateDetector(date);
            }
            else {
                return new DateDetector(date_man);
            }
        }
        else {
            if (ascendingnode) {
                return new EventSlopeFilter<>(new NodeDetector(initialOrbit, frame), FilterType.TRIGGER_ONLY_INCREASING_EVENTS);
            } else {
                return new EventSlopeFilter<>(new NodeDetector(initialOrbit, frame), FilterType.TRIGGER_ONLY_DECREASING_EVENTS);
            }
        }
    }

    public ImpulseManeuver computeManeuver() {

        double deltaV = getDeltaVManeuver();

        Vector3D impulse = new Vector3D(0, 0, deltaV);

        return new ImpulseManeuver(getEventDetector(), impulse, 400); // Isp
    }
}
