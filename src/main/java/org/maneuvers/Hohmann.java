package org.maneuvers;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.ode.nonstiff.DormandPrince853Integrator;
import org.hipparchus.util.FastMath;
import org.orekit.attitudes.AttitudeProvider;
import org.orekit.attitudes.LofOffset;
import org.orekit.forces.maneuvers.ImpulseManeuver;
import org.orekit.frames.LOFType;
import org.orekit.orbits.KeplerianOrbit;
import org.orekit.orbits.OrbitType;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.events.DateDetector;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.numerical.NumericalPropagator;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import java.util.ArrayList;
import java.util.List;

public class Hohmann {

    private final double altfinal;
    private final KeplerianOrbit initialOrbit;
    private final AbsoluteDate initialDate;
    private final double initialmass;
    private final double ispsat;

    //Constructor for the Hohmann transfer
    public Hohmann(double afinal, KeplerianOrbit initialOrbit) {
        this.altfinal = afinal;                                      // Set altitude objective
        this.initialOrbit = initialOrbit;                              // Set the initial Orbit
        this.initialDate = initialOrbit.getDate().shiftedBy(1);     // Set the date for the first maneuver
        this.initialmass = 1000;                                       // Set the initial ergol mass of the satellite
        this.ispsat = 400;                                             // Set the satellite motor Isp
    }


    //Constructor for the Hohmann transfert with a given date for the fisrt maneuver
    public Hohmann(double afinal, KeplerianOrbit initialOrbit, AbsoluteDate initialDate) {
        this.altfinal = afinal;
        this.initialOrbit = initialOrbit;
        this.initialDate = initialDate;
        this.initialmass = 100000;
        this.ispsat = 400;
    }

    //Constructor for the Hohmann transfert with propulsion data
    public Hohmann(double afinal, KeplerianOrbit initialOrbit, double ispsat, double initialmass) {
        this.altfinal = afinal;
        this.initialOrbit = initialOrbit;
        this.initialDate = initialOrbit.getDate().shiftedBy(1);
        this.initialmass = initialmass;
        this.ispsat = ispsat;
    }

    //Constructor for the Hohmann transfert with date for the first maneuver and propulsion data
    public Hohmann(double afinal, KeplerianOrbit initialOrbit, AbsoluteDate initialDate, double ispsat, double initialmass) {
        this.altfinal = afinal;
        this.initialOrbit = initialOrbit;
        this.initialDate = initialDate;
        this.initialmass = initialmass;
        this.ispsat = ispsat;
    }

    //Find the initial Velocity of the initial Orbit
    private double getVinitial(){
        return initialOrbit.getPVCoordinates().getVelocity().getNorm();
    }

    //Find the semi major-axis of the transfert orbit
    private double getATransfert(){
        return  (initialOrbit.getA()+(altfinal))/2;
    }

    // Find the specific mechanical energy of the transfert orbit
    private double getTransfertEnergy(){
        return -(Constants.EGM96_EARTH_MU)/(2*getATransfert());

    }

    // Find velocity after the first impulse (periapsis velocity of the transfert orbit)
    private double getVTransfert1(){
        return Math.sqrt(2*((Constants.EGM96_EARTH_MU/initialOrbit.getA())+getTransfertEnergy()));
    }

    // Find the velocity before the second impulse (apoapsis velocity of the transfert orbit)
    private double getVTransfert2(){
        return Math.sqrt(2*((Constants.EGM96_EARTH_MU/(altfinal))+getTransfertEnergy()));
    }

    // Find the  mechanical energy of the final orbit
    private double getFinalEnergy(){
        return -(Constants.EGM96_EARTH_MU)/(2*(altfinal));
    }

    // Determine the specific mechanical energy of the final orbit
    private double getVfinal(){
        return Math.sqrt(2*((Constants.EGM96_EARTH_MU/(altfinal))+getFinalEnergy()));
    }

    // Determine the DV needed for the first impulse maneuver
    public double getDV1(){
        return getVTransfert1()- getVinitial();
    }

    // Determine the DV needed for the second impulse maneuver
    public double getDV2(){
        return getVfinal()-getVTransfert2();
    }
    // Determine the DV needed for the two impulse maneuver
    public double getDVtotal(){
        return getDV1()+getDV2();
    }

    // Get the date of the first maneuver
    public AbsoluteDate getFirstManeuverDate() {
        return initialDate;
    }

    // Get the Date of the second maneuver
    public AbsoluteDate getSecondManeuverDate() {
        return initialDate.shiftedBy(tof());
    }

    // Create a DateDetector for the first impulse maneuver
    private EventDetector eventDetector1(){
        AbsoluteDate date = getFirstManeuverDate();
        return new DateDetector(date);
    }

    // Create a DateDetector for the second maneuver
    private EventDetector eventDetector2(){
        AbsoluteDate date2 =  getSecondManeuverDate();
        return new DateDetector(date2);
    }

    // Compute the first impulse maneuver in TWC satellite frame
    private ImpulseManeuver impulseManeuver1(){
        final Vector3D impulse = new Vector3D(getDV1(),0,0);
        return new ImpulseManeuver(eventDetector1(),impulse,ispsat);
    }

    // Compute the second impulse maneuver in TWC satellite frame
    private ImpulseManeuver impulseManeuver2(){
        final Vector3D impulse = new Vector3D(getDV2(),0,0);
        return new ImpulseManeuver(eventDetector2(),impulse,ispsat);
    }

    // Create a list with the two impulse maneuvers
    public List<ImpulseManeuver> computeManeuvers() {
        final List<ImpulseManeuver> maneuvers = new ArrayList<>();
        maneuvers.add(impulseManeuver1());
        maneuvers.add(impulseManeuver2());

        return maneuvers;
    }

    // Compute the amout of ergol used during the first maneuver
    public double getFirstManeuverErgolUsed(){
        double deltaV = Math.abs(getDV1());
        double g0 = Constants.G0_STANDARD_GRAVITY; // 9.80665 m/s²

        return initialmass * (1 - FastMath.exp(-deltaV / (g0 * ispsat)));
    }

    //Compute the ergol mass after the first maneuver
    private double satMassAfterFirstManeuver(){
        return initialmass - getFirstManeuverErgolUsed();
    }

    // Compute the amout of ergol used during the second maneuver
    public double getSecondManeuverErgolUsed(){
        double deltaV = Math.abs(getDV2());
        double g0 = Constants.G0_STANDARD_GRAVITY; // 9.80665 m/s²

        return satMassAfterFirstManeuver() * (1 - FastMath.exp(-deltaV / (g0 * ispsat)));
    }

    // Compute the ergol mass after the second maneuver
    private double satMassAfterSecondManeuver(){
        return satMassAfterFirstManeuver() - getSecondManeuverErgolUsed();
    }

    // Get the total amont of ergol mass used durinf the Hohmann transfert
    public double getTotalErgolUsed(){
        return initialmass - satMassAfterSecondManeuver();
    }

    // Determine the time of transfert (half of the elliptical orbit period)
    public double tof(){
        return FastMath.PI*Math.sqrt((Math.pow(getATransfert(),3))/Constants.EGM96_EARTH_MU);
    }

}
