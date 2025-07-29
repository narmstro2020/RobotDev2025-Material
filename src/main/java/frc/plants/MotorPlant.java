package frc.plants;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class MotorPlant {

    public static final PerUnit<TorqueUnit, CurrentUnit> NewtonMetersPerAmp = NewtonMeters.per(Amp);
    public static final PerUnit<VoltageUnit, AngularVelocityUnit> VoltsPerRadiansPerSecond = Volts.per(RadiansPerSecond);

    private final Per<TorqueUnit, CurrentUnit> Kt;
    private final Per<VoltageUnit, AngularVelocityUnit> Ke;
    private final Resistance R;

    private final MutVoltage appliedVoltage = Volts.mutable(0.0);

    public MotorPlant(
            Voltage nominalVoltage,
            Torque stallTorque,
            Current stallCurrent,
            Current freeCurrent,
            AngularVelocity freeSpeed) {


        Kt = stallTorque.div(stallCurrent);
        R = nominalVoltage.div(stallCurrent);
        Ke = (nominalVoltage.div(freeSpeed).times(Value.one().minus(freeCurrent.div(stallCurrent))));
    }

    public Voltage getAppliedVoltage (AngularVelocity angularVelocity, Current current){
        return appliedVoltage
    }

}
