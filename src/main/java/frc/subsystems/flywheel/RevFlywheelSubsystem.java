package frc.subsystems.flywheel;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Volts;

public class RevFlywheelSubsystem extends FlywheelSubsystem {

    private final SparkMax sparkMax;
    private final SparkMaxSim sparkMaxSim;
    private final String name;
    private final MutVoltage voltage = Volts.mutable(0.0);

    public RevFlywheelSubsystem(String name, int deviceId, MotorType motorType, DCMotor dcMotor) {
        this.name = name;
        sparkMax = new SparkMax(deviceId, motorType);
        sparkMaxSim = new SparkMaxSim(sparkMax, dcMotor);
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public void periodic() {
        if (state == State.Stopped) {
            sparkMax.stopMotor();
            SmartDashboard.putString(getName() + ": " + "Current State: ", state.name());
        } else if (state == State.Voltage) {
            sparkMax.setVoltage(voltageSetpoint.baseUnitMagnitude());
            SmartDashboard.putString(getName() + ": " + "Current State: ", state.name() + ": " + voltageSetpoint);
        }
        super.periodic();
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
        double voltage = sparkMax.getBusVoltage() * sparkMax.getAppliedOutput();
        double velocity = voltage / RobotController.getBatteryVoltage() * DCMotor.getNeo550(1).freeSpeedRadPerSec;
        sparkMaxSim.iterate(velocity, RobotController.getBatteryVoltage(), dtSeconds);
    }

    @Override
    protected Voltage getVoltage() {
        return voltage.mut_setBaseUnitMagnitude(
                sparkMax.getBusVoltage() * sparkMax.getAppliedOutput());
    }
}
