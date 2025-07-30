package frc.robot;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;

public class IntakeRevSubsystem extends IntakeSubsystem {

    private final SparkMax sparkMax;
    private final SparkMaxSim sparkMaxSim;

    public IntakeRevSubsystem(String name, int deviceId, MotorType motorType, DCMotor dcMotor) {
        super(name);
        sparkMax = new SparkMax(deviceId, motorType);
        sparkMaxSim = new SparkMaxSim(sparkMax, dcMotor);
    }

    @Override
    protected final void updateStateVariables() {
        voltage.mut_setBaseUnitMagnitude(sparkMax.getBusVoltage() * sparkMax.getAppliedOutput());
        current.mut_setBaseUnitMagnitude(sparkMax.getOutputCurrent());
    }

    @Override
    protected final void updateMotorSim() {
        double voltage = sparkMax.getBusVoltage() * sparkMax.getAppliedOutput();
        double velocity = voltage / RobotController.getBatteryVoltage() * DCMotor.getNeo550(1).freeSpeedRadPerSec;
        sparkMaxSim.iterate(velocity, RobotController.getBatteryVoltage(), dtSeconds);
    }

    @Override
    protected final void setVoltage() {
        sparkMax.setVoltage(voltageSetpoint);
    }

    @Override
    protected final void stopMotor() {
        sparkMax.stopMotor();
    }
}
