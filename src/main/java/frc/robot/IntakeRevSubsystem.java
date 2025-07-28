package frc.robot;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.IntakeRevSubsystem.State.Stopped;
import static frc.robot.IntakeRevSubsystem.State.VoltageControl;

public class IntakeRevSubsystem implements Subsystem {


    public enum State {
        Stopped,
        VoltageControl
    }

    private final SparkMax sparkMax;
    private final SparkMaxSim sparkMaxSim;
    private final MutVoltage voltage = Volts.mutable(0.0);
    private final MutVoltage voltageSetpoint = Volts.mutable(0.0);
    private final String name;
    private final DoublePublisher voltagePublisher;
    private final DoublePublisher voltageSetpointPublisher;
    private final StringPublisher statePublisher;
    private State state = Stopped;

    private double lastTimeSeconds;

    public IntakeRevSubsystem(String name, int deviceId, MotorType motorType, DCMotor dcMotor) {
        this.name = name;
        sparkMax = new SparkMax(deviceId, motorType);
        sparkMaxSim = new SparkMaxSim(sparkMax, dcMotor);
        NetworkTable table = NetworkTableInstance.getDefault().getTable(getName());
        voltagePublisher = table.getDoubleTopic("Voltage [V]").publish();
        voltageSetpointPublisher = table.getDoubleTopic("Voltage Setpoint [V]").publish();
        statePublisher = table.getStringTopic("State").publish();
        voltagePublisher.set(voltage.baseUnitMagnitude());
        voltageSetpointPublisher.set(voltageSetpoint.baseUnitMagnitude());
        statePublisher.set(state.name());
        register();
    }

    @Override
    public void periodic() {
        updateVoltage();
        voltagePublisher.set(voltage.baseUnitMagnitude());
        voltageSetpointPublisher.set(voltageSetpoint.baseUnitMagnitude());
        statePublisher.set(state.name());
        checkDisabled();
    }


    @Override
    public void simulationPeriodic() {
        double currentTimeSeconds = Timer.getFPGATimestamp();
        double dtSeconds = currentTimeSeconds - lastTimeSeconds;
        lastTimeSeconds = currentTimeSeconds;

        double voltage = sparkMax.getBusVoltage() * sparkMax.getAppliedOutput();
        double velocity = voltage / RobotController.getBatteryVoltage() * DCMotor.getNeo550(1).freeSpeedRadPerSec;
        sparkMaxSim.iterate(velocity, RobotController.getBatteryVoltage(), dtSeconds);
    }

    @Override
    public String getName() {
        return name;
    }

    private void updateVoltage() {
        voltage.mut_setBaseUnitMagnitude(sparkMax.getBusVoltage() * sparkMax.getAppliedOutput());
    }

    private void checkDisabled() {
        if (DriverStation.isDisabled() && state != Stopped) {
            state = Stopped;
            voltageSetpoint.mut_replace(Volts.zero());
            sparkMax.stopMotor();
        }
    }

    public Command createSetVoltage(Voltage voltage) {
        return runOnce(() -> state = VoltageControl)
                .andThen(runOnce(() -> voltageSetpoint.mut_replace(voltage)))
                .andThen(runOnce(() -> sparkMax.setVoltage(voltageSetpoint)))
                .withName("Set Voltage: " + voltage);
    }

    public Command createStop() {
        return runOnce(() -> state = Stopped)
                .andThen(runOnce(sparkMax::stopMotor))
                .andThen(runOnce(() -> voltageSetpoint.mut_replace(Volts.zero())))
                .withName("Stop Intake");
    }
}
