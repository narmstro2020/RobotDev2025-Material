package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
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

public class IntakeCTRESubsystem implements Subsystem {

    public enum State {
        Stopped,
        VoltageControl
    }

    private final TalonFX talonFX;
    private final TalonFXSimState talonFXSimState;
    private final MutVoltage voltage = Volts.mutable(0.0);
    private final MutVoltage voltageSetpoint = Volts.mutable(0.0);
    private final String name;
    private final DoublePublisher voltagePublisher;
    private final DoublePublisher voltageSetpointPublisher;
    private final StringPublisher statePublisher;
    private IntakeRevSubsystem.State state = Stopped;

    private double lastTimeSeconds;


    public IntakeCTRESubsystem(String name, int deviceId, CANBus canbus) {
        this.name = name;
        talonFX = new TalonFX(deviceId, canbus);
        talonFXSimState = talonFX.getSimState();
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
        checkDisabled();    }

    @Override
    public void simulationPeriodic() {
        double currentTimeSeconds = Timer.getFPGATimestamp();
        double dtSeconds = currentTimeSeconds - lastTimeSeconds;
        lastTimeSeconds = currentTimeSeconds;

        double voltageCTRE = talonFX.getMotorVoltage().getValueAsDouble();
        double velocityCTRE = voltageCTRE / RobotController.getBatteryVoltage() * DCMotor.getKrakenX60(1).freeSpeedRadPerSec;
        talonFXSimState.setRotorVelocity(velocityCTRE);
    }

    @Override
    public String getName() {
        return name;
    }

    private void updateVoltage() {
        voltage.mut_replace(talonFX.getMotorVoltage().getValue());
    }

    private void checkDisabled() {
        if (DriverStation.isDisabled() && state != Stopped) {
            state = Stopped;
            voltageSetpoint.mut_replace(Volts.zero());
            talonFX.stopMotor();
        }
    }

    public Command createSetVoltage(Voltage voltage) {
        return runOnce(() -> state = VoltageControl)
                .andThen(runOnce(() -> voltageSetpoint.mut_replace(voltage)))
                .andThen(runOnce(() -> talonFX.setVoltage(voltageSetpoint.baseUnitMagnitude())))
                .withName("Set Voltage: " + voltage);
    }

    public Command createStop() {
        return runOnce(() -> state = Stopped)
                .andThen(runOnce(talonFX::stopMotor))
                .andThen(runOnce(() -> voltageSetpoint.mut_replace(Volts.zero())))
                .withName("Stop Intake");
    }
}
