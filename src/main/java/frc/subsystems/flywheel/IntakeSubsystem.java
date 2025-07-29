package frc.subsystems.flywheel;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.subsystems.flywheel.IntakeSubsystem.State.Stopped;
import static frc.subsystems.flywheel.IntakeSubsystem.State.VoltageControl;

public abstract class IntakeSubsystem implements Subsystem {

    public enum State {
        Stopped,
        VoltageControl
    }

    private final String name;
    protected final MutCurrent current = Amps.mutable(0.0);
    protected final MutVoltage voltage = Volts.mutable(0.0);
    protected final MutVoltage voltageSetpoint = Volts.mutable(0.0);
    private final DoublePublisher currentPublisher;
    private final DoublePublisher voltagePublisher;
    private final DoublePublisher voltageSetpointPublisher;
    private final StringPublisher statePublisher;
    protected IntakeRevSubsystem.State state = Stopped;
    private double lastTimeSeconds;
    protected double dtSeconds;

    protected IntakeSubsystem(String name) {
        this.name = name;
        NetworkTable table = NetworkTableInstance.getDefault().getTable(getName());
        currentPublisher = table.getDoubleTopic("Current [A]").publish();
        voltagePublisher = table.getDoubleTopic("Voltage [V]").publish();
        voltageSetpointPublisher = table.getDoubleTopic("Voltage Setpoint [V]").publish();
        statePublisher = table.getStringTopic("State").publish();
        currentPublisher.set(current.baseUnitMagnitude());
        voltagePublisher.set(voltage.baseUnitMagnitude());
        voltageSetpointPublisher.set(voltageSetpoint.baseUnitMagnitude());
        statePublisher.set(state.name());
        register();
    }

    protected abstract void updateStateVariables();

    private void checkDisabled(){
        if (DriverStation.isDisabled() && state != Stopped) {
            state = Stopped;
            voltageSetpoint.mut_replace(Volts.zero());
            stopMotor();
        }
    }

    @Override
    public final void periodic() {
        updateStateVariables();
        currentPublisher.set(current.baseUnitMagnitude());
        voltagePublisher.set(voltage.baseUnitMagnitude());
        voltageSetpointPublisher.set(voltageSetpoint.baseUnitMagnitude());
        statePublisher.set(state.name());
        checkDisabled();
    }

    protected abstract void updateMotorSim();

    @Override
    public final void simulationPeriodic() {
        double currentTimeSeconds = Timer.getFPGATimestamp();
        dtSeconds = currentTimeSeconds - lastTimeSeconds;
        lastTimeSeconds = currentTimeSeconds;
        updateMotorSim();
    }

    @Override
    public final String getName() {
        return name;
    }

    protected abstract void setVoltage();

    protected abstract void stopMotor();

    private void setStateToVoltage(){
        state = VoltageControl;
    }

    private void setStateToStopped(){
        state = Stopped;
    }

    public final Command createSetVoltage(Voltage voltage) {
        return runOnce(this::setStateToVoltage)
                .andThen(runOnce(() -> voltageSetpoint.mut_replace(voltage)))
                .andThen(runOnce(this::setVoltage))
                .withName("Set Voltage: " + voltage);
    }

    public final Command createStop() {
        return runOnce(this::setStateToStopped)
                .andThen(runOnce(this::stopMotor))
                .andThen(runOnce(() -> voltageSetpoint.mut_replace(Volts.zero())))
                .withName("Stop Intake");
    }
}
