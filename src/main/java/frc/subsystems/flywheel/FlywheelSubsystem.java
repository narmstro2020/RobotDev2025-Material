package frc.subsystems.flywheel;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static edu.wpi.first.units.Units.Volts;
import static frc.subsystems.flywheel.FlywheelSubsystem.State.*;

public abstract class FlywheelSubsystem implements Subsystem {

    protected enum State {
        Voltage,
        Stopped,
    }

    protected final MutVoltage voltageSetpoint = Volts.mutable(0.0);

    protected State state = Stopped;
    private double lastTimeSeconds;
    protected double dtSeconds;
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable(getName());
    private final DoublePublisher voltagePublisher = table.getDoubleTopic("Voltage [V]")
            .publish();
    private final StringPublisher statePublisher = table.getStringTopic("State")
            .publish();


    protected FlywheelSubsystem() {
        register();
        voltagePublisher.set(voltageSetpoint.baseUnitMagnitude());
        statePublisher.set(state.name());
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        double currentTimeSeconds = Timer.getFPGATimestamp();
        dtSeconds = currentTimeSeconds - lastTimeSeconds;
        lastTimeSeconds = currentTimeSeconds;
    }

    protected abstract Voltage getVoltage();


    public Command createSetVoltage(Voltage voltage) {
        return runOnce(() -> voltageSetpoint.mut_replace(voltage))
                .andThen(() -> state = Voltage)
                .withName("Set Voltage: " + voltage.toString());
    }

    public Command createStop() {
        return runOnce(() -> voltageSetpoint.mut_setBaseUnitMagnitude(0.0))
                .andThen(() -> state = Stopped)
                .withName("Stop Flywheel");
    }
}
