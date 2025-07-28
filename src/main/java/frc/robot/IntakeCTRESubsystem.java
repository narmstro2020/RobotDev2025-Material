package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeCTRESubsystem implements Subsystem {

    private final TalonFX talonFX;
    private final TalonFXSimState talonFXSimState;
    private double lastTimeSeconds;

    public IntakeCTRESubsystem(int deviceId, CANBus canbus) {
        talonFX = new TalonFX(deviceId, canbus);
        talonFXSimState = talonFX.getSimState();
        setDefaultCommand(createStop());
    }

    @Override
    public void periodic() {
        // Useful for updating subsystem-specific state.
    }

    @Override
    public void simulationPeriodic() {
        double currentTimeSeconds = Timer.getFPGATimestamp();
        double dtSeconds = currentTimeSeconds - lastTimeSeconds;
        lastTimeSeconds = currentTimeSeconds;

        double voltageCTRE = talonFX.getMotorVoltage().getValueAsDouble();
        double velocityCTRE = voltageCTRE / RobotController.getBatteryVoltage() * DCMotor.getKrakenX60(1).freeSpeedRadPerSec;
        talonFXSimState.setRotorVelocity(velocityCTRE);
    }

    public Command createSetVoltage(double voltage){
        return run(() -> talonFX.setVoltage(voltage)).withName("CTRE Set Voltage Command");
    }

    public Command createStop(){
        return run(talonFX::stopMotor).withName("CTRE Stop Motor Command");
    }
}
