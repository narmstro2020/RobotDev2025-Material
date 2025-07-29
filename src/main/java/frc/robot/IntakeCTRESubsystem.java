package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;

public class IntakeCTRESubsystem extends IntakeSubsystem {

    private final TalonFX talonFX;
    private final TalonFXSimState talonFXSimState;

    public IntakeCTRESubsystem(String name, int deviceId, CANBus canbus) {
        super(name);
        talonFX = new TalonFX(deviceId, canbus);
        talonFXSimState = talonFX.getSimState();
    }

    @Override
    protected final void updateStateVariables() {
        voltage.mut_replace(talonFX.getMotorVoltage().getValue());
        current.mut_replace(talonFX.getTorqueCurrent().getValue());
    }

    @Override
    protected final void updateMotorSim() {
        double voltageCTRE = talonFX.getMotorVoltage().getValueAsDouble();
        double velocityCTRE = voltageCTRE / RobotController.getBatteryVoltage() * DCMotor.getKrakenX60(1).freeSpeedRadPerSec;
        talonFXSimState.setRotorVelocity(velocityCTRE);
    }

    @Override
    protected final void setVoltage() {
        talonFX.setVoltage(voltageSetpoint.baseUnitMagnitude());
    }

    @Override
    protected final void stopMotor() {
        talonFX.stopMotor();
    }
}
