package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;

public class IntakeCTRE {

    private final TalonFX talonFX;
    private final TalonFXSimState talonFXSimState;

    public IntakeCTRE(int deviceId, CANBus canbus) {
        talonFX = new TalonFX(deviceId, canbus);
        talonFXSimState = talonFX.getSimState();

    }

    public void setVoltage(double voltage) {
        talonFX.setVoltage(voltage);
    }

    public void simulate(double dtSeconds) {
        double voltageCTRE = talonFX.getMotorVoltage().getValueAsDouble();
        double velocityCTRE = voltageCTRE / RobotController.getBatteryVoltage() * DCMotor.getKrakenX60(1).freeSpeedRadPerSec;
        talonFXSimState.setRotorVelocity(velocityCTRE);
    }
}
