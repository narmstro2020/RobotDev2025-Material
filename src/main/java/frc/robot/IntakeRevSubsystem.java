package frc.robot;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeRevSubsystem implements Subsystem {

    private final SparkMax sparkMax;
    private final SparkMaxSim sparkMaxSim;
    private double lastTimeSeconds;

    public IntakeRevSubsystem(int deviceId, MotorType motorType, DCMotor dcMotor) {
        sparkMax = new SparkMax(deviceId, motorType);
        sparkMaxSim = new SparkMaxSim(sparkMax, dcMotor);
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

        double voltage = sparkMax.getBusVoltage() * sparkMax.getAppliedOutput();
        double velocity = voltage / RobotController.getBatteryVoltage() * DCMotor.getNeo550(1).freeSpeedRadPerSec;
        sparkMaxSim.iterate(velocity, RobotController.getBatteryVoltage(), dtSeconds);
    }

    public Command createSetVoltage(double voltage){
        return run(() -> sparkMax.setVoltage(voltage)).withName("REV Set Voltage Command");
    }

    public Command createStop(){
        return run(sparkMax::stopMotor).withName("REV Stop Motor Command");
    }
}
