package frc.robot;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;

public class IntakeRev {

    private final SparkMax sparkMax;
    private final SparkMaxSim sparkMaxSim;

    public IntakeRev(int deviceId, MotorType motorType, DCMotor dcMotor) {
        sparkMax = new SparkMax(deviceId, motorType);
        sparkMaxSim = new SparkMaxSim(sparkMax, dcMotor);
    }

    public void setVoltage(double voltage) {
        sparkMax.setVoltage(voltage);
    }

    public void simulate(double dtSeconds){
        double voltage = sparkMax.getBusVoltage() * sparkMax.getAppliedOutput();
        double velocity = voltage / RobotController.getBatteryVoltage() * DCMotor.getNeo550(1).freeSpeedRadPerSec;
        sparkMaxSim.iterate(velocity, RobotController.getBatteryVoltage(), dtSeconds);
    }
}
