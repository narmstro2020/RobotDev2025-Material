// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.*;

import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;

public class Robot extends TimedRobot {

    private final SparkMax intakeRev = new SparkMax(15, kBrushless);
    private final SparkMaxSim intakeRevSim = new SparkMaxSim(intakeRev, DCMotor.getNeo550(1));
    private final XboxController xboxController = new XboxController(0);
    private double lastTimeSeconds;

    public Robot() {
    }


    @Override
    public void robotPeriodic() {
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopPeriodic() {
        if(xboxController.getAButton()){
            intakeRev.setVoltage(6.0);
        }

        if(xboxController.getAButtonReleased()){
            intakeRev.setVoltage(0.0);
        }

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void simulationPeriodic() {
        double currentTimeSeconds = Timer.getFPGATimestamp();
        double dt = currentTimeSeconds - lastTimeSeconds;
        lastTimeSeconds = currentTimeSeconds;
        double voltage = intakeRev.getBusVoltage() * intakeRev.getAppliedOutput();
        double velocity = voltage / RobotController.getBatteryVoltage() * DCMotor.getNeo550(1).freeSpeedRadPerSec;
        intakeRevSim.iterate(velocity, RobotController.getBatteryVoltage(), dt);
    }
}
