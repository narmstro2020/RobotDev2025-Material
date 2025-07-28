// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.*;

import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;

public class Robot extends TimedRobot {

    private final CANBus canbus = new CANBus("rio");
    private final IntakeCTRE intakeCTRE = new IntakeCTRE(15, canbus);
    private final IntakeRev intakeRev = new IntakeRev(14, kBrushless, DCMotor.getNeo550(1));

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

        if (xboxController.getBButton()){
            intakeCTRE.setVoltage(6.0);
        }

        if (xboxController.getBButtonReleased()){
            intakeCTRE.setVoltage(0.0);
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
        double dtSeconds = currentTimeSeconds - lastTimeSeconds;
        lastTimeSeconds = currentTimeSeconds;

        intakeRev.simulate(dtSeconds);
        intakeCTRE.simulate(dtSeconds);
    }
}
