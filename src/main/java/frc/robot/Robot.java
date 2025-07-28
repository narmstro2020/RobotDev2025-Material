// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;

public class Robot extends TimedRobot {

    public Robot() {
        CommandXboxController controller = new CommandXboxController(0);

        CANBus canbus = new CANBus("rio");
        IntakeCTRESubsystem intakeCTRESubsystem = new IntakeCTRESubsystem(15, canbus);
        IntakeRevSubsystem intakeRevSubsystem = new IntakeRevSubsystem(14, kBrushless, DCMotor.getNeo550(1));

        controller.a().whileTrue(intakeRevSubsystem.createSetVoltage(6.0));
        controller.b().whileTrue(intakeCTRESubsystem.createSetVoltage(6.0));

        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());


    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
