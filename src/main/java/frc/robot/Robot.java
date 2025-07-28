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
import frc.subsystems.flywheel.FlywheelSubsystem;
import frc.subsystems.flywheel.RevFlywheelSubsystem;

import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.units.Units.Volts;

public class Robot extends TimedRobot {

    public Robot() {
        CommandXboxController controller = new CommandXboxController(0);

        CANBus canbus = new CANBus("rio");
        IntakeCTRESubsystem intakeCTRESubsystem = new IntakeCTRESubsystem(15, canbus);
        FlywheelSubsystem intakeRevSubsystem = new RevFlywheelSubsystem("REV Flywheel", 14, kBrushless, DCMotor.getNeo550(1));

        controller.a().onTrue(intakeRevSubsystem.createSetVoltage(Volts.of(6.0)));
        controller.a().onFalse(intakeRevSubsystem.createStop());
        controller.b().onTrue(intakeCTRESubsystem.createSetVoltage(6.0));
        controller.b().onFalse(intakeRevSubsystem.createStop());


        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());


    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
