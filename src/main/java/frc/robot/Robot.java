// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.units.Units.Volts;

public class Robot extends TimedRobot {

    public Robot() {
        CommandXboxController controller = new CommandXboxController(0);

        CANBus canbus = new CANBus("rio");
        IntakeSubsystem intakeCTRESubsystem = new IntakeCTRESubsystem("CTRE Intake", 15, canbus);
        IntakeSubsystem intakeRevSubsystem = new IntakeRevSubsystem("Rev Intake", 14, kBrushless, DCMotor.getNeo550(1));

        controller.a().onTrue(intakeRevSubsystem.createSetVoltage(Volts.of(6.0)));
        controller.a().onFalse(intakeRevSubsystem.createStop());
        RobotModeTriggers.teleop().onFalse(intakeRevSubsystem.createStop());


        controller.b().onTrue(intakeCTRESubsystem.createSetVoltage(Volts.of(6.0)));
        controller.b().onFalse(intakeCTRESubsystem.createStop());
        RobotModeTriggers.teleop().onFalse(intakeCTRESubsystem.createStop());



        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());


    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
