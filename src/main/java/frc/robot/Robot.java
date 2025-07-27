// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class Robot extends TimedRobot {

    private final XboxController xboxController = new XboxController(0);
    private final LEDPattern red = LEDPattern.solid(Color.kRed);
    private final LEDPattern black = LEDPattern.solid(Color.kBlack);
    private final LEDPattern rainbow = LEDPattern.rainbow(255, 255);
    private final Distance ledSpacing = Meters.of(1 / 100.0);
    private final LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), ledSpacing);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(100);
    private final AddressableLED addressableLED = new AddressableLED(0);

    public Robot() {
        addressableLED.setLength(buffer.getLength());
        addressableLED.setData(buffer);
        addressableLED.start();
    }

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopPeriodic() {
        if (xboxController.getAButton()) {
            red.applyTo(buffer);
            addressableLED.setData(buffer);
        }

        if (xboxController.getAButtonReleased()) {
            black.applyTo(buffer);
            addressableLED.setData(buffer);
        }

        if (xboxController.getBButton()) {
            rainbow.applyTo(buffer);
            addressableLED.setData(buffer);
        }

        if (xboxController.getBButtonReleased()) {
            black.applyTo(buffer);
            addressableLED.setData(buffer);
        }

        if (xboxController.getXButton()) {
            scrollingRainbow.applyTo(buffer);
            addressableLED.setData(buffer);
        }

        if(xboxController.getXButtonReleased()){
            black.applyTo(buffer);
            addressableLED.setData(buffer);
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

    }
}
