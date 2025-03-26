// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.LEDConstants;


public class LEDLights extends SubsystemBase {
  private final AddressableLED m_led = new AddressableLED(LEDConstants.kLEDPort);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kNumLEDs);

  /** Creates a new LEDLights. */
  public LEDLights() {
    m_led.setLength(LEDConstants.kNumLEDs);
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void setLEDColorGradient(Color... colors) {
    LEDPattern m_gradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, colors);
    m_gradient.applyTo(m_ledBuffer);
    m_led.setData(m_ledBuffer);
  }

  public void setLEDRainbow() {
    LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
    Distance ledSpacing = Meters.of(1 / 120.0)
    LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), ledSpacing);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
