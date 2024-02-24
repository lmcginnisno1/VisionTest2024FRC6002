// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;
import frc.utils.led.*;
import frc.utils.led.pattern.LEDPattern;

public class SUB_Led extends SubsystemBase {
  final LEDStrip m_ledStrip;
  public SUB_Led() {
    m_ledStrip = new LEDStrip(LedConstants.kLedStripLength, LedConstants.kLedStripOffest);
  }

  public void setPattern(LEDPattern pattern){
    m_ledStrip.setPattern(pattern);
  }

  public void setPatternDuration(double duration){
    m_ledStrip.setPatternDuration(duration);
  }

  public void setPrimaryColor(Color color){
    m_ledStrip.setPrimaryColor(color);
  }

  public void setSecondaryColor(Color color){
    m_ledStrip.setSecondaryColor(color);
  }



  @Override
  public void periodic() {

  }
}
