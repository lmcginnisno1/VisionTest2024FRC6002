// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;
import frc.utils.led.*;
import frc.utils.led.pattern.LEDPattern;

public class SUB_Led extends SubsystemBase {
  final PWMLEDController m_ledController;
  public SUB_Led() {
    m_ledController = new PWMLEDController(LedConstants.kLedStripPWMPort);
    try{
      m_ledController.addStrip(new LEDStrip(LedConstants.kLedStripLength, LedConstants.kLedStripOffest));
    }catch(DuplicateLEDAssignmentException e){
      DriverStation.reportError("duplicate LED asignment", false);
      e.printStackTrace();
    }
  }

  public void setPattern(LEDPattern pattern){
    m_ledController.getStrip(LedConstants.kLedStripControllerIndex).setPattern(pattern);
  }

  public void setPatternDuration(double duration){
    m_ledController.getStrip(LedConstants.kLedStripControllerIndex).setPatternDuration(duration);
  }

  public void setLedColors(Color PrimaryColor, Color SecondaryColor){
    m_ledController.getStrip(LedConstants.kLedStripControllerIndex).setPrimaryColor(PrimaryColor);
    m_ledController.getStrip(LedConstants.kLedStripControllerIndex).setSecondaryColor(SecondaryColor);
  }

  @Override
  public void periodic(){
    m_ledController.updateStrips();
  }
}
