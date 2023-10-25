// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
  //private final XboxController m_controller = new XboxController(0);
  private final PS4Controller m_driver = new PS4Controller(0);
  private final PS4Controller m_operator = new PS4Controller(1);
  private final Drivetrain m_drive = new Drivetrain();
  private final Intake m_intake = new Intake();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
 

  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
   
  }

  @Override
  public void teleopPeriodic() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double forward = m_driver.getLeftY();
    double rotation = m_driver.getRightX();

    forward = m_drive.Deadband(forward, 0.05);
    rotation = m_drive.Deadband(rotation, 0.05);

    m_drive.Drive(forward * 1.0, rotation * 1.0);

    double intakevalue = m_driver.getR2Axis() - m_driver.getL2Axis();
    m_intake.Set(intakevalue);

  

    
  //}
   
  }
}
