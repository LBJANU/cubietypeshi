// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/** Represents a differential drive style drivetrain. */
public class Drivetrain extends TimedRobot {
  WPI_TalonFX drive_L1 = new WPI_TalonFX(0);
  WPI_TalonFX drive_L2 = new WPI_TalonFX(1);
  WPI_TalonFX drive_L3 = new WPI_TalonFX(2);

  WPI_TalonFX drive_R1 = new WPI_TalonFX(3);
  WPI_TalonFX drive_R2 = new WPI_TalonFX(4);
  WPI_TalonFX drive_R3 = new WPI_TalonFX(5);
  /**
   * Constructs a differential drive object. Sets the encoder distance per pulse and resets the
   * gyro.
   */
  public Drivetrain() {
    drive_L1.configFactoryDefault();
    drive_L2.configFactoryDefault();
    drive_L3.configFactoryDefault();

    drive_R1.configFactoryDefault();
    drive_R2.configFactoryDefault();
    drive_R3.configFactoryDefault();

    drive_R1.setInverted(true);
    drive_R2.setInverted(true);
    drive_R3.setInverted(true);
  }
  public void Drive(double forward, double rotation)
  {
    drive_L1.set(forward - rotation);
    drive_L2.set(forward - rotation);
    drive_L3.set(forward - rotation);

    drive_R1.set(forward + rotation);
    drive_R2.set(forward + rotation);
    drive_R3.set(forward + rotation);

   
  }

  public double Deadband(double stick, double val)
  {
   if (Math.abs(stick) < val)
   {
     return 0.0;
   }
   else if (stick > 0.95)
   {
     return 1.0;
   }

   else if (stick < -0.95)
   {
     return -1.0;
   }
   else
   {
     return stick;
   }
  }
}
