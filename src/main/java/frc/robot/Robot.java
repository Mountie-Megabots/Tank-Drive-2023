// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private XboxController controller= new XboxController(0);

  
  private final CANSparkMax m_leftLead = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax m_leftFollower = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax m_rightLead = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax m_rightFollower = new CANSparkMax(4, MotorType.kBrushless);
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftLead, m_rightLead);
  

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //m_rightMotor.setInverted(true);

    m_leftLead.restoreFactoryDefaults();
    m_leftFollower.restoreFactoryDefaults();
    m_rightLead.restoreFactoryDefaults();
    m_rightFollower.restoreFactoryDefaults();

    m_leftFollower.follow(m_leftLead);
    m_rightFollower.follow(m_rightLead);

  }

  @Override
  public void teleopPeriodic() {

    //m_leftLead.set(controller.getLeftY());
    //m_rightLead.set(controller.getRightY());
    m_drive.arcadeDrive(controller.getRightX(),controller.getLeftY(), true);
  }
}
