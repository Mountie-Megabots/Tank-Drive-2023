// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private final CANSparkMax m_leftLead = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax m_leftFollower = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax m_rightLead = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax m_rightFollower = new CANSparkMax(4, MotorType.kBrushless);




  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftLead, m_rightLead);

  // The left-side drive encoder
  private final RelativeEncoder m_leftEncoder;

  // The right-side drive encoder
  private final RelativeEncoder m_rightEncoder;

  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(5);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_leftLead.restoreFactoryDefaults();
    m_leftFollower.restoreFactoryDefaults();
    m_rightLead.restoreFactoryDefaults();
    m_rightFollower.restoreFactoryDefaults();
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightLead.setInverted(true);

    m_leftFollower.follow(m_leftLead);
    m_rightFollower.follow(m_rightLead);

    // Sets the distance per pulse for the encoders
    m_leftEncoder = m_leftLead.getEncoder();
    m_rightEncoder = m_rightLead.getEncoder();
    //m_rightEncoder.setInverted(true);

    // TODO: Fix encoder scale
    m_leftEncoder.setPositionConversionFactor(0.04468674122);
    m_rightEncoder.setPositionConversionFactor(0.04468674122);
    //m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    //m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry =
        new DifferentialDriveOdometry(
            this.getGyroRotation(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    SmartDashboard.putNumber("Left Encoder", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder", m_rightEncoder.getPosition());

    double gravVector[] = {0,0,0};
    m_gyro.getGravityVector(gravVector);
    SmartDashboard.putNumber("Grav X", gravVector[0]);
    SmartDashboard.putNumber("Grav Y", gravVector[1]);
    SmartDashboard.putNumber("Grav Z", gravVector[2]);
    m_odometry.update(
        this.getGyroRotation(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());

    SmartDashboard.putNumber("Pose X", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Pose Y", m_odometry.getPoseMeters().getY());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        this.getGyroRotation(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftLead.setVoltage(leftVolts);
    m_rightLead.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
  }

  public Rotation2d getGyroRotation(){
    return Rotation2d.fromDegrees(m_gyro.getYaw());
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  //public Encoder getLeftEncoder() {
  //  return m_leftEncoder;
  //}

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  //public Encoder getRightEncoder() {
  //  return m_rightEncoder;
  //}

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.setYaw(0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return this.getGyroRotation().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  //public double getTurnRate() {
  //  return -m_gyro.get
  //}
}