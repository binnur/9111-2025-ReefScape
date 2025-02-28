// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;;

@Logged
public class DriveSubsystem extends SubsystemBase {
  //@Logged(name="Leader Left")
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  //@Logged(name="Leader Right")
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

   // setup closed loop controller
  private final SparkClosedLoopController leftController;
  private final SparkClosedLoopController rightController;

 

  //@Logged(name="Differential Drive")
  private final DifferentialDrive drive;

  /** Creates a new ExampleSubsystem. */
  
  public DriveSubsystem() {
    leftLeader = new SparkMax(DriveConstants.LEFT_LEADER_ID, MotorType.kBrushed);
    leftFollower = new SparkMax(DriveConstants.LEFT_FOLLOWER_ID, MotorType.kBrushed);
    rightLeader = new SparkMax(DriveConstants.RIGHT_LEADER_ID, MotorType.kBrushed);
    rightFollower = new SparkMax(DriveConstants.RIGHT_FOLLOWER_ID, MotorType.kBrushed);

    leftController = leftLeader.getClosedLoopController();
    rightController = rightLeader.getClosedLoopController();

    drive = new DifferentialDrive(leftLeader, rightLeader);

    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    globalConfig.voltageCompensation(12);
    globalConfig.smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    
    //TODO check motor invert when testing
    rightLeaderConfig.apply(globalConfig);

    leftLeaderConfig.apply(globalConfig);

    leftFollowerConfig.apply(globalConfig).follow(leftLeader);

    rightFollowerConfig.apply(globalConfig).follow(rightLeader);



    // configure encoders
            // velocityConversionFactor returns rotations per min -- divide by 60.0 for rotation per seconds
            // FIX?: in setReference() left encoder is counting backwards -- multiplying by -1
            leftLeaderConfig.encoder
                .positionConversionFactor(DriveConstants.kDrivePositionConversionFactor)
                .velocityConversionFactor(DriveConstants.kDriveVelocityConversionFactor / 60.0);
            rightLeaderConfig.encoder
                .positionConversionFactor(DriveConstants.kDrivePositionConversionFactor)
                .velocityConversionFactor(DriveConstants.kDriveVelocityConversionFactor / 60.0);

            // configure closed loop controllers for velocity -- by default written to slot 0
            leftLeaderConfig.closedLoop
                // set PID values for position control. Closed loop slot defaults to slot 0
                .p(DriveConstants.reakKp)
                .i(DriveConstants.realKi)
                .d(DriveConstants.realKd)
                .velocityFF(1.0 / DriveConstants.kEncoderCountsPerRevolution)        // FIXME: 1.0 / 5767 REV example
                .outputRange(-1, 1);
 
            rightLeaderConfig.closedLoop
                .p(DriveConstants.reakKp)
                .i(0.0)
                .d(DriveConstants.realKd)
                .velocityFF(1.0 / DriveConstants.kEncoderCountsPerRevolution)        // FIXME: 1.0 / 5767 REV example
                .outputRange(-1, 1);

    

    leftLeader.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



    resetEncoders();

  }
  @Logged(name="DriveIOInfo")
  private final DriveIOInfo ioInfo = new DriveIOInfo();
  @Logged
  public static class DriveIOInfo {
    public double leftPositionInMeters = 0.0;
    public double leftVelocityInMetersPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;

    public double rightPositionInMeters = 0.0;
    public double rightVelocityInMetersPerSec = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightCurrentAmps = 0.0;

    // TODO: add updateInputs() to this class?
  }

  private void updateDriveIOInfo() {
    ioInfo.leftPositionInMeters = leftLeader.getEncoder().getPosition();
    ioInfo.leftVelocityInMetersPerSec = leftLeader.get();
    ioInfo.leftAppliedVolts = leftLeader.getAppliedOutput();
    ioInfo.leftCurrentAmps = leftLeader.getOutputCurrent();

    ioInfo.rightPositionInMeters = rightLeader.getEncoder().getPosition();
    ioInfo.rightVelocityInMetersPerSec = rightLeader.get();
    ioInfo.rightAppliedVolts = rightLeader.getAppliedOutput();
    ioInfo.rightCurrentAmps = rightLeader.getOutputCurrent();  
  }

  public void setVelocity(double leftVelocity, double rightVelocity) {
    leftController.setReference(leftVelocity, ControlType.kVelocity);
    rightController.setReference(rightVelocity, ControlType.kVelocity);  
    
  }

  public BooleanSupplier isAtDistance(double desiredDistanceInMeters) {
    return () -> ((Math.abs(leftLeader.getEncoder().getPosition()) >= desiredDistanceInMeters) || 
                  (Math.abs(rightLeader.getEncoder().getPosition()) >= desiredDistanceInMeters)); 
  }


  public void resetEncoders() {
    
    // note in simulation there is a small drift 
    leftLeader.getEncoder().setPosition(0.0);
    rightLeader.getEncoder().setPosition(0.0);

    
  }  

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  public Command driveArcade(DriveSubsystem driveA, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return Commands.run(
      () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()), driveA);
  
  }

  public void stop()
  {
    leftLeader.set(0.0);
    rightLeader.set(0.0);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateDriveIOInfo();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Command driveFwdInMetersCmd(DriveSubsystem driveSubsystem, DoubleSupplier distanceInMeters) {
    return Commands.startRun(
      this::resetEncoders, 
      () -> this.setVelocity(DriveConstants.walkingSpeedMetersPerSec, DriveConstants.walkingSpeedMetersPerSec), 
      driveSubsystem)
      .until(this.isAtDistance(distanceInMeters.getAsDouble()))
      .andThen(this::stop)
      .withName("Drive/CMD/driveFwd");
  }
}