// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int LEFT_LEADER_ID = 3;
    public static final int LEFT_FOLLOWER_ID = 4;
    public static final int RIGHT_LEADER_ID = 1;
    public static final int RIGHT_FOLLOWER_ID = 2;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;

    public static final double wheelRadiusInches = 3.0;
    public static final double wheelDiameterInches = 2 * wheelRadiusInches;
    public static final double wheelCircumferenceInches = wheelDiameterInches * Math.PI;

    // chassis configuration in meters
    public static final double wheelRadiusMeters = Units.inchesToMeters(wheelRadiusInches);
    public static final double wheelDiameterMeters = Units.inchesToMeters(wheelDiameterInches);
    public static final double wheelCircumferenceMeters = Units.inchesToMeters(wheelCircumferenceInches);

    // speed references meters/sec
    public static final double walkingSpeedMetersPerSec = 1.0;  
    public static final double maxSpeedMetersPerSec = 3.0;  // FIXME
    public static final int kEncoderCountsPerRevolution = 2048 * 4;     // one rotation is 8192 ticks of the hardware encoder
    public static final double kDrivePositionConversionFactor = DriveConstants.wheelCircumferenceMeters / kEncoderCountsPerRevolution;
    public static final double kDriveVelocityConversionFactor = DriveConstants.wheelCircumferenceMeters / kEncoderCountsPerRevolution;     // RPM (per minute)
    
    public static final double reakKp = 0.1; // was: 0.0001
    public static final double realKd = 0.0;
    public static final double realKi = 0.0;
    
  }
  
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int coralToReef = 5;
    public static final int intakeGamePiece = 3;
    public static final int armUp = 6;
    public static final int armDown = 4;
   // public static final int armDownDebouncer = 7;
    public static final int elevatorToL1 = 7;
    public static final int elevatorToL2 = 8;
    // public static final int elevatorToTop = 8;
    public static final int resetLiftToBottomPosition = 9;
    
  }
  public static final class ArmConstants{
    public static final int ARM_MOTOR_ID = 5;
    public static final int ARM_MOTOR_CURRENT_LIMIT = 9;
    public static final double ARM_MOTOR_VOLTAGE_COMP = 10;
    public static final double ARM_SPEED_DOWN = -0.5;
    public static final double ARM_SPEED_UP = 0.5;
    public static final double ARM_HOLD_DOWN = 0.0;
    public static final double ARM_HOLD_UP = -0.0;
  }

  public static final class ArmRollerConstants {
      public static final int ROLLER_MOTOR_ID = 6;
      public static final int ROLLER_MOTOR_CURRENT_LIMIT = 9;
      public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
      public static final double ROLLER_EJECT_VALUE = 0.44;

      public static final double rollerGamePieceInSpeed = -0.8;
      public static final double rollerCoralOutSpeed = 0.5;
  }
  
  public static final class ElevatorConstants {
      public static final int ELEVATOR_MOTOR_ID = 7; //TODO
      public static final int ELEVATOR_FOLLOWER_MOTOR_ID = 8;
      public static final int ELEVATOR_ARM_MOTOR_ID = 9;
      public static final int gearing = 12;
      public static final double drumRadiusInMeters = Units.inchesToMeters(1);     // sprocket diameter is 2"
      public static final double drumCircumferenceInMeters = 2.0 * Math.PI * drumRadiusInMeters;


      public static final double MOTION_LIMIT = -0.7;
      public static final double SCORING_MOVEMENT = -0.8;

      public static final int MOTOR_ID = 12;
      public static final boolean MOTOR_INVERTED = false;
      public static final boolean FOLLOWER_MOTOR_INVERTED = true;
      public static final boolean INVERT_FOLLOWER_OUTPUT = false;
      
      public static enum ElevatorPosition {
        BOTTOM(0.0),      // min height will trigger limit switch
        INTAKE(0.35),     // coral intake
        CORAL_L1(0.8),
        CORAL_L2(1.2),
        TOP(1.5);        // max height
  
        public final double value;
  
        private ElevatorPosition(double value) {
          this.value = value;
        }
      }

      public static final double MIN_HEIGHT_METERS = 0.005;
        public static final double MAX_HEIGHT_METERS = 1.57;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNEO(1);
        public static final double GEARING = 40.0; // TODO
        public static final double MASS_KG = Units.lbsToKilograms(10); // TODO
        public static final double COM_DISTANCE_METERS = Units.inchesToMeters(6); // TODO
        public static final double MOI = SingleJointedArmSim.estimateMOI(COM_DISTANCE_METERS, MASS_KG);
        public static final int encoderCountsPerRevolution = 42 * (int)ElevatorConstants.gearing;     // one rotation is 8192 ticks of the hardware encoder
        public static final double liftPositionConversionFactor = ElevatorConstants.drumCircumferenceInMeters / encoderCountsPerRevolution;
        public static final double liftVelocityConversionFactor = ElevatorConstants.drumCircumferenceInMeters / encoderCountsPerRevolution;     // RPM (per minute)

        public static final double MIN_ANGLE_RADIANS = -Math.PI / 2.0;
        public static final double MAX_ANGLE_RADIANS = Math.PI / 2.0;

        public static final int CURRENT_LIMIT = 4;

        public static final double kP = 10; // TODO
        public static final double kI = 0.01; // TODO
        public static final double kD = 0.01; // TODO
        public static final double kS = 0.017964; // TODO
        public static final double kG = 0.321192; // TODO
        public static final double kV = 0.876084;// TODO
        public static final double kA = 0.206676;// TODO
        public static final double TOLERANCE = 0.02;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 8; // TODO
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 4; // TODO
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    
  }

}

