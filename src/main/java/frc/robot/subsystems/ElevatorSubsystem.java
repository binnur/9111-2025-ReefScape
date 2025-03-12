package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

//import frc.robot.PositionTracker;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
//import frc.robot.GlobalStates;
//import frc.robot.Constants.Elevator.ElevatorPosition;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
//import static frc.robot.Constants.Elevator.*;

@Logged
public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax liftMotor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax liftFollowerMotor = new SparkMax(ElevatorConstants.ELEVATOR_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax armMotor = new SparkMax(ElevatorConstants.ELEVATOR_ARM_MOTOR_ID, MotorType.kBrushless);
    private SparkMaxConfig liftMotorConfig;

    @Logged(name="control")
    private final ProfiledPIDController pidController = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, ElevatorConstants.MOVEMENT_CONSTRAINTS);

    @Logged(name="Feedback voltage")
    private double feedbackVoltage = 0;

    @Logged(name="Feed foward voltage")
    private double feedforwardVoltage = 0;

    private ProfiledPIDController liftPidController = new ProfiledPIDController(ElevatorConstants.kP, 
                                                    ElevatorConstants.kI, 
                                                    ElevatorConstants.kD,
                                                    ElevatorConstants.MOVEMENT_CONSTRAINTS);

    private final ElevatorFeedforward liftFFController = new ElevatorFeedforward(ElevatorConstants.kS, 
                                                    ElevatorConstants.kG,
                                                    ElevatorConstants.kV,
                                                    ElevatorConstants.kA);                                                

    public ElevatorSubsystem() {
        liftMotorConfig = new SparkMaxConfig();
        liftMotorConfig
                .inverted(ElevatorConstants.MOTOR_INVERTED)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ElevatorConstants.CURRENT_LIMIT);
        liftMotorConfig.encoder
                .positionConversionFactor(ElevatorConstants.ENCODER_ROTATIONS_TO_METERS)
                .velocityConversionFactor(ElevatorConstants.ENCODER_ROTATIONS_TO_METERS / 60.0);
        liftMotorConfig = new SparkMaxConfig();
        liftMotor.configure(liftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    @Logged(name="Elevator: Lift IOInfo")
    private final ElevatorIOInfo ioInfo = new ElevatorIOInfo();
    @Logged
    public static class ElevatorIOInfo {
        public double liftAtPositionInMeters = 0.0;
       // public double liftDesiredPositionInMeters = ElevatorPosition.BOTTOM.value;
        public double liftSimVelocityInMetersPerSec = 0.0;
        public double liftVelocityInMetersPerSec = 0.0;
        public double liftAppliedVolts = 0.0;
        public double liftCurrentAmps = 0.0;
    }

    /*private void updateElevatorIOInfo() {
        ioInfo.liftAtPositionInMeters = liftMotor.getEncoder().getPosition();
        ioInfo.liftVelocityInMetersPerSec = liftMotor.get();     // note: does not get updated during simulation use corresponding liftSimVelocity
        ioInfo.liftAppliedVolts = liftMotor.getAppliedOutput();
        ioInfo.liftCurrentAmps = liftMotor.getOutputCurrent();
        // note: ioInfo.liftDesiredPositionMeters updated with the operator control commands

        if (Robot.isSimulatio.n()) {
            ioInfo.liftSimVelocityInMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
        }
    }*/

    private void resetEncoders() 
    {
        liftMotor.getEncoder().setPosition(0.0);
    }


    public double getPosition()
    {
        return liftMotor.getEncoder().getPosition();
       
    }

    public Command coastMotorsCommand() {
        return runOnce(liftMotor::stopMotor)
                .andThen(() -> {
                    liftMotorConfig.idleMode(IdleMode.kCoast);
                    liftMotor.configure(liftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                })
                .finallyDo((d) -> {
                    liftMotorConfig.idleMode(IdleMode.kBrake);
                    liftMotor.configure(liftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                    pidController.reset(getPosition());
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("elevator.coastMotorsCommand");
    }

    public void Periodic(){
        // note: default command moveToSetPointCommand() automatically runs
        //updateElevatorIOInfo();
        
    }

    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);

        if ((voltage > 0.0 && getPosition() >= ElevatorConstants.MAX_HEIGHT_METERS) ||
            (voltage < 0.0 && getPosition() <= ElevatorConstants.MIN_HEIGHT_METERS)) {
               voltage = 0.0;
        }

        liftMotor.setVoltage(voltage);
    }

    

    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            feedbackVoltage = pidController.calculate(getPosition());
         //   feedforwardVoltage = feedforwardController.calculate(pidController.getSetpoint().velocity);
            setVoltage(feedbackVoltage + feedforwardVoltage);
        }).withName("elevator.moveToCurrentGoal");
    }

    public Command moveToSetPointCommand() {
        return run( () -> {
            feedbackVoltage = liftPidController.calculate(getPosition());
            feedforwardVoltage = liftFFController.calculate(liftPidController.getSetpoint().velocity);

            // TODO: replace this code with Math.signum function
            // feedforwardVoltage = 0.0;
            // if (feedbackVoltage < 0) {
            //     if (feedbackVoltage > -1) {
            //         feedforwardVoltage = -1;
            //     }
            // }
            // else if (feedbackVoltage > 0) {
            //     if (feedbackVoltage < 1) {
            //         feedforwardVoltage = 1;
            //     }
            // }

            setVoltage(feedbackVoltage+feedforwardVoltage);
        }).withName("elevator.moveToCurrentGoal");
    }

    
}