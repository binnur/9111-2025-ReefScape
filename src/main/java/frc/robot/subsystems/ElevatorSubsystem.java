package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import frc.robot.PositionTracker;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
//import frc.robot.GlobalStates;
//import frc.robot.Constants.Elevator.ElevatorPosition;
import frc.robot.Robot;

//import static frc.robot.Constants.Elevator.*;

@Logged
public class ElevatorSubsystem extends SubsystemBase {
    // construct the brushless motors
    private final SparkMax liftMotor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax liftFollowerMotor = new SparkMax(ElevatorConstants.ELEVATOR_FOLLOWER_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax armMotor = new SparkMax(ElevatorConstants.ELEVATOR_ARM_MOTOR_ID, MotorType.kBrushless);

    private final SparkMaxConfig liftMotorConfig;
    private final SparkMaxConfig liftFollowerMotorConfig;

    @Logged(name = "Desired Lift Level")
    private double desiredLiftLevel = 0.0;
    

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

    /**
     * The representation of the "elevator" for simulation. (even though this is a
     * rotational mechanism w.r.t. its setpoints, we still control it as a linear
     * mechanism since that is the cloest physical mechanism to this)
     */
    private final ElevatorSim elevatorSim = new ElevatorSim(
        DCMotor.getNEO(2),                  // number of motors in the gearbox
        ElevatorConstants.gearing,                  // elevator gearing                  
        Units.lbsToKilograms(20),                // carriage weight
        ElevatorConstants.drumDiameterInMeters/2,   // radius of drum where elevator spool is wrapped around
        ElevatorConstants.MIN_HEIGHT_METERS,
        ElevatorConstants.MAX_HEIGHT_METERS,
        true,
        ElevatorPosition.BOTTOM.value);


    // setup simulation support
    private DCMotor dcMotorLift = DCMotor.getNEO(1);
    private DCMotor dcMotorLiftFollower = DCMotor.getNEO(1);
    private SparkMaxSim simDcMotorLift;
    private SparkMaxSim simDcMotorLiftFollower;

    public ElevatorSubsystem() {
        /** Configure each motor for: 
         *      - current limit & voltage compensation
         *      - if the motor is inverted
         *      - coast & break mode
         *      - encoder conversion factors -- assumes encoder is primary encoder built into motor controller
        */

        // liftMotor -- this is the leader motor w/ ID 7. It is not inverted, so encoder and velocity values need to be positive 
        liftMotorConfig = new SparkMaxConfig();
        liftMotorConfig
            .inverted(ElevatorConstants.MOTOR_INVERTED)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ElevatorConstants.CURRENT_LIMIT)     // setting to 50 per rev code
            .voltageCompensation(12.0);
        liftMotorConfig.encoder
            .positionConversionFactor(ElevatorConstants.liftPositionConversionFactor)
            .velocityConversionFactor(ElevatorConstants.liftVelocityConversionFactor / 60.0);
        liftMotor.configure(liftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Follower configuration did not work as expected -- instead we are not applying voltage to both motors
        // liftFollowerMotor is w/ ID 8. It is inverted, so encoder and velocity values should be negative
        liftFollowerMotorConfig = new SparkMaxConfig();
        liftFollowerMotorConfig
                .inverted(ElevatorConstants.FOLLOWER_MOTOR_INVERTED)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ElevatorConstants.CURRENT_LIMIT)     // setting to 50 per rev code
                .voltageCompensation(12.0);
        //followerConfig.follow(liftMotor  , ElevatorConstants.FOLLOWER_MOTOR_INVERTED);    // regardless of invert fallower output, i.e. true/false value, motor always moved positive direction
        // liftFollowerMotorConfig.encoder      // using native rotation values for comparasion/debugging
        //     .positionConversionFactor(1)
        //     .velocityConversionFactor(1 / 60.0);
        liftFollowerMotor.configure(liftFollowerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // construct simulation 
        simDcMotorLift = new SparkMaxSim(liftMotor, dcMotorLift);
        simDcMotorLiftFollower = new SparkMaxSim(liftFollowerMotor, dcMotorLiftFollower);

        // at startup, elevator is naturally coasts to the bottom -- resetting encoders
        resetEncoders();     
    }

    @Logged(name="Lift IOInfo")
    private final ElevatorIOInfo ioInfo = new ElevatorIOInfo();
    @Logged
    public static class ElevatorIOInfo {
        public double liftAtPositionInMeters = 0.0;
        public double liftSimVelocityInMetersPerSec = 0.0;
        public double liftVelocityInMetersPerSec = 0.0;
        public double liftAppliedVolts = 0.0;
        public double liftCurrentAmps = 0.0;
        public double followerLiftAtPositionInMeters = 0.0;
        public double followerLiftAppliedVolts = 0.0;
    }

    private void setElevatorMotorTest() {
        liftMotor.set(ElevatorConstants.elevatorSpeed);
    }

    private void stopElevatorMotorTest() {
        liftMotor.set(ElevatorConstants.stopElevatorMotor);
    }

    public Command runElevatorMotorTest () {
        return this.startEnd(this::setElevatorMotorTest, this::stopElevatorMotorTest)
            .withTimeout(1.0);
    }

    // This is the debug information for AdvantageScope
    private void updateElevatorIOInfo() {
        ioInfo.liftAtPositionInMeters = liftMotor.getEncoder().getPosition();
        ioInfo.liftVelocityInMetersPerSec = liftMotor.get();     // note: does not get updated during simulation use corresponding liftSimVelocity
        ioInfo.liftAppliedVolts = liftMotor.getAppliedOutput();
        ioInfo.liftCurrentAmps = liftMotor.getOutputCurrent();
        ioInfo.followerLiftAtPositionInMeters = liftFollowerMotor.getEncoder().getPosition();
        ioInfo.followerLiftAppliedVolts = liftFollowerMotor.getAppliedOutput();
        if (Robot.isSimulation()) {
           ioInfo.liftSimVelocityInMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
        }
    }

    /*
     * Interesting behavior observed in simulation -- 
     *  liftMotor w/ positionConversion factor is settling at 0.005, i.e not 0
     *  liftFollowerMotor is reset to 0, using positionConversion of 1, i.e. native rotations
     */
    public void resetEncoders() 
    {
        desiredLiftLevel = ElevatorPosition.BOTTOM.value;

        liftMotor.getEncoder().setPosition(0.0);
        liftFollowerMotor.getEncoder().setPosition(0.0);

        if (Robot.isSimulation()) {
            simDcMotorLift.setPosition(0.0);
            simDcMotorLiftFollower.setPosition(0.0);
        }
    }

    // PID values calculated using liftMotor 
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
            })
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
            .withName("elevator.coastMotorsCommand");
    }

    public void periodic(){
        // note: default command moveToSetPointCommand() automatically runs
        updateElevatorIOInfo();
    }

    @Override
    public void simulationPeriodic() {
        // note: using liftMotor (instead of simDCMotorLift) provided better results
        elevatorSim.setInput(simDcMotorLift.getAppliedOutput() * RobotController.getInputVoltage());
        //elevatorSim.setInput(liftMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
        elevatorSim.update(0.020);

        // note: liftVelocityInMetersPerSec values are not updated via iterate -- liftMoter.get() returns 0
        simDcMotorLift.iterate(
            elevatorSim.getVelocityMetersPerSecond(),       // see liftConfig conversion factors
            RobotController.getBatteryVoltage(),
            0.02);

        // negate as follower is inverted?
        simDcMotorLiftFollower.iterate(
            elevatorSim.getVelocityMetersPerSecond(), 
            RobotController.getBatteryVoltage(),
            0.02);

        // note: .iterate should reset encoder position, but this seem to simulate better
        liftMotor.getEncoder().setPosition(elevatorSim.getPositionMeters());
        // convert follower encoder to rotations, not meters
        liftFollowerMotor.getEncoder().setPosition(elevatorSim.getPositionMeters()* 85.9);  // 85.9 rotations == 1 meters
    }

    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);

        if ((voltage > 0.0 && getPosition() >= ElevatorConstants.MAX_HEIGHT_METERS) ||
            (voltage < 0.0 && getPosition() <= ElevatorConstants.MIN_HEIGHT_METERS)) {
               voltage = 0.0;
        }

        liftMotor.setVoltage(voltage);
        liftFollowerMotor.setVoltage(0-voltage);
    }

    // Runs motors
    public Command moveToSetPointCommand() {
        return run( () -> {
            feedbackVoltage = liftPidController.calculate(getPosition());
            feedforwardVoltage = liftFFController.calculate(liftPidController.getSetpoint().velocity);     
            setVoltage(feedbackVoltage+feedforwardVoltage);
        }).withName("elevator.moveToCurrentGoal");
    }
   
    public boolean liftAtGoal() {
        return liftPidController.atGoal();
    }

    public Command moveToPositionCommand(Supplier<ElevatorPosition> goalPositionSupplier) {
        desiredLiftLevel = goalPositionSupplier.get().value;

        // stop current motion and clear integral values
        // specify the new goal position to the PID controller 
        liftPidController.reset(getPosition());
        liftPidController.setGoal((goalPositionSupplier.get().value));

        // run the motors until target goal is reached
        return Commands.sequence(
                moveToSetPointCommand()
                    .until( () -> liftAtGoal() ) 
        )
        .withTimeout(3)
        .withName("elevator.moveToPosition");
    }

    public Command setTargetPositionCommand(ElevatorPosition level) {
        ElevatorPosition liftLevelTarget;
        // TODO add arm position & command to setup arm
        switch (level) {
            case INTAKE:
                liftLevelTarget = ElevatorPosition.INTAKE;
                break;
            case TOP:
                liftLevelTarget = ElevatorPosition.TOP;
                break;
            case CORAL_L1:
                liftLevelTarget = ElevatorPosition.CORAL_L1;
                break;
            case CORAL_L2:
                liftLevelTarget = ElevatorPosition.CORAL_L2;
                break;
            default: 
                liftLevelTarget = ElevatorPosition.BOTTOM;
                break;   
        }

        return Commands.runOnce( 
            () -> {
                System.out.println("Running setTargetPositionCommand " + liftLevelTarget);
                moveToPositionCommand( () -> liftLevelTarget);
            }
        );
    }    
}