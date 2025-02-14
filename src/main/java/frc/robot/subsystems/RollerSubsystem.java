package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

// import frc.robot.Configs;
import frc.robot.Constants.RollerConstants;


@Logged
public class RollerSubsystem extends SubsystemBase {

    // Enum for roller motor state
    public static enum RollerState {
        STOPPED,
        FORWARD,
        REVERSE
    }

    // Instance variables
    private final SparkMax rollerMotor;

    @Logged(name = "Roller State")
    private RollerState rollerState;  // To track the motor's state
    
    @Logged(name = "Motor Speed")
    private double speed;  // Speed value for the motor

    // Constructor to initialize motor and configurations
    public RollerSubsystem() {
        rollerMotor = new SparkMax(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
        rollerState = RollerState.STOPPED;  // Initialize the state as STOPPED

        rollerMotor.setCANTimeout(250);  // Set timeout for CAN communication

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(RollerConstants.ROLLER_MOTOR_VOLTAGE_COMP)
            .smartCurrentLimit(RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT);

        rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Run the roller motor. Arguments are constants, rollerAlgaeInSpeed, rollerAlgaeOutSpeed
    //@Logged(name = "Roller Motor Speed")



    private void runRollerMotorForward() {
        rollerState = RollerState.FORWARD;
        rollerMotor.set(Math.abs(RollerConstants.rollerCoralOutSpeed));
      }



      private void stopRollerMotor() {
        rollerState = RollerState.STOPPED;
        rollerMotor.set(0.0);
      }



      private void runRollerMotorReverse() {
        rollerState = RollerState.REVERSE;
        rollerMotor.set(-Math.abs(RollerConstants.rollerGamePieceInSpeed));
      }



    public Command runRollerForward() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return this.startEnd(this::runRollerMotorForward, this::stopRollerMotor)
                .withName("Roller/CMD/runRollerForward");
    }


    public Command runRollerReverse() {
        return this.startEnd(this::runRollerMotorReverse, this::stopRollerMotor)
                    .withName("Roller/CMD/runRollerReverse");
      }


    
      public Command runRollerStop() {
        return this.runOnce(this::stopRollerMotor)
                    .withName("Roller/CMD/runRollerStop");
      }
}
