package frc.robot.subsystems;

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

public class RollerSubsystem extends SubsystemBase {

    public static enum RollerState {
        STOPPED,
        FORWARD,
        REVERSE
      };


    private final SparkMax rollerMotor;

    public RollerSubsystem() {
        rollerMotor = new SparkMax(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);

        rollerMotor.setCANTimeout(250);

        SparkMaxConfig config = new SparkMaxConfig();

        config
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(RollerConstants.ROLLER_MOTOR_VOLTAGE_COMP)
            .smartCurrentLimit(RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT);

        rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        

        // Run motor forward and backward
        
    }
}
