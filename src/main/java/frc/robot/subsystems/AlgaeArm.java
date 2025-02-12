package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;;

public class AlgaeArm {
    private final SparkMax algaeMotor;

    public AlgaeArm(){
        algaeMotor = new SparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);

        algaeMotor.setCANTimeout(250);

        SparkMaxConfig algaeConfig = new SparkMaxConfig();
        algaeConfig.voltageCompensation(10);
        algaeConfig.smartCurrentLimit(ArmConstants.ARM_MOTOR_CURRENT_LIMIT);
        algaeConfig.idleMode(IdleMode.kBrake);
        algaeMotor.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    public void runArm(double speed){
        algaeMotor.set(speed);
    }

}

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;;

public class AlgaeArm {
    private final SparkMax algaeMotor;

    public AlgaeArm(){
        algaeMotor = new SparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);

        algaeMotor.setCANTimeout(250);

        SparkMaxConfig algaeConfig = new SparkMaxConfig();
        algaeConfig.voltageCompensation(10);
        algaeConfig.smartCurrentLimit(ArmConstants.ARM_MOTOR_CURRENT_LIMIT);
        algaeConfig.idleMode(IdleMode.kBrake);
        algaeMotor.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    public void runArm(double speed){
        algaeMotor.set(speed);
    }

    public static enum armState
    {
        UP,
        DOWN
    };
    
    
}
