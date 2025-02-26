
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;;


public class AlgaeArm extends SubsystemBase {
    private final SparkMax algaeMotor;
    
    private ArmState armState;

    @Logged(name="RollerMotorInfo")
    private final MotorIOInfo ioInfo = new MotorIOInfo();
    @Logged
    public static class MotorIOInfo {
      public double motorPositionInMeters = 0.0;
      public double motorVelocityInMetersPerSec = 0.0;
      public double motorAppliedVolts = 0.0;
      public double motorCurrentAmps = 0.0;
    }
    
        // Debouncer for current stall detection
    LinearFilter currenFilter = LinearFilter.movingAverage(10);
    private double filteredCurrent;

    public AlgaeArm() {
        algaeMotor = new SparkMax(ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);

        algaeMotor.setCANTimeout(250);

        SparkMaxConfig algaeConfig = new SparkMaxConfig();
        algaeConfig.voltageCompensation(10);
        algaeConfig.smartCurrentLimit(ArmConstants.ARM_MOTOR_CURRENT_LIMIT);
        algaeConfig.idleMode(IdleMode.kBrake);
        algaeMotor.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        
    }

    private void updateMotorIOInfo() {
        //ioInfo.leftPositionInMeters = motor.getEncoder().getPosition();
        ioInfo.motorVelocityInMetersPerSec = algaeMotor.get();
        ioInfo.motorAppliedVolts = algaeMotor.getAppliedOutput();
        ioInfo.motorCurrentAmps = algaeMotor.getOutputCurrent();
    }

    public void periodic()
    {
        updateMotorIOInfo();

    }


    public void armUp()
    {
        armState = ArmState.UP;
        algaeMotor.set(ArmConstants.ARM_SPEED_UP);
       
    }

    public void armHoldUp()
    {
        algaeMotor.set(ArmConstants.ARM_HOLD_UP);

    }
    public void armDown()
    {
        armState = ArmState.DOWN;
        algaeMotor.set(ArmConstants.ARM_SPEED_DOWN);

    }

    public void armHoldDown()
    {
        algaeMotor.set(ArmConstants.ARM_HOLD_DOWN);

    }

    public Command ArmUp()
    {
        return this.startEnd(this::armUp, this::armHoldUp);

    }
    public Command ArmDown()
    {
        return this.startEnd(this::armDown, this::armHoldDown);

    }


    public static enum ArmState
    {
        UP,
        DOWN
    }

     public Command runDebounceArmDownCmd() {
    Debouncer debounce = new Debouncer(1, Debouncer.DebounceType.kRising);

    // Run roller
    return runOnce(
     () -> {
      // initialize
      debounce.calculate(false);
     })
     .andThen(
      // set intake to algae intaking speed
      run( () -> {
        armDown();
      })
        // wait until current spike is detected for more than 1s
        .until( () -> debounce.calculate(filteredCurrent > 1 )))    // INTAKE_STALL_DETECTION is set to 1
      .finallyDo(
        // reduce power to holding state
       (interrupted) -> {
        armHoldDown(); // hold arm down speed
       }); 
  }
    
    
}
