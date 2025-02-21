// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.epilogue.Logged;
import frc.robot.subsystems.ArmRollerSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  @Logged(name = "ArmRoller")
  public final ArmRollerSubsystem armRoller = new ArmRollerSubsystem(); // Rename the rollersubsystem class to armRollerSubsystem

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick driverController =

      new Joystick(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);

  // Trigger declarations
  // Trigger button2 =  driverController.button(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }



  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

   
        
// Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value)
        driveSubsystem.setDefaultCommand(
          driveSubsystem.driveArcade(
              driveSubsystem, () -> -driverController.getRawAxis(0), () -> -driverController.getRawAxis(1)));
       
   // RollerSubsystem. TODO:
   // Add condition that roller may only roll out to eject coral when the arm is in a down position
   // Add another condition that roller roll in or out when the arm is a down position
   // Discuss what button to bind the armRoller to with drivers for rolling in and out

    //driverController.button(2).onTrue(armRoller.runRoller());
    // armRoller.runRollerMotor(Constants.RollerConstants.rollerAlgaeInSpeed);
    // armRoller.runRollerMotor( () -> Constants.RollerConstants.rollerAlgaeInSpeed).withTimeout(1.0);

    new JoystickButton(driverController, OperatorConstants.coralToLevel1)
      .whileTrue(armRoller.runRollerForward());

    new JoystickButton(driverController, OperatorConstants.intakeGamePiece)
      .whileTrue(armRoller.runRollerReverse());

  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}