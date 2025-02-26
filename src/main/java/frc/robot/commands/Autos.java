// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static final Command doNothing()
  {
    return null;

  }

  public static final Command driveArcadeCmd(DriveSubsystem driveSubsystem) {
    return driveSubsystem.driveArcade(driveSubsystem, () -> 0.5, () -> 0.0)
      .withTimeout(1.0);
  }

  public static final Command resetEncoders(DriveSubsystem driveSubsystem) {
   return Commands.run( () -> driveSubsystem.resetEncoders() );
  }

  public static final Command driveFwd3meters(DriveSubsystem driveSubsystem) {
    return driveSubsystem.driveFwdInMetersCmd(driveSubsystem, () -> 3.0);
  }

}
