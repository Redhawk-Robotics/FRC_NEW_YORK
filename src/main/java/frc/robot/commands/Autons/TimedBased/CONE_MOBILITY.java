// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons.TimedBased;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Swerve.DriveForward;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CONE_MOBILITY extends SequentialCommandGroup {
  /** Creates a new CONE_MOBILITY. */
  private final SwerveSubsystem SwerveDrive = new SwerveSubsystem();

  public CONE_MOBILITY() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new DriveForward(SwerveDrive, .5, -25, 1),
        new WaitCommand(1),
        new DriveForward(SwerveDrive, 4, 15, 7));
  }
}
