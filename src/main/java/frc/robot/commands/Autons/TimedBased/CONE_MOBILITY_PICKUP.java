// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons.TimedBased;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Positions.groundIntake;
import frc.robot.commands.Positions.stowAway;
import frc.robot.commands.Swerve.DriveForward;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CONE_MOBILITY_PICKUP extends SequentialCommandGroup {
  /** Creates a new CONE_MOBILITY_PICKUP. */
  private final SwerveSubsystem SwerveDrive = new SwerveSubsystem();
  private final ClawSubsystem claw = new ClawSubsystem();

  public CONE_MOBILITY_PICKUP() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new DriveForward(SwerveDrive, .5, -30, 1),
        new WaitCommand(1),
        new ParallelDeadlineGroup(
            new DriveForward(SwerveDrive, 4, 15, 7),
            new groundIntake()),
        new InstantCommand(() -> claw.stopClaw()),
        new stowAway());
  }
}
