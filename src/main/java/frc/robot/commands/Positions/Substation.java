// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Substation extends SequentialCommandGroup {
  /** Creates a new Substation. */
  private ExtenderSubsystem extender;
  private ArmSubsystem arm;
  private WristSubsystem wrist;

  public Substation() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new InstantCommand(() -> extender.setPosition(0)),
            new InstantCommand(() -> wrist.setPosition(5))),
        new ParallelCommandGroup(
            new InstantCommand(() -> arm.setPosition(0)),
            new InstantCommand(() -> wrist.setPosition(5)),
            new InstantCommand(() -> extender.setPosition(0))));

  }
}
