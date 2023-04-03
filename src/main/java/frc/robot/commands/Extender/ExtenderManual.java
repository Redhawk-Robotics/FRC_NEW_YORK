// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Extender;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtenderSubsystem;

public class ExtenderManual extends CommandBase {
  /** Creates a new ExtenderManual. */
  private ExtenderSubsystem extender;
  private DoubleSupplier speed;

  public ExtenderManual(ExtenderSubsystem extender,DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.extender = extender;
    this.speed = speed;

    addRequirements(extender);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    extender.setMotor(speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
