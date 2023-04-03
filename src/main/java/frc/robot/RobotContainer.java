// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Arm.ArmManual;
import frc.robot.commands.Autons.DoNothingAuton;
import frc.robot.commands.Autons.TimedBased.CONE_MOBILITY;
import frc.robot.commands.Autons.TimedBased.CONE_MOBILITY_PICKUP;
import frc.robot.commands.Autons.TimedBased.JUST_CHARGE_PAD;
import frc.robot.commands.Autons.TimedBased.LOW_ENGAGE;
import frc.robot.commands.Autons.TimedBased.MIDSCORE;
import frc.robot.commands.Extender.ExtenderManual;
import frc.robot.commands.Positions.Substation;
import frc.robot.commands.Positions.groundIntake;
import frc.robot.commands.Positions.stowAway;
import frc.robot.commands.Swerve.Drive;
import frc.robot.constants.Ports;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  /* Subsystems */
  private final SwerveSubsystem SwerveDrive = new SwerveSubsystem();

  private final WristSubsystem wrist = new WristSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final ExtenderSubsystem extender = new ExtenderSubsystem();

  private ClawSubsystem claw = new ClawSubsystem();

  private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  /* Controllers */
  private final XboxController DRIVER = new XboxController(Ports.Gamepad.DRIVER);
  private final XboxController OPERATOR = new XboxController(Ports.Gamepad.OPERATOR);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final int rightYAxis1 = XboxController.Axis.kRightY.value;

  private final int leftTrigger1 = XboxController.Axis.kLeftTrigger.value;
  private final int rightTrigger1 = XboxController.Axis.kRightTrigger.value;

  /* Driver Buttons */
  private final Trigger driver_A_zeroGyro = new JoystickButton(DRIVER, XboxController.Button.kA.value);
  private final Trigger driver_X_robotCentric = new JoystickButton(DRIVER, XboxController.Button.kX.value);

  private final Trigger driver_slowSpeed_rightBumper = new JoystickButton(DRIVER,
      XboxController.Button.kRightBumper.value);

  // Additional buttons
  private final Trigger driver_leftBumper = new JoystickButton(DRIVER, XboxController.Button.kLeftBumper.value);

  private final Trigger driver_B = new JoystickButton(DRIVER, XboxController.Button.kB.value);

  private final Trigger driver_Y = new JoystickButton(DRIVER, XboxController.Button.kY.value);

  private final Trigger driver_BottomRightRearButton = new JoystickButton(DRIVER, XboxController.Button.kStart.value);
  private final Trigger driver_BottomLeftRearButton = new JoystickButton(DRIVER, XboxController.Button.kBack.value);

  private final Trigger driver_TopRightRearButton = new JoystickButton(DRIVER,
      XboxController.Button.kRightStick.value);

  private final Trigger driver_START = new JoystickButton(DRIVER, XboxController.Button.kStart.value);
  private final Trigger driver_BACK = new JoystickButton(DRIVER, XboxController.Button.kBack.value);

  // Controller two - Operator
  private final int leftYAxis2 = XboxController.Axis.kLeftY.value;
  private final int leftXAxis2 = XboxController.Axis.kLeftX.value;

  private final int rightYAxis2 = XboxController.Axis.kRightY.value;
  private final int rightXAxis2 = XboxController.Axis.kRightX.value;

  private final int leftTrigger2 = XboxController.Axis.kLeftTrigger.value;
  private final int rightTrigger2 = XboxController.Axis.kRightTrigger.value;

  private final Trigger opperator_A = new JoystickButton(OPERATOR, XboxController.Button.kA.value);
  private final Trigger opperator_B = new JoystickButton(OPERATOR, XboxController.Button.kB.value);
  private final Trigger opperator_X = new JoystickButton(OPERATOR, XboxController.Button.kX.value);
  private final Trigger opperator_Y = new JoystickButton(OPERATOR, XboxController.Button.kY.value);

  private final Trigger opperator_RightBumper = new JoystickButton(OPERATOR,
      XboxController.Button.kRightBumper.value);
  private final Trigger opperator_leftBumper = new JoystickButton(OPERATOR, XboxController.Button.kLeftBumper.value);

  private final Trigger opperator_BottomRightRearButton = new JoystickButton(OPERATOR,
      XboxController.Button.kStart.value);
  private final Trigger opperator_BottomLeftRearButton = new JoystickButton(OPERATOR,
      XboxController.Button.kBack.value);

  private final Trigger opperator_TopLeftRearButton = new JoystickButton(OPERATOR,
      XboxController.Button.kLeftStick.value);
  private final Trigger opperator_TopRightRearButton = new JoystickButton(OPERATOR,
      XboxController.Button.kRightStick.value);

  // Create SmartDashboard chooser for autonomous routines
  private static SendableChooser<Command> Autons = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    // Configure the trigger bindings
    /*************/
    /*** DRIVE ***/
    /*************/

    SwerveDrive.setDefaultCommand(
        new Drive(
            SwerveDrive,
            () -> -DRIVER.getRawAxis(translationAxis),
            () -> -DRIVER.getRawAxis(strafeAxis),
            () -> -DRIVER.getRawAxis(rotationAxis),
            () -> driver_X_robotCentric.getAsBoolean(),
            () -> driver_slowSpeed_rightBumper.getAsBoolean()));

    // armTest//
    arm.setDefaultCommand(
        new ArmManual(
            arm,
            () -> OPERATOR.getRawAxis(leftYAxis2)));

    // extederTest//
    extender.setDefaultCommand(
        new ExtenderManual(
            extender,
            () -> OPERATOR.getRawAxis(rightYAxis2)));

    // Configure the trigger bindings, defaults, Autons
    configureDefaultCommands();
    configureButtonBindings();
    configureAutons();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureDefaultCommands() {
    // Compressor code
    compressor.enableAnalog(100, 120);

    // compressor.disableCompressor();

    // Camera Server
    // CameraServer.startAutomaticCapture();
  }

  private void configureButtonBindings() {
    // Driver Controls
    // ------------------------------------- GYRO
    driver_A_zeroGyro.onTrue(new InstantCommand(() -> SwerveDrive.zeroGyro()));

    // Operator Controls
    opperator_Y.onTrue(new Substation());
    opperator_A.onTrue(new stowAway());
    opperator_X.onTrue(new groundIntake());
    // opperator_B.onTrue(new );

    // ------------------------------------- ARM
    opperator_leftBumper.onTrue(new InstantCommand(() -> arm.setPosition(44)));

    opperator_leftBumper.whileFalse(new InstantCommand(() -> arm.stopArm()));
    // ------------------------------------- CLAW
    opperator_BottomLeftRearButton.onTrue(new InstantCommand(() -> claw.coneIntake()));
    opperator_TopLeftRearButton.onTrue(new InstantCommand(() -> claw.cubeIntake()));

    opperator_BottomLeftRearButton.whileFalse(new InstantCommand(() -> claw.stopClaw()));
    opperator_TopLeftRearButton.whileFalse(new InstantCommand(() -> claw.stopClaw()));

    opperator_RightBumper.onTrue(new InstantCommand(() -> claw.outTake()));

    // ------------------------------------- WRIST
    opperator_TopRightRearButton.onTrue(new InstantCommand(() -> wrist.upGoWrist()));
    opperator_TopRightRearButton.onFalse(new InstantCommand(() -> wrist.stopWrist()));

    opperator_BottomRightRearButton.onTrue(new InstantCommand(() -> wrist.downGoWrist()));
    opperator_BottomRightRearButton.onFalse(new InstantCommand(() -> wrist.stopWrist()));
    // ------------------------------------- EXTENDER

  }

  // /**************/
  // /*** AUTONS ***/
  // /**************/
  public void configureAutons() {
    SmartDashboard.putData("Autonomous: ", Autons);

    Autons.setDefaultOption("Do Nothing", new DoNothingAuton());
    Autons.addOption(" CONE_MOBILITYv", new CONE_MOBILITY());

    Autons.addOption("CONE_MOBILITY_PICKUP", new CONE_MOBILITY_PICKUP());

    Autons.addOption("JUST_CHARGE_PAD", new JUST_CHARGE_PAD());
    Autons.addOption("LOW_ENGAGE", new LOW_ENGAGE());

    Autons.addOption("MIDSCORE", new MIDSCORE());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autons.getSelected();
  }
}
