// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.Settings;
import frc.robot.lib.util.CANSparkMaxUtil;
import frc.robot.lib.util.CANSparkMaxUtil.Usage;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
  private final CANSparkMax wristMotor;
  private final RelativeEncoder wristEncoder;

  private final SparkMaxPIDController wristAngleController;

  private double wristSpeed = 0.3;// FIXME tune later
  private double wristSpeedReverse = -0.3;// FIXME tune later

  private double stop = 0;

  public WristSubsystem() {
    wristMotor = new CANSparkMax(Ports.Wrist.wrist, MotorType.kBrushless);
    wristEncoder = wristMotor.getEncoder();

    wristAngleController = wristMotor.getPIDController();

    configWristMotor(wristMotor, wristEncoder, wristAngleController, Ports.Wrist.wristMotorInvert);

    wristMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
        true);
    wristMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
        true);

    wristMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
        5);

    wristMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
        -30);
    setSmartMotionParams();
    PID();

    /// resetEncoder();
    // enableMotors(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Encoder Value", wristEncoder.getPosition());
  }

  private void setSmartMotionParams() {
    wristAngleController.setSmartMotionMaxVelocity(Settings.wristSetting.SmartMotionParameters.maxVel,
        Settings.wristSetting.SmartMotionParameters.smartMotionSlot);
    wristAngleController.setSmartMotionMinOutputVelocity(Settings.wristSetting.SmartMotionParameters.minVel,
        Settings.wristSetting.SmartMotionParameters.smartMotionSlot);
    wristAngleController.setSmartMotionMaxAccel(Settings.wristSetting.SmartMotionParameters.maxAccel,
        Settings.wristSetting.SmartMotionParameters.smartMotionSlot);
    wristAngleController.setSmartMotionAllowedClosedLoopError(Settings.wristSetting.SmartMotionParameters.maxErr,
        Settings.wristSetting.SmartMotionParameters.smartMotionSlot);
  }

  // Methods for config for the motors used in this subsystems
  private void configWristMotor(CANSparkMax wristMotor, RelativeEncoder wristEncoder,
      SparkMaxPIDController wristAngleController, boolean Invert) {
    wristMotor.restoreFactoryDefaults();
    wristMotor.clearFaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(wristMotor, Usage.kPositionOnly);
    wristMotor.setSmartCurrentLimit(Settings.wristSetting.wristContinousCurrentLimit);
    wristMotor.setInverted(Invert);
    wristMotor.setIdleMode(Settings.wristSetting.wristNeutralMode);
    wristMotor.enableVoltageCompensation(Settings.wristSetting.maxVoltage);
    wristAngleController.setFeedbackDevice(wristEncoder);
    wristMotor.burnFlash();
    Timer.delay(1);
  }

  private void PID() {
    wristAngleController.setP(Settings.wristSetting.wristP);
    wristAngleController.setI(Settings.wristSetting.wristI);
    wristAngleController.setD(Settings.wristSetting.wristD);
    wristAngleController.setFF(Settings.wristSetting.wristFF);
  }

  public void setPosition(double targetPosition) {
    wristAngleController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
  }

  public void setSmartMotionPosition(double targetPosition) {
    wristAngleController.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion);
  }

  // Getters
  public double getCurrentPosition() {
    return wristEncoder.getPosition();
  }

  public void setMotor(double speed) {
    wristMotor.set(speed);
  }

  public double getWristVelocity() {
    return wristMotor.getEncoder().getVelocity();
  }

  public double getWristCurrent() {
    return wristMotor.getOutputCurrent();
  }

  public void upGoWrist() {
    wristMotor.set(wristSpeed);
  }

  public void stopWrist() {
    wristMotor.set(stop);
  }

  public void downGoWrist() {
    wristMotor.set(wristSpeedReverse);
  }

  // TODO try with the wrist that if its in code that its coast, and moves freely,
  // then this method is not needed
  public void enableMotors(boolean on) {
    IdleMode mode;
    if (on) {
      mode = IdleMode.kBrake;
    } else {
      mode = IdleMode.kCoast;
    }
    wristMotor.setIdleMode(mode);
  }

  public void resetEncoder() {
    wristEncoder.setPosition(0);
  }
}
