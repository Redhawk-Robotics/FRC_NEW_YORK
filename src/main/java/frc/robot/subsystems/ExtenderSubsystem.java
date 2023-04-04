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

public class ExtenderSubsystem extends SubsystemBase {
  /** Creates a new extenderSubsystem. */
  private final CANSparkMax extenderMotor;
  private final RelativeEncoder extenderEncoder;

  private final SparkMaxPIDController extenderController;

  public ExtenderSubsystem() {
    extenderMotor = new CANSparkMax(Ports.Extender.extender, MotorType.kBrushless);
    extenderMotor.setIdleMode(IdleMode.kCoast);
    extenderMotor.setInverted(true);
    extenderEncoder = extenderMotor.getEncoder();

    extenderController = extenderMotor.getPIDController();
    configExtenderMotor(extenderMotor, extenderEncoder, extenderController, Ports.Extender.ExtenderMotorInvert);

    extenderMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    extenderMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    extenderMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 270);// TODO check the value for both forward
                                                                             // and
    // reverse
    extenderMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

    setSmartMotionParams();
    PID();

    // resetEncoder();

    // enableMotors(true);//TODO test later
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Extender Encoder Value", extenderEncoder.getPosition());

  }

  // Methods for config for the motors used in this subsystems
  private void setSmartMotionParams() {
    extenderController.setSmartMotionMaxVelocity(Settings.extenderSetting.SmartMotionParameters.maxVel,
        Settings.extenderSetting.SmartMotionParameters.smartMotionSlot);
    extenderController.setSmartMotionMinOutputVelocity(Settings.extenderSetting.SmartMotionParameters.minVel,
        Settings.extenderSetting.SmartMotionParameters.smartMotionSlot);
    extenderController.setSmartMotionMaxAccel(Settings.extenderSetting.SmartMotionParameters.maxAccel,
        Settings.extenderSetting.SmartMotionParameters.smartMotionSlot);
    extenderController.setSmartMotionAllowedClosedLoopError(Settings.extenderSetting.SmartMotionParameters.maxErr,
        Settings.extenderSetting.SmartMotionParameters.smartMotionSlot);
  }

  private void configExtenderMotor(CANSparkMax extenderMotor, RelativeEncoder extenderEncoder,
      SparkMaxPIDController extenderController, boolean Invert) {
    extenderMotor.restoreFactoryDefaults();
    extenderMotor.clearFaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(extenderMotor, Usage.kPositionOnly);
    extenderMotor.setSmartCurrentLimit(Settings.extenderSetting.extenderContinousCurrentLimit);
    extenderMotor.setInverted(Invert);
    extenderMotor.setIdleMode(Settings.extenderSetting.extenderNeutralMode);
    extenderMotor.enableVoltageCompensation(Settings.extenderSetting.maxVoltage);
    extenderController.setFeedbackDevice(extenderEncoder);
    extenderMotor.burnFlash();
    Timer.delay(1);
    // resetToAbsolute();//FIXME if we are adding a canCODER to the shaft of the arm
  }

  private void PID() {
    extenderController.setP(Settings.extenderSetting.extenderP);
    extenderController.setI(Settings.extenderSetting.extenderI);
    extenderController.setD(Settings.extenderSetting.extenderD);
    extenderController.setFF(Settings.extenderSetting.extenderFF);
  }

  public void setPosition(double targetPosition) {
    extenderController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
  }

  public void setSmartMotionPosition(double targetPosition) {
    extenderController.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion);
  }

  // Getters
  public double getCurrentPosition() {
    return extenderEncoder.getPosition();
  }

  public void setMotor(double speed) {
    extenderMotor.set(speed);
  }

  public double getExtenderVelocity() {
    return extenderMotor.getEncoder().getVelocity();
  }

  public double getExtenderCurrent() {
    return extenderMotor.getOutputCurrent();
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
    extenderMotor.setIdleMode(mode);
  }

  public void resetEncoder() {
    extenderEncoder.setPosition(0);
  }
}
