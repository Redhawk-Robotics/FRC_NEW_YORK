// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.lib.util.CANSparkMaxUtil;
import frc.robot.lib.util.CANSparkMaxUtil.Usage;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Ports;
import frc.robot.constants.Settings;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private final CANSparkMax leftArmMotor, rightArmMotor;
  private final RelativeEncoder leftArmEncoder, rightArmEncoder;

  private final SparkMaxPIDController armAngleController;

  private double stop = 0;

  public ArmSubsystem() {

    leftArmMotor = new CANSparkMax(Ports.Arm.leftArm, MotorType.kBrushless);
    leftArmEncoder = leftArmMotor.getEncoder();

    rightArmMotor = new CANSparkMax(Ports.Arm.rightArm, MotorType.kBrushless);
    rightArmEncoder = rightArmMotor.getEncoder();

    leftArmMotor.follow(rightArmMotor, true); 

    armAngleController = rightArmMotor.getPIDController();
    
    configArmMotor(rightArmMotor, rightArmEncoder, armAngleController, Ports.Arm.rightArmMotorInvert);

    rightArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    rightArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    rightArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 0);// TODO check the value for both forward and
                                                                           // reverse
    rightArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
    
    //resetEncoder();

    // enableMotors(true);//TODO test later
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("RightArm Encoder Value", leftArmEncoder.getPosition());

  }

  // Methods for config for the motors used in this subsystems
  private void configArmMotor(CANSparkMax ArmMotor, RelativeEncoder ArmEncoder,
      SparkMaxPIDController armAngleController, boolean Invert) {
    ArmMotor.restoreFactoryDefaults();
    ArmMotor.clearFaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(ArmMotor, Usage.kPositionOnly);
    ArmMotor.setSmartCurrentLimit(Settings.armSetting.armContinousCurrentLimit);
    ArmMotor.setInverted(Invert);
    ArmMotor.setIdleMode(Settings.armSetting.armNeutralMode);
    ArmMotor.enableVoltageCompensation(Settings.armSetting.maxVoltage);
    armAngleController.setFeedbackDevice(ArmEncoder);
    ArmMotor.burnFlash();
    Timer.delay(1);
    // resetToAbsolute();//FIXME if we are adding a canCODER to the shaft of the arm
  }
  private void pidUp(){
    armAngleController.setP(Settings.armSetting.armPup);
    armAngleController.setI(Settings.armSetting.armIup);
    armAngleController.setD(Settings.armSetting.armDup);
    armAngleController.setFF(Settings.armSetting.armFFup);
  }

  private void pidDown(){
    armAngleController.setP(Settings.armSetting.armPdown);
    armAngleController.setI(Settings.armSetting.armIdown);
    armAngleController.setD(Settings.armSetting.armDdown);
    armAngleController.setFF(Settings.armSetting.armFFdown);
  }
  
  private void manageMotion(double targetPosition) {
    double currentPosition = getCurrentPosition();
      if (currentPosition < targetPosition) {
        pidUp();
      } else {
        pidDown();
      }
  }

  public void setPosition(double targetPosition) {
    manageMotion(targetPosition);
    armAngleController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
  }

  public void setSmartMotionPosition(double targetPosition) {
    manageMotion(targetPosition);
    armAngleController.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion);
  }

  //Getters
 public double getCurrentPosition() {
    return rightArmEncoder.getPosition();
  }
  public void setMotor(double speed) {
    rightArmMotor.set(speed);
  }

  public double getArmVelocity() {
    return rightArmMotor.getEncoder().getVelocity();
  }

  public double getArmCurrent() {
    return rightArmMotor.getOutputCurrent();
  }

public void stopArm(){
  rightArmMotor.set(stop);
  leftArmMotor.set(stop);
}
  // TODO try with the wrist that if its in code that its coast, and moves freely,
  // then this method is not needed
  public void enableMotors(boolean on) {
    IdleMode mode = on ? IdleMode.kBrake: IdleMode.kCoast;

    leftArmMotor.setIdleMode(mode);
    rightArmMotor.setIdleMode(mode);
  }

  public void resetEncoder() {
    leftArmEncoder.setPosition(0);
    rightArmEncoder.setPosition(0);
  }
}
