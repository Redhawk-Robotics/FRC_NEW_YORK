// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.constants.Ports;
import frc.robot.constants.Settings;
import frc.robot.lib.util.CANSparkMaxUtil;
import frc.robot.lib.util.CANSparkMaxUtil.Usage;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new ClawSubsystem. */
  private final CANSparkMax leftNeo550, rightNeo550;
  private final RelativeEncoder leftEncoder, rightEncoder;
  
  private final SparkMaxPIDController clawSpeedPIDController; 

  private final DoubleSolenoid clawSolenoid;

  public ClawSubsystem() {
    leftNeo550 = new CANSparkMax(Ports.Claw.leftClawMotor, MotorType.kBrushless);
    rightNeo550 = new CANSparkMax(Ports.Claw.rightClawMotor, MotorType.kBrushless);

    clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,  Ports.Claw.clawPneumatic.clawForwardChannel, Ports.Claw.clawPneumatic.clawReverseChannel);//FIXME to fix pneumatics
    
    leftEncoder = leftNeo550.getEncoder();
    rightEncoder = rightNeo550.getEncoder();

    leftNeo550.follow(rightNeo550,false);
    
    clawSpeedPIDController = leftNeo550.getPIDController();
    
    configClawMotor(leftNeo550, leftEncoder, clawSpeedPIDController, Ports.Claw.leftClawMotorInvert);

    PID();
    //enableMotors(true);//TODO test later
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void enableMotors(boolean on){
    IdleMode mode;
    if(on){
      mode = IdleMode.kBrake;
    }else{
      mode = IdleMode.kCoast;
    }
    leftNeo550.setIdleMode(mode);
    rightNeo550.setIdleMode(mode);
  }
  
  public void coneIntake() {
    clawSolenoid.set(Value.kForward);
    leftNeo550.set(-.5);
    rightNeo550.set(.5);
  }

  public void outTake() {
    clawSolenoid.set(Value.kReverse);
    // leftClaw.set(-.25);
    // rightClaw.set(-.25);
  }

  public void cubeIntake() {
    clawSolenoid.set(Value.kReverse);
    leftNeo550.set(.75);
    rightNeo550.set(-.75);
  }

  public void stopClaw() {
    clawSolenoid.set(Value.kOff);
  }
  public void openClaw(){
    clawSolenoid.set(Value.kForward);
  }

  public void closeClaw(){
    clawSolenoid.set(Value.kReverse);
  }

  public void configClawMotor(CANSparkMax clawMotor, RelativeEncoder clawEncoder, SparkMaxPIDController clawController, boolean invert) {
    clawMotor.restoreFactoryDefaults();
    clawMotor.clearFaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(clawMotor, Usage.kAll);
    clawMotor.setSmartCurrentLimit(Settings.clawSetting.clawContinousCurrentLimit);
    clawMotor.setIdleMode(Settings.clawSetting.clawNeutralMode);
    clawMotor.setInverted(invert);
    clawEncoder.setVelocityConversionFactor(Settings.clawSetting.clawConversionVelocityFactor);
    clawEncoder.setPositionConversionFactor(Settings.clawSetting.clawConversionPositionFactor);
    clawMotor.enableVoltageCompensation(Settings.clawSetting.maxVoltage);
    clawController.setFeedbackDevice(clawEncoder);
    clawMotor.burnFlash();
  }

  private void PID(){
    clawSpeedPIDController.setP(Settings.clawSetting.clawP);
    clawSpeedPIDController.setI(Settings.clawSetting.clawI);
    clawSpeedPIDController.setD(Settings.clawSetting.clawD);
    clawSpeedPIDController.setFF(Settings.clawSetting.clawFF);
  }
}
