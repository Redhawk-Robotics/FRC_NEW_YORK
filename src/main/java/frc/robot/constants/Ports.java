// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * @Filename:Ports.java
 * @Purpose:This file contains the different ports of motors, and sensors
 * @Version:1.0
 * @Author: TEAM 8739 RedhawkRobotics
 * @Date: 04/02/23
 */

/*Subsystems have an order with corresponding ports
 * Swerve - 1
 * Arm - 2
 * Extender - 3
 * Wrist - 4
 * Rollers - 5
 * Claw - 6
 * SwerveEncoders - 7
 */

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.lib.SwerveModuleConstants;

public interface Ports {

    public static final double stickDeadband = 0.1;// FIXME need to change perhaps

    public static final class Gamepad {
        public static final int DRIVER = 0;
        public static final int OPERATOR = 1;
    }

    public static final class Gyro {
        public static final int drivetrainPigeonID = 19;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
    }

    public static final class PDH {
        public static final int portPDH = 1;
    }

    public static final class Compressor {
        public static final int portCompressor = 1;
    }

    public static final class Arm {
        public static final int leftArm = 21;
        public static final int rightArm = 22;

        public static final boolean leftArmMotorInvert = false;
        public static final boolean rightArmMotorInvert = true;
    }

    public static final class Extender {
        public static final int extender = 31;

        public static final boolean ExtenderMotorInvert = false;
    }

    public static final class Claw {
        public static final int leftClawMotor = 61;
        public static final int rightClawMotor = 62;

        public static final boolean leftClawMotorInvert = false;
        public static final boolean rightClawMotorInvert = false;

        public static final class clawPneumatic {
            public static final int clawForwardChannel = 0;//FIXME
            public static final int clawReverseChannel = 1;//FIXME
        }
    }

    public static final class Wrist {
        public static final int wrist = 41;

        public static final boolean wristMotorInvert = true;
    }

    // FRONT_LEFT_MODULE
    public static final class frontLeftModule0 {
        public static final int frontLeftModuleDriveMotor = 11; // Set front left module drive motor ID
        public static final int frontLeftModuleSteerMotor = 12; // Set front left module steer motor ID
        public static final int frontLeftModuleSteerEncoder = 71; // Set front left steer encoder ID
        public static final Rotation2d frontLeftModuleSteerOffSet = Rotation2d.fromDegrees(0.0);

        public static final SwerveModuleConstants constants = new SwerveModuleConstants(frontLeftModuleDriveMotor,
                frontLeftModuleSteerMotor, frontLeftModuleSteerEncoder, frontLeftModuleSteerOffSet);
    }

    // FRONT_RIGHT_MODULE
    public static final class frontRightModule1 {
        public static final int frontRightModuleDriveMotor = 13; // Set front right drive motor ID
        public static final int frontRightModuleSteerMotor = 14; // Set front right steer motor ID
        public static final int frontRightModuleSteerEncoder = 72; // Set front right steer encoder ID
        public static final Rotation2d frontRightModuleSteerOffSet = Rotation2d.fromDegrees(0.0);

        public static final SwerveModuleConstants constants = new SwerveModuleConstants(frontRightModuleDriveMotor,
                frontRightModuleSteerMotor, frontRightModuleSteerEncoder, frontRightModuleSteerOffSet);
    }

    // BACK_LEFT_MODULE
    public static final class backLeftModule2 {
        public static final int backLeftModuleDriveMotor = 15; // Set back left drive motor ID
        public static final int backLeftModuleSteerMotor = 16; // Set back left steer motor ID
        public static final int backleftModuleSteerEncoder = 73; // Set back left steer encoder ID
        public static final Rotation2d backLeftModuleSteerOffSet = Rotation2d.fromDegrees(0.0);

        public static final SwerveModuleConstants constants = new SwerveModuleConstants(backLeftModuleDriveMotor,
                backLeftModuleSteerMotor, backleftModuleSteerEncoder, backLeftModuleSteerOffSet);
    }

    // BACK_RIGHT_MODULE
    public static final class backRightModule3 {
        public static final int backRightModuleDriveMotor = 17; // Set back right drive motor ID
        public static final int backRightModuleSteerMotor = 18; // Set back right steer motor ID
        public static final int backRightModuleSteerEncoder = 74; // Set back right steer encoder ID
        public static final Rotation2d backRightModuleSteerOffSet = Rotation2d.fromDegrees(0.0);

        public static final SwerveModuleConstants constants = new SwerveModuleConstants(backRightModuleDriveMotor,
                backRightModuleSteerMotor, backRightModuleSteerEncoder, backRightModuleSteerOffSet);
    }
}
