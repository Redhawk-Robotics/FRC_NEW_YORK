package frc.robot.constants;

import java.util.HashMap;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public interface Settings {
    /* Swerve Voltage Compensation */
    public static final double voltageComp = 8.5;// 8.5

    // RobotCharacterizations
    public static final double drivetrainTrackWidthMeters = Units.inchesToMeters(28.5);
    public static final double drivetrainWheelBaseMeters = Units.inchesToMeters(28.5);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kCoast;// kCoast
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveMotorInvert = false;
    public static final boolean angleMotorInvert = true;

    /* Swerve Profiling Values */
    public static final double maxVelocityMetersPerSecond = 5880.0 / 60.0 /
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) * 0.10033 * Math.PI;

    public static final double maxAngularVelocityRadiansPerSecond = maxVelocityMetersPerSecond /
            Math.hypot(drivetrainTrackWidthMeters / 2.0, drivetrainWheelBaseMeters / 2.0);

    /* Swerve Current Limiting */
    public static final int angleContinousCurentLimit = 20;// FIXME
    public static final int driveContinousCurrentLimit = 40;// FIXME

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;// FIXME //try 1.0
    public static final double angleKI = 0.0;// FIXME
    public static final double angleKD = 0.0;// FIXME //try 0.1
    public static final double angleKFF = 0.0;// FIXME

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;// FIXME maybe 0.01
    public static final double driveKI = 0.0;// FIXME
    public static final double driveKD = 0.0;// FIXME
    public static final double driveKFF = 0.0;// FIXME

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.0;// FIXME
    public static final double driveKV = 0.0;// FIXME Volt-seconds per meter (max voltage divided by max speed) 12/MAX
                                             // SPEED
    public static final double driveKA = 0.0;// FIXME Volt-seconds^2 per meter (max voltage divided by max accel) 12/max
                                             // acceleration

    /** SDS MK4i l1 - 8.14 : 1 */
    /** SDS MK4i l2 - 6.75 : 1 */
    /** SDS MK4i l3 - 6.12 : 1 */
    /// public static final double driveGearRatio = (50.0/14.0) * (17.0/27.0) *
    /// (45.0/15.0); // 6.75:1
    public static final double driveGearRatio = (6.75 / 1.0);

    /** SDS MK4i l1 - (150 / 7) : 1 */
    /** SDS MK4i l2 - (150 / 7) : 1 */
    /** SDS MK4i l3 - (150 / 7) : 1 */
    public static final double angleGearRatio = (150.0 / 7.0); // Same for all of the MK4i modules

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    public static final class SoftLimits {
        public static final double armRatio = 5 * 4 * 3; // 5, 4, 3 OR 60
        public static final double armSprocketRatio = 64. / 22; // top sprocket over lower sprocket (physically on
                                                                // robot)
        public static final double armMotorRotationsPerEveryFullRotation = armRatio * armSprocketRatio; // 174.545

        public static final double extenderRatio = Math.pow(3, 3); // 3, 3, 3 OR 27
        public static final double extenderSprocketRatio = 1; // top sprocket over lower sprocket (physically on robot)
        public static final double extenderMotorRotationsPerEveryFullRotation = extenderRatio * extenderSprocketRatio; // 27

        public static final double pivotRatio = Math.pow(5, 3); // 5, 5, 5 OR 125
        public static final double pivotpSrocketRatio = 16. / 22; // top sprocket over lower sprocket (physically on
                                                                  // robot)
        public static final double pivotMotorRotationsPerEveryFullRotation = pivotRatio * pivotpSrocketRatio; // 90.909

        public static final float armForwardLimit = 44;
        public static final float armReverseLimit = -2;

        public static final float extenderForwardLimit = 0;
        public static final float extenderReverseLimit = -270;

        public static final float wristForwardLimit = 5;
        public static final float wristReverseLimit = -30;
        public static final float wristMAXReverseLimit = -40;
    }

    /*------- CANcoder Config ------- */
    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;// Change to true if moving the module right isnt positive

    /*
     * Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional
     * rectangular/square 4 module swerve
     */
    public final SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(drivetrainTrackWidthMeters / 2.0, drivetrainWheelBaseMeters / 2.0),
            // Front right
            new Translation2d(drivetrainTrackWidthMeters / 2.0, -drivetrainWheelBaseMeters / 2.0),
            // Back left
            new Translation2d(-drivetrainTrackWidthMeters / 2.0, drivetrainWheelBaseMeters / 2.0),
            // Back right
            new Translation2d(-drivetrainTrackWidthMeters / 2.0, -drivetrainWheelBaseMeters / 2.0));

    public static final class clawSetting {
        public static final double clawP = 0;
        public static final double clawI = 0;
        public static final double clawD = 0;
        public static final double clawFF = 0;
        public static final double maxVoltage = 12;
        public static final double clawConversionPositionFactor = 0;// FIXME need to find the conversion Factor
        public static final double clawConversionVelocityFactor = clawConversionPositionFactor / 60;

        public static final int clawContinousCurrentLimit = 20;

        public static final IdleMode clawNeutralMode = IdleMode.kBrake;

        public static class SmartMotionParameters {
            public static final int smartMotionSlot = 0;
            public static final double maxVel = 5200.0;
            public static final double minVel = 0.0;
            public static final double maxAccel = 1000.0;
            public static final double maxErr = 100.0;
        }

    }

    public static final class armSetting {
        public static final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.1 * Math.PI;

        public static final double armPup = 0.01;
        public static final double armIup = 0;
        public static final double armDup = 0;
        public static final double armFFup = 0;

        public static final double armPdown = 0.05;
        public static final double armIdown = 0;
        public static final double armDdown = 0;
        public static final double armFFdown = 0;

        public static final double maxVoltage = 12;
        // public static final double armConversionFactor = 0;// FIXME need to find the
        // conversion Factor

        public static final int armContinousCurrentLimit = 20;
        public static final double armSpeed = 0.3;
        public static final double armSpeedReverse = -0.1;

        public static final IdleMode armNeutralMode = IdleMode.kBrake;

        public static class SmartMotionParameters {
            public static final int smartMotionSlot = 0;
            public static final double maxVel = 5200.0;
            public static final double minVel = 0.0;
            public static final double maxAccel = 1000.0;
            public static final double maxErr = 100.0;
        }

    }

    public static final class extenderSetting {
        public static final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.1 * Math.PI;

        public static final double extenderP = 0.05;
        public static final double extenderI = 0;
        public static final double extenderD = 0;
        public static final double extenderFF = 0;
        public static final double maxVoltage = 12;
        // public static final double extenderConversionFactor = 0;// FIXME need to find
        // the conversion Factor

        public static final int extenderContinousCurrentLimit = 20;
        public static final double extenderSpeed = 1;
        public static final double extenderSpeedReverse = -1;

        public static final IdleMode extenderNeutralMode = IdleMode.kBrake;

        public static class SmartMotionParameters {
            public static final int smartMotionSlot = 0;
            public static final double maxVel = 5200.0;
            public static final double minVel = 0.0;
            public static final double maxAccel = 1000.0;
            public static final double maxErr = 100.0;
        }
    }

    public static final class wristSetting {
        public static final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.1 * Math.PI;

        public static final double wristP = 0.05;
        public static final double wristI = 0;
        public static final double wristD = 0;
        public static final double wristFF = 0;

        public static final double maxVoltage = 12;
        // public static final double wristConversionFactor = 0;// FIXME need to find
        // the conversion Factor

        public static final int wristContinousCurrentLimit = 20;
        public static final double wristSpeed = 0.1;
        public static final double wristSpeedReverse = -0.1;

        public static final IdleMode wristNeutralMode = IdleMode.kBrake;

        public static class SmartMotionParameters {
            public static final int smartMotionSlot = 0;
            public static final double maxVel = 5200.0;
            public static final double minVel = 0.0;
            public static final double maxAccel = 1000.0;
            public static final double maxErr = 100.0;
        }
    }

    public static final class setPoint {
        public static final int armMax = 44;// TODO FIX LATER
    }

    public static final class AutoConstants {
        // tune later

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1.05;
        public static final double kPYController = 1.05;
        public static final double kPThetaController = 1;

        public static final HashMap<String, Command> EventMap = new HashMap<>();

    }
}
