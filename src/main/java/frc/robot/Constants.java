// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {


    public static final class ControllerConstants{
        public static double controlDeadband = 0.05;
        public static double triggerDeadband = 0.1;

    }

    public static final class ShooterConstants{
        public static final int kShooterId = 2;
        public static boolean kShooterActive = false;
        public static boolean kShooterReversed = true;

        public static int kShooterActiveState = 1;
        public static int kShooterIdle = 0;
        public static int kShooterChargedState = 2;
        public static int kShooterManualReverseState = 3;

        public static double SHOOTER_LUT_MIN = 0.6;
        public static double SHOOTER_LUT_MAX = 0.9;
    }

    public static final class intakeConstants{
        public static final int kpivotId = 31;
        public static final int krollerId = 32;
        public static boolean kpivotInverted = true;
        public static boolean krollerInverted = false;
        public static double kgearRatio = 80/14;
        public static double kReductionFactor = 25*kgearRatio;
        public static double kAngleFactor = 1 / kReductionFactor * 360;
        public static double intakeposition = 62;
        public static double kHomePosition = -0.4;

        public static int kintakeIdleState = 0;
        public static int kIntakingState = 1;
        public static int kintakeHomeState = 2;

        public static double krollerIntakeRPM = 4000.0;

    }

    public static final class DriveConstants{


        public static final double kDirectionSlewRate = 2.4; // radians per second   original es 1.2
        public static final double kMagnitudeSlewRate = 3.6; // percent per second (1 = 100%)  original es 1.8
        public static final double kRotationalSlewRate = 4.0; // percent per second (1 = 100%)   original es 2.0
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static  double kMaxSpeedMetersPerSecond = 4.8;
        public static  double kMaxAngularSpeed = Math.PI*2


                ; // radians per second

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(27.1);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(27.1);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;
        //    public static final double kFrontLeftChassisAngularOffset = Math.PI;
        // public static final double kFrontRightChassisAngularOffset = -Math.PI /2 ;
        // public static final double kBackLeftChassisAngularOffset = Math.PI/2;
        // public static final double kBackRightChassisAngularOffset = 0;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 21;
        public static final int kFrontRightDrivingCanId = 22;
        public static final int kRearRightDrivingCanId = 23;
        public static final int kRearLeftDrivingCanId = 24;


        public static final int kFrontLeftTurningCanId = 11;
        public static final int kFrontRightTurningCanId = 12;
        public static final int kRearRightTurningCanId = 13;
        public static final int kRearLeftTurningCanId =14;

        public static boolean kfieldRelative = true;
        public static final boolean kGyroReversed = false;
        public static boolean kSlowMode = false;
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
        // more teeth will result in a robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;
    }


    public static final class LimelightConstats{
        public static double xPosSetpoint = 0 ;
        public static final double xPosTolerance = 0.2;
        public static double yPosSetpoint = 11;
        public static final double yPosTolerance = 0.02;
        public static final double rotPosTolerance = 0.02;


    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }

    public static final class LEDConstants {
        public static final int TEJUINO_CAN_ID = 0;
    }
}