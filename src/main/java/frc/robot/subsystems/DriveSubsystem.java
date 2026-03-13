// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    // Create MAXSwerveModules
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);

    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);

    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);

    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);
    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            getGyroAngle(),
            new SwerveModulePosition[]{
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
            });
    private double xSpeedGlobal;
    private double ySpeedGlobal;
    private double rotSpeedGlobal;
    // Slew rate filter variables for controlling lateral acceleration
    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;
    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {
        // Usage reporting for MAXSwerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getPose, // Robot pose supplier
                    this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                    ),
                    config, // The robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
    }

    public Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(-m_gyro.getAngle());
    }

    public static double StepTowards(double _current, double _target, double _stepsize) {
        if (Math.abs(_current - _target) <= _stepsize) {
            return _target;
        } else if (_target < _current) {
            return _current - _stepsize;
        } else {
            return _current + _stepsize;
        }
    }

    /**
     * Steps a value (angle) towards a target (angle) taking the shortest path with a specified step
     * size.
     *
     * @param _current  The current or starting angle (in radians). Can lie outside the 0 to 2*PI
     *                  range.
     * @param _target   The target angle (in radians) the algorithm will step towards. Can lie outside
     *                  the 0 to 2*PI range.
     * @param _stepsize The maximum step size that can be taken (in radians).
     * @return The new angle (in radians) for {@code _current} after performing the specified step
     * towards the specified target. This value will always lie in the range 0 to 2*PI
     * (exclusive).
     */
    public static double StepTowardsCircular(double _current, double _target, double _stepsize) {
        _current = WrapAngle(_current);
        _target = WrapAngle(_target);

        double stepDirection = Math.signum(_target - _current);
        double difference = Math.abs(_current - _target);

        if (difference <= _stepsize) {
            return _target;
        } else if (difference > Math.PI) { // does the system need to wrap over eventually?
            // handle the special case where you can reach the target in one step while also wrapping
            if (_current + 2 * Math.PI - _target < _stepsize
                    || _target + 2 * Math.PI - _current < _stepsize) {
                return _target;
            } else {
                return WrapAngle(
                        _current - stepDirection * _stepsize); // this will handle wrapping gracefully
            }

        } else {
            return _current + stepDirection * _stepsize;
        }
    }

    /**
     * Finds the (unsigned) minimum difference between two angles including calculating across 0.
     *
     * @param _angleA An angle (in radians).
     * @param _angleB An angle (in radians).
     * @return The (unsigned) minimum difference between the two angles (in radians).
     */
    public static double AngleDifference(double _angleA, double _angleB) {
        double difference = Math.abs(_angleA - _angleB);
        return difference > Math.PI ? (2 * Math.PI) - difference : difference;
    }

    /**
     * Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).
     *
     * @param _angle The angle (in radians) to wrap. Can be positive or negative and can lie multiple
     *               wraps outside the output range.
     * @return An angle (in radians) from 0 and 2*PI (exclusive).
     */
    public static double WrapAngle(double _angle) {
        double twoPi = 2 * Math.PI;

        if (_angle
                == twoPi) { // Handle this case separately to avoid floating point errors with the floor
            // after the division in the case below
            return 0.0;
        } else if (_angle > twoPi) {
            return _angle - twoPi * Math.floor(_angle / twoPi);
        } else if (_angle < 0.0) {
            return _angle + twoPi * (Math.floor((-_angle) / twoPi) + 1);
        } else {
            return _angle;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("GyroAngle", getGyroAngle().getDegrees());
        // Update the odometry in the periodic block
        m_odometry.update(
                getGyroAngle(),
                new SwerveModulePosition[]{
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });

        if (DriveConstants.kSlowMode) {
            DriveConstants.kMaxSpeedMetersPerSecond = 0.5;
            DriveConstants.kMaxAngularSpeed = Math.PI * 0.6;
        } else {
            DriveConstants.kMaxSpeedMetersPerSecond = 4.8;
            DriveConstants.kMaxAngularSpeed = 2 * Math.PI;
        }

        SmartDashboard.putBoolean("FieldRelative", DriveConstants.kfieldRelative);
        SmartDashboard.putBoolean("SlowMode", DriveConstants.kSlowMode);

        SmartDashboard.putNumberArray("RobotPose", new double[]{
                getPose().getX(),
                getPose().getY(),
                getPose().getRotation().getRadians()
        });
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
                getGyroAngle(),
                new SwerveModulePosition[]{
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                pose);
    }

    public void setGyroAngle(double angle) {
        this.m_gyro.setAngleAdjustment(angle);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {


        double xSpeedCommanded;
        double ySpeedCommanded;

        // Convert XY to polar for rate limiting
        double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
        double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

        // Calculate the direction slew rate based on an estimate of the lateral acceleration
        double directionSlewRate;
        if (m_currentTranslationMag != 0.0) {
            directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
        } else {
            directionSlewRate =
                    500.0; // some high number that means the slew rate is effectively instantaneous
        }

        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - m_prevTime;
        double angleDif = AngleDifference(inputTranslationDir, m_currentTranslationDir);
        if (angleDif < 0.45 * Math.PI) {
            m_currentTranslationDir =
                    StepTowardsCircular(
                            m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
            m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        } else if (angleDif > 0.85 * Math.PI) {
            if (m_currentTranslationMag
                    > 1e-4) { // some small number to avoid floating-point errors with equality checking
                // keep currentTranslationDir unchanged
                m_currentTranslationMag = m_magLimiter.calculate(0.0);
            } else {
                m_currentTranslationDir = WrapAngle(m_currentTranslationDir + Math.PI);
                m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            }
        } else {
            m_currentTranslationDir =
                    StepTowardsCircular(
                            m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
            m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        m_prevTime = currentTime;

        xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
        ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
        m_currentRotation = m_rotLimiter.calculate(rot);


        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;


        xSpeedGlobal = xSpeedDelivered;
        ySpeedGlobal = ySpeedDelivered;
        rotSpeedGlobal = rotDelivered;

        SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                        getGyroAngle())
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return new ChassisSpeeds(xSpeedGlobal, ySpeedGlobal, rotSpeedGlobal);
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));

    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public Command changeDrivingMode() {
        return Commands.runOnce(() -> {
            DriveConstants.kfieldRelative = !DriveConstants.kfieldRelative;
        }, this);
    }

    public Command changeSpeed() {
        return Commands.runOnce(() -> {
            DriveConstants.kSlowMode = !DriveConstants.kSlowMode;
        }, this);


    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        m_gyro.reset();
    }

    public Command resetGyro() {
        return Commands.runOnce(m_gyro::reset, this);
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return -m_gyro.getYaw();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }
}
