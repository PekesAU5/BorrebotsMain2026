// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstats;
import frc.robot.LimelightHelpers;


/**
 * Add your docs here.
 */
public class Limelight extends SubsystemBase {
    public String LL = "limelight-ll";
    double xoutput;
    double youtput;
    double rotoutput;
    DriveSubsystem driveSubsystem;
    PIDController zPidController;
    PIDController yPidController;
    PIDController xPidController;


    public Limelight(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        zPidController = new PIDController(0.0075, 0.0, 0.0);
        yPidController = new PIDController(0.035, 0.0, 0.00);
        xPidController = new PIDController(0.035, 0.0, 0.0);

        // xPidController.setSetpoint(limelightConstants.xRightReefSetpoint);
        xPidController.setTolerance(LimelightConstats.xPosTolerance);


        yPidController.setSetpoint(LimelightConstats.yPosSetpoint);

        yPidController.setTolerance(LimelightConstats.yPosTolerance);

        zPidController.setSetpoint(LimelightConstats.rotPosTolerance);
        // zPidController.setTolerance(limelightConstants.rotReefTolerance);

        zPidController.enableContinuousInput(-180.0, 180);


    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Ty", getTy());
        SmartDashboard.putNumber("Tx", getTx());
        SmartDashboard.putBoolean("HasTarget", hasTarget());
        SmartDashboard.putBoolean("isAlligned", isAlligned());
        SmartDashboard.putNumber("yError", yPidController.getError());
        SmartDashboard.putNumber("xError", xPidController.getError());
        SmartDashboard.putNumber("xSetpoint", LimelightConstats.xPosSetpoint);
        SmartDashboard.putNumber("zSetpoint", LimelightConstats.yPosSetpoint);
        SmartDashboard.putNumber("DesiredRotation", getDesiredRotation());
        SmartDashboard.putNumber("Rotation Error", zPidController.getError());

        if (!hasTarget()) {
            xoutput = 0.0;
            youtput = 0.0;
            rotoutput = 0.0;
        }
    }

    public int priorityTag() {
        int id = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? 25 : 10;
        return id;

    }

    public double getTy() {
        double positions = LimelightHelpers.getTargetPose3d_RobotSpace(LL).getZ();
        return positions * 10;
    }


    public double getTx() {

        double positions = LimelightHelpers.getTargetPose3d_RobotSpace(LL).getX();

        return positions * 10;


    }

    public double getRy() {
        double rotation = LimelightHelpers.getTargetPose3d_RobotSpace(LL).getRotation().getY();

        return rotation;
    }

    public double getDesiredRotation() {

        double Ty = LimelightHelpers.getTargetPose3d_RobotSpace(LL).getZ();
        double Tx = LimelightHelpers.getTargetPose3d_RobotSpace(LL).getX();


        double vx = driveSubsystem.getRobotRelativeSpeeds().vxMetersPerSecond;
        double vy = driveSubsystem.getRobotRelativeSpeeds().vyMetersPerSecond;


        switch (getId()) {
            case 8:
                Tx = Tx - Units.inchesToMeters(5);
                break;

            case 24:
                Tx = Tx - Units.inchesToMeters(5);
                break;

            case 11:
                Tx = Tx + Units.inchesToMeters(5);
                break;

            case 9:
                Tx = Tx - Units.inchesToMeters(12);
                break;
            case 27:
                Tx = Tx + Units.inchesToMeters(5);
                break;
            default:
                break;
        }

        double velocityoffset = Math.atan2(vy, vx);

        double desiredRotation = Math.atan2(Ty, Tx);

        desiredRotation = 90 - Math.toDegrees(desiredRotation) + velocityoffset;

        return desiredRotation;
    }

    public double getTa() {
        return LimelightHelpers.getTA(LL);
    }

    public double getTxnc() {
        return NetworkTableInstance.getDefault().getTable("limelight-ll").getEntry("txnc").getDouble(0);
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(LL);
    }

    public int getId() {
        return (int) LimelightHelpers.getFiducialID(LL);
    }

    public void stopCommand() {


        driveSubsystem.drive(0, 0, 0, false);

    }


    public void ResetPids() {
        xPidController.reset();
        yPidController.reset();
        zPidController.reset();
    }


    public boolean isAlligned() {
        if (xPidController.atSetpoint() && yPidController.atSetpoint()
        ) {
            return true;
        } else {
            return false;
        }
    }

    public Command AllignXAxis(CommandXboxController controller) {

        return Commands.run(() -> {
            if (hasTarget()) {
                // double youtput = yPidController.calculate(getTy
                // double xoutput = xPidController.calculate(getTx());
                double rotoutput = zPidController.calculate(getRy(), getDesiredRotation());
                driveSubsystem.drive(0.2 * MathUtil.applyDeadband(-controller.getLeftX(), 0.1),
                        0.2 * MathUtil.applyDeadband(controller.getLeftY(), 0.1) //   youtput *0.2
                        , -rotoutput * 0.5
                        , false);
            } else {

                driveSubsystem.drive(0.2 * MathUtil.applyDeadband(-controller.getLeftX(), 0.1),
                        0.2 * MathUtil.applyDeadband(controller.getLeftY(), 0.1),
                        0.2 * MathUtil.applyDeadband(-controller.getRightX(), 0.1), DriveConstants.kfieldRelative);
            }

        }, driveSubsystem).beforeStarting(() -> ResetPids());

    }
}