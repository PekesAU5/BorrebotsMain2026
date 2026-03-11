// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.intakeConstants;
import frc.robot.subsystems.LEDHandler;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;

import frc.robot.subsystems.shooterSubsystem;


/** Add your docs here. */
public class RobotContainer {

    DriveSubsystem Chassis = new DriveSubsystem();
    shooterSubsystem shooter = new shooterSubsystem();

    Limelight limelight = new Limelight(Chassis);

    IntakeSubsystem intake = new IntakeSubsystem();

    private LEDHandler ledHandler;
    private final SendableChooser<Command> autoChooser;

    CommandXboxController m_Controller = new CommandXboxController(0);
    CommandXboxController m_Controller1 = new CommandXboxController(1);


    public RobotContainer(){

        configureButtons();

        Chassis.setDefaultCommand(new RunCommand(()-> Chassis.drive(
                MathUtil.applyDeadband(-m_Controller.getLeftX(), ControllerConstants.controlDeadband),
                MathUtil.applyDeadband(m_Controller.getLeftY(), ControllerConstants.controlDeadband),
                MathUtil.applyDeadband(-m_Controller.getRightX(), ControllerConstants.controlDeadband),
                DriveConstants.kfieldRelative), Chassis));

//  limelight.setDefaultCommand(new RunCommand(()->limelight.stopCommand(), limelight));

        shooter.setDefaultCommand(new InstantCommand(()->shooter.setState(ShooterConstants.kShooterIdle), shooter));


//  intake.setDefaultCommand(new InstantCommand(()->intake.setState(intakeConstants.kintakeIdleState), intake));
//  shooter.setDefaultCommand(new RunCommand(()-> shooter.setShootingPower(m_Controller.getLeftX()), shooter));


        ledHandler = new LEDHandler();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureButtons(){

        // m_Controller.start().onTrue(Chassis.changeDrivingMode());

        // m_Controller.leftBumper().onChange(Chassis.changeSpeed());

        m_Controller.rightBumper().whileTrue(limelight.AllignXAxis(m_Controller));

        m_Controller.rightTrigger(0.05).whileTrue(new RunCommand(()->shooter.setState(ShooterConstants.kShooterActiveState), shooter)).whileFalse(new RunCommand(()->shooter.setState(ShooterConstants.kShooterIdle), shooter));


        // m_Controller.rightTrigger(0.05).whileTrue(Commands.run(()->{transfer.activateTransfer(0.6);}, transfer).unless(()->!shooter.ShooterCharged()));

        m_Controller.a().onTrue(Chassis.resetGyro());

        m_Controller.x().whileTrue(Chassis.changeSpeed());

        m_Controller.start().onTrue(Chassis.changeDrivingMode());


        // m_Controller.a().onTrue(new InstantCommand(()-> Chassis.zeroHeading(), Chassis));

        m_Controller.leftTrigger(0.05)
                .whileTrue(new RunCommand(()->intake.setState(intakeConstants.kIntakingState), intake))
                .whileFalse(new RunCommand(()->intake.setState(intakeConstants.kintakeIdleState), intake));
        m_Controller.rightBumper().whileTrue(new RunCommand(()-> intake.setRollerPower(-0.8), intake));
        m_Controller.leftBumper().onTrue(new RunCommand(()->intake.setState(intakeConstants.kintakeHomeState), intake));

        m_Controller.b()
                .toggleOnTrue(Commands.runOnce(() -> ledHandler.setState(LEDHandler.State.IDLE)))
                .toggleOnFalse(Commands.runOnce(() -> ledHandler.setState(LEDHandler.State.OFF)));
    }


    public Command getAutonomousCommand(){
        return autoChooser.getSelected();
    }

    public void shuffleboardData(){

    }

}