// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.transferSubsystem;

/** Add your docs here. */
public class RobotContainer {

    DriveSubsystem Chassis = new DriveSubsystem();
    shooterSubsystem shooter = new shooterSubsystem();
    Limelight limelight = new Limelight(Chassis);
    
    transferSubsystem transfer = new transferSubsystem();
    //  private final SendableChooser<Command> autoChooser;

CommandXboxController m_Controller = new CommandXboxController(0); 
// CommandPS5Controller m_Controller = new CommandPS5Controller(0);

public RobotContainer(){

 configureButtons();

Chassis.setDefaultCommand(new RunCommand(()-> Chassis.drive(
MathUtil.applyDeadband(-m_Controller.getLeftX()*0.4, ControllerConstants.controlDeadband),
MathUtil.applyDeadband(m_Controller.getLeftY()*0.4, ControllerConstants.controlDeadband),
MathUtil.applyDeadband(-m_Controller.getRightX()*0.2, ControllerConstants.controlDeadband),
 DriveConstants.kfieldRelative), Chassis));
 
//  limelight.setDefaultCommand(new RunCommand(()->limelight.stopCommand(), limelight));

 shooter.setDefaultCommand(new RunCommand(()-> shooter.setShootingPower(0.0), shooter));

//  shooter.setDefaultCommand(new RunCommand(()-> shooter.setShootingPower(m_Controller.getLeftX()), shooter));


// turret.setDefaultCommand(new RunCommand(()-> turret.stopMotors(), turret));
// transfer.setDefaultCommand(new RunCommand(()-> transfer.stopTransfer(), transfer));
//   autoChooser = AutoBuilder.buildAutoChooser();

    // SmartDashboard.putData("Auto Chooser", autoChooser);

}

private void configureButtons(){

    // m_Controller.start().onTrue(Chassis.changeDrivingMode());

    // m_Controller.leftBumper().onChange(Chassis.changeSpeed());

    m_Controller.b().whileTrue(limelight.AllignXAxis(m_Controller));
m_Controller.x().onTrue(new InstantCommand(()->shooter.reverseShoot(), shooter));
m_Controller.rightTrigger(0.05).whileTrue(new RunCommand(()-> shooter.setShootingPower(0.8), shooter));
    // m_Controller.rightTrigger(0.05).whileTrue(Commands.run(()->{transfer.activateTransfer(0.6);}, transfer).unless(()->!shooter.ShooterCharged()));
m_Controller.leftTrigger(0.05).whileTrue(new RunCommand(()->transfer.activateTransfer(0.6), transfer));


    m_Controller.leftBumper().onTrue(Chassis.changeSpeed());

    m_Controller.rightBumper().onTrue(Chassis.changeDrivingMode());

    // m_Controller.a().onTrue(new InstantCommand(()-> Chassis.zeroHeading(), Chassis));
}   


public Command getAutonomousCommand(){


    return null;
}

public void shuffleboardData(){
    
}

}
