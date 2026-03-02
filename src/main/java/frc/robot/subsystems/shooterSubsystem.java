// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.ModuleLayer.Controller;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ShooterConstants;

public class shooterSubsystem extends SubsystemBase {

 

  private VictorSPX leftMotor;
  
  private PIDController pid;

  
  double output = 0.0;
  
  /** Creates a new shooterSubsystem. */
  public shooterSubsystem() {

    leftMotor = new VictorSPX(ShooterConstants.kShooterId);

    pid = new PIDController(0.05, 0.0, 0.0);
  }


  public void setShootingPower(double output){

    this.output = output;

    

if(ShooterConstants.kShooterReversed){leftMotor.set(VictorSPXControlMode.PercentOutput, output);}
else{leftMotor.set(VictorSPXControlMode.PercentOutput, -output);}
}

  public void reverseShoot(){
    ShooterConstants.kShooterReversed = !ShooterConstants.kShooterReversed;
  }

  public boolean ShooterActive(){
    if(output > 0.1){
      return true;
    }
    return false;
  }

  public boolean ShooterCharged(){
    if(leftMotor.getMotorOutputPercent()> 0.8){
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("ShooterReversed", ShooterConstants.kShooterReversed);
    SmartDashboard.putBoolean("ShooterActive", ShooterCharged());
 
  }
}
