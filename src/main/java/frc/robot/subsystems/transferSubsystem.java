// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class transferSubsystem extends SubsystemBase {
   private VictorSPX transferMotor;


   private PIDController pidController;
  /** Creates a new transferSubsystem. */
  public transferSubsystem() {

    transferMotor = new VictorSPX(3);

    pidController = new PIDController(0.09, 0, 0);
   


      
  }
  public void activateTransfer(double output){
    pidController.calculate(transferMotor.getMotorOutputPercent(),output);
    transferMotor.set(VictorSPXControlMode.PercentOutput, output);
  }

  public void stopTransfer(){
    transferMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
  }
  @Override
  public void periodic() {

    transferMotor.set(VictorSPXControlMode.PercentOutput, 0.0);

    SmartDashboard.putNumber("transferoutput", transferMotor.getMotorOutputPercent());
    // This method will be called once per scheduler run
  }
}
