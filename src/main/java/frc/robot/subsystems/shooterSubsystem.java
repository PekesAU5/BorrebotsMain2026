// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class shooterSubsystem extends SubsystemBase {

    public enum ShootingStates{
        ACTIVE,
        CHARGED,
        IDLE
    }

    ShootingStates state = ShootingStates.IDLE;
    private VictorSPX leftMotor;

    private PIDController shooterpid;

    private VictorSPX transferMotor;
    private VictorSPX transfer2Motor;

    private PIDController transferpid;
    private PIDController transfer2pid;

    Timer timer = new Timer();

    double shooteroutput = 0.95;
    double transferoutput = 0.8;
    double transfer2output = 0.8;


    /** Creates a new shooterSubsystem. */
    public shooterSubsystem() {

        leftMotor = new VictorSPX(ShooterConstants.kShooterId);

        shooterpid = new PIDController(1, 0.0, 0.0);

        transferMotor = new VictorSPX(3);
        transfer2Motor = new VictorSPX(4);

        transferpid = new PIDController(1, 0.0, 0.0);
        transfer2pid = new PIDController(1, 0.0, 0.0);
    }


    public void setShootingPower(double output){
        if(ShooterConstants.kShooterReversed){leftMotor.set(VictorSPXControlMode.PercentOutput, output);}
        else{leftMotor.set(VictorSPXControlMode.PercentOutput, output);}
    }

    public void reverseShoot(){
        ShooterConstants.kShooterReversed = !ShooterConstants.kShooterReversed;
    }

    public boolean ShooterActive(){
        if(shooteroutput > 0.4){
            return true;

        }
        return false;
    }

    public boolean ShooterCharged(){
        if( MathUtil.isNear(shooteroutput, leftMotor.getMotorOutputPercent(), 0.075) ){
            return true;
        }
        return false;
    }

    public void setState(int intstate){
        switch (intstate) {
            case 0:
                state = ShootingStates.IDLE;
                break;

            case 1:
                state = ShootingStates.ACTIVE;
                break;

            case 2:
                state = ShootingStates.CHARGED;
                break;
        }

    }

    public void setPowers(double shooteroutput, double transferoutput, double transfer2output){
        this.shooteroutput = shooteroutput;
        this.transferoutput = transferoutput;
        this.transfer2output = transfer2output;

    }

    public void Transferpower(double output, double output2){

        transferpid.calculate(transferMotor.getMotorOutputPercent(),output);
        transferMotor.set(VictorSPXControlMode.PercentOutput, output);

        transfer2pid.calculate(transfer2Motor.getMotorOutputPercent(),output2);
        transfer2Motor.set(VictorSPXControlMode.PercentOutput, output2);
    }

    public void stopTransfer(){
        transferMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
        transfer2Motor.set(VictorSPXControlMode.PercentOutput, 0.0);
    }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("ShooterReversed", ShooterConstants.kShooterReversed);
        SmartDashboard.putBoolean("ShooterActive", ShooterCharged());
        SmartDashboard.putString("shooterState", state.toString());

        switch (state) {
            case IDLE:
                setShootingPower(0.15);

                Transferpower(0.0, 0.0);

                if(timer.isRunning()){
                    timer.stop();
                    timer.reset();
                }

                break;

            case ACTIVE:

                timer.start();
                setShootingPower(shooteroutput);

                if(timer.advanceIfElapsed(1)){state = ShootingStates.CHARGED;
                    Transferpower(transferoutput,transfer2output);}


            case CHARGED:

                break;
        }

        SmartDashboard.putNumber("time", timer.get());
        SmartDashboard.putNumber("transferoutput", transferMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("transfer2output", transfer2Motor.getMotorOutputPercent());
        SmartDashboard.putNumber("shooterOutput", leftMotor.getMotorOutputPercent());
    }
}