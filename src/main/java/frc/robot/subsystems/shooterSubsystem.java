// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class shooterSubsystem extends SubsystemBase {

    public enum ShootingStates {
        ACTIVE,
        READY,
        IDLE,
        REVERSE
    }

    public enum RunMode {
        AUTOMATIC,
        STATIC
    }

    private static final InterpolatingDoubleTreeMap SHOOTER_LUT = new InterpolatingDoubleTreeMap();
    static {
        //aqui se añaden los datos del tiro (EJEMPLO)
        SHOOTER_LUT.put(
                5.0, //distancia (metros)
                0.85 //poder motor
        );
        SHOOTER_LUT.put(
                3.0,
                0.65
        );
        //añadir los mas datos posibles
    }

    ShootingStates state = ShootingStates.IDLE;
    RunMode runMode = RunMode.STATIC;
    private VictorSPX leftMotor;

    private PIDController shooterpid;

    private VictorSPX transferMotor;

    private PIDController transferpid;

    Timer spoolUpTimer = new Timer();
    Timer preReadyTimer = new Timer();

    double staticShooterOutput = 0.8;
    double lutShooterOutput = 0.0;

    double transferoutput = 0.8;
    double transferReverseOutput = -0.4;
    double transferReverseDuration = 1.5;

    /** Creates a new shooterSubsystem. */
    public shooterSubsystem() {

        leftMotor = new VictorSPX(ShooterConstants.kShooterId);

        shooterpid = new PIDController(1, 0.0, 0.0);

        transferMotor = new VictorSPX(3);

        transferpid = new PIDController(1, 0.0, 0.0);
    }

    /**
     * A distance meter se le pasa la distancia obtenida a partir de la limelight (en metros)
     * @param distanceMeters distance from limelight
     */
    public void updateLUT(double distanceMeters) {
        lutShooterOutput = Math.clamp(SHOOTER_LUT.get(distanceMeters), ShooterConstants.SHOOTER_LUT_MIN, ShooterConstants.SHOOTER_LUT_MAX);
    }

    public void setShootingPower(double output){
        if(ShooterConstants.kShooterReversed){leftMotor.set(VictorSPXControlMode.PercentOutput, output);}
        else{leftMotor.set(VictorSPXControlMode.PercentOutput, output);}
    }

    public void reverseShoot(){
        ShooterConstants.kShooterReversed = !ShooterConstants.kShooterReversed;
    }

    public boolean ShooterActive(){
        if(staticShooterOutput > 0.4){
            return true;

        }
        return false;
    }

    public boolean ShooterCharged(){
        if( MathUtil.isNear(staticShooterOutput, leftMotor.getMotorOutputPercent(), 0.075) ){
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
                state = ShootingStates.READY;
                break;

            case 3:
                state = ShootingStates.REVERSE;
                break;
        }
    }

    public void toggleRunMode() {
        if(this.runMode == RunMode.AUTOMATIC) {
            this.runMode = RunMode.STATIC;
         } else if(this.runMode == RunMode.STATIC) {
            this.runMode = RunMode.AUTOMATIC;
        }
    }

    public void setPowers(double shooteroutput, double transferoutput){
        this.staticShooterOutput = shooteroutput;
        this.transferoutput = transferoutput;

    }

    public void Transferpower(double output){

        transferpid.calculate(transferMotor.getMotorOutputPercent(),output);
        transferMotor.set(VictorSPXControlMode.PercentOutput, output);

    }

    public void stopTransfer(){
        transferMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
    }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("ShooterReversed", ShooterConstants.kShooterReversed);
        SmartDashboard.putBoolean("ShooterActive", ShooterCharged());
        SmartDashboard.putString("shooterState", state.toString());

        //updateLUT(dist)
        switch (state) {
            case IDLE:
                setShootingPower(0.15);

                Transferpower(0.0);

                if(spoolUpTimer.isRunning()){
                    spoolUpTimer.stop();
                    spoolUpTimer.reset();
                }

                //update LUT data
                break;

            case ACTIVE:

                spoolUpTimer.start();
                setShootingPower(this.runMode == RunMode.AUTOMATIC ? lutShooterOutput : staticShooterOutput);

                if(spoolUpTimer.get() < transferReverseDuration) {
                    Transferpower(transferReverseOutput);
                } else {
                    Transferpower(0.0);
                }

                if(spoolUpTimer.advanceIfElapsed(1.5)) {
                    state = ShootingStates.READY;
                    Transferpower(transferoutput);
                }
                break;

            case READY:

                break;

            case REVERSE:
                Transferpower(transferReverseOutput);
                break;
        }

        SmartDashboard.putNumber("time", spoolUpTimer.get());
        SmartDashboard.putNumber("transferoutput", transferMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("shooterOutput", leftMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("LUT Shooter Output", this.lutShooterOutput);
    }
}