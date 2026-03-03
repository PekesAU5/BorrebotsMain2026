// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  public enum IntakeState {
    INTAKING,
    HOME,
    IDLE
  }

  public IntakeState state = IntakeState.IDLE;
  private SparkMax pivotSpark;
  private SparkMax rollerSpark;

  private RelativeEncoder pivotSparkEncoder;
  private RelativeEncoder rollerSparkEncoder;

  private SparkMaxConfig pivotConfig;
  private SparkMaxConfig rollerConfig;

  private ProfiledPIDController pivotPid = new ProfiledPIDController(0.085, 0.0, 0.0, new Constraints(1000, 375));

// private PIDController pivotPid = new PIDController(0.08, 0.0, 0.0);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    pivotSpark = new SparkMax(IntakeConstants.kPivotId, MotorType.kBrushless);
    rollerSpark = new SparkMax(IntakeConstants.kRollerId, MotorType.kBrushless);

    pivotSparkEncoder = pivotSpark.getEncoder();
    rollerSparkEncoder = rollerSpark.getEncoder();

    pivotConfig = new SparkMaxConfig();
    rollerConfig = new SparkMaxConfig();

    pivotConfig.idleMode(IdleMode.kBrake)
            .inverted(IntakeConstants.kPivotInverted)
            .smartCurrentLimit(50);

    pivotConfig.encoder.positionConversionFactor(IntakeConstants.kAngleFactor)
            .velocityConversionFactor(IntakeConstants.kAngleFactor / 60);

    pivotSpark.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rollerSpark.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


    public double getAngle(){
    return pivotSparkEncoder.getPosition();
  }

    public double getVelocity(){
    return pivotSparkEncoder.getVelocity();
  }

    public void setPower(double output){
  pivotSpark.set(output*0.2);
}

    public void setState(int intstate){
        switch (intstate) {
            case 0:
                state = IntakeState.IDLE;
            break;
  
            case 1:
                state = IntakeState.INTAKING;
            break;

            case 2:
                state = IntakeState.HOME;
            break;
        }
    }

    public Command setAngle(double angle){
 

        return Commands.run(()->{
            double output = pivotPid.calculate(getAngle(), angle);
            pivotSpark.setVoltage(MathUtil.clamp(-1, output, 1));
    }, this).beforeStarting(()->pivotPid.reset(getAngle()), this);}



    public void setRollerPower(double output){
  rollerSpark.set(output);
}

    public void stopMotors(){
        pivotSpark.stopMotor();
        rollerSpark.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("pivotAngle", getAngle());
        SmartDashboard.putBoolean("atSetpoint", pivotPid.atSetpoint());
        SmartDashboard.putNumber("setpoint", pivotPid.getGoal().position);
        double output = 0.0;
        double rolleroutput = 0.0;

        switch (state) {
            case INTAKING:
                pivotPid.setGoal(IntakeConstants.intakePos);
                output = pivotPid.calculate(getAngle());
                pivotSpark.set(MathUtil.clamp(-1, output*0.1, 1));
                if(MathUtil.isNear(IntakeConstants.intakePos, getAngle(), 5.0)) { rolleroutput = 0.7; }
                rollerSpark.set(rolleroutput);
            break;
            case HOME:
                pivotPid.setGoal(IntakeConstants.kHomePosition);
                if(!MathUtil.isNear(IntakeConstants.intakePos, getAngle(), 5.0)){rolleroutput = 0.0;}
                rollerSpark.set(rolleroutput);
                output = pivotPid.calculate(getAngle(), IntakeConstants.kHomePosition);
                // if(pivotPid.atSetpoint()) output = 0.0;
                pivotSpark.set(MathUtil.clamp(-1, output*0.1, 1));
                break;
            case IDLE:
                stopMotors();
            break;
        }
        SmartDashboard.putString("intakeState", state.toString());
        // This method will be called once per scheduler run
  }
}
