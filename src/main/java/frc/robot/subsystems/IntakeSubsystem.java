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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    public enum IntakeState{
        INTAKING,
        HOME,
        IDLE
    }

    public IntakeState state;
    private SparkMax pivotSpark;
    private SparkMax rollerSpark;

    private RelativeEncoder pivotSparkEncoder;
    private RelativeEncoder rollerSparkEncoder;

    private SparkMaxConfig pivotConfig;
    private SparkMaxConfig rollerConfig;

    // private SparkClosedLoopController pivotPid;

    private ProfiledPIDController pivotPid = new ProfiledPIDController(0.0975, 0.0, 0.0, new Constraints(105, 175));

    // private PIDController pivotPid = new PIDController(0.12, 0.0, 0.0);
    private double desiredAngle;

    double output =0.0;
    double rolleroutput = 0.0;

// private PIDController pivotPid = new PIDController(0.08, 0.0, 0.0);
    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        pivotSpark = new SparkMax(intakeConstants.kpivotId, MotorType.kBrushless);

        rollerSpark = new SparkMax(intakeConstants.krollerId, MotorType.kBrushless);

        pivotSparkEncoder = pivotSpark.getEncoder();

        rollerSparkEncoder = rollerSpark.getEncoder();


        pivotConfig = new SparkMaxConfig();

        rollerConfig = new SparkMaxConfig();






        pivotConfig.
                idleMode(IdleMode.kBrake).inverted(intakeConstants.kpivotInverted).smartCurrentLimit(50).openLoopRampRate(0.1);

        pivotConfig.softLimit.forwardSoftLimit(65).reverseSoftLimit(-2.0).forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);
        pivotConfig.encoder.positionConversionFactor(intakeConstants.kAngleFactor)
                .velocityConversionFactor(intakeConstants.kAngleFactor/60);

        //  pivotConfig.closedLoop.outputRange(-50, desiredAngle)

        rollerConfig.inverted(true).smartCurrentLimit(50).idleMode(IdleMode.kBrake);
        pivotSpark.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rollerSpark.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        state = IntakeState.IDLE;

        pivotPid.setTolerance(0.2);

        desiredAngle = 0.0;



    }


    public double getpivotAngle(){
        return pivotSparkEncoder.getPosition();
    }

    public double getpivotVelocity(){
        return pivotSparkEncoder.getVelocity();

    }


    public double getRollersVelocity(){
        return rollerSparkEncoder.getVelocity();


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
        }

    }

    public boolean aroundAngle(double angle){
        return MathUtil.isNear(angle, getpivotAngle(), 1.0);
    }

    public void setRollerPower(double output){
        rollerSpark.set(output);
    }

    public void stopMotors(){
        pivotSpark.stopMotor();
        rollerSpark.stopMotor();
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("pivotAngle", getpivotAngle());
        SmartDashboard.putBoolean("atSetpoint", pivotPid.atSetpoint());
        SmartDashboard.putNumber("setpoint", pivotPid.getSetpoint().position);

        SmartDashboard.putNumber("pivotVelocitt", getpivotVelocity());
// SmartDashboard.putNumber("p", desiredAngle)
        SmartDashboard.putNumber("RollersVelocity", getRollersVelocity());


// pivotPid.setSetpoint(desiredAngle);

//  double output = pivotPid.calculate(getpivotAngle());



        switch (state) {
            case INTAKING:

                if(desiredAngle != intakeConstants.intakeposition && pivotPid.atSetpoint()){pivotPid.reset(getpivotAngle());pivotPid.setGoal(intakeConstants.intakeposition); desiredAngle = intakeConstants.intakeposition;}





                output = MathUtil.clamp(pivotPid.calculate(getpivotAngle()), -1, 1);

                // pivotPid.setSetpoint(desiredAngle, ControlType.kMAXMotionPositionControl);

                pivotSpark.set(output);
                if(aroundAngle(intakeConstants.intakeposition)){rolleroutput = 0.8;}


                rollerSpark.set(rolleroutput);
                break;

            case HOME:
                if(desiredAngle != intakeConstants.kHomePosition && pivotPid.atSetpoint()){pivotPid.reset(getpivotAngle());pivotPid.setGoal(intakeConstants.kHomePosition);desiredAngle = intakeConstants.kHomePosition;}


                if(!aroundAngle(intakeConstants.intakeposition)){rolleroutput = 0.0;}

                rollerSpark.set(rolleroutput);
                output = MathUtil.clamp(pivotPid.calculate(getpivotAngle()), -1, 1);


                pivotSpark.set(output);





                break;

            case IDLE:
                pivotPid.reset(getpivotAngle());
                rollerSpark.set(0.0);
                // pivotSpark.set(MathUtil.clamp(-0.3, output, 0.3));%


                break;


        }

        SmartDashboard.putString("intakeState", state.toString());
        // This method will be called once per scheduler run
    }
}