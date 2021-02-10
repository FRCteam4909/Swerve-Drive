package frc.robot.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

//Implemented as a copypasta of Jack-in-the-Bot's code, with a bunch of edits.

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class SwerveModule extends PIDSubsystem{

    private double mLastTargetAngle;
    private int mModuleNumber;
    private DutyCycleEncoder mEncoder;
    public TalonFX mTopMotor, mBottomMotor;

    //TODO Check if inverts are needed. (on both)

    public SwerveModule(int moduleNumber, TalonFX topMotor, TalonFX bottomMotor, DutyCycleEncoder encoder){
        super(new PIDController(RobotConstants.swerveTurningkP, RobotConstants.swerveTurningkI, RobotConstants.swerveTurningkD));

        mModuleNumber = moduleNumber;
        mTopMotor = topMotor;
        mBottomMotor = bottomMotor;
        mEncoder = encoder;
        //Config Encoder

        //Config Motors
        mTopMotor.configFactoryDefault();
        mTopMotor.setNeutralMode(NeutralMode.Brake);
        mTopMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        mTopMotor.setSelectedSensorPosition(0);

        mBottomMotor.configFactoryDefault();
        mBottomMotor.setNeutralMode(NeutralMode.Brake);
        mBottomMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        mBottomMotor.setSelectedSensorPosition(0);
    }

    public double getError(){
        return 0.01;
    }

    public void setSpeeds(double linearSpeedTop, double linearSpeedBottom){
        mTopMotor.set(ControlMode.Velocity, linearSpeedTop);
        mBottomMotor.set(ControlMode.Velocity, linearSpeedBottom);
    }

    public void anglePID(double absoluteAngle){

    }

    public void setTargetAngle(double targetAngle){
        mLastTargetAngle = targetAngle;
        setSetpoint(mLastTargetAngle);
    }

    public double getTargetAngle(){
        return mLastTargetAngle;
    }

    public double getCurrentAngle(){
        return mEncoder.get();
    }

    @Override
    public void useOutput(double output, double setpoint) {
        
    }

    @Override
    public double getMeasurement() {
        return this.getCurrentAngle();
    }
}