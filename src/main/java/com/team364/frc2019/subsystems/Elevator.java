package com.team364.frc2019.subsystems;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.team1323.loops.*;
import com.team1323.subsystems.requests.*;
import com.team1323.lib.util.Util;
import com.team254.drivers.LazyTalonSRX;
import com.team364.frc2019.RobotMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends Subsystem {
	private static Elevator instance = null;

	public static Elevator getInstance() {
		if(instance == null)
			instance = new Elevator();
		return instance;
	}
	
	LazyTalonSRX lift, liftSlave, arm;
	private double targetHeight = 0.0;
	public double getTargetHeight(){
		return targetHeight;
	}
	private boolean configuredForAscent = true;
	private boolean limitsEnabled = false;
	public boolean limitsEnabled(){
		return limitsEnabled;
	}
	
/*	public TalonSRX getPigeonTalon(){
		return motor2;
	}*/

	
	public enum ControlState{
		Neutral, Position, OpenLoop, Locked
	}
	private ControlState state = ControlState.Neutral;
	public ControlState getState(){
		return state;
	}
	public void setState(ControlState newState){
		state = newState;
	}

	PeriodicIO periodicIO = new PeriodicIO();

  private static Elevator Instance = null;
  public double TargetHeight;
  public double TargetAngle;
  private PWM servoCamera; 
  private PigeonIMU pigeon;
  private PigeonIMU.GeneralStatus gen_status;
  double[] yaw = new double[3]; 
  

  public Elevator() {
    lift = new LazyTalonSRX(RobotMap.topLift);
    liftSlave = new LazyTalonSRX(RobotMap.bottomLift);
    liftSlave.follow(lift);
    liftSlave.setInverted(true);
    lift.setInverted(false);
    liftSlave.setNeutralMode(NeutralMode.Brake);
    servoCamera = new PWM(RobotMap.servoCamera);
    pigeon = new PigeonIMU(liftSlave);
    gen_status = new PigeonIMU.GeneralStatus();
    pigeon.getGeneralStatus(gen_status);
    pigeon.setYaw(0);
    arm = new LazyTalonSRX(RobotMap.arm);
    lift.configFactoryDefault();
    arm.configFactoryDefault();
    lift.configPeakCurrentLimit(RobotMap.liftCurrentCeiling);
    lift.configPeakCurrentDuration(RobotMap.liftCurrentCeilingDuration);
    liftSlave.configPeakCurrentLimit(RobotMap.liftCurrentCeiling);
    lift.configPeakCurrentLimit(RobotMap.liftCurrentCeilingDuration);

    lift.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PIDLoopIdx, RobotMap.TimeoutMs);
    arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.PIDLoopIdx, RobotMap.TimeoutMs);

    lift.setSensorPhase(RobotMap.liftReverse);
    lift.setInverted(RobotMap.liftReverseEncoder);
    arm.setSensorPhase(RobotMap.armReverse);
    arm.setInverted(RobotMap.armReverseEncoder);

    lift.setNeutralMode(NeutralMode.Brake);
    arm.setNeutralMode(NeutralMode.Brake);

    lift.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, RobotMap.TimeoutMs);
    lift.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, RobotMap.TimeoutMs);
    arm.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, RobotMap.TimeoutMs);
    arm.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, RobotMap.TimeoutMs);

    lift.configNominalOutputForward(RobotMap.liftNominalOutputForward, RobotMap.TimeoutMs);
    lift.configNominalOutputReverse(RobotMap.liftNominalOutputReverse, RobotMap.TimeoutMs);
    lift.configPeakOutputForward(RobotMap.liftPeakOutputForward, RobotMap.TimeoutMs);
    lift.configPeakOutputReverse(RobotMap.liftPeakOutputReverse, RobotMap.TimeoutMs);
    arm.configNominalOutputForward(RobotMap.armNominalOutputForward, RobotMap.TimeoutMs);
    arm.configNominalOutputReverse(RobotMap.armNominalOutputReverse, RobotMap.TimeoutMs);
    arm.configPeakOutputForward(RobotMap.armPeakOutputForward, RobotMap.TimeoutMs);
    arm.configPeakOutputReverse(RobotMap.armPeakOutputReverse, RobotMap.TimeoutMs);

    lift.selectProfileSlot(RobotMap.SlotIdx, RobotMap.PIDLoopIdx);
    lift.config_kF(RobotMap.SlotIdx, RobotMap.liftFgain, RobotMap.TimeoutMs);
    lift.config_kP(RobotMap.SlotIdx, RobotMap.liftPgain, RobotMap.TimeoutMs);
    lift.config_kI(RobotMap.SlotIdx, RobotMap.liftIgain, RobotMap.TimeoutMs);
    lift.config_kD(RobotMap.SlotIdx, RobotMap.liftDgain, RobotMap.TimeoutMs);

    arm.selectProfileSlot(RobotMap.SlotIdx, RobotMap.PIDLoopIdx);
    arm.config_kF(RobotMap.SlotIdx, RobotMap.armFgain, RobotMap.TimeoutMs);
    arm.config_kP(RobotMap.SlotIdx, RobotMap.armPgain, RobotMap.TimeoutMs);
    arm.config_kI(RobotMap.SlotIdx, RobotMap.armIgain, RobotMap.TimeoutMs);
    arm.config_kD(RobotMap.SlotIdx, RobotMap.armDgain, RobotMap.TimeoutMs);

    lift.configMotionCruiseVelocity(RobotMap.liftCruiseVelocity, RobotMap.TimeoutMs);
    lift.configMotionAcceleration(RobotMap.liftAcceleration, RobotMap.TimeoutMs);
    arm.configMotionCruiseVelocity(RobotMap.armCruiseVelocity, RobotMap.TimeoutMs);
    arm.configMotionAcceleration(RobotMap.armAcceleration, RobotMap.TimeoutMs);

    lift.setSelectedSensorPosition(0);
    arm.setSelectedSensorPosition(0);
  }

  public void setClimbCruiseVelocity() {
    lift.configMotionCruiseVelocity(RobotMap.liftCruiseVelocityClimb, RobotMap.TimeoutMs);
  }

  public void setPlayCruiseVelocity() {
    lift.configMotionCruiseVelocity(RobotMap.liftCruiseVelocity, RobotMap.TimeoutMs);
  }

	public void enableLimits(boolean enable){
		lift.overrideSoftLimitsEnable(enable);
		limitsEnabled = enable;
	}	
	
	public void setOpenLoop(double output){
		setState(ControlState.OpenLoop);
		periodicIO.demand = output;
	}
	
	public boolean isOpenLoop(){
		return getState() == ControlState.OpenLoop;
	}
	
	public synchronized void setTargetHeight(double height, double tArm){
		setState(ControlState.Position);
		if(height > RobotMap.liftUpperBound);
			height = RobotMap.liftUpperBound;
		if(height < RobotMap.liftLowerBound)
			height = RobotMap.liftLowerBound;
		targetHeight = height;
		periodicIO.demand = height;
		arm.set(ControlMode.MotionMagic, tArm);
		System.out.println("Set elevator height to: " + height);
		onTarget = false;
	}
	
	public synchronized void lockHeight(){
		setState(ControlState.Locked);
		targetHeight = getCounts();
		periodicIO.demand = periodicIO.position;
	}
	
	public Request openLoopRequest(double output){
		return new Request(){
			
			@Override
			public void act(){
				setOpenLoop(output);
			}
			
		};
	}
	
	public Request ElevateToRequest(double height, double tArm){
		return new Request(){
			
			@Override
			public void act() {
				setTargetHeight(height,  tArm);
			}

			@Override
			public boolean isFinished() {
				return hasReachedTargetHeight() || isOpenLoop();
			}
			
		};
	}

	
	public Request lockHeightRequest(){
		return new Request(){
			
			@Override
			public void act(){
				lockHeight();
			}
			
		};
	}
	
	public Prerequisite heightRequisite(double height, boolean above){
		return new Prerequisite(){
		
			@Override
			public boolean met() {
				return Util.epsilonEquals(Math.signum(height - getCounts()), above ? -1.0 : 1.0);
			}

		};
	}

	public double getCounts(){
		return periodicIO.position;
	}
	
	
	private final Loop loop  = new Loop(){

		@Override
		public void onStart(double timestamp) {
			
		}

		@Override
		public void onLoop(double timestamp) {

		}

		@Override
		public void onStop(double timestamp) {
			
		}
		
	};


	@Override
	public synchronized void readPeriodicInputs(){
		periodicIO.position = lift.getSelectedSensorPosition(0);
			periodicIO.velocity = lift.getSelectedSensorVelocity(0);
			periodicIO.voltage = lift.getMotorOutputVoltage();
			periodicIO.current = lift.getOutputCurrent();
	}

	@Override
	public synchronized void writePeriodicOutputs(){
		if(getState() == ControlState.Position || getState() == ControlState.Locked)
			lift.set(ControlMode.MotionMagic, periodicIO.demand);
		else
			lift.set(ControlMode.PercentOutput, periodicIO.demand);
	}


	boolean onTarget = false;
	public boolean hasReachedTargetHeight(){
		if(Math.abs(getTargetHeight() - getCounts()) <= 200 && !onTarget){	
			onTarget = true;
			return true;
		}
		return false;
	}


	@Override
	public void stop() {
		setOpenLoop(0.0);
	}

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(loop);
	}
	
	@Override
	public void outputTelemetry() {
	}

	public static class PeriodicIO{
		//Inputs
		public int position = 0;
		public double velocity = 0.0;
		public double voltage = 0.0;
		public double current = 0.0;

		//outputs
		public double demand;
	}
}
