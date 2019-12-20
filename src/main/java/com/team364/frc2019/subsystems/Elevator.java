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
	
	LazyTalonSRX master, motor2;
	List<LazyTalonSRX> motors, slaves;
	private double targetHeight = 0.0;
	public double getTargetHeight(){
		return targetHeight;
	}
	private boolean configuredForAscent = true;
	private boolean limitsEnabled = false;
	public boolean limitsEnabled(){
		return limitsEnabled;
	}
	
	public TalonSRX getPigeonTalon(){
		return motor2;
	}

	
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
	
	double manualSpeed = Constants.kElevatorTeleopManualSpeed;
	public void setManualSpeed(double speed){
		manualSpeed = speed;
	}

	PeriodicIO periodicIO = new PeriodicIO();

  private static Elevator Instance = null;
  private TalonSRX arm;
  private TalonSRX lift;
  private TalonSRX liftSlave;
  public double TargetHeight;
  public double TargetAngle;
  private PWM servoCamera; 
  private PigeonIMU pigeon;
  private PigeonIMU.GeneralStatus gen_status;
  double[] yaw = new double[3]; 
  

  public Elevator() {
    lift = new TalonSRX(RobotMap.topLift);
    liftSlave = new TalonSRX(RobotMap.bottomLift);
    liftSlave.follow(lift);
    liftSlave.setInverted(true);
    lift.setInverted(false);
    liftSlave.setNeutralMode(NeutralMode.Brake);
    servoCamera = new PWM(RobotMap.servoCamera);
    pigeon = new PigeonIMU(liftSlave);
    gen_status = new PigeonIMU.GeneralStatus();
    pigeon.getGeneralStatus(gen_status);
    pigeon.setYaw(0);
    arm = new TalonSRX(RobotMap.arm);
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
	private void configForAscent(){		
		manualSpeed = Constants.kElevatorTeleopManualSpeed;

		master.config_kP(0, 1.75, 10);//0.75 going up
		master.config_kI(0, 0.0, 10);
		master.config_kD(0, 40.0, 10);//20.0
		master.config_kF(0, 1023.0/Constants.kElevatorMaxSpeedHighGear, 10);
		
		master.config_kP(1, 1.5, 10);//2.5 going down
		master.config_kI(1, 0.0, 10);
		master.config_kD(1, 30.0, 10);//20.0
		master.config_kF(1, 1023.0/Constants.kElevatorMaxSpeedHighGear, 10);

		master.configMotionCruiseVelocity((int)(Constants.kElevatorMaxSpeedHighGear * 1.0), 10);
		master.configMotionAcceleration((int)(Constants.kElevatorMaxSpeedHighGear * 3.0), 10);
		master.configMotionSCurveStrength(0);

		configuredForAscent = true;
	}

	private void configforDescent(){
		master.configMotionSCurveStrength(4);

		configuredForAscent = false;
	}
	
	public void configForTeleopSpeed(){
		configForAscent();
	}
	
	public void configForAutoSpeed(){
		configForAscent();
	}
	
	public void enableLimits(boolean enable){
		master.overrideSoftLimitsEnable(enable);
		limitsEnabled = enable;
	}
	
	public void setCurrentLimit(int amps){
		for(LazyTalonSRX motor : motors){
			motor.configContinuousCurrentLimit(amps, 10);
			motor.configPeakCurrentLimit(amps, 10);
			motor.configPeakCurrentDuration(10, 10);
			motor.enableCurrentLimit(true);
		}
	}
	
	public void setOpenLoop(double output){
		setState(ControlState.OpenLoop);
		periodicIO.demand = output * manualSpeed;
	}
	
	public boolean isOpenLoop(){
		return getState() == ControlState.OpenLoop;
	}
	
	public synchronized void setTargetHeight(double heightFeet){
		setState(ControlState.Position);
		if(heightFeet > Constants.kElevatorMaxHeight)
			heightFeet = Constants.kElevatorMaxHeight;
		else if(heightFeet < Constants.kElevatorMinHeight)
			heightFeet = Constants.kElevatorMinHeight;
		if(isSensorConnected()){
			if(heightFeet > getHeight()){
				master.selectProfileSlot(0, 0);
				configForAscent();
			}
			else{
				master.selectProfileSlot(1, 0);
				configforDescent();
			}
			targetHeight = heightFeet;
			periodicIO.demand = elevatorHeightToEncUnits(heightFeet);
			System.out.println("Set elevator height to: " + heightFeet);
			onTarget = false;
			startTime = Timer.getFPGATimestamp();
		}else{
			DriverStation.reportError("Elevator encoder not detected!", false);
			stop();
		}
	}
	
	public synchronized void lockHeight(){
		setState(ControlState.Locked);
		if(isSensorConnected()){
			targetHeight = getHeight();
			periodicIO.demand = periodicIO.position;
		}else{
			DriverStation.reportError("Elevator encoder not detected!", false);
			stop();
		}
	}
	
	public Request openLoopRequest(double output){
		return new Request(){
			
			@Override
			public void act(){
				setOpenLoop(output);
			}
			
		};
	}
	
	public Request heightRequest(double height){
		return new Request(){
			
			@Override
			public void act() {
				setTargetHeight(height);
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
				return Util.epsilonEquals(Math.signum(height - getHeight()), above ? -1.0 : 1.0);
			}

		};
	}

	public double getHeight(){
		return encUnitsToElevatorHeight(periodicIO.position);
	}
	
	public double getVelocityFeetPerSecond(){
		return encUnitsToInches(periodicIO.velocity) * 10.0;
	}
	
	boolean onTarget = false;
	double startTime = 0.0;
	public boolean hasReachedTargetHeight(){
		if(master.getControlMode() == ControlMode.MotionMagic){
			if((Math.abs(targetHeight - getHeight()) <= Constants.kElevatorHeightTolerance)){
				if(!onTarget){
					//System.out.println("Elevator done in: " + (Timer.getFPGATimestamp() - startTime)); //TODO uncomment this if desired
					onTarget = true;
				}
				return true;
			}
		}
		return false;
	}
	
	private int inchesToEncUnits(double inches){
		return (int) (inches * Constants.kElevatorTicksPerInch);
	}

	private double encUnitsToInches(double encUnits){
		return encUnits / Constants.kElevatorTicksPerInch;
	}

	private double elevatorHeightToEncUnits(double elevatorHeight){
		return Constants.kElevatorEncoderStartingPosition + inchesToEncUnits(elevatorHeight);
	}

	private double encUnitsToElevatorHeight(double encUnits){
		return encUnitsToInches(encUnits - Constants.kElevatorEncoderStartingPosition);
	}

	public boolean inVisionRange(List<double[]> ranges){
		return inVisionRange(getHeight(), ranges);
	}

	public boolean inVisionRange(double height, List<double[]> ranges){
		boolean inRange = false;
		for(double[] range : ranges){
			inRange |= (height >= range[0]) && (height <= range[1]);
		}
		return inRange;
	}

	public double nearestVisionHeight(List<double[]> ranges){
		return nearestVisionHeight(getHeight(), ranges);
	}

	public double nearestVisionHeight(double height, List<double[]> ranges){
		if(inVisionRange(height, ranges))
			return height;
		double nearestHeight = Constants.kElevatorMaxHeight;
		double smallestDistance = Math.abs(height - nearestHeight);
		for(double[] range : ranges){
			for(int i=0; i<2; i++){
				if(Math.abs(height - range[i]) < smallestDistance){
					smallestDistance = Math.abs(height - range[i]);
					nearestHeight = (i == 0) ? range[i] + 0.5 : range[i] - 0.5;
				}
			}
		}
		if(Util.epsilonEquals(nearestHeight, Constants.kElevatorMaxHeight))
			return height;
		return nearestHeight;
	}
	
	private boolean getMotorsWithHighCurrent(){
		return periodicIO.current >= Constants.kElevatorMaxCurrent;
	}
	
	private final Loop loop  = new Loop(){

		@Override
		public void onStart(double timestamp) {
			
		}

		@Override
		public void onLoop(double timestamp) {
			if(getMotorsWithHighCurrent()){
				DriverStation.reportError("Elevator current too high", false);
				//stop();
			}
		}

		@Override
		public void onStop(double timestamp) {
			
		}
		
	};
	
	public boolean isSensorConnected(){
		int pulseWidthPeriod = master.getSensorCollection().getPulseWidthRiseToRiseUs();
		boolean connected = pulseWidthPeriod != 0;
		if(!connected)
			hasEmergency = true;
		return connected;
	}

	public void resetToAbsolutePosition(){
		int absolutePosition = (int) Util.boundToScope(0, 4096, master.getSensorCollection().getPulseWidthPosition());
		if(encUnitsToElevatorHeight(absolutePosition) > Constants.kElevatorMaxInitialHeight){
			absolutePosition -= 4096;
		}else if(encUnitsToElevatorHeight(absolutePosition) < Constants.kElevatorMinInitialHeight){
			absolutePosition += 4096;
		
		double height = encUnitsToElevatorHeight(absolutePosition);
		if(height > Constants.kElevatorMaxInitialHeight || height < Constants.kElevatorMinInitialHeight){
			DriverStation.reportError("Elevator height is out of bounds", false);
			hasEmergency = true;
		}
		master.setSelectedSensorPosition(absolutePosition, 0, 10);
	}

	@Override
	public synchronized void readPeriodicInputs(){
		periodicIO.position = master.getSelectedSensorPosition(0);
			periodicIO.velocity = master.getSelectedSensorVelocity(0);
			periodicIO.voltage = master.getMotorOutputVoltage();
			periodicIO.current = master.getOutputCurrent();
	}

	@Override
	public synchronized void writePeriodicOutputs(){
		if(getState() == ControlState.Position || getState() == ControlState.Locked)
			master.set(ControlMode.MotionMagic, periodicIO.demand);
		else
			master.set(ControlMode.PercentOutput, periodicIO.demand);
	}

	@Override
	public void stop() {
		setOpenLoop(0.0);
	}

	@Override
	public void zeroSensors() {
		//master.setSelectedSensorPosition(0, 0, 10);
		resetToAbsolutePosition();
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
