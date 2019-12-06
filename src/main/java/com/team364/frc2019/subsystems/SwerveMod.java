package com.team364.frc2019.subsystems;

import static com.team364.frc2019.Conversions.*;
import static com.team364.frc2019.RobotMap.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1323.lib.util.Util;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.team364.frc2019.misc.math.Vector2;


public class SwerveMod extends Subsystem{
    private double lastTargetAngle = 0;
    public final int moduleNumber;

    public final int mZeroOffset;

    private final TalonSRX mAngleMotor;
    private final TalonSRX mDriveMotor;
    private Vector2 modulePosition;
    private boolean driveInverted = false;
    
    public double targetAngle;
    public double targetSpeed;
    public double smartAngle;
    public Vector2 velocity;
    public double currentAngle;

    private PeriodicIO periodicIO = new PeriodicIO();


    public SwerveMod(int moduleNumber, Vector2 modulePosition, TalonSRX angleMotor, TalonSRX driveMotor, boolean invertDrive, boolean invertSensorPhase, int zeroOffset) {
        this.moduleNumber = moduleNumber;
        this.modulePosition = modulePosition;
        mAngleMotor = angleMotor;
        mDriveMotor = driveMotor;
        mZeroOffset = zeroOffset;
        targetAngle = 0;
        targetSpeed = 0;
        currentAngle = 0;

        


        // Configure Angle Motor
        angleMotor.configFactoryDefault();
        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, SLOTIDX, SWERVETIMEOUT);
        angleMotor.selectProfileSlot(SLOTIDX, SWERVETIMEOUT);
        angleMotor.setSensorPhase(invertSensorPhase);
        angleMotor.config_kP(SLOTIDX, ANGLEP, SWERVETIMEOUT);
        angleMotor.config_kI(SLOTIDX, ANGLEI, SWERVETIMEOUT);
        angleMotor.config_kD(SLOTIDX, ANGLED, SWERVETIMEOUT);
        angleMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, 10);
        angleMotor.configMotionAcceleration((int)(kSwerveRotationMaxSpeed*12.5), 10);
    	angleMotor.configMotionCruiseVelocity((int)(kSwerveRotationMaxSpeed), 10);
        angleMotor.set(ControlMode.Position, angleMotor.getSelectedSensorPosition(0));


        //Configure Drive Motor
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setInverted(invertDrive);

        
        // Setup Current Limiting
        angleMotor.configContinuousCurrentLimit(ANGLECONTINUOUSCURRENTLIMIT, SWERVETIMEOUT);
        angleMotor.configPeakCurrentLimit(ANGLEPEAKCURRENT, SWERVETIMEOUT);
        angleMotor.configPeakCurrentDuration(ANGLEPEAKCURRENTDURATION, SWERVETIMEOUT);
        angleMotor.enableCurrentLimit(ANGLEENABLECURRENTLIMIT);

        driveMotor.configContinuousCurrentLimit(DRIVECONTINUOUSCURRENTLIMIT, SWERVETIMEOUT);
        driveMotor.configPeakCurrentLimit(DRIVEPEAKCURRENT, SWERVETIMEOUT);
        driveMotor.configPeakCurrentDuration(DRIVEPEAKCURRENTDURATION, SWERVETIMEOUT);
        driveMotor.enableCurrentLimit(DRIVEENABLECURRENTLIMIT);

    }

    @Override
	public synchronized void stop(){
        setTargetSpeed(0);
    }

    public void setTargetVelocity(Vector2 velocity, boolean speed, double rotation){
            this.velocity = velocity;
            targetAngle = velocity.getAngle().toDegrees();
            smartAngle = targetAngle;
            if(speed){
            targetSpeed = velocity.length;
            }
            else{
            targetSpeed = 0;
            }

    }

    public Vector2 getModulePosition(){
        return modulePosition;
    }


    public synchronized void setTargetAngle(double fTargetAngle) {
        double newAngle = Util.placeInAppropriate0To360Scope(periodicIO.periodicAngle, fTargetAngle + mZeroOffset);
		double setpoint = toCounts(newAngle);
        periodicIO.setPosition(setpoint);
    }

    public synchronized void setTargetSpeed(double fSpeed) {
        periodicIO.setSpeed(fSpeed);
    } 



    @Override
	public synchronized void readPeriodicInputs() {
        periodicIO.periodicAngle = mAngleMotor.getSelectedSensorPosition(0) * (360.0/1024.0);
        //TODO: *CB* need to add periodicSpeed when we use speed encoders
	}

	@Override
	public synchronized void writePeriodicOutputs() {
        mDriveMotor.set(ControlMode.PercentOutput, periodicIO.speedDemand);
        mAngleMotor.set(ControlMode.MotionMagic, periodicIO.positionDemand);
    }

    @Override
	public void outputTelemetry() {
    }

    public double getModuleAngle(){
        return periodicIO.periodicAngle - mZeroOffset;
    }

    public static class PeriodicIO{
		//Inputs
        public double periodicSpeed = 0.0;
        public double periodicAngle = 0.0;
		

		//Outputs
		public double speedDemand;
        public double positionDemand;
        public void setPosition(double outputPosition){
            positionDemand = outputPosition;
        }
        public void setSpeed(double outputSpeed){
            speedDemand = outputSpeed;
        }
	}
	

}
