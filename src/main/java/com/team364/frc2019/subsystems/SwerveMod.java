package com.team364.frc2019.subsystems;

import static com.team364.frc2019.Conversions.*;
import static com.team364.frc2019.RobotMap.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
    
    public TalonSRX getAngleMotor(){
        return mAngleMotor;
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

    public TalonSRX getDriveMotor() {
        return mDriveMotor;
    }

    public double getDrivePos(){
        return mDriveMotor.getSelectedSensorPosition(SLOTIDX);
    }
    
    public double getTargetAngle() {
        return lastTargetAngle;
    }

    public void setDriveInverted(boolean inverted) {
        driveInverted = inverted;
    }

    public synchronized void setTargetAngle(double targetAngle) {
        
        targetAngle = modulate360(targetAngle);
        targetAngle += mZeroOffset;
        double currentAngle = periodicIO.periodicPosition;
        double currentAngleMod = modulate360(currentAngle);
        if (currentAngleMod < 0) currentAngleMod += 360;

        double delta = currentAngleMod - targetAngle;
        if (delta > 180) {
            targetAngle += 360;
        } else if (delta < -180) {
            targetAngle -= 360;
        }

        if(Util.shouldReverse(targetAngle, periodicIO.periodicPosition)){
            periodicIO.setPosition(toCounts(targetAngle + 180.0));
            setDriveInverted(true);
        }else{
            periodicIO.setPosition(toCounts(targetAngle));
            setDriveInverted(false);
        }
        
        /*
        delta = currentAngleMod - targetAngle;
        if (delta > 90 || delta < -90) {
            if(delta > 90){
                targetAngle += 180;
            }
            else if(delta < -90){
                targetAngle -= 180;
            }            
            setDriveInverted(false);

        } else { 
            setDriveInverted(true);
        }
        

        targetAngle += currentAngle - currentAngleMod;
        lastTargetAngle = targetAngle;
        */
        //periodicIO.setPosition(toCounts(targetAngle));
    }

    public synchronized void setTargetSpeed(double speed) {
        if (driveInverted) {speed = -speed;}
        periodicIO.setSpeed(speed);
    } 




    /**
     * @return Current Angle of Module
     */
    public  double getPos(){
        double relativePosition = modulate360(toDegrees(mAngleMotor.getSelectedSensorPosition()));
        if ( relativePosition < 0){ relativePosition += 360;}
        return relativePosition;
    }

    @Override
	public synchronized void readPeriodicInputs() {
        periodicIO.periodicPosition = mAngleMotor.getSelectedSensorPosition(0) * (360.0/1024.0);
        //TODO: *CB* need to add periodicSpeed when we use speed encoders
	}

	@Override
	public synchronized void writePeriodicOutputs() {
        mDriveMotor.set(ControlMode.PercentOutput, periodicIO.speedDemand);
        mAngleMotor.set(ControlMode.Position, periodicIO.positionDemand);
    }

    @Override
	public void outputTelemetry() {
    }


    public static class PeriodicIO{
		//Inputs
        public double periodicSpeed = 0.0;
        public double periodicPosition = 0.0;
		

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
