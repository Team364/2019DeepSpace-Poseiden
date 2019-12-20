package com.team364.frc2019.subsystems;

import static com.team364.frc2019.Conversions.*;
import static com.team364.frc2019.RobotMap.*;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team1323.lib.util.Util;
import com.team1323.loops.ILooper;
import com.team1323.loops.Loop;

import edu.wpi.first.networktables.NetworkTableInstance;

import com.team364.frc2019.OI.DriverOI;
import com.team364.frc2019.misc.math.Vector2;

public class Swerve extends Subsystem {

    private static Swerve Instance = null;
	/*
	 * 0 is Front Right
	 * 1 is Front Left
	 * 2 is Back Left
	 * 3 is Back Right
	 */
    private SwerveMod[] mSwerveModules;
    private List<SwerveMod> modules;

    public Swerve() {
            mSwerveModules = new SwerveMod[] {
                    new SwerveMod(0,
                            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                            new TalonSRX(FRANGLE),
                            new TalonSRX(FRDRIVE),
                            MOD0DRIVEINVERT, 
                            false,
                            MOD0OFFSET),
                    new SwerveMod(1,
                            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                            new TalonSRX(FLANGLE),
                            new TalonSRX(FLDRIVE),
                            MOD1DRIVEINVERT,
                            false,
                            MOD1OFFSET),
                    new SwerveMod(2,
                            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                            new TalonSRX(BLANGLE),
                            new TalonSRX(BLDRIVE),
                            MOD2DRIVEINVERT,
                            false,
                            MOD2OFFSET),
                    new SwerveMod(3,
                            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                            new TalonSRX(BRANGLE),
                            new TalonSRX(BRDRIVE),
                            MOD3DRIVEINVERT,
                            false,
                            MOD3OFFSET)
            };
         
            modules = Arrays.asList(mSwerveModules[0], mSwerveModules[1], mSwerveModules[2], mSwerveModules[3]);

    } 
    public String getName(){
        return "Swerve";
    }
    public synchronized static Swerve getInstance() {
        if (Instance == null) {
          Instance = new Swerve();
        }
        return Instance;
      }


    public SwerveMod getSwerveModule(int i) {
        return mSwerveModules[i];
    }

    public void holonomicDrive(Vector2 translation, double rotation, boolean speed) {
            Vector2 velocity;
            for(SwerveMod mod : getSwerveModules()){
                Vector2 newTranslation = null;
                //newTranslation = translation.rotateBy(Rotation2.fromDegrees(getGyro()));
                newTranslation = translation;
                velocity = mod.getModulePosition().normal().scale(deadband(rotation)).add(newTranslation);
                mod.setTargetVelocity(velocity, speed, rotation);
            }        
    }
    public void updateKinematics(){
        for (SwerveMod mod : getSwerveModules()){
            if(Util.shouldReverse(mod.targetAngle, mod.getModuleAngle())){
                mod.setTargetAngle(mod.targetAngle += 180);
                mod.setTargetSpeed(mod.targetSpeed * -1);
            }else{
                mod.setTargetAngle(mod.targetAngle);
                mod.setTargetSpeed(mod.targetSpeed);
            }
        }
    }


    public SwerveMod[] getSwerveModules() {
        return mSwerveModules;
    }

    public double closestGyroSetPoint(double[] gyroSet){
        double checkPoint = 0;
        double returnSetPoint = 0;
        for(Double setPoint : gyroSet){
            double initial = setPoint /*- .getFittedYaw()*/;
            if(Math.abs(initial) < Math.abs(checkPoint) || checkPoint == 0) {
                checkPoint = initial;
                returnSetPoint = setPoint;
            }
        }
        return returnSetPoint;
    }

    public void setTrackingMode(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); //Turns LED off
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); //Begin Processing Vision
    }

    public void setDriverCamMode(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); //Turns LED off
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1); //Disable Vision Processing
    }
    
    @Override
	public synchronized void stop() {
        modules.forEach((m) -> m.stop());

    }
    @Override
	public synchronized void outputTelemetry() {
        modules.forEach((m) -> m.outputTelemetry());
    }
    @Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(loop);
	}


    //-------------------------------------LOOP--------------------------------------------
    
	public int cycles = 0;

	double forward;
	double strafe;
	double rotation;
	private Vector2 translation;

	Vector2 lastTranslation;
    double lastRotation;
    public void sendInput(double forward, double strafe, double rotation){
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;
    }
    public synchronized void updateControlCycle(){
        boolean zeroPoint = false;
        if(zeroPoint){
            translation = new Vector2(-1, 0);
        }
        else{
            translation = new Vector2(forward, strafe);
        }
        if (Math.abs(forward) > STICKDEADBAND || Math.abs(strafe) > STICKDEADBAND || Math.abs(rotation) > STICKDEADBAND) {
            holonomicDrive(translation, rotation, !zeroPoint);
            lastTranslation = translation;
            lastRotation = rotation;
            cycles++;

        } else {
            if(cycles != 0){
                holonomicDrive(lastTranslation, lastRotation, false);
            }
        }	
        if(cycles != 0){
        updateKinematics();
        }
    }
    public Loop getLoop(){
        return loop;
    }
	private final Loop loop = new Loop(){

		@Override
		public void onStart(double timestamp) {
			synchronized(Swerve.this){
				stop();
			}
		}

		@Override
		public void onLoop(double timestamp) {
			synchronized(Swerve.this){
				updateControlCycle();
			}
        }

		@Override
		public void onStop(double timestamp) {
			synchronized(Swerve.this){
				stop();
			}
		}
		
    };
    
	@Override
	public synchronized void readPeriodicInputs() {
        modules.forEach((m) -> m.readPeriodicInputs());
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		modules.forEach((m) -> m.writePeriodicOutputs());
    }
    //--------------------------------------SIGNAL----------------------------------
    
}
