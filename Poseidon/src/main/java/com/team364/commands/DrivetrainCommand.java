package com.team364.commands;

import edu.wpi.first.wpilibj.command.Command;
import com.team364.Poseidon;
import com.team364.subsystems.Swerve;
import com.team364.misc.math.Vector2;
import com.team1323.loops.Loop;

import static com.team364.RobotMap.*;

public class DrivetrainCommand {

	private Swerve mSwerve;
	public int cycles;

	double forward;
	double strafe;
	double rotation;
	private Vector2 translation;

	Vector2 lastTranslation;
	double lastRotation;
	double plzStop = 0;

	public DrivetrainCommand(Swerve swerve){
		mSwerve = swerve;
		cycles = 0;
	}
	private final Loop loop = new Loop(){

		@Override
		public void onStart(double timestamp) {
			synchronized(DrivetrainCommand.this){
				mSwerve.stopDriveMotors();
			}
		}

		@Override
		public void onLoop(double timestamp) {
			synchronized(DrivetrainCommand.this){
				forward = -Poseidon.oi.controller.getRawAxis(1);
				strafe = Poseidon.oi.controller.getRawAxis(0);
				rotation = Poseidon.oi.controller.getRawAxis(4);
				boolean zeroPoint = false;
				if(zeroPoint){
					translation = new Vector2(-1, 0);
				}
				else{
					translation = new Vector2(forward, strafe);
				}
				if (Math.abs(forward) > STICKDEADBAND || Math.abs(strafe) > STICKDEADBAND || Math.abs(rotation) > STICKDEADBAND) {
					mSwerve.holonomicDrive(translation, rotation, !zeroPoint);
					lastTranslation = translation;
					lastRotation = rotation;
					cycles++;

				} else {
					if(cycles != 0){

						mSwerve.holonomicDrive(lastTranslation, lastRotation, false);
					}
				}	
				if(cycles != 0){
				mSwerve.updateKinematics();
				}
			}
		}

		@Override
		public void onStop(double timestamp) {
			synchronized(DrivetrainCommand.this){
				mSwerve.stopDriveMotors();
			}
		}
		
	};
}
