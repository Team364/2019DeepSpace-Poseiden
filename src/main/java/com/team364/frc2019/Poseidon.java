/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team364.frc2019;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.util.Arrays;

import com.team1323.lib.util.CrashTracker;
import com.team1323.loops.Looper;
import com.team364.frc2019.OI.*;
import com.team364.frc2019.subsystems.*;
import static com.team364.frc2019.RobotMap.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Poseidon extends TimedRobot {
  public static DriverOI driverOi;
  public static OperatorOI operatorOi;

  public static Superstructure s;
  public static Swerve swerve;
  public static Elevator elevator;

	public static SubsystemManager subsystems;

	private Looper enabledLooper = new Looper();
	private Looper disabledLooper = new Looper();


  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    driverOi = new DriverOI();
    operatorOi = new OperatorOI();
    s = new Superstructure();
    swerve = Swerve.getInstance();
    elevator = Elevator.getInstance();
    subsystems = new SubsystemManager(
      Arrays.asList(s, swerve, elevator));

    subsystems.registerEnabledLoops(enabledLooper);
    subsystems.registerDisabledLoops(disabledLooper);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    operatorOi.updateControl();
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }


  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
		try {
			disabledLooper.stop();
      enabledLooper.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
  }

  @Override
  public void teleopPeriodic() {
    try {
      subsystems.outputToSmartDashboard();
      swerve.sendInput(-driverOi.controller.getRawAxis(1), driverOi.controller.getRawAxis(0), driverOi.controller.getRawAxis(4));
      standardControl();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
  }

  
	@Override
	public void disabledInit() {
		try {
			enabledLooper.stop();
			subsystems.stop();
			disabledLooper.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

 
  @Override
  public void testPeriodic() {
  }

  public void standardControl(){
    frontCam = RobotMap.fCam;
    if (States.objState == States.ObjectStates.HATCH_OBJ) {
        low = RobotMap.liftLowH;
        med = RobotMap.liftMedH;
        high = RobotMap.liftHighH;
        cargo = low;
        intake = low;

        perpendicularToGround = RobotMap.armPerpindicularToGround;
        scoreOnHigh = perpendicularToGround;
        intakeCargo = 3300;

        lvlone = perpendicularToGround;
        lvlthree = perpendicularToGround;

        l1cam = RobotMap.l1Hcam;
        l2cam = RobotMap.l2Hcam;
        l3cam = RobotMap.l3Hcam;
        intakeCam = RobotMap.l1Ccam;
    } else if (States.objState == States.ObjectStates.CARGO_OBJ) {
        intake = RobotMap.liftIntake;
        low = RobotMap.liftLowC;
        med = RobotMap.liftMedC;
        high = RobotMap.liftHighC;
        cargo = RobotMap.liftCargoC;

        intakeCargo = RobotMap.armIntakeCargo;
        perpendicularToGround = RobotMap.armPerpindicularToGround;
        scoreOnHigh = perpendicularToGround;
        custom = 5000;
        lvlone = 3300;
        lvlthree = 2100;

        l1cam = RobotMap.l1Ccam;
        l2cam = RobotMap.l2Ccam;
        l3cam = RobotMap.l3Ccam;
        intakeCam = RobotMap.l1Ccam;

    }

    if (desiredHeight == 0) {
        elevator.setPlayCruiseVelocity();
        wantedPosition = intake;
        wantedAngle = intakeCargo;
        camera = l1cam;
        States.led = States.LEDstates.INTAKE_MODE;
    } else if (operatorOi.setLiftPositionLow) {
        elevator.setPlayCruiseVelocity();
        wantedPosition = low;
        wantedAngle = lvlone;
        camera = l1cam;
    } else if (operatorOi.setLiftPositionMedium) {
        elevator.setPlayCruiseVelocity();
        wantedPosition = med;
        wantedAngle = perpendicularToGround;
        camera = l2cam;
    } else if (operatorOi.setLiftPositionHigh) {
        elevator.setPlayCruiseVelocity();
        wantedPosition = high;
        wantedAngle = lvlthree;
        camera = l3cam;
    } else if (operatorOi.setLiftPositionCargo) {
        elevator.setPlayCruiseVelocity();
        wantedPosition = cargo;
        wantedAngle = 4200;
        camera = frontCam;
    } else if (desiredHeight == 5) {
        elevator.setPlayCruiseVelocity();
        wantedPosition = liftStartConfig;
        wantedAngle = 300;
        camera = frontCam;
        if(States.objState == States.ObjectStates.CARGO_OBJ){
            States.led = States.LEDstates.HAS_OBJ;
        }else if(States.objState == States.ObjectStates.HATCH_OBJ){
            States.led = States.LEDstates.PASSIVE;
        }   
    }
    if (wantedPosition > 132000) {
        wantedPosition = 132000;
    }
    s.SimpleElevatorState(wantedPosition);
    //elevator.elevateTo(wantedPosition, wantedAngle);
    //elevator.setCamera(camera);

  }
}
