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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.util.Arrays;

import com.team1323.lib.util.CrashTracker;
import com.team1323.loops.Looper;
import com.team364.frc2019.OI.*;
import com.team364.frc2019.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Poseidon extends TimedRobot {
  public static DriverOI oi;

  private Swerve swerve;
  private Superstructure s;
	private SubsystemManager subsystems;

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
    oi = new DriverOI();
    s = Superstructure.getInstance();
    swerve = Swerve.getInstance();
    subsystems = new SubsystemManager(
      Arrays.asList(swerve));

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
      //driver.update();
      subsytems.signal.sendInput(-oi.controller.getRawAxis(1), oi.controller.getRawAxis(0), oi.controller.getRawAxis(4));
      //close signal if done
      if(subsystems.signal.returnSignal == true){
        subsystem.signal.close();
      }
      subsystems.outputToSmartDashboard();
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
}
