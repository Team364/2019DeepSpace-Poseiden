package com.team364.frc2019.OI;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;


public class DriverOI {
    //Driver Controller
    //Xbox One Wired Controller
    public Joystick controller;

    /**
     * OI()
     * <p>Initializes Joysticks and buttons thereof
     * <p>assigns commands to buttons when pressed or held
     */
    public DriverOI() {
        controller = new Joystick(0);
    }
    public Joystick sendInput(){
        return controller;
    }

}
