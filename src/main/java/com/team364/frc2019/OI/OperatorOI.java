package com.team364.frc2019.OI;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

import com.team364.frc2019.Commands.*;


public class OperatorOI {
    //Driver Controller
    //Xbox One Wired Controller
    public Joystick buttoBoxo;
    //Lift Buttons
    public JoystickButton setLiftPositionLow;
    public JoystickButton setLiftPositionMedium;
    public JoystickButton setLiftPositionHigh;
    public JoystickButton setLiftPositionCargo;
    public JoystickButton setIntakePosition;
    public java.lang.Integer height;
    /**
     * OI()
     * <p>Initializes Joysticks and buttons thereof
     * <p>assigns commands to buttons when pressed or held
     */
    public OperatorOI() {

        //Initialize Operator Controller
        buttoBoxo = new Joystick(1);
        //Set Lift Position to level 1 for scoring in rocket and hatches on cargo ship
        setLiftPositionLow = new JoystickButton(buttoBoxo, 8);
        setLiftPositionLow.whenPressed(new ElevateToPosition(1));
        //Set Lift Position to level 2 for scoring in rocket
        setLiftPositionMedium = new JoystickButton(buttoBoxo, 9);
        setLiftPositionMedium.whenPressed(new ElevateToPosition(2));
        //Set Lift Position to level 3 for scoring in rocket
        setLiftPositionHigh = new JoystickButton(buttoBoxo, 10);
        setLiftPositionHigh.whenPressed(new ElevateToPosition(3));
        //Set Lift Position to level 4 for scoring Cargo in Cargo Ship
        setLiftPositionCargo = new JoystickButton(buttoBoxo, 7);
        setLiftPositionCargo.whenPressed(new ElevateToPosition(4));
        //Set Lift Position to level 0 for intaking
        setIntakePosition = new JoystickButton(buttoBoxo, 3);
        setIntakePosition.whenPressed(new SetIntakePos());
    }
    public Joystick sendInput(){
        return buttoBoxo;
    }

}