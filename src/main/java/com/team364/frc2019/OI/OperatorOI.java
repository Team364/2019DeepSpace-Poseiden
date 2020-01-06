package com.team364.frc2019.OI;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

import com.team364.frc2019.States;
import com.team364.frc2019.Commands.*;


public class OperatorOI {
    //Driver Controller
    //Xbox One Wired Controller
    public Joystick buttoBoxo;
    //Lift Buttons
    public boolean setLiftPositionLow;
    public boolean setLiftPositionMedium;
    public boolean setLiftPositionHigh;
    public boolean setLiftPositionCargo;
    public boolean setIntakePosition;
    public java.lang.Integer height;
    /**
     * OI()
     * <p>Initializes Joysticks and buttons thereof
     * <p>assigns commands to buttons when pressed or held
     */
    public OperatorOI() {
        updateControl();
    }
    public void updateControl(){

        //Initialize Operator Controller
        buttoBoxo = new Joystick(1);
        //Set Lift Position to level 1 for scoring in rocket and hatches on cargo ship
        setLiftPositionLow = buttoBoxo.getRawButton(1);
        //Set Lift Position to level 2 for scoring in rocket
        setLiftPositionMedium = buttoBoxo.getRawButton(9);
        //Set Lift Position to level 3 for scoring in rocket
        setLiftPositionHigh = buttoBoxo.getRawButton(10);
        //Set Lift Position to level 4 for scoring Cargo in Cargo Ship
        setLiftPositionCargo = buttoBoxo.getRawButton(7);
        //Set Lift Position to level 0 for intaking
        setIntakePosition = buttoBoxo.getRawButton(3);
        
        //State Buttons
        if(buttoBoxo.getRawButton(1)){
            States.objState = States.ObjectStates.CARGO_OBJ;
        }else if(buttoBoxo.getRawButton(2)){
            States.objState = States.ObjectStates.HATCH_OBJ;
        }
    }
    public Joystick sendInput(){
        return buttoBoxo;
    }

}