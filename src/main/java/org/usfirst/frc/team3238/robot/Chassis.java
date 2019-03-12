package org.usfirst.frc.team3238.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;

/**
 * Constructs a chassis object to control the drivetrain and chassis movement
 * 
 * @author FRC Team 3238
 * 
 * @version 1.0
 */ 
public class Chassis
{
    RobotDrive driveTrain;

    double mainX, mainY, mainTwist;

    double speedMult, turnMult, twistMult;

    int flip;

    WPI_TalonSRX leftMotorControllerA, rightMotorControllerA, leftMotorControllerB,
            rightMotorControllerB;

    /**
     * Constructs a chassis object with all the drive motors that are required 
     */
    Chassis(WPI_TalonSRX leftMotorControllerA, WPI_TalonSRX leftMotorControllerB,
            WPI_TalonSRX rightMotorControllerA, WPI_TalonSRX rightMotorControllerB)
    {
        this.leftMotorControllerA = leftMotorControllerA;
        this.rightMotorControllerA = rightMotorControllerA;
        this.leftMotorControllerB = leftMotorControllerB;
        this.rightMotorControllerB = rightMotorControllerB;
        invertMotors(false);
        driveTrain = new RobotDrive(leftMotorControllerA, leftMotorControllerB,
                rightMotorControllerA, rightMotorControllerB);
        speedMult = Constants.Chassis.yMultiplier;
        turnMult = Constants.Chassis.twistMultiplier;
    }

    /**
     * Sets the class values for easier access from the maindriver's joystick
     * 
     * @param mainDriver - the main joystick
     */ 
    void setJoystickData(Joystick mainDriver)
    {
        mainX = mainDriver.getX();
        mainY = mainDriver.getY();
        mainTwist = mainDriver.getTwist();
        setMotorInversion(mainDriver);
    }
    /**
     * Moves the chassis based on joystick input in arcadedrive fashion
     * 
     * @param mainDriver - the main joystick
     */ 
    void idle(Joystick mainDriver)
    {
        setJoystickData(mainDriver);
        setMotorInversion(mainDriver);
        arcadeDrive();
    }

    /**
     * Changes chassis motor speed
     * 
     * @param power : the power of the drive motors
     */
    void setPower(double power)
    {
        rightMotorControllerA.set(-power);
        rightMotorControllerB.set(-power);
        leftMotorControllerA.set(power);
        leftMotorControllerB.set(power);

    }
    /**
     * The main driver uses the y axis of the joystick to determine where he's moving, so this inverts the axis values incase the
     * driver wants to move the back or front of the robot forward by pushing the joystick forward
     * 
     * @param mainDriver the main joystick, used for the throttle value
     */ 
    void setMotorInversion(Joystick mainDriver)
    {
        if(mainDriver.getThrottle() > 0.0)
        {
            flip = 1;
        } else
        {
            flip = -1;
        }
    }
    /**
     * A drive system where the robot can move based on one joystick, y axis for speed and x axis for rotating
     */ 
    void arcadeDrive()
    {
        double mappedTwist = (mainTwist * turnMult); //Don't want to overshoot on turns
        double mappedY = mainY * speedMult; //Don't want to drive exceedingly fast
        driveTrain.arcadeDrive(mappedY, mappedTwist, true); //the built in function provided by WPI for arcadeDrive
    }
    /**
     * Calles arcade drive except uses the arduino attached to our driver station in order to change the turn multiplier so that 
     * the drivers can actively change how much they want to turn
     * 
     * @param mainDriver : the main joystick
     */
    void arcadeDrive(Joystick mainDriver)
    {
    
        
       double mappedTwist = -mainDriver.getTwist() * turnMult;
        double mappedY = -mainDriver.getY() * speedMult;
        driveTrain.arcadeDrive(mappedY, mappedTwist, true);
    }
    /**
     * The functionality of arcadedrive but absolute instead of algorithmic input values in order to have effective autonomous
     * @param y : speed of chassis 
     * @param twist : turn speed
     */ 
    void arcadeDriveAuto(double y, double twist)
    {
        driveTrain.arcadeDrive(y, twist, false); //last param squares inputs if true
    }
    /**
     * changes motor inversion
     * @param inv : chooses whether to invert motors or not
     */ 
    void invertMotors(boolean inv)
    {
        this.leftMotorControllerA.setInverted(inv);
        this.leftMotorControllerB.setInverted(inv);
        this.rightMotorControllerA.setInverted(inv);
        this.rightMotorControllerB.setInverted(inv);
    }
    /**
     * turns off drive motors
     */ 
    public void disable()
    {
        this.leftMotorControllerA.set(0);
        this.leftMotorControllerB.set(0);
        this.rightMotorControllerA.set(0);
        this.rightMotorControllerB.set(0);
    }
}
