package org.usfirst.frc.team3238.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.IterativeRobot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * The main WPI provided class, iterative and holding the configuration of subsystems and attempting to integrate them effectively.
 * 
 * @author FRC Team 3238
 * 
 * @version 1.0
 */ 
public class Robot extends IterativeRobot
{ //Lots of configuration, possible to use static classes instead for most of these.
    Breacher breacherArm;
 
    Chassis chassis;
    Collector collector;
    Shooter shooter;
    Joystick assistantJoystick, mainJoystick, launchPad;
    WPI_TalonSRX leftDriveTalonA, leftDriveTalonB, rightDriveTalonA,
            rightDriveTalonB;
    WPI_TalonSRX breacherTalon;
    WPI_TalonSRX collectorTalon;
    WPI_TalonSRX shooterTalonLeft, shooterTalonRight;
    DigitalInput ballDetectSwitch;
    Timer timer, tim;
    NetworkTable netTab; //a way of transmitting values
    public static boolean camChanging;
    public static boolean camDead;

    public void robotInit()
    {
        try
        {
            assistantJoystick = new Joystick(
                    Constants.Joysticks.joystickZeroPort);
            mainJoystick = new Joystick(Constants.Joysticks.joystickOnePort);
            launchPad = new Joystick(Constants.Joysticks.launchpadPort);

            ballDetectSwitch = new DigitalInput(
                    Constants.Collector.ballDetectPort);

            leftDriveTalonA = new WPI_TalonSRX(Constants.Chassis.leftMotorOneID);
            leftDriveTalonB = new WPI_TalonSRX(Constants.Chassis.leftMotorTwoID);
            rightDriveTalonA = new WPI_TalonSRX(Constants.Chassis.rightMotorOneID);
            rightDriveTalonB = new WPI_TalonSRX(Constants.Chassis.rightMotorTwoID);
            rightDriveTalonA.setInverted(true);
            rightDriveTalonB.setInverted(true);
            breacherTalon = new WPI_TalonSRX(Constants.Breacher.breacherTalonID);
            collectorTalon = new WPI_TalonSRX(Constants.Collector.collectorTalonID);
            shooterTalonLeft = new WPI_TalonSRX(
                    Constants.Shooter.shooterLeftTalonID);
            shooterTalonRight = new WPI_TalonSRX(
                    Constants.Shooter.shooterRightTalonID);
            breacherArm = new Breacher(breacherTalon);
            netTab = NetworkTable.getTable("GRIP");
            netTab.putBoolean("run", true); //turns vision processing on, although does not work in autonomous
           
            chassis = new Chassis(leftDriveTalonA, leftDriveTalonB,
                    rightDriveTalonA, rightDriveTalonB);
            shooter = new Shooter(shooterTalonLeft, shooterTalonRight,
                    mainJoystick, assistantJoystick, launchPad);
            collector = new Collector(collectorTalon, ballDetectSwitch,
                    shooter, mainJoystick, assistantJoystick, launchPad);
            
            timer = new Timer();
        } catch(Exception e)
        {
            DriverStation.reportError(e.getMessage(), true);
        }
    }

    public void autonomousInit()
    {
     
        shooter.reset();
        netTab.putBoolean("run", true); //vision processing
    }

    public void autonomousPeriodic()
    {
        // auto.switchingAuto();
            netTab.putBoolean("run", true); //vision processing
    }

    public void teleopInit()
    {
		collector.setStick(mainJoystick);

        collector.init(); //allow driver to use the collector, set to automatic
        shooter.reset();
        SmartDashboard.putNumber("DB/Slider 0", 0); //reset the DB/Slider that is preset before matches to determine autonomous routine
        netTab.putBoolean("run", false); //turn off vision processing
    }
boolean override = false;
    public void teleopPeriodic()
    {
    	if(Math.abs(assistantJoystick.getY()) > 0.15 || Math.abs(assistantJoystick.getTwist()) > 0.15) {
			collector.setStick(assistantJoystick);

    		if(!override)collector.init();
    		override = true;
    	} else if(assistantJoystick.getRawButton(11)) {
			collector.setStick(mainJoystick);

    		if(override) collector.init();
    		override = false;
    		
    	}
		DriverStation.reportWarning("CollectorState: " + collector.state, false);

    	if(!override) {
    		breacherArm.run(mainJoystick);
    		if(shooter.autoShoot(mainJoystick.getRawButton(1),mainJoystick.getRawButton(3))) 
    			collector.shoot();
    			chassis.setMotorInversion(mainJoystick);
    			chassis.arcadeDrive(mainJoystick);
    			collector.idle();
    		
    	} else {
    		breacherArm.run(assistantJoystick);
    		if(shooter.autoShoot(assistantJoystick.getRawButton(1), assistantJoystick.getRawButton(3))) 
    			collector.shoot();
    			chassis.setMotorInversion(assistantJoystick);
    			chassis.arcadeDrive(assistantJoystick);
    			collector.idle();
    		
    	}
        
    }
   public void checkOverride() {
   }

    // Drive system
    

    public void disabledPeriodic()
    {
        
        chassis.setPower(0.0); //disable everything else
        breacherArm.moveArm(0.0);
      
        netTab.putBoolean("run", false);
    }

    public void testPeriodic()
    {
      
        netTab.putBoolean("run", false);
    }

}
