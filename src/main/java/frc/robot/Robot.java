package frc.robot;

import java.nio.channels.Channel;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
//import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
//import com.revrobotics.WPI_VictorSPX;
//import com.revrobotics.WPI_VictorSPX.MotorType;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.ctr.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
//import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CAN;

//Network tables for limelight 3
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {
    private static final String kDefaultAuto = "Default";
    private static final String goStraightAuto = "goStraightAuto";
    private static final String goLeftAuto = "goLeftAuto";
    private static final String  goRightAuto = "goRightAuto";
    
    
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    
    // variables used within the program
    // private double deadzone = 0.05;
    
    // timer used for autonomous
    //private Timer autoTimer;
    private double startTime;
    private double armSpeed = 0.64;
    private final XboxController driver = new XboxController(0);
    
    // create the VictorSPX motor controllers and assign their ports
    private final DigitalInput limitSwitchUpper = new DigitalInput(8);
    private final DigitalInput limitSwitchLower = new DigitalInput(9);
    private final VictorSPX leftArmMotor = new VictorSPX(25);
    private final VictorSPX rightArmMotor = new VictorSPX(26);
    private final VictorSPX shooterMotor1 = new VictorSPX(27);
    private final VictorSPX shooterMotor2 = new VictorSPX(28);
    private final VictorSPX intakeMotor = new VictorSPX(29);
    private final VictorSPX leftDriveMotor1 = new VictorSPX(21);
    private final VictorSPX leftDriveMotor2 = new VictorSPX(22);
    //leftDriveMotor2.follow(m_left);
    private final VictorSPX rightDriveMotor1 = new VictorSPX(23);
    private final VictorSPX rightDriveMotor2 = new VictorSPX(24);
    private final DifferentialDrive m_robotDrive = new DifferentialDrive((double output) -> {
        leftDriveMotor1.set(ControlMode.PercentOutput,output);
        leftDriveMotor2.set(ControlMode.PercentOutput,output);
    },
    (double output2) -> {
        rightDriveMotor1.set(ControlMode.PercentOutput,-output2);
        rightDriveMotor2.set(ControlMode.PercentOutput,-output2);
    });

    @Override
    public void robotInit() {
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    //    m_chooser.addOption("Go middle", goStraightAuto);
        m_chooser.addOption("Go Left", goLeftAuto);
        m_chooser.addOption("Go Right", goRightAuto);

        SmartDashboard.putData("Auto choices", m_chooser);
       
        CameraServer.startAutomaticCapture("cam1",0);  
    }

    @Override
    public void robotPeriodic() {
    }


    //Initial startup for autonomous
    @Override
    public void autonomousInit() {
        m_autoSelected = m_chooser.getSelected();
        //m_autoSelected = SmartDashboard.getString("Auto Selector",
        // kDefaultAuto);
        System.out.println("Auto selected: " + m_autoSelected);
        // Reset the autonomous timer when auto starts

        //autoTimer.reset();
        startTime = Timer.getFPGATimestamp();

        
    }

    @Override
    public void autonomousPeriodic() {
        double time = Timer.getFPGATimestamp();
      
        switch (m_autoSelected) {
        case goStraightAuto:
        break;

        case goLeftAuto:
            
        break;
      
        case goRightAuto:
             
        break;

        default:
            System.out.println("Auto does not exist");
            break;
        }
    }


    @Override
    public void teleopInit() {
    }

    /* (non-Javadoc)
     * @see edu.wpi.first.wpilibj.IterativeRobotBase#teleopPeriodic()
     */
    // bentroll is used for inverting controls (Dan)
    int bentroll=-1;
    double drive_speed=1;
    int car=0;
    @Override
    public void teleopPeriodic() {
        
        // Invert Controls when a button is pressed
        if(driver.getXButtonPressed()) {
            bentroll*=-1;
        }
        //Set Drive Speed to 100%
        if(driver.getBButtonPressed()) {
            drive_speed=1;
        }

        //Set Drive Speed to 50%
        if(driver.getYButtonPressed()) {
            drive_speed=.5;
        }

        SmartDashboard.putNumber("car", car);
        SmartDashboard.putNumber("inverted?", bentroll);
        SmartDashboard.putNumber("drive_speed", drive_speed);

        // New driving method is being used, same concept but
        // Except using getLeftX & getLeftY from the Joysticks
        double xAxisScaled;
        double yAxisScaled;
        // getLeftX referring to the left joysticks left-right motion, from 1 to -1
        // This if statement checks if the joystick is left (negative) or right (positive)
        // If the joystick is left (negative) (or less than zero same thing) it squares the xbox contollers value (for smooth motion)
        // But if the joystick is right (positive) it not only squares the value, but it multiplies it by negative one
        // This is because when you square it, the value from 1 to -1 (left-right), will always be positive, so it
        // doesn't matter if you move the joystick left or right, you will always go one direction.
        // So if the joystick is right, it will multiply the value by -1 to make the robot move the other direction.
        // This is the same for driver.getLeftY, but except on the up-down axis.
        if (driver.getLeftX() < 0) {
            xAxisScaled = (driver.getLeftX()*driver.getLeftX());
        }
        else {
            xAxisScaled = -1*(driver.getLeftX()*driver.getLeftX());
        }
        
        if (driver.getLeftY() < 0) {
            yAxisScaled = -drive_speed*bentroll*(driver.getLeftY()*driver.getLeftY());
        }
        else {
            yAxisScaled = drive_speed*bentroll*(driver.getLeftY()*driver.getLeftY());
        }    
        m_robotDrive.arcadeDrive(yAxisScaled, xAxisScaled);
     // try this instead, squared inputs may smooth the controls somewhat
     //   m_robotDrive.arcadeDrive(driver.getRawAxis(0)*driver.getRawAxis(0),driver.getRawAxis(1)*driver.getRawAxis(1));
      //now for the flapper (wrong '20s')
      //we would use armspeed to determine how fast the flapper works
        // we will use armMotor.set() to give speed. - is in?
                
        if (driver.getRightBumper()) {
            //intake on
            intakeMotor.set(ControlMode.PercentOutput,-1); 
        }
        else {
            if (driver.getLeftBumper()) {
            //intake reverse
                intakeMotor.set(ControlMode.PercentOutput,1); }
            else {
                intakeMotor.set(ControlMode.PercentOutput,0.0);
            } 
        }               
        if (driver.getRightTriggerAxis()==1){
            //shooter on
            shooterMotor1.set(ControlMode.PercentOutput,1.0);
            shooterMotor2.set(ControlMode.PercentOutput,1.0); }
        else {
            shooterMotor1.set(ControlMode.PercentOutput,0.0);
            shooterMotor2.set(ControlMode.PercentOutput,0.0);}

        //the arm control,

        //Set ben to 1 to see if the code works
        //When you press A the arm will lift until the "limitSwitchUpper" is triggered (This is temporary, for firing presets)
        if((driver.getAButtonPressed()) && (limitSwitchUpper.get())) {
            car = 1;
            leftArmMotor.set(ControlMode.PercentOutput,1.0);
            rightArmMotor.set(ControlMode.PercentOutput,1.0);
        }

        if((driver.getRawAxis(5) < -0.1) && (limitSwitchUpper.get())) {
            leftArmMotor.set(ControlMode.PercentOutput,(armSpeed*driver.getRawAxis(5)));
            rightArmMotor.set(ControlMode.PercentOutput,(armSpeed*driver.getRawAxis(5)));
        } else {
            if((driver.getRawAxis(5) > 0.1) && (limitSwitchLower.get())) {
                leftArmMotor.set(ControlMode.PercentOutput,(armSpeed*driver.getRawAxis(5)));
                rightArmMotor.set(ControlMode.PercentOutput,(armSpeed*driver.getRawAxis(5)));
            }
            else {
                leftArmMotor.set(ControlMode.PercentOutput,0.0);
                rightArmMotor.set(ControlMode.PercentOutput,0.0);
            }
        }
    }
    


   //     armMotor.set(ControlMode.PercentOutput,armSpeed);


public void goForward(double inVal){
   
}
public void goBackward(double inVal){
   
}
public void goLeft(double inVal){
   
}
public void goRight(double inVal){
   
}
public void goStop(double inVal){   
}
public void goTimer(int inVal){
    int i=0;
    while (i<(100*inVal)) {
    i=i+1;}
}



    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

        

        
        
    }
}