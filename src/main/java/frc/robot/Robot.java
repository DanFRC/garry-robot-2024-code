package frc.robot;

import java.nio.channels.Channel;

import javax.print.attribute.standard.MediaSize.Other;

import org.photonvision.PhotonCamera;

//test
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
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CAN;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;





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
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(7);
    private NetworkTable limelightTable;
    
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

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("Shuffleboard");
        NetworkTableEntry textEntry = table.getEntry("Text");
        
        textEntry.setString("ben");
    }

    @Override
    public void robotPeriodic() {

        if (unidegrees != 0) {
        diff = encoder.get() - unidegrees;
        }

        
        if (start == 1 & diff > 0 & upodwn == 0) {
            upodwn = 1;
            dir = -1;
        } else if (start == 1 & diff < 0 & upodwn == 0) {
            upodwn = -1;
            dir = 1;
        }
        
        differenceval = Math.abs(diff);
    
        if (dir != 0) {
            raising = 1;
        }
        else {
            raising = 0;
        }

        if (upodwn == 1) {
                if (!limitSwitchLower.get() || diff < 0) {

                leftArmMotor.set(ControlMode.PercentOutput, 0.0);
                rightArmMotor.set(ControlMode.PercentOutput, 0.0);
                start = 0;
                unidegrees = 0;
                dir = 0;
                differenceval = 0;
                upodwn = 0;
                return;
            }
        }
        else if (upodwn == -1) {
                if (!limitSwitchUpper.get() || diff > 0) {

                leftArmMotor.set(ControlMode.PercentOutput, 0.0);
                rightArmMotor.set(ControlMode.PercentOutput, 0.0);
                start = 0;
                unidegrees = 0;
                dir = 0;
                differenceval = 0;
                upodwn = 0;
                return;
            }
        }

        if (upodwn != 0) { 
            if (!limitSwitchLower.get()) {
                leftArmMotor.set(ControlMode.PercentOutput, 0.0);
                rightArmMotor.set(ControlMode.PercentOutput, 0.0);

            }
            else if (!limitSwitchUpper.get()) {
                leftArmMotor.set(ControlMode.PercentOutput, 0.0);
                rightArmMotor.set(ControlMode.PercentOutput, 0.0);
                
            }
        }
        

        // Calculate the speed based on half of armSpeed
        double speed = armSpeed * 1;
        if (dir != 0) {
        leftArmMotor.set(ControlMode.PercentOutput, speed * -dir);
        rightArmMotor.set(ControlMode.PercentOutput, speed * -dir);
        }

        if (moving == 1) {
            m_robotDrive.arcadeDrive(rb_speed, rb_turn);
        }
        else if (moving == 0 & autoEnabled == 1) {
            m_robotDrive.arcadeDrive(0.0, 0.0);
        }
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

        PortForwarder.add(5800, "photonvision.local", 5800);
        

        
    }

    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putNumber("moving?", moving);
        SmartDashboard.putNumber("speed", rb_speed);
        SmartDashboard.putNumber("speed", rb_turn);
        double time = Timer.getFPGATimestamp();
        
      
        switch (m_autoSelected) {
        case goStraightAuto:


            
        break;



        case goLeftAuto:
            
        break;
      
        case goRightAuto:
             
        break;

        default:
                    //Autonomous Move Forward
                    if (doOnce == 0) {
                        doOnce = 1;
                        autoEnabled = 1;
                        new Thread(() ->{
                            Timer.delay(0.3);
                            AUTOraiseArmandShoot(0.185);
                            autoEnabled = 0;
                        }).start();
                    }

 

            break;
        }
    }

    private void drive_to(double seconds, double mov_turn, double mov_speed) {
        rb_speed = mov_speed;
        rb_turn = mov_turn;
        moving = 1;
        
        new Thread(() -> {
            Timer.delay(seconds);
            moving = 0;
            rb_speed = 0;
            rb_turn = 0;
        }).start();
    }


    @Override
    public void teleopInit() {
    }


    private static void vribrate() {
        //hello
    }

    /* (non-Javadoc)
     * @see edu.wpi.first.wpilibj.IterativeRobotBase#teleopPeriodic()
     */
    // bentroll is used for inverting controls (Dan)

    int bentroll=-1;
    double hello = 1;
    double drive_speed=1;
    int car=0;
    int pov = -1;
    int rumbling = 0;
    int intakeOn = 0;
    int raising = 0;
    int db = 0;
    double unidegrees = 0;
    double differenceval = 0;
    double diff = 0;
    int stop = 1;
    int start = 0;
    int arming = 1;
    int dir = 0;
    int upodwn = 0;
    int doOnce = 0;
    double shootNow = 0;
    double waittime = 0;
    int driving = 0;
    //Moving Forward Function
    int moving = 0;
    double rb_speed = 0;
    double rb_turn = 0;
    int autoEnabled = 0;
    //For AUTOshooting
    int shooterDB = 0;

    //POV Variables:
    int povup = 0;
    int povdown = 180;
    int povleft = 270;
    int povright = 90;
    int currentPOV = 0;
    double test = 0.5;
    double test2 = 0.6;
    //

    //  ____  ____  _____ ____  _____ _____ ____  _ 
    // |  _ \|  _ \| ____/ ___|| ____|_   _/ ___|| |
    // | |_) | |_) |  _| \___ \|  _|   | | \___ \| |
    // |  __/|  _ <| |___ ___) | |___  | |  ___) |_|
    // |_|   |_| \_\_____|____/|_____| |_| |____/(_)
                                                 
    //DAN NGUYEN

    //PRESETS FOR RAISING THE ARM ARE TO BE PUT HERE//
    double preset1 = 1;
    
    //SECURITY FEATURES:
    //I've added a debounce time to the code, to make sure only one preset can run at a time.
    //WARNING DO NOT CHANGE VALUES OUT OF THIS RANGE: 0.38 - 0.16

    //TO ADD MORE PRESETS:
    //Make a new double variable and set it to any time e.g., 1.5 Seconds, name it in order of how many presets there are
    //If there are 10 Presets, name the new one preset11, if there are 2 presets, name it preset3.
    //Go below and find the main code. Copy this code and place it under the pre-existing code:
    //    if (driver.getPOV() == povup & db == 0) {
    //       bentroll*=-1;
    //       raiseArmfor(preset1);
    //       debounce(preset1 + 0.2);
    //    }
    //Change presetx to the corresponding preset for both debounce, and raiseArmfor.
    //Set the new pov, e.g., preset2 is set to povleft. so in the first line:
    //if (driver.getPOV() == povup & db == 0) {
    //                         ^
    //Change the word that the arrow is pointing to, to the corresponding pov, for this case, it would be changed to povleft:
    //if (driver.getPOV() == povleft & db == 0) {
    
    //preset1 is the first preset, which is triggered by POV 0 Degrees, See Below in teleopPeriodic() to see the code.
    //Change the preset1 value to anything, it will raise the arm at 50% speed for preset1 seconds.



    @Override
    public void teleopPeriodic() {

        // Invert Controls when a button is pressed
        if(driver.getXButtonPressed()) {
            rumbleController(0.1, 0.4);
            bentroll*=-1;
            doOnce = 0;
        }
        //Set Drive Speed to 100%
        if(driver.getBButtonPressed()) {
            rumbleController(0.4, 0.2);
            drive_speed=1;
        }
        
        //Set Drive Speed to 50%
        if(driver.getYButtonPressed()) {
            rumbleController(0.2, 0.2);
            drive_speed=.5;
        }

        if (driver.getPOV() == povup) {
            AUTOraiseArmandShoot(0.165);
        }
        if (driver.getPOV() == povdown) {
            raiseArmto(0.3);
        }


        SmartDashboard.putNumber("car", car);
        SmartDashboard.putNumber("inverted?", bentroll);
        SmartDashboard.putNumber("drive_speed", drive_speed);
        SmartDashboard.putNumber("rumbling?", rumbling);
        SmartDashboard.putNumber("encoder", encoder.get());
        SmartDashboard.putNumber("What is diff?", diff);
        SmartDashboard.putNumber("upodwn", upodwn);
        SmartDashboard.putBoolean("limitswitchlower", limitSwitchLower.get());


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

        //Math.abs(xAxisScaled)/4
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

        double absXAXIS = Math.abs(xAxisScaled);
        double absYAXIS = Math.abs(yAxisScaled);

        double otherabsXASIX = Math.abs(driver.getRightX());
        double otherabsYASIX = Math.abs(driver.getRightY());



        if (absYAXIS > 0.2) {
            driving = 1;
            rumbling = 1;
            driver.setRumble(RumbleType.kLeftRumble, absYAXIS);
        }
        else if (absXAXIS > 0.2) {
            driving = 1;
            rumbling = 1;
            driver.setRumble(RumbleType.kLeftRumble, absXAXIS);
        }
        else if (otherabsXASIX > 0.07) {
            driving = 1;
            rumbling = 1;
            driver.setRumble(RumbleType.kLeftRumble, otherabsXASIX/4);
        }
        else if (otherabsYASIX > 0.07) {
            driving = 1;
            rumbling = 1;
            driver.setRumble(RumbleType.kLeftRumble, otherabsYASIX/4);
        }
        else {
            if (driving == 1) {
                rumbling = 0;
            }
            driving = 0;
        }


        SmartDashboard.putNumber("xAxisDriver", xAxisScaled);
        SmartDashboard.putNumber("yAxisDriver", yAxisScaled);
        SmartDashboard.putNumber("RMB", driver.getRightTriggerAxis());
        SmartDashboard.putNumber("Intake ON?", intakeOn);
        SmartDashboard.putNumber("raising", raising);
        SmartDashboard.putBoolean("limitswitchUp", limitSwitchUpper.get());
     // try this instead, squared inputs may smooth the controls somewhat
     //   m_robotDrive.arcadeDrive(driver.getRawAxis(0)*driver.getRawAxis(0),driver.getRawAxis(1)*driver.getRawAxis(1));
      //now for the flapper (wrong '20s')
      //we would use armspeed to determine how fast the flapper works
        // we will use armMotor.set() to give speed. - is in?
              
        if (driver.getRightBumper() || driver.getAButton()) {
            //intake on
            intakeOn = 1;
            intakeMotor.set(ControlMode.PercentOutput,-1); 
        }
        else {
            if (driver.getLeftBumper() || driver.getLeftTriggerAxis() > 0) {
            //intake reverse
                if (driver.getLeftBumper()) {
                intakeOn = 1;
                intakeMotor.set(ControlMode.PercentOutput,0.4);
                }
                else if (driver.getLeftTriggerAxis() > 0) {
                intakeOn = 1;
                intakeMotor.set(ControlMode.PercentOutput,driver.getLeftTriggerAxis());
                }
            }
            else if (shooterDB == 0) {
                intakeOn = 0;
                if (intakeOn == 0) {
                    if (rumbling == 0) {
                        driver.setRumble(RumbleType.kLeftRumble, 0.0);
                    }
                    
                }
                intakeMotor.set(ControlMode.PercentOutput,0.0);
            } 
        }   
        if (intakeOn == 1) {
            driver.setRumble(RumbleType.kLeftRumble, 0.1);
        }
        

        int onshooter = 0;
        if (driver.getRightTriggerAxis() > 0) {
            onshooter = 1;
            //shooter on
            shooterMotor1.set(ControlMode.PercentOutput,driver.getRightTriggerAxis());
            if (driver.getRightTriggerAxis() < 1 & driver.getRightTriggerAxis() != 1) {
                driver.setRumble(RumbleType.kBothRumble, 0.0);
                driver.setRumble(RumbleType.kLeftRumble, driver.getRightTriggerAxis());
            }
            else if (driver.getRightTriggerAxis() == 1) {
                driver.setRumble(RumbleType.kLeftRumble, 0.0);
                driver.setRumble(RumbleType.kBothRumble, 1.0);
            }
            
            shooterMotor2.set(ControlMode.PercentOutput,driver.getRightTriggerAxis()); }
        else if (shooterDB == 0) {
            onshooter = 0;
            shooterMotor1.set(ControlMode.PercentOutput,0.0);
            if (rumbling == 0 & intakeOn == 0) {
                driver.setRumble(RumbleType.kLeftRumble, 0.0);
                driver.setRumble(RumbleType.kBothRumble, 0.0);
            }
            shooterMotor2.set(ControlMode.PercentOutput,0.0);}

            SmartDashboard.putNumber("Shooter On?", onshooter);

        //the arm control,

        //Set ben to 1 to see if the code works
        //When you press A the arm will lift until the "limitSwitchUpper" is triggered (This is temporary, for firing presets)
        if((driver.getRawAxis(5) < -0.1) && (limitSwitchUpper.get())) {
            leftArmMotor.set(ControlMode.PercentOutput,(armSpeed*driver.getRawAxis(5)));
            rightArmMotor.set(ControlMode.PercentOutput,(armSpeed*driver.getRawAxis(5)));
        } else {
            if((driver.getRawAxis(5) > 0.1) && (limitSwitchLower.get())) {
                leftArmMotor.set(ControlMode.PercentOutput,(armSpeed*driver.getRawAxis(5)));
                rightArmMotor.set(ControlMode.PercentOutput,(armSpeed*driver.getRawAxis(5)));
            }
            else if (raising == 0){
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





   private void rumbleController(double seconds, double value) {
       driver.setRumble(RumbleType.kLeftRumble, value);
       rumbling = 1;

       new Thread(() -> {
           Timer.delay(seconds);
           driver.setRumble(RumbleType.kLeftRumble, 0.0);
           rumbling = 0;
       }).start();
   }
    private void raiseArmfor(double seconds) {
        if ((limitSwitchUpper.get()) == true) {
            leftArmMotor.set(ControlMode.PercentOutput,-0.32);
            rightArmMotor.set(ControlMode.PercentOutput,-0.32);
            driver.setRumble(RumbleType.kLeftRumble, 0.05);
            raising = 1;
            rumbling = 1;
        }
        else {
            leftArmMotor.set(ControlMode.PercentOutput,0.0);
            rightArmMotor.set(ControlMode.PercentOutput,0.0);
            driver.setRumble(RumbleType.kLeftRumble, 0.0);
            raising = 0;
            rumbling = 0;
        }

       new Thread(() -> {
            Timer.delay(seconds);
            leftArmMotor.set(ControlMode.PercentOutput,0.0);
            rightArmMotor.set(ControlMode.PercentOutput,0.0);
            driver.setRumble(RumbleType.kLeftRumble, 0.0);
            raising = 0;
            rumbling = 0;
       }).start();
    }


    private void raiseArmto(double degrees) {
        unidegrees = degrees;
        start = 1;

    }


    private void AUTOraiseArmandShoot(double degrees) {
        shooterDB = 1;
        unidegrees = degrees;
        start = 1;
        shootNow = encoder.get() - unidegrees;

        if (Math.abs(shootNow) < 0.15) {
            waittime = 2;
        }
        else if (Math.abs(shootNow) < 0.3 && (Math.abs(shootNow) > 0.15)) {
            waittime = 2;
        }
        else {
            waittime = 2;
        }

        shooterMotor2.set(ControlMode.PercentOutput, 1);
        shooterMotor1.set(ControlMode.PercentOutput, 1);
        new Thread(() -> {
            Timer.delay(waittime);

            intakeMotor.set(ControlMode.PercentOutput,-1);
            new Thread(() -> {
                Timer.delay(1);
                intakeMotor.set(ControlMode.PercentOutput,0.0);
                shooterMotor2.set(ControlMode.PercentOutput, 0.0);
                shooterMotor1.set(ControlMode.PercentOutput, 0.0);
                shooterDB = 0;
            }).start();
        }).start();

    }

    


    
    private void debounce(double seconds) {
        db = 1;

       new Thread(() -> {
            Timer.delay(seconds);
            db = 0;
       }).start();
    }
}
