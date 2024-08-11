package frc.robot;

import java.nio.channels.Channel;
import java.util.Random;

import javax.print.attribute.standard.MediaSize.Other;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

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
import edu.wpi.first.math.geometry.Transform3d;
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
    Random random = new Random();
    PhotonCamera camera = new PhotonCamera("6509limelight3");
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
        m_chooser.addOption("Go Straight", goStraightAuto);

        SmartDashboard.putData("Auto choices", m_chooser);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("Shuffleboard");
        NetworkTableEntry textEntry = table.getEntry("Text");



    }

    @Override
    public void robotPeriodic() {

        var result = camera.getLatestResult();
        SmartDashboard.putBoolean("photon-targets", result.hasTargets());
        
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            int targetID = target.getFiducialId();
            if (targetID == 7 || targetID == 4) {
                SmartDashboard.putNumber("targetID", targetID);
                double poseAmbiguity = target.getPoseAmbiguity();
                Transform3d bestCameraToTarget = target.getBestCameraToTarget();
                Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
                double scale = Math.pow(10, 2);
                double roundedgetX = Math.round(bestCameraToTarget.getX() * scale) / scale - 0.50;
                SmartDashboard.putNumber("x", roundedgetX);
                xvalue = roundedgetX;
                yLL = bestCameraToTarget.getY();
                SmartDashboard.putNumber("y", yLL);
                targetlocked = 1;
                SmartDashboard.putNumber("targlock", targetlocked);
            }
            else {
                targetlocked = 0;
            }
        }

        if (turn_to_y0 == 1) {
            move_db = true;
            if (targetlocked == 1) {
            if (yLL < -0.06 & yLL > -0.5) {
                test_turning = -1;
                m_robotDrive.arcadeDrive(0.0, -0.483);
            }
            else if (yLL < -0.05) {
                test_turning = -2;
                m_robotDrive.arcadeDrive(0.0, -0.555);
            }
            else if (yLL > 0.06 && yLL < 0.5) {
                m_robotDrive.arcadeDrive(0.0, 0.483);
                test_turning = 1;
            }
            else if (yLL > 0.5) {
                m_robotDrive.arcadeDrive(0.0, 0.555);
                test_turning = 2;
            }
            else if (yLL <= 0.1 && yLL >= -0.1) {
                m_robotDrive.arcadeDrive(0.0, 0.0);
                test_turning = 0;
            }
            
            }
            else {
                m_robotDrive.arcadeDrive(0.0, 0.755);
                test_turning = 100;
            }
        }
        else if (turn_to_y0 == -1) {
            test_turning = 0;
            move_db = false;
        }

            //AUTO Turning
        SmartDashboard.putNumber("AutoTurning?", test_turning);
        SmartDashboard.putNumber("enabledAUTOTURNING", turn_to_y0);
        SmartDashboard.putBoolean("move_db", move_db);
        SmartDashboard.putNumber("test_driverleftX", driver.getLeftY());

        if (unidegrees != 0) {
        diff = degree - unidegrees;
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
                if (!limitSwitchLower.get() || diff-4.7 < 0) {

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
                if (!limitSwitchUpper.get() || diff+2 > 0) {

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
        else if (moving < 1 & autoEnabled == 1 & move_db == false) {
            m_robotDrive.arcadeDrive(0.0, 0.0);
        }
        
        double encodervalue = encoder.get();
            // Calculations For Converting Encoder Values to Degrees (Becareful When changing the values)
            if (encodervalue > 0) {

                degree = ((encodervalue - minValue) / (maxValue - minValue)) * (maxDegree - minDegree) + minDegree;

                SmartDashboard.putNumber("Encoder Degrees", degree);
            }
    }

    private void check4AprilTagandTurn() {
            turn_to_y0 *= -1;
    }

    


    //Initial startup for autonomous
    @Override
    public void autonomousInit() {
        m_autoSelected = m_chooser.getSelected();
        //m_autoSelected = SmartDashboard.getString("Auto Selector",
        // kDefaultAuto);
        // Reset the autonomous timer when auto starts

        //autoTimer.reset();
        startTime = Timer.getFPGATimestamp();    
    }

    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putNumber("moving?", moving);
        SmartDashboard.putNumber("speed", rb_speed);
        SmartDashboard.putNumber("speed", rb_turn);
        SmartDashboard.putString("automode", autonomous_selected);
        double time = Timer.getFPGATimestamp();
        
      
        switch (m_autoSelected) {
        case goStraightAuto:
            autonomous_selected = "goStraightAuto";
        break;



        case goLeftAuto:
            autonomous_selected = "goLeftAuto";
            turn_to_y0 = 1;
        break;
      
        case goRightAuto:
             autonomous_selected = "goRightAuto";
        break;

        default:
                    autonomous_selected = "default";
                    //Autonomous Move Forward
                    //mov_turn 0.07 = ~90 degrees

                    if (doOnce == 0) {
                        doOnce = 1;
                        autoEnabled = 1;
                        new Thread(() ->{
                            Timer.delay(0.3);
                            AUTOraiseArmandShoot(12);
                            Timer.delay(3);
                            intakeMotor.set(ControlMode.PercentOutput,-1);
                            drive_to(1.2, 0.0, -0.721);
                            raiseArmto(-2);
                            Timer.delay(0.8);
                            drive_to(0.25, 0.0, -0.621);
                            shooterMotor2.set(ControlMode.PercentOutput, -1);
                            shooterMotor1.set(ControlMode.PercentOutput, -1);
                            Timer.delay(0.55);
                            drive_to(0.25, 0.0, 0.721);
                            intakeMotor.set(ControlMode.PercentOutput,0.3);
                            Timer.delay(0.5);
                            shooterMotor2.set(ControlMode.PercentOutput, 0.0);
                            shooterMotor1.set(ControlMode.PercentOutput, 0.0);
                            Timer.delay(0.1);
                            intakeMotor.set(ControlMode.PercentOutput,0);
                            //Timer.delay(1);
                            getShootingAngleandFire(xvalue);
                            check4AprilTagandTurn();
                            Timer.delay(1.8);
                            check4AprilTagandTurn();
                            drive_to(0.75, -0.7, 0.0);
                            raiseArmto(-2);
                            Timer.delay(1);
                            intakeMotor.set(ControlMode.PercentOutput,-1);
                            drive_to(1, 0.0, -0.721);
                            Timer.delay(0.8);
                            shooterMotor2.set(ControlMode.PercentOutput, -1);
                            shooterMotor1.set(ControlMode.PercentOutput, -1);
                            Timer.delay(0.8);
                            shooterMotor2.set(ControlMode.PercentOutput, 0);
                            shooterMotor1.set(ControlMode.PercentOutput, 0);
                            intakeMotor.set(ControlMode.PercentOutput,0.0);
                            Timer.delay(0.2);
                            intakeMotor.set(ControlMode.PercentOutput,-0.3);
                            drive_to(0.9, 0.65, 0.0);
                            Timer.delay(0.5);
                            intakeMotor.set(ControlMode.PercentOutput,0.0);
                            tap_back();
                            drive_to(2, 0.0, 0.655);
                            Timer.delay(2);
                            getShootingAngleandFire(xvalue);
                            check4AprilTagandTurn();
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

    private void tap_back() {
        intakeMotor.set(ControlMode.PercentOutput,0.3);
        new Thread(() -> {
            Timer.delay(0.5);
            intakeMotor.set(ControlMode.PercentOutput,0.0);
        }).start();
    }

    @Override
    public void teleopInit() {
    }

    /* (non-Javadoc)
     * @see edu.wpi.first.wpilibj.IterativeRobotBase#teleopPeriodic()
     */
    // bentroll is used for inverting controls (Dan)
    //AUTONOMOUS SELECTION VARIABLES
    String autonomous_selected = "nil";
    //Limelight Variables
    double auto_turning_speed = 0.25;
    int targetlocked = 0;
    int pov_down_db = 0;
    //AUTONOMOUS INTAKE MODE
    int intake_mode = 0;
    //AUTONOMOUS INTAKE MODE
    //SET-UP FOR AUTOMATIC ARM ALIGNMENT
    double AUTO_distance = 0;
    double AUTO_angle = 0;
    double xvalue = 0;
    //Negative = Disabled
    //Positive = Enabled
    int turn_to_y0 = -1;
    boolean move_db = false;
    int test_turning = 0;
    //Encoder Calculations:
    //EDIT THE VALUES WITHIN THESE COMMENTS
    double minValue = 0.15;
    double maxValue = 0.4;
    double minDegree = 0;
    double maxDegree = 90;
    //EDIT THE VALUES WITHIN THESE COMMENTS
    double degree = 0;
    //Encoder Calculations:
    //Limelight Calculations:
    double ideal_scoring_height = 2.21;
    double distance_from_apriltag = 0;
    //Limelight Calculations:
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
    double xAxisScaled;
    double yAxisScaled;
    double yLL;

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
            if (targetlocked == 1) {
                if (turn_to_y0 == 1) {
                rumbleController(1.6, 0.5);
                }
                else if (turn_to_y0 == -1) {
                rumbleController(0.3, 0.5);
                }
                getShootingAngleandFire(xvalue);
                check4AprilTagandTurn();
            }
        }

        if (driver.getPOV() == povup) {
            AUTOraiseArmandShoot(12.6);
        }
        if (driver.getPOV() == povdown) {
            if (pov_down_db == 0) {
                pov_down_db = 1;
                rumbleController(0.5, 0.5);
                check4AprilTagandTurn();
        new Thread(() -> {
            Timer.delay(0.3);
            pov_down_db = 0;
        }).start();

            }
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
        // getLeftX referring to the left joysticks left-right motion, from 1 to -1
        // This if statement checks if the joystick is left (negative) or right (positive)
        // If the joystick is left (negative) (or less than zero same thing) it squares the xbox contollers value (for smooth motion)
        // But if the joystick is right (positive) it not only squares the value, but it multiplies it by negative one
        // This is because when you square it, the value from 1 to -1 (left-right), will always be positive, so it
        // doesn't matter if you move the joystick left or right, you will always go one direction.
        // So if the joystick is right, it will multiply the value by -1 to make the robot move the other direction.
        // This is the same for driver.getLeftY, but except on the up-down axis.

        //Math.abs(xAxisScaled)/4
        if (driver.getLeftX() < 0 & move_db == false) {
            xAxisScaled = (driver.getLeftX()*driver.getLeftX());
        }
        else if (driver.getLeftX() > 0 & move_db == false){
            xAxisScaled = -1*(driver.getLeftX()*driver.getLeftX());
        }
        
        if (driver.getLeftY() < 0) {
            yAxisScaled = -drive_speed*bentroll*(driver.getLeftY()*driver.getLeftY());
        }
        else if (driver.getLeftY() > 0) {
            yAxisScaled = drive_speed*bentroll*(driver.getLeftY()*driver.getLeftY());
        }
        if (move_db == false) {
        m_robotDrive.arcadeDrive(yAxisScaled, xAxisScaled);
        }

        double absXAXIS = Math.abs(xAxisScaled);
        double absYAXIS = Math.abs(yAxisScaled);

        double otherabsXASIX = Math.abs(driver.getRightX());
        double otherabsYASIX = Math.abs(driver.getRightY());


        if (absYAXIS > 0.55) {
            driving = 1;
            rumbling = 1;
            driver.setRumble(RumbleType.kLeftRumble, absYAXIS/8);
        }
        else if (absXAXIS > 0.55) {
            driving = 1;
            rumbling = 1;
            driver.setRumble(RumbleType.kLeftRumble, absXAXIS/8);
        }
        else if (otherabsXASIX > 0.2) {
            driving = 1;
            rumbling = 1;
            driver.setRumble(RumbleType.kLeftRumble, otherabsYASIX/14);
        }
        else if (otherabsYASIX > 0.2) {
            driving = 1;
            rumbling = 1;
            driver.setRumble(RumbleType.kLeftRumble, otherabsYASIX/14);
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
        intake_mode = 0;
    }

    @Override
    public void testInit() {
    }





    @Override
    public void testPeriodic() {
    }
    
    private void getShootingAngleandFire(double dis) {

        double adjustedDistance = dis;
        double angle = 0;

            if (dis < 1.1 && dis > 0) {
                angle = 12.6;
            }
            else if (dis == 0 || dis < 0) {
                angle = 12.6;
            }
            else if (adjustedDistance >= RANGE1_D1 && adjustedDistance <= RANGE1_D2) {
                double slope = (RANGE1_A2 - RANGE1_A1) / (RANGE1_D2 - RANGE1_D1);
                angle = RANGE1_A1 + (adjustedDistance - RANGE1_D1) * slope;
            } else if (adjustedDistance >= RANGE2_D1 && adjustedDistance <= RANGE2_D2) {
                double slope = (RANGE2_A2 - RANGE2_A1) / (RANGE2_D2 - RANGE2_D1);
                angle = RANGE2_A1 + (adjustedDistance - RANGE2_D1) * slope;
            }
            else if (adjustedDistance >= RANGE3_D1 && adjustedDistance <= RANGE3_D2) {
                double slope = (RANGE3_A2 - RANGE3_A1) / (RANGE3_D2 - RANGE3_D1);
                angle = RANGE3_A1 + (adjustedDistance - RANGE3_D1) * slope;
            }
            else if (adjustedDistance >= RANGE4_D1 && adjustedDistance <= RANGE4_D2) {
                double slope = (RANGE4_A2 - RANGE4_A1) / (RANGE4_D2 - RANGE4_D1);
                angle = RANGE4_A1 + (adjustedDistance - RANGE4_D1) * slope;
            }
            AUTO_angle = angle;
            SmartDashboard.putNumber("AngleTestPeriodic", angle);
            AUTOraiseArmandShoot(angle);
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

   private static final double RANGE1_D1 = 1.0;
   private static final double RANGE1_A1 = 12.6;
   private static final double RANGE1_D2 = 2.0;
   private static final double RANGE1_A2 = 25.5;

   private static final double RANGE2_D1 = 0.0;
   private static final double RANGE2_A1 = 12.6;
   private static final double RANGE2_D2 = 0.99;
   private static final double RANGE2_A2 = 25.5;

   private static final double RANGE3_D1 = 2.0;
   private static final double RANGE3_A1 = 22.5;
   private static final double RANGE3_D2 = 3.0;
   private static final double RANGE3_A2 = 28;

   private static final double RANGE4_D1 = 3.0;
   private static final double RANGE4_A1 = 28;
   private static final double RANGE4_D2 = 3.5;
   private static final double RANGE4_A2 = 30.5;


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

        shooterMotor2.set(ControlMode.PercentOutput, 1);
        shooterMotor1.set(ControlMode.PercentOutput, 1);
        new Thread(() -> {
            Timer.delay(1.6);

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
}