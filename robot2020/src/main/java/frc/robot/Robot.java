/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.               test123                                                  */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.livewindow.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.UsbCamera;
import java.util.Map;
import edu.wpi.first.wpilibj.controller.*;
import javax.print.attribute.standard.JobPrioritySupported;

import com.ctre.phoenix.motorcontrol.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;


public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String Auton2 = "Auton-2";
  private static final String Auton1 = "Auton-1";
  private static final String Auton3 = "Auton-3";
  private static final String Auton4 = "Auton-4";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  /*
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 color_sensor = new ColorSensorV3(i2cPort);
  private final ColorMatch color_matcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  */

  PIDController gyroPID ;
  PIDController limepid ;
  PIDController drivepid ;
  PIDController tiltpid ;

  
  boolean targeting = false;
  UsbCamera camera;
  NetworkTableEntry camstream;
  boolean limelight_target_button = false;
  boolean xbox_override = false;
  boolean xbox_low_dump_mode_on = false ;
  boolean xbox_low_dump_mode_on_really_fast = false;
  boolean outtake_trigger_mode = false;
  boolean xbox_shooter_wheels_out_high_really_fast_mode = false;
  

  // MOTORS:
  CANSparkMax shooterWheel1;
  CANSparkMax shooterWheel2;
  WPI_TalonSRX talonLeftDrive;
  WPI_TalonSRX talonRightDrive;
  WPI_VictorSPX tiltMotor ;
  WPI_VictorSPX leftDriveFollow ;
  WPI_VictorSPX rightDriveFollow ;
  WPI_VictorSPX dialMotor ;
  WPI_VictorSPX intakeMotor;
  WPI_VictorSPX wheelSpinnerMotor ;
  WPI_VictorSPX wheelElevatorMotor ;
  CANSparkMax winchMotor ;
  WPI_VictorSPX deployMotor ;

  // CONTROLLERS:
  XboxController xbox = new XboxController(1);
  Joystick joystick = new Joystick(0);


  // ENCODERS:
  Encoder dialEncoder ;
  Encoder tiltEncoder ;
  Encoder wheelElevatorEncoder ;
  AnalogInput climbSwitch = new AnalogInput(2) ;



  // global'ish Variables used in routines below:
  Timer myClock = new Timer();
  double throttle; 
  double turn_throttle;
  DifferentialDrive drive ;
  boolean tab_intake_mode_on = false ;
  boolean tab_low_dump_mode_on = false ;
  boolean intake_mode_xbox_on = false;
  boolean doing_manual_tilt = false ;
  double intake_roller_speed_in = .3 ;
  double jump_backwards_starttime = 0 ;
  boolean jump_backwards_finished = false ;  
  double jump_forwards_starttime = 0 ;
  boolean jump_forwards_finished = false ;
  boolean done_moving = false ;
  boolean done_moving2 = false ;
  boolean done_auton_turning = false ;
  double done_auton_turning_time = 0 ;
  boolean done_auton_turning2 = false ;
  boolean ready_to_fire = false ;
  double old_shooter_speed_1 = 0 ;
  double old_shooter_speed_2 = 0 ;
  double old_dial_speed = 0 ;
  boolean continual_outtake_mode = true ;
  boolean only_once = false ;
  double target_found_score = 0 ; 
  double target_found_distance_score = 0 ; 
  boolean target_and_fire_mode = false ;
  double ball_count = 0 ;
  boolean limelightLEFT = true ;
  double gyro_turn_minspeed = .35 ;
  double lime_turn_minspeed = .25 ;
  double lime_turn_speed = .55 ;
  double limepid_x_score = 0 ;
  double limepid_distance_score = 0 ;
  double auton_started_firing_time = 0 ;


  // MPOSE
  boolean mpose_turn_1_done = false ;
  boolean mpose_turn_2_done = false ;
  boolean auton_mpose_1 = false ;
  boolean auton_mpose_2 = false ;

  /// DIAL
  boolean move_dial_one_notch = false ;
  double dial_notch_begin_encoder_value = 0 ;
  boolean dial_is_moving_one_notch = false ;
  AnalogInput beam1 = new AnalogInput(0) ;
  AnalogInput beam2 = new AnalogInput(1) ;
  boolean dial_by_sensor_is_on =false ;
  boolean tab_spinDialOut = false ;

  // color wheel
  String wheelHeightGoal = "down" ;
  String current_wheel_height = "" ;

  // TILT:
  String tiltGoal = "high" ;
  String current_tilt_location = "" ;
  double startedLoweringTime = 0 ;
  double tiltToAngleValue = 0 ;

  //network tables:
  private NetworkTableEntry intakeControl;
  private NetworkTableEntry shooterControl;
  private NetworkTableEntry shooterControlLow;
  private NetworkTableEntry dialNotchControl ;

  private NetworkTableEntry tiltFloorControl ;
  private NetworkTableEntry tiltLowControl ;
  private NetworkTableEntry tiltHighControl ;
  private NetworkTableEntry tiltMotorRightControl ;
  private NetworkTableEntry tiltMotorLeftControl ;

  private NetworkTableEntry beamIntakeControl ;
  private NetworkTableEntry allIntakeControl ;
  private NetworkTableEntry lowDumpControl ;
  private NetworkTableEntry spinDialOutControl ;
  private NetworkTableEntry spinDialOutControl2 ;
 
  private NetworkTableEntry colorWheelSpinControl ;
  private NetworkTableEntry colorWheelElevatorDownControl; 
  private NetworkTableEntry colorWheelElevatorUpControl ;
  private NetworkTableEntry colorWheelElevatorAtWheelControl;

  private NetworkTableEntry winchLeftControl ;
  private NetworkTableEntry winchRightControl;
  private NetworkTableEntry usePanelControl ;
  private NetworkTableEntry deployControl ;
  private NetworkTableEntry deployInControl ;
  private NetworkTableEntry deployOutControl ;

  private NetworkTableEntry pidKpControl;
  private NetworkTableEntry pidKdControl ;
  private NetworkTableEntry pidKiControl ;
  private NetworkTableEntry extraControl ;
  
  private NetworkTableEntry zeroControl ;
  private NetworkTableEntry shooterSpeedControl ;
  private NetworkTableEntry shooterSpeedOnOffControl ;
  private NetworkTableEntry tiltAngleControl ;
  private NetworkTableEntry tiltAngleOnOffControl ;

  private NetworkTableEntry limelightModeControl ;
  private NetworkTableEntry limelightLEDControl ;
  private NetworkTableEntry limeXControl ;
  private NetworkTableEntry limeYControl ;
  private NetworkTableEntry limeTAControl ;
  private NetworkTableEntry limeTControl ;
  private NetworkTableEntry limeAControl ;
  private NetworkTableEntry limeSteerControl ;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  Faults _faults = new Faults() ;

  Pose2d mpose ;
  DifferentialDriveOdometry od ;
  

  
  @Override
  public void robotInit() {
    myClock.start();
    myClock.reset();
    table.getEntry("pipeline").setNumber(0);
    CameraServer.getInstance().startAutomaticCapture("Camera", 0);

      /*
    color_matcher.addColorMatch(kBlueTarget);
    color_matcher.addColorMatch(kGreenTarget);
    color_matcher.addColorMatch(kRedTarget);
    color_matcher.addColorMatch(kYellowTarget);
    */


    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable CamPub = inst.getTable("CameraPublisher");
    camstream = CamPub.getEntry("camera_stream");

    // define autonomous options:
    m_chooser.setDefaultOption("Default Auton", kDefaultAuto);
    m_chooser.addOption("Auton1", Auton1);
    m_chooser.addOption("Auton2", Auton2);
    m_chooser.addOption("Auton3", Auton3);
    m_chooser.addOption("Auton4", Auton4);
    SmartDashboard.putData("Auto choices", m_chooser);


    // motors:
    shooterWheel1 = new CANSparkMax(11, MotorType.kBrushless);
    shooterWheel2 = new CANSparkMax(10, MotorType.kBrushless);
    talonLeftDrive = new WPI_TalonSRX(2);
      talonLeftDrive.configFactoryDefault() ;
      talonLeftDrive.setSensorPhase(true);
    talonRightDrive = new WPI_TalonSRX(1);
      talonRightDrive.configFactoryDefault() ;
      talonRightDrive.setSensorPhase(true);
      talonRightDrive.setInverted(true) ;

    leftDriveFollow = new WPI_VictorSPX(25);
    rightDriveFollow = new WPI_VictorSPX(24);
      rightDriveFollow.setInverted(true) ;
    dialMotor = new WPI_VictorSPX(21);
    tiltMotor = new WPI_VictorSPX(22);
    intakeMotor = new WPI_VictorSPX(23);

    wheelElevatorMotor = new WPI_VictorSPX(20);
    wheelSpinnerMotor = new WPI_VictorSPX(26);
    winchMotor = new CANSparkMax(12,MotorType.kBrushless) ;
    deployMotor = new WPI_VictorSPX(27) ;

    setupTestButtons() ; // test mode buttons on shuffleboard.


    // encoders:
    tiltEncoder = new Encoder (0,1);
    dialEncoder = new Encoder (2,3) ;
    wheelElevatorEncoder = new Encoder (4,5) ;

    // driving:
    talonLeftDrive.configFactoryDefault() ;
    talonRightDrive.configFactoryDefault() ;
    leftDriveFollow.configFactoryDefault();  
    leftDriveFollow.follow(talonLeftDrive);
    rightDriveFollow.configFactoryDefault();  
    rightDriveFollow.follow(talonRightDrive);
    drive = new DifferentialDrive(talonLeftDrive,talonRightDrive);
    drive.setRightSideInverted(false);

    
    limelight_LED_mode(false);
    limelight_camera_mode(true);

    talonRightDrive.configMotionCruiseVelocity(8000,30);
    talonRightDrive.configMotionAcceleration(8000,30);
    talonLeftDrive.configMotionCruiseVelocity(8000,30);
    talonLeftDrive.configMotionAcceleration(8000,30);

    talonLeftDrive.setSelectedSensorPosition(0,0,30);
    talonRightDrive.setSelectedSensorPosition(0,0,30);

    talonRightDrive.configClosedloopRamp(.5);
    talonLeftDrive.configClosedloopRamp(.5);

double P = 0.08 ;
double I = 0 ;
double D = 0.0 ;
double F = 0.2 ;
    
talonRightDrive.selectProfileSlot(0,0);
talonRightDrive.config_kF(0,F,30);
talonRightDrive.config_kP(0,P,30);
talonRightDrive.config_kI(0,I,30);
talonRightDrive.config_kD(0,D,30);    

talonLeftDrive.selectProfileSlot(0,0);
talonLeftDrive.config_kF(0,F,30);
talonLeftDrive.config_kP(0,P,30);
talonLeftDrive.config_kI(0,I,30);
talonLeftDrive.config_kD(0,D,30);

gyroPID = new PIDController(.05,0,.01) ;
tiltpid = new PIDController(.03,0,0) ;
mpose = new Pose2d(); 



  }


  @Override
  public void robotPeriodic() {
    all_values_to_dashboard();
  
  }


  @Override
  public void autonomousInit() {
      tiltEncoder.reset();
      dialEncoder.reset() ;
      wheelElevatorEncoder.reset();

      jump_backwards_finished = false ;
      jump_backwards_starttime = 0 ;
      jump_forwards_finished = false ;
      jump_forwards_starttime = 0 ;
      done_auton_turning = false ;
      done_auton_turning2 = false ;
      done_moving = false ;
      done_moving2 = false ;

          // mpose:
      talonLeftDrive.setSelectedSensorPosition(0,0,30);
      talonRightDrive.setSelectedSensorPosition(0,0,30);
      gyro.reset();
      SmartDashboard.putString("MPOSE STATUS", "not started");
      do_mpose_init() ;

      target_found_score = 0 ;
      target_found_distance_score = 0 ;
      limelight_LED_mode(true);    limelight_camera_mode(false);
      continual_outtake_mode = true ;


    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);


  }
 

  @Override
  public void autonomousPeriodic() {

    switch (m_autoSelected) {
      case Auton1:
        // Put custom auto code here
        SmartDashboard.putString("Running Auton:", "Auton-1");
          do_auton_1() ;
        break;

        case kDefaultAuto:
        default:
        // Put default auto code here
        SmartDashboard.putString("Running Auton:", "DEFAULT");
          do_auton_default() ;
        break;

        case Auton2:
        SmartDashboard.putString("Running Auton:", "Auton-2");
          do_auton_2() ;

        break ;

        case Auton3:
        SmartDashboard.putString("Running Auton:", "Auton-3");
          do_auton_3() ;

        break ;

        case Auton4:
        SmartDashboard.putString("Running Auton:", "Auton-3");
          do_auton_4() ;

        break ;

      }

  }


    // -- ALL AUTON HERE:
  public void do_auton_default() {
    if (done_moving == false) {
      done_moving = moveRobot(4); // go X feet forward from where we started.
      return ;
    } 
  }
  public void do_auton_1() {
    tiltGoal = "low";
    do_tilting_pid() ;
    if (done_moving == false) {
      done_moving = moveRobot(5); // go X feet forward from where we started.
      return ;
    } 
    if (done_auton_turning == false) {
      done_auton_turning = gyro_turn_to(0); // adjust back to zero.
      if (done_auton_turning == true) {
        talonLeftDrive.setSelectedSensorPosition(0,0,30);
        talonRightDrive.setSelectedSensorPosition(0,0,30);
        done_auton_turning_time = myClock.get() ;
      }
      return ;
    }
    if ((myClock.get() - done_auton_turning_time) < 1) { return ;}

    if (done_moving2 == false) {
      SmartDashboard.putString("HERE:", "got here");
      done_moving2 = moveRobot(4); // go X feet forward from where we started.
      return ;
    } 

    if (done_auton_turning2 == false) {
      SmartDashboard.putString("HERE2:", "got here");
      done_auton_turning2 = gyro_turn_to(0); // adjust back to zero.
      return ;
    }
    drive.arcadeDrive(.3, 0);
    do_low_dump();

  }

  public void do_auton_2() {
    tiltGoal = "angle" ;
    tiltToAngleValue = 0 ;
    do_tilting_pid() ;
    if (current_tilt_location != "floor") { intakeMotor.set(-.7);}

    //jump_backwards() ;  if (jump_backwards_finished == false) { return ;}
   //  jump_forwards() ;  if (jump_forwards_finished == false) { return ;}
    targeting = true ;
    if (ready_to_fire == false) {
      limelight_targeting_left();
      return ;
    }
    run_shooter_wheels_for_shooting_out(.75);
    dialMotor.set(-1);

  }
  public void do_auton_3() {

    jump_forwards() ;  if (jump_forwards_finished == false) { return ;}

    if (current_tilt_location != "floor") { intakeMotor.set(-.7);}

    targeting = true ;
    if (done_auton_turning == false ) {
      done_auton_turning = gyro_turn_to(-23) ;
      return ;
    }

    run_shooter_wheels_for_shooting_out(.65);
    if (ready_to_fire == false) {
      limelight_targeting_right();
      auton_started_firing_time = myClock.get() ;
      return ;
    }
    if ( (myClock.get() - auton_started_firing_time) < 2) {
      dialMotor.set(-1);
      return ;
    }
    if ( (myClock.get() - auton_started_firing_time) < 3) {
      dialMotor.set(0) ;
      stop_shooter_wheels();
    }

    
    if (auton_mpose_1 == false) {
      auton_mpose_1 = do_mpose(-21591,-2748) ;
      if (auton_mpose_1 == true) {
        mpose_turn_1_done = false ;
      }
      return ;
    } 

      // turn south
    if (done_auton_turning2 == false ) {
      done_auton_turning2 = gyro_turn_to(180) ;
      return ;
    }

    run_all_intake();

    if (current_tilt_location != "floor"){
      return ;
    } 

    /*
    if (auton_mpose_2 == false) {
      auton_mpose_2 = do_mpose(-10000,-10200) ;
      return ;
    }
*/
      SmartDashboard.putString("AUTON:", "arrivedddd");
  }

  public void do_auton_4() {

    jump_forwards() ;  if (jump_forwards_finished == false) { return ;}

    if (current_tilt_location != "floor") { intakeMotor.set(-.7);}

    targeting = true ;
    if (done_auton_turning == false ) {
      done_auton_turning = gyro_turn_to(23) ;
      return ;
    }

    run_shooter_wheels_for_shooting_out(.65);
    if (ready_to_fire == false) {
      limelight_targeting_left();
      auton_started_firing_time = myClock.get() ;
      return ;
    }
    if ( (myClock.get() - auton_started_firing_time) < 2) {
      dialMotor.set(-1);
      return ;
    }
    if ( (myClock.get() - auton_started_firing_time) < 3) {
      dialMotor.set(0) ;
      stop_shooter_wheels();
    }

    
    if (auton_mpose_1 == false) {
      auton_mpose_1 = do_mpose(-23645,22175) ;
      if (auton_mpose_1 == true) {
        mpose_turn_1_done = false ;
      }
      return ;
    } 

      // turn south
    if (done_auton_turning2 == false ) {
      done_auton_turning2 = gyro_turn_to(-120) ;
      return ;
    }

    run_all_intake();
    
    if (current_tilt_location != "floor"){
      return ;
    } 

    //drive.arcadeDrive(.3, 0);

    /*
    if (auton_mpose_2 == false) {
      auton_mpose_2 = do_mpose(-10000,-10200) ;
      return ;
    }
*/
      SmartDashboard.putString("AUTON:", "arrivedddd");
  }


  public void jump_backwards() {
    if (jump_backwards_finished == true) { return ;}
    if (jump_backwards_starttime == 0){
      drive.arcadeDrive(-1, 0);
      jump_backwards_starttime = myClock.get() ;
      return ;
    }
    if ( (myClock.get() - jump_backwards_starttime) > .2) {
      drive.arcadeDrive(0, 0);
      jump_backwards_finished = true ;
    }

  }

  public void jump_forwards() {
    if (jump_forwards_finished == true) { return ;}
    if (jump_forwards_starttime == 0){
      drive.arcadeDrive(1, 0);
      jump_forwards_starttime = myClock.get() ;
      return ;
    }
    if ( (myClock.get() - jump_forwards_starttime) > .2) {
      drive.arcadeDrive(0, 0);
      jump_forwards_finished = true ;
    }

  }


  @Override
  public void teleopInit() {
    pid_playground() ;
    stop_low_dump() ; // from autonomous
    limelight_LED_mode(false);
    limelight_camera_mode(true);
    targeting=false ;
    stop_shooter_wheels();
    dialMotor.set(0);

    // mpose:
    talonLeftDrive.setSelectedSensorPosition(0,0,30);
    talonRightDrive.setSelectedSensorPosition(0,0,30);
    gyro.reset();
    SmartDashboard.putString("MPOSE STATUS", "not started");
    do_mpose_init() ;

    target_and_fire_mode = false ;
    targeting = false ;
    ready_to_fire = false ;
    stop_shooter_wheels();
    dialMotor.set(0);
    drive.arcadeDrive(0, 0);

    // CHANGE THESE BEFORE COMPETITION: uncomment
    intake_mode_xbox_on = true ;
    continual_outtake_mode = true ;

  }

  @Override
  public void teleopPeriodic() {

    //do_Colors();
   do_mpose_update() ;


    if (zeroControl.getBoolean(false)){ // zero all encoders if ZERO button pushed.
      zero_everything();
    }

    //intake_rollers_continual_out() ; // bypass with thumb on joystick

    if (usePanelControl.getBoolean(false)){ // "USE CONTROL PANEL" button is pressed.
      controlTabStuff() ;
      normal_driving();
      return;
    }

    do_control_maps() ;

    if (joystick.getRawButtonPressed(12) == true){
      limepid_x_score = 0 ;
      limepid_distance_score = 0 ;
      ready_to_fire = false;
      limelight_LED_mode(true);    limelight_camera_mode(false);
    }
    if (joystick.getRawButtonReleased(12) == true){
   //   limelight_LED_mode(false);    limelight_camera_mode(true);
    }
    if (joystick.getRawButton(12) == true){
      //limepidtargeting(0,0) ;
      limepidtargeting(2.5,2.8) ;
      return ;
    }


      // switch back and forth between left and right targeting.
    if (joystick.getRawButtonPressed(8) == true) {
      limelightLEFT = true ;
    } else if (joystick.getRawButtonPressed(12) == true){ // RIGHT!
      limelightLEFT = false ;
    }
   // SmartDashboard.putBoolean("TARGET-which-way:", limelightLEFT);




    if (joystick.getRawButtonPressed(7)) {  // go into target and fire mode.
        ready_to_fire = false ;
        target_and_fire_mode = true ;
        targeting = true ;
        tiltToAngleValue = 0 ;
        tiltGoal = "angle";
        do_tilting_pid() ;
        run_shooter_wheels_for_shooting_out(.65);
        target_found_score = 0 ;
        target_found_distance_score = 0 ;
        limelight_LED_mode(true);    limelight_camera_mode(false);

    } else if (joystick.getRawButtonReleased(7)){  // leave target and fire mode.
        target_and_fire_mode = false ;
        targeting = false ;
        ready_to_fire = false ;
        limelight_LED_mode(false);    limelight_camera_mode(true);
        stop_shooter_wheels();
        dialMotor.set(0);
        drive.arcadeDrive(0, 0);
    } 


    if (targeting == true) {  
        if (limelightLEFT == true) {
          limelight_targeting_left();
        } else {
          limelight_targeting_right();
        }
        //SmartDashboard.putBoolean("RTF:", ready_to_fire) ;
        //SmartDashboard.putNumber("TFS:", target_found_score) ;
        //SmartDashboard.putNumber("TFDS:", target_found_distance_score) ;
        if (target_and_fire_mode == true && ready_to_fire == true && current_tilt_location == "angle"){
          // FIRE!
          dialMotor.set(-1) ;
        }

    } else { // we are NOT targeting:
      normal_driving() ;
    }
    
    
  }

  // -------------------------------------------------------------
  // -------------------------------------------------------------
  // -------------------------------------------------------------
  // -------------------------------------------------------------

  public void talonpidtest() {
    //limepid.setTolerance(.01);
    //double answerR = limepid.calculate(talonRightDrive.getSelectedSensorPosition() / 10000,1) ;
    //double answerL = limepid.calculate(talonLeftDrive.getSelectedSensorPosition() / 10000,-1) ;
    //SmartDashboard.putNumber("TALONSTEER:", answerR) ;
    //SmartDashboard.putNumber("TALONSTEEL:", answerL) ;
    if (joystick.getRawButton(11)) {
      SmartDashboard.putString("HERE:", "got here");
      double dist = 10000 ;
      talonRightDrive.set(ControlMode.MotionMagic, dist);
      talonLeftDrive.set(ControlMode.MotionMagic, dist);
    //  talonRightDrive.set(ControlMode.MotionMagic, demand0, demand1Type, demand1);
 /*    talonRightDrive.set(.2) ;
    talonLeftDrive.set(.2) ;
    talonLeftDrive.getFaults(_faults);
    System.out.println("OUT OF PHASE:" + _faults.SensorOutOfPhase); */
    }
  }
  public boolean moveRobot(double feet) {  
    double dist = feet * 2500 ;
    talonRightDrive.set(ControlMode.MotionMagic, dist);
    talonLeftDrive.set(ControlMode.MotionMagic, dist);
    if (
        Math.abs(dist - talonRightDrive.getSelectedSensorPosition()) < 1000
        && 
        Math.abs(dist - talonLeftDrive.getSelectedSensorPosition()) < 1000
       ) {
       return true ;
    }
    return false ;
  }

  // -------------------------------------------------------------
  public void drive_straight( double targetAngle , double targetSpeed) {

  //  double answer = gyroPID.calculate(gyro.getAngle(),0) ;
//    SmartDashboard.putNumber("answer:", answer) ;

    // STRAIGHT LINE:
   // double Kp = pidKpControl.getDouble(.03) ;
    double Kp = .1 ;

    double turningValue = (targetAngle - gyro.getAngle()) * Kp ;
    double diff = Math.abs(gyro.getAngle() - targetAngle) ;

    SmartDashboard.putString("TV:", "tv:" + turningValue + " diff:" + diff) ;
    SmartDashboard.putString("STRAIGHT:", "angle:" + targetAngle + " speed:" + targetSpeed) ;
    //double extrastuff = extraControl.getDouble(.12) ;
    //SmartDashboard.putNumber("EXTRA", extrastuff) ;
      drive.arcadeDrive(targetSpeed, turningValue);

  }
  
  public boolean gyro_turn_to(double degrees) {
    double answer = gyroPID.calculate(gyro.getAngle(),degrees) ;
    double diff = Math.abs(gyro.getAngle() - degrees) ;
   SmartDashboard.putString("turnto:", "DEG:" + degrees + " diff:" + diff + " ans:" + answer);
   if (talonRightDrive.getSelectedSensorVelocity() < 50) {
     gyro_turn_minspeed = gyro_turn_minspeed + .01 ;
   }
    if (answer < 0) {
      if (answer > -gyro_turn_minspeed) { answer = -gyro_turn_minspeed;}
      drive.arcadeDrive(0, answer);
    } else {
      if (answer < gyro_turn_minspeed) { answer = gyro_turn_minspeed ;}
      drive.arcadeDrive(0, answer);
    }
    if (  Math.abs(gyro.getAngle() - degrees) < 2) {
      drive.arcadeDrive(0, 0);
      //SmartDashboard.putString("finished gyro turn:", "yes");
      return true;
    }
    return false ;
  }
  // -------------------------------------------------------------
  public void do_control_maps() {

    boolean override_button = xbox.getRawButtonPressed(10);
    int tilting_to_floor = xbox.getPOV();
    int tilting_low = xbox.getPOV();
    int tilting_high = xbox.getPOV();
    double wheel_elevator = xbox.getRawAxis(3);
    double vertical_dial_override = xbox.getRawAxis(1);
    boolean spinning_color_wheel = xbox.getRawButton(12);
    boolean intake_out = joystick.getRawButton(1);
    boolean winch_up = xbox.getRawButton(4);
    boolean winch_down = xbox.getRawButton(2);
    double deploy = xbox.getRawAxis(1);


  // set an override button to go into override mode
  SmartDashboard.putBoolean("xbox override", xbox_override);
    if (override_button == true && xbox_override == false){
      xbox_override = true;
    } else if (override_button == true){
      xbox_override = false;
    }

    
    if (xbox_override == true){     // override mode

      // zero out all encoders:
      if (xbox.getRawButtonPressed(9) ){
        zero_everything();
      }
      
      // manual override for the wheel elevator going up and down that ignores the sensor. CAREFUL!!
      if (Math.abs(wheel_elevator) > 0.05){
        wheelElevatorMotor.set(wheel_elevator);
      } else {
        wheelElevatorMotor.set(0);
      }
      // Button that you have to hold down to spin the wheel
      if (spinning_color_wheel == true){
        wheelSpinnerMotor.set(0.7);
      } else {
        wheelSpinnerMotor.set(0);
      }

      // move dial up and down manually
      if (Math.abs(vertical_dial_override) > 0.05){
        tiltMotor.set(vertical_dial_override);
      } else if ( vertical_dial_override < 0.05){
        tiltMotor.set(0);
      }

      boolean button6pressed = xbox.getRawButtonPressed(6) ;
      if ( button6pressed == true && xbox_low_dump_mode_on_really_fast == false){
        xbox_low_dump_mode_on_really_fast = true ;
      } else if (button6pressed == true) {
        xbox_low_dump_mode_on_really_fast = false;
        stop_low_dump_really_fast();
      }
      if (xbox_low_dump_mode_on_really_fast == true){
        do_low_dump_really_fast();
      }
      
      /*
      boolean button8pressed = xbox.getRawButtonPressed(8) ;
      if ( button8pressed == true && xbox_shooter_wheels_out_high_really_fast_mode == false){
        xbox_shooter_wheels_out_high_really_fast_mode = true ;
      } else if (button8pressed == true) {
        xbox_shooter_wheels_out_high_really_fast_mode = false;
        stop_shoot_out_high_really_fast();
      }
      if (xbox_shooter_wheels_out_high_really_fast_mode == true){
        do_shooting_out_high_really_fast();
      }
      */

    }



    // code for tilting the dial to set value
    if ( xbox_override == false){

        // TILTING: 
      if (tilting_to_floor == 180){
        tiltGoal = "floor" ;
      } else if ( tilting_low == 90){
        tiltGoal = "low" ;
      } else if (tilting_high == 0){
        tiltGoal = "high" ;
      } // end code for tilting dial to set value
      do_tilting_pid();


      // this sets the trigger on the joystick to run the intake backwards so we don't get stuck on balls
      if (intake_out == true && outtake_trigger_mode == false){
        outtake_trigger_mode = true;
      } else if (joystick.getRawButtonReleased(1) == true )  {
        intakeMotor.set(0);
        outtake_trigger_mode = false;
        //return ;
      }
      if (outtake_trigger_mode == true) {
        intakeMotor.set(-1);
        return ; // prevent the running of run_all_intake.
      }




      // temporarily reverse the shooter wheels in case they are stuck.
      if (joystick.getRawButtonPressed(5)) {
        shooter_wheels_reverse_unjam() ; 
      }
      if (joystick.getRawButtonReleased(5)) {
        shooter_wheels_reverse_unjam_done() ; 
      }
      if (joystick.getRawButton(5)) {
        return ; // to prevent any other function from operating the wheels while we unjam.
      }


      // temporarily reverse the DIAL direction AND the shooter wheels, for when shooting is jammed.
      if (joystick.getRawButtonPressed(6)) {
        shooting_unjam();
      }
      if (joystick.getRawButtonReleased(6)) {
        shooting_unjam_done() ; 
      }
      if (joystick.getRawButton(6)) {
        return ; // to prevent any other function from operating the DIAL while we unjam.
      }


      // hit button, all intake mechanisms will turn on, hit it again, and they will turn off
       boolean button7pressed = xbox.getRawButtonPressed(7) ;
      if ( button7pressed == true){
        ball_count = 1 ;
      }
      
      if (xbox.getRawButton(7)){
        run_all_intake();
        intake_mode_xbox_on = true ;
        //ball_count = 1 ;
      } 
      if (xbox.getRawButtonReleased(7) == true) {
        intake_mode_xbox_on = false;
        stop_all_intake();
      }

      //   if (intake_mode_xbox_on == true) {
   //     run_all_intake();
    //  }


      // hit button, low dump mode will turn on, hit button again, low dump mode will turn off
      boolean button6pressed = xbox.getRawButtonPressed(6) ;
   /*   if ( button6pressed == true && xbox_low_dump_mode_on == false){
        xbox_low_dump_mode_on = true ;
      } else if (button6pressed == true) {
        xbox_low_dump_mode_on = false;
        stop_low_dump();
      }
      if (xbox_low_dump_mode_on == true){
        do_low_dump();
      } */
      if (xbox.getRawButton(6) == true) {
        do_low_dump();
        xbox_low_dump_mode_on = true ;
      }
      if (xbox.getRawButtonReleased(6) == true){
        stop_low_dump();
        xbox_low_dump_mode_on = false ;
      }
       
      
      
      if ( xbox.getRawButton(8) == true) {
        xbox_shooter_wheels_out_high_really_fast_mode = true ;
        do_shooting_out_high_really_fast();
      } 
      if(xbox.getRawButtonReleased(8)) {
        xbox_shooter_wheels_out_high_really_fast_mode = false;
        stop_shoot_out_high_really_fast();
      }
      
      
      // setting wheel elavator to the right joystick on the xbox controller. It's a manual up and down, with stops at the ends
      // Click joystick in to spin the wheel
      if (Math.abs(wheel_elevator) > 0.05 ){
        double wheelHeight = wheelElevatorEncoder.getDistance() ;
        if ( wheelHeight > 850 && wheel_elevator < 0) {
          wheelElevatorMotor.set(0);
        } else if (wheelHeight < 5 && wheel_elevator > 0) {
          wheelElevatorMotor.set(0);
        } else {
            if (wheelHeight > 770){
              wheelElevatorMotor.set(wheel_elevator * 0.6);
            } else{
              wheelElevatorMotor.set(wheel_elevator);
            }
        }
      }
      else {
        wheelElevatorMotor.set(0);
      }
      if (spinning_color_wheel == true){
        wheelSpinnerMotor.set(1);
      } else {
        wheelSpinnerMotor.set(0);
      }

      // CLIMBING:
      if (winch_up == true) {
        winchMotor.set(-1) ;
      } else if (winch_down) {
        winchMotor.set(1);
      } else {
        winchMotor.set(0);
      }
      // DEPLOY motor.
      if (Math.abs(deploy) > .01) {
        if ( deploy < 0 && climbSwitch.getAverageVoltage() < 2 ) {
          deployMotor.set(0) ;
        } else {
          deployMotor.set(deploy);
        }
      } else {
        deployMotor.set(0) ;
      }
  


    }  // END non-override mode.

  }
  
  // -------------------------------------------------------------
  public void intake_rollers_on() {
    // never run in IN unless tilt is at floor.
    if (current_tilt_location == "floor") {
      intakeMotor.set(intake_roller_speed_in) ;
    } //else {
      //intakeMotor.set(0);
    //}
  }
  public void intake_rollers_off() {
    intakeMotor.set(0) ;
  }
  public void intake_rollers_continual_out() {
    // if we are not on the floor, run the roller bar OUT.
    // but this can be annoying, so allow turn off with button.
    if (joystick.getRawButtonPressed(2)){
      if (continual_outtake_mode == true) {
        continual_outtake_mode = false ;
        intakeMotor.set(0);
      } else {
        continual_outtake_mode = true ;
      }
    }

    if ( continual_outtake_mode == true &&  current_tilt_location != "floor") {
      intakeMotor.set(-.4) ; 

    } 
  }

  // -------------------------------------------------------------
  public void run_all_intake() {
    tiltGoal = "floor" ;
    do_tilting_pid();
    intake_rollers_on() ;
    run_shooter_wheels_for_intake() ;
    dial_by_sensor() ;
    return ;
  }
  public void stop_all_intake() {
    intake_rollers_off();
    stop_shooter_wheels();
    dialMotor.set(0);
    return ;
  }
  // ----------------------------------------
  public void do_low_dump() {
    tiltGoal = "low" ;
    do_tilting_pid();
    if (current_tilt_location != "low") { return ;} // wait until tilt is correct.
    // tilt is now OK, fire.
    run_shooter_wheels_for_shooting_out_low() ;
    dialMotor.set(-1) ;
  }

  public void do_low_dump_really_fast(){
    tiltGoal = "low" ;
    do_tilting_pid();
    if (current_tilt_location != "low") { return ;} // wait until tilt is correct.
    // tilt is now OK, fire.
    run_shooter_wheels_for_shooting_out_low_really_fast() ;
    dialMotor.set(-1) ;
  }
  public void stop_low_dump () {
    dialMotor.set(0);
    stop_shooter_wheels();
  }
  public void stop_low_dump_really_fast(){
    dialMotor.set(0);
    stop_shooter_wheels();
  }
  public void do_shooting_out_high_really_fast(){
    tiltGoal = "high" ;
    do_tilting_pid();
    if (current_tilt_location != "high") { return ;} // wait until tilt is correct.
    // tilt is now OK, fire.
    run_shooter_wheels_for_shooting_out_high_really_fast() ;
    dialMotor.set(-1) ;
  }
  public void stop_shoot_out_high_really_fast(){
    dialMotor.set(0);
    stop_shooter_wheels();
  }
  
  // --------------------------------------
  public void do_wheel_height() {
    double wheel_up_goal = 850 ;
    double wheel_atwheel_goal = 805 ;
    double wheelHeight = wheelElevatorEncoder.getDistance() ;

    current_wheel_height = "" ;

    if (wheelHeightGoal == "down") {
      if (wheelHeight < 10) {
        current_wheel_height = "down" ;
        wheelElevatorMotor.set(0) ;
        return ;
      }
      wheelElevatorMotor.set(.8);
      return ;
    }

    if (wheelHeightGoal == "up") {
      if ( wheelHeight > (wheel_up_goal - 15) ) {
        current_wheel_height = "up";
        wheelElevatorMotor.set(0) ;
        return;
      }
      wheelElevatorMotor.set(-.8) ;
      return ;
    }

    if (wheelHeightGoal == "atwheel") {
      if ( (Math.abs(wheelHeight - wheel_atwheel_goal)) < 15) {
        current_wheel_height = "atwheel" ;
        wheelElevatorMotor.set(0) ;
        return ;
      }
      if (wheelHeight > wheel_atwheel_goal) {
        wheelElevatorMotor.set(.5) ;
        return ;
      }
      if (wheelHeight < wheel_atwheel_goal) {
        wheelElevatorMotor.set(-.7) ;
      }
    }
  }
  
  // --------------------------------------


  public void controlTabStuff() {

   /* if (limelightLEDControl.getBoolean(false)){
      limelight_LED_mode(true);
    } else {
      limelight_LED_mode(false);
    }
    if (limelightModeControl.getBoolean(false)){
      limelight_camera_mode(true);
    } else {
      limelight_camera_mode(false);
    }
*/

    if (winchLeftControl.getBoolean(false)){
      winchMotor.set(.8) ;
    } else if (winchRightControl.getBoolean(false)){
      winchMotor.set(-.8) ;
    } else {
      winchMotor.set(0);
    }
    if (deployInControl.getBoolean(false)) {
      deployMotor.set(.8);
    } else if (deployOutControl.getBoolean(false)){
      if (climbSwitch.getAverageVoltage() > 2){
        deployMotor.set(-.8) ;
      } else {
        deployMotor.set(0) ;
      }
    } else {
      deployMotor.set(0);
    }


  

    if (colorWheelSpinControl.getBoolean(false)) {
      wheelSpinnerMotor.set(.7) ;
    } else {
      wheelSpinnerMotor.set(0) ;
    }

    // color wheel elevator.
    if (colorWheelElevatorDownControl.getBoolean(false)){
      wheelHeightGoal = "down" ;
      colorWheelElevatorDownControl.setBoolean(false) ;
    } else if (colorWheelElevatorUpControl.getBoolean(false)){
      wheelHeightGoal = "up" ;
      colorWheelElevatorUpControl.setBoolean(false) ;
    } else if (colorWheelElevatorAtWheelControl.getBoolean(false)){
      wheelHeightGoal = "atwheel" ;
      colorWheelElevatorAtWheelControl.setBoolean(false) ;
    } 
    do_wheel_height() ;



    // INTAKE MODE:
    if (allIntakeControl.getBoolean(false) ) {
      tab_intake_mode_on = true ;
      run_all_intake() ;
      return ;
    } else if (tab_intake_mode_on == true) {
      tab_intake_mode_on = false ;
      stop_all_intake() ;
      return ;
    }


    // spin the dial OUT:
    if (spinDialOutControl.getBoolean(false) || spinDialOutControl2.getBoolean(false)
        && tab_intake_mode_on == false  
        && move_dial_one_notch == false
        && Math.abs(dialMotor.get()) < .05
        ) {
          tab_spinDialOut = true ;
          dialMotor.set(-1) ;
          return ;
    } else if (tab_spinDialOut == true){
      tab_spinDialOut = false ;
      dialMotor.set(0) ;
      return;
    }


    // do all stuff for LOW dump.
    if (lowDumpControl.getBoolean(false)) {
      tab_low_dump_mode_on = true ;
      do_low_dump() ;
      return ;
    } else if (tab_low_dump_mode_on == true) {
      tab_low_dump_mode_on = false ;
      stop_low_dump() ;
      return ;
    }


    // SHOOTER WHEELS
    if (intakeControl.getBoolean(false)){
      run_shooter_wheels_for_intake() ;
    } else if (shooterControl.getBoolean(false)){
      run_shooter_wheels_for_shooting_out_high() ;
    } else if (shooterControlLow.getBoolean(false)){
      run_shooter_wheels_for_shooting_out_low() ;
    } else if (shooterSpeedOnOffControl.getBoolean(false)) {
      double thespeed = shooterSpeedControl.getDouble(0) ;
      shooterWheel1.set(-thespeed);
      shooterWheel2.set(thespeed) ;
    } else {
      stop_shooter_wheels();
    }


    // DIAL by increments: 
    if (dialNotchControl.getBoolean(false))  {
      move_dial_one_notch = true ;
      dialNotchControl.setBoolean(false) ;
    }
    dial_intake_turning_by_degrees();

    // DIAL by beam control:
    //  if I want to run JUST the beams and dial:
    if (beamIntakeControl.getBoolean(false)) {
      dial_by_sensor_is_on = true ;
      dial_by_sensor() ;
    } else if (dial_by_sensor_is_on == true) {
      dial_by_sensor_is_on = false ;
      dialMotor.set(0) ;
    }
    


    // TILTING:
    if (tiltFloorControl.getBoolean(false)) {
      tiltGoal = "floor" ;
      tiltFloorControl.setBoolean(false) ;
    } else if (tiltLowControl.getBoolean(false)) {
      tiltGoal = "low" ;
      tiltLowControl.setBoolean(false) ;
    } else if (tiltHighControl.getBoolean(false)) {
      tiltGoal = "high" ;
      tiltHighControl.setBoolean(false) ;
    } else if (tiltAngleOnOffControl.getBoolean(false)){
      tiltToAngleValue = tiltAngleControl.getDouble(0) ;
      tiltGoal = "angle" ;
      tiltAngleOnOffControl.setBoolean(false) ;
    } 
   
    // manual tilt:
    if (tiltMotorLeftControl.getBoolean(false)){
      tiltMotor.set(.7);
      tiltGoal = "" ;
      doing_manual_tilt = true ;
    } else if (tiltMotorRightControl.getBoolean(false)){
      tiltMotor.set(-.7);
      tiltGoal = "" ;
      doing_manual_tilt = true ;
    } else {
      if (doing_manual_tilt == true) {
        doing_manual_tilt = false ;
        tiltMotor.set(0) ;
      }
      //do_tilting();
      do_tilting_pid();
    }

    // Color Wheel

  }

  @Override
  public void testPeriodic() {

    //controlTabStuff();

    limelight_LED_mode(true);
    limelight_camera_mode(false);

  }
  // -------------------------------------------------------------
  public void do_tilting_pid() {
    double tiltPosition = tiltEncoder.getDistance();


      // Robot starts HIGH, at 0.  increase to get to low, increase more to get to floor.
      double low_goal = 140 ;
      double floor_goal = 410 ;
  
      current_tilt_location = "";
      if (tiltGoal == "") { tiltMotor.set(0);  return ;}

      /* if the motor is lowering the dial, (ie, has a positive set value), ie, moving toward "low" or "floor",
          it should reach its goal within 3 seconds, otherwise there is probably a ball stuck under it.
          It won't reach the encoder value, so it will keep running until it unwinds.
          If that's the case, stop the motor and set tiltgoal to null so that it stops trying.
      */
      if ((tiltGoal == "low" || tiltGoal == "floor") && tiltMotor.get() > 0) {
        if (startedLoweringTime > 0) {
          double elapsed = myClock.get() - startedLoweringTime;
          if (elapsed > 3) {
            tiltMotor.set(0);
            tiltGoal = "" ;
            System.out.println("lowering failed after 3 sec, bailing");
            startedLoweringTime = 0 ;
            return ;
          }
        } else {
          startedLoweringTime = myClock.get() ;
        }
      } else {
        startedLoweringTime = 0 ;
      }
  
      if (tiltGoal == "high"){
        tiltToAngleValue = 0 ;
      } else if (tiltGoal == "low") {
        tiltToAngleValue = low_goal;
      } else if (tiltGoal == "floor") {
         tiltToAngleValue = floor_goal ;
      }

      double answer = Math.round(tiltpid.calculate(tiltPosition,tiltToAngleValue));
      SmartDashboard.putString("TILTPID:", "pos:"+tiltPosition+" val:"+tiltToAngleValue+" ans:"+answer+" speed:"+tiltMotor.get());
      tiltMotor.set(answer) ;
      if (Math.abs(tiltToAngleValue - tiltPosition) < 10) { 
        current_tilt_location = tiltGoal ;
        tiltMotor.set(0);
      }

      if (Math.abs(answer) < .05) {
        current_tilt_location = tiltGoal ;
        tiltMotor.set(0);
      }
  
  }
  // -------------------------------------------------------------
  // TILT CONTROL
  public void do_tilting() {

    double tiltPosition = tiltEncoder.getDistance();
    // Robot starts HIGH, at 0.  increase to get to low, increase more to get to floor.
    double low_goal = 140 ;
    double floor_goal = 410 ;

    current_tilt_location = "";

    /* if the motor is lowering the dial, (ie, has a positive set value), ie, moving toward "low" or "floor",
        it should reach its goal within 3 seconds, otherwise there is probably a ball stuck under it.
        It won't reach the encoder value, so it will keep running until it unwinds.
        If that's the case, stop the motor and set tiltgoal to null so that it stops trying.
    */
    if ((tiltGoal == "low" || tiltGoal == "floor") && tiltMotor.get() > 0) {
      if (startedLoweringTime > 0) {
        double elapsed = myClock.get() - startedLoweringTime;
        if (elapsed > 3) {
          tiltMotor.set(0);
          tiltGoal = "" ;
          System.out.println("lowering failed after 3 sec, bailing");
          startedLoweringTime = 0 ;
          return ;
        }
      } else {
        startedLoweringTime = myClock.get() ;
      }
    } else {
      startedLoweringTime = 0 ;
    }



    if (tiltGoal == "high"){
      if (tiltPosition < 10)  {
        // then we are AT high.  so stop doing anything.
        current_tilt_location = "high" ;
        tiltMotor.set(0);
        return ;
      }
      // need to raise it to get to high
      tiltMotor.set(-.8);
      return ;
    }

    if (tiltGoal == "low") {
      if (  Math.abs( low_goal - tiltPosition) < 15) {
        current_tilt_location = "low";
        tiltMotor.set(0);
        return ;
      }
      // it's too high, lower it.
      if ( tiltPosition < low_goal) {
        tiltMotor.set(.7) ;
        return ;
      }
      if ( tiltPosition > low_goal) { // too low, raise it.
        tiltMotor.set(-.8);
        return ;
      }
    }
    

    if (tiltGoal == "floor") {
      if (  Math.abs( floor_goal - tiltPosition) < 15) {
        current_tilt_location = "floor";
        tiltMotor.set(0);
        return ;
      }
      if ( tiltPosition < floor_goal) {
        tiltMotor.set(.7) ;
        return ;
      }
      if ( tiltPosition > floor_goal) {
        tiltMotor.set(-.8);
        return ;
      }
    }

    if (tiltGoal == "angle") {
      if (Math.abs(tiltPosition - tiltToAngleValue) < 15) {
        current_tilt_location = "angle";
        tiltMotor.set(0) ;
        return;
      }
      if (tiltPosition < tiltToAngleValue) {
        tiltMotor.set(.7) ;
        return ;
      }
      if (tiltPosition > tiltToAngleValue) {
        tiltMotor.set(-.8) ;
        return;
      }
    }

  }

  //  dial around one ball at a time.
  public void dial_intake_turning_by_degrees() {
    if (move_dial_one_notch == true) {
      dial_notch_begin_encoder_value = dialEncoder.getDistance();
      dial_is_moving_one_notch = true ;
      dialMotor.set(1) ;
      move_dial_one_notch = false ;
    }
    if (dial_is_moving_one_notch == true) {
      // it's already moving.  decide if we should keep moving it, or we've finished.
      double current_encoder_value = dialEncoder.getDistance();
      double how_far = Math.abs(dial_notch_begin_encoder_value - current_encoder_value) ;
      SmartDashboard.putNumber("how far", how_far);
      SmartDashboard.putNumber("begin", dial_notch_begin_encoder_value) ;
      if (how_far > 150) {
        dialMotor.set(0);
        dial_is_moving_one_notch = false ;
      } else {
        // keep on turning....
      }
    } else {
      // nothing to do here.. 
    }
  }

  // -------------------------------------------------------------
  public void dial_by_sensor() {
      // when a ball breaks beam 1, turn until both beam 1 and beam 2 are not broken.
    double motorspeed = 1 ;
    if (ball_count < 2) { motorspeed = .6 ;} else { motorspeed = 1 ;}

    if ( beam1.getAverageVoltage() < .1) {
      dialMotor.set(motorspeed) ;
      return;
    }
    if (beam1.getAverageVoltage() > .1 && beam2.getAverageVoltage() > .1){
      if (dialMotor.get() > 0) { ball_count++ ;}
      dialMotor.set(0);
    }

  }

  // -------------------------------------------------------------
  // shooter wheel controls:
  public void stop_shooter_wheels(){
    shooterWheel1.set(0);
    shooterWheel2.set(0) ;
  }
  public void run_shooter_wheels_for_intake() {
    double SPEED = .8 ;
    shooterWheel1.set(SPEED);
    shooterWheel2.set(-SPEED) ;
  }
  public void run_shooter_wheels_for_shooting_out_high() {
    shooterWheel1.set(-0.58);
    shooterWheel2.set(0.5802) ;
  }
  public void run_shooter_wheels_for_shooting_out_high_really_fast(){
    shooterWheel1.set(-0.65);
    shooterWheel2.set(0.65);
  }
  public void run_shooter_wheels_for_shooting_out_low() {
    shooterWheel1.set(-.3);
    shooterWheel2.set(.3F) ;
  }
  public void run_shooter_wheels_for_shooting_out_low_really_fast(){
    shooterWheel1.set(-1);
    shooterWheel2.set(1) ;
  }
  public void run_shooter_wheels_for_shooting_out(double speed){
    shooterWheel1.set(-speed);
    shooterWheel2.set(speed) ;
  }
  //   -------------------------------------------------------------
  public void all_values_to_dashboard() {
    SmartDashboard.putNumber("TILT:",tiltEncoder.getDistance());
//    SmartDashboard.putNumber("TILT-SPEED:",tiltEncoder.getRate());
    SmartDashboard.putString("CURRENT TILT:",current_tilt_location);
 
    SmartDashboard.putNumber("DIAL:",dialEncoder.getDistance());
    SmartDashboard.putNumber("beam-1:", beam1.getAverageVoltage()) ;
    SmartDashboard.putNumber("beam-2:", beam2.getAverageVoltage()) ;
    SmartDashboard.putNumber("color-wheel-elevator:", wheelElevatorEncoder.getDistance());

    SmartDashboard.putNumber("angle:", gyro.getAngle()) ;
    //SmartDashboard.putNumber("throttle:", joystick.getThrottle());
    SmartDashboard.putNumber("climb switch:", climbSwitch.getAverageVoltage());

    limeXControl.setNumber(tx.getDouble(0.0));
    limeYControl.setNumber(ty.getDouble(0.0));
    limeTAControl.setNumber(ta.getDouble(0.0));
    limeAControl.setNumber( table.getEntry("tv").getDouble(0.0));
    limeTControl.setBoolean(targeting);

    double Rposition = talonRightDrive.getSelectedSensorPosition();
    double Lposition = talonLeftDrive.getSelectedSensorPosition();
    SmartDashboard.putNumber("RIGHT POSITION:", Rposition);
    SmartDashboard.putNumber("LEFT POSITION:", Lposition);
    SmartDashboard.putNumber("RIGHTSPEED", talonRightDrive.getSelectedSensorVelocity()) ;
    SmartDashboard.putNumber("LEFTSPEED", talonLeftDrive.getSelectedSensorVelocity()) ;


  }

  // -------------------------------------------------------------
  public void setupTestButtons() {
    intakeControl = Shuffleboard.getTab("controls")
    .add("shooter wheels IN",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(1, 0)
    .withSize(3, 1)
    .getEntry();

    shooterControl = Shuffleboard.getTab("controls")
    .add("shooter wheels out fast",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(1, 2)
    .withSize(3, 1)
    .getEntry();

    shooterControlLow = Shuffleboard.getTab("controls")
    .add("shooter wheels out LOW",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(1, 4)
    .withSize(3, 1)
    .getEntry();



    tiltFloorControl = Shuffleboard.getTab("controls")
    .add("tilt-to-floor",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(4, 0)
    .withSize(3, 1)
    .getEntry();

    tiltLowControl = Shuffleboard.getTab("controls")
    .add("tilt-LOW",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(4, 2)
    .withSize(3, 1)
    .getEntry();

    tiltHighControl = Shuffleboard.getTab("controls")
    .add("tilt-HIGH",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(4, 4)
    .withSize(3, 1)
    .getEntry();


    
    beamIntakeControl = Shuffleboard.getTab("controls")
    .add("BEAM intake",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(4, 6)
    .withSize(3, 1)
    .getEntry(); 

    allIntakeControl = Shuffleboard.getTab("controls")
    .add("ALL intake mechanisms",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(7, 0)
    .withSize(3, 1)
    .getEntry();

    lowDumpControl = Shuffleboard.getTab("controls")
    .add("Dump Low",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(7, 2)
    .withSize(3, 1)
    .getEntry();

    spinDialOutControl = Shuffleboard.getTab("controls")
    .add("Spin dial OUT",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(7, 4)
    .withSize(3, 1)
    .getEntry();

    dialNotchControl = Shuffleboard.getTab("controls")
    .add("move-dial one click",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(7, 6)
    .withSize(3, 1)
    .getEntry();

    colorWheelSpinControl = Shuffleboard.getTab("controls")
    .add("Spin color wheel",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(10, 0)
    .withSize(3, 1)
    .getEntry();

    colorWheelElevatorDownControl = Shuffleboard.getTab("controls")
    .add("Wheel elevator DOWN",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(10, 2)
    .withSize(3, 1)
    .getEntry();

    colorWheelElevatorUpControl = Shuffleboard.getTab("controls")
    .add("Wheel elevator UP",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(10, 4)
    .withSize(3, 1)
    .getEntry();  

    colorWheelElevatorAtWheelControl = Shuffleboard.getTab("controls")
    .add("Wheel elevator AT COLOR WHEEL",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(10, 6)
    .withSize(3, 1)
    .getEntry();

    
    tiltMotorLeftControl = Shuffleboard.getTab("controls")
    .add("TILT MOTOR LEFT",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(13, 0)
    .withSize(3, 1)
    .getEntry();
    tiltMotorRightControl = Shuffleboard.getTab("controls")
    .add("TILT MOTOR RIGHT",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(13, 2)
    .withSize(3, 1)
    .getEntry();


    usePanelControl = Shuffleboard.getTab("controls")
    .add("USE PANEL",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(16, 0)
    .withSize(3, 1)
    .getEntry();

    zeroControl = Shuffleboard.getTab("controls")
    .add("Zero Encoders",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(16, 2)
    .withSize(3, 1)
    .getEntry();
    

    winchLeftControl = Shuffleboard.getTab("climb")
    .add("Winch LEFT",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(10, 0)
    .withSize(3, 1)
    .getEntry();

    winchRightControl = Shuffleboard.getTab("climb")
    .add("Winch RIGHT",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(10, 2)
    .withSize(3, 1)
    .getEntry();


    deployOutControl = Shuffleboard.getTab("climb")
    .add("Deploy Motor OUT",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(16, 0)
    .withSize(3, 1)
    .getEntry();

    deployInControl = Shuffleboard.getTab("climb")
    .add("Deploy Motor IN",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(16, 2)
    .withSize(3, 1)
    .getEntry();

  /*  deployControl = Shuffleboard.getTab("climb")
    .add("Deploy Motor",0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withPosition(10, 5)
    .getEntry();*/


    limelightModeControl = Shuffleboard.getTab("LIME")
    .add("Camera Target mode switch",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(8, 0)
    .withSize(3, 1)
    .getEntry();

    limelightLEDControl = Shuffleboard.getTab("LIME")
    .add("LEDs on-off",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(8, 2)
    .withSize(3, 1)
    .getEntry();

    shooterSpeedControl = Shuffleboard.getTab("LIME")
    .add("shooter speed",.5)
    .withPosition(11,0)
    .getEntry();

    shooterSpeedOnOffControl = Shuffleboard.getTab("LIME")
    .add("Shoot at this speed",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(14, 0)
    .withSize(3, 1)
    .getEntry();

    spinDialOutControl2 = Shuffleboard.getTab("LIME")
    .add("Spin dial for shooting",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(14, 2)
    .withSize(3, 1)
    .getEntry();

    tiltAngleControl = Shuffleboard.getTab("LIME")
    .add("tilt 0-400",300)
    .withPosition(11,2)
    .getEntry();

    tiltAngleOnOffControl = Shuffleboard.getTab("LIME")
    .add("tilt to angle",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(10,4)
    .withSize(3,1)
    .getEntry();

    limeYControl = Shuffleboard.getTab("LIME")
    .add("Y:",0)
    .withPosition(8,4)
    .getEntry() ;
    limeXControl = Shuffleboard.getTab("LIME")
    .add("X:",0)
    .withPosition(8,5)
    .getEntry() ;
    limeTAControl = Shuffleboard.getTab("LIME")
    .add("TA:",0)
    .withPosition(8,6)
    .getEntry() ;
    limeAControl = Shuffleboard.getTab("LIME")
    .add("Acquired:",0)
    .withPosition(8,7)
    .getEntry() ;
    limeSteerControl = Shuffleboard.getTab("LIME")
    .add("Steer:",0)
    .withPosition(10,6)
    .getEntry() ;
    limeTControl = Shuffleboard.getTab("LIME")
    .add("targeting:",false)
    .withPosition(10,6)
    .withSize(3, 1)
    .getEntry() ;



    pidKpControl = Shuffleboard.getTab("PID")
    .add("KP",.03)
    .getEntry() ;     
    pidKdControl = Shuffleboard.getTab("PID")
    .add("KD",.05)
    .getEntry() ;
    extraControl = Shuffleboard.getTab("PID")
    .add("extra",.2)
    .getEntry() ;

    }

  // -------------------------------------------------------------
  public void limelight_camera_mode(boolean CAM){
    if (CAM == true) {
      table.getEntry("camMode").setNumber(1);
      return;
    }
    table.getEntry("camMode").setNumber(0);
  }
  public void limelight_LED_mode(boolean CAM){
    if (CAM == true) {
      table.getEntry("ledMode").setNumber(3);
      return;
    }
    table.getEntry("ledMode").setNumber(1);
  }

  // -------------------------------------------------------------
  public void normal_driving() {
    double throttle = (1 - joystick.getThrottle()) /  2.0 ;
    double turn_throttle = throttle + .2 ;
    // we should repace this with some math.... :-)
    if (throttle > .9) { throttle = .95 ;}
    else if (throttle > .8) { throttle = .89 ;}
    else if (throttle > .7) { throttle = .83 ;}
    else if (throttle > .6) { throttle = .77 ;}
    else if (throttle > .5) { throttle = .72 ;}
    else if (throttle > .4) { throttle = .66 ;}
    else if (throttle > .3) { throttle = .6 ;}
    else if (throttle > .2) { throttle = .54 ;}
    else if (throttle > .1) { throttle = .49 ;}
    else if (throttle > 0) { throttle = .43 ;}
    else { throttle= 0; }

    double joyy = joystick.getY();
    double joyz = joystick.getZ();
    if (  Math.abs(joyy) < .03) { joyy = 0 ;} // deadzone.
    if (  Math.abs(joyz) < .03) { joyz = 0 ;} // deadzone.
    drive.arcadeDrive(-joyy * throttle , joyz * turn_throttle );
  }
  // -------------------------------------------------------------------
  // -------------------------------------------------------------------
  // -------------------------------------------------------------------
  public void limepidtargeting( double distanceMin, double distanceMax) {
    
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    // this indicates whether or not the limelight can see the target at all.
    double havetarget = table.getEntry("tv").getDouble(0.0);

    // use zeros to NOT do distance. 
    boolean doing_distance_too = false ;
    if (distanceMax != 0 && distanceMin != 0) { 
      doing_distance_too = true ;
    }

    // if you don't see the target at all, do nothing.  rumble joystick?
    if (havetarget < 1){  drive.arcadeDrive(0, 0);  return ; }
  
    //double answer = Math.round(limepid.calculate(-x,0));
    //SmartDashboard.putString("LIMEPID x:", x+" ans:"+answer+" speed:" +  talonRightDrive.getSelectedSensorVelocity() );

    // check scores:
    if (Math.abs(x) < 1) {
      limepid_x_score++ ;
    } else {
      limepid_x_score = 0 ;
    }
    if (doing_distance_too){
      if (area > distanceMin && area < distanceMax){
        limepid_distance_score++ ;
      } else {
        limepid_distance_score = 0 ;
      }
    }
    /// if both scores match, we're done.
    if (doing_distance_too && limepid_distance_score > 25 && limepid_x_score > 25) {
      drive.arcadeDrive(0, 0);
      ready_to_fire = true ;
      return ;  
    } else if (doing_distance_too == false &&  limepid_x_score > 25) {
      drive.arcadeDrive(0, 0);
      ready_to_fire = true ;
      return ;  
    }


    // DISTANCE:  
    double straight = 0;
    if (doing_distance_too){ 
      if (area < distanceMax && area > distanceMin){ // IN proper area:
        straight = 0 ;
      } else if (area > distanceMax){ // too close, backup.
        if ( (area - distanceMax) > 2) {
          straight = -.7 ;
        } else if ((area - distanceMax) > 1) {
          straight = -.6 ;
        } else {
          straight = -.5 ;
        }
      } else if (area < distanceMin){
        if (( distanceMin - area) > 2) { // to far away, go forard.
          straight = .7 ;
        } else if ((distanceMin - area) > 1){
          straight = .6 ;
        } else {
          straight = .5 ;
        }
      }
    }


    if (joystick.getRawButton(11) == false) {
      return ; 
    }

    // TURNING:
    double turnspeed ;
    if (Math.abs(x) < 1) {
      turnspeed = 0 ;
    } else if (Math.abs(x) < 5) {
      turnspeed = get_lime_turn_minspeed();
    } else {
      turnspeed = get_lime_turn_speed();
    } 
    turnspeed = Math.copySign(turnspeed,x) ;
    SmartDashboard.putString("LPscore", limepid_x_score + " TS:" + turnspeed + " SPscore:" + limepid_distance_score );
    drive.arcadeDrive(straight, turnspeed);


  } 

  public double get_lime_turn_speed() {
      //if (joystick.getRawButton(11) == false) {return lime_turn_speed ;}
      if (Math.abs(talonRightDrive.getSelectedSensorVelocity()) < 60) {
         lime_turn_speed = lime_turn_speed + .005 ;
      } else if (Math.abs(talonRightDrive.getSelectedSensorVelocity()) > 100) {
        lime_turn_speed = lime_turn_speed - .005 ;
      }
    if (lime_turn_speed > .6) { lime_turn_speed = .6 ;}
    return lime_turn_speed ;
  }

  public double get_lime_turn_minspeed() {
    //if (joystick.getRawButton(11) == false) {return lime_turn_minspeed ;}

      if (Math.abs(talonRightDrive.getSelectedSensorVelocity()) < 10) {
        System.out.println("got here:" + lime_turn_minspeed);
        lime_turn_minspeed = lime_turn_minspeed + .005 ;
      } else if (Math.abs(talonRightDrive.getSelectedSensorVelocity()) > 20) {
        System.out.println("got here2:" + lime_turn_minspeed);
        lime_turn_minspeed = lime_turn_minspeed - .005 ;
      }
      if (lime_turn_minspeed > .5) { lime_turn_minspeed = .5 ;}

    return lime_turn_minspeed ;
  }

  public void limelight_targeting_left() {

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    // this indicates whether or not the limelight can see the target at all.
    double havetarget = table.getEntry("tv").getDouble(0.0);

    // if you don't see the target at all, do nothing.  rumble joystick?
    if (havetarget < 1){  drive.arcadeDrive(0, 0);  return ; }
  
    // now turn toward target:

    double steer = 0;
    if (x > 10){
      steer = 0.6;
    }
    else if (x < -10){
      steer = -0.6;
    }
    else if (x < 10 && x > 1){
      steer=0.5;
    }
    else if (x < 1 && x > 0){
      steer = 0.4;
    }
    else if (x > -10 && x < -3){
      steer = -0.5;
    }
    else if (x > -3 && x < -2){
      steer = 0.4;
    }

    else if ( x > -2 && x < 0 ){
      target_found_score = target_found_score + 1 ;
      steer = 0;
    } else {
      target_found_score = 0 ;
    }
   

    double straight = 0;
    if (area > 4){
      straight = -0.7;
    }
    else if (area > 2.8 && area < 4){
      straight = -0.6;
    }
    else if (area < 1){
      straight = 0.7;
    }
    else if (area < 2 && area > 1){
      straight = 0.6;
    }
    else if (area < 2.5 && area > 2){
      straight = 0.5;
    }

    else if ( area > 2.5 && area < 2.8 ){
      target_found_distance_score = target_found_distance_score + 1 ;
      straight = 0;
    } else {
      target_found_distance_score = 0 ;
    }
    
    drive.arcadeDrive(straight, steer);
    

    if (target_found_score > 40 && target_found_distance_score > 40){
      ready_to_fire = true ;
    }

 

  }
  public void limelight_targeting_right() {

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    // this indicates whether or not the limelight can see the target at all.
    double havetarget = table.getEntry("tv").getDouble(0.0);

    // if you don't see the target at all, do nothing.
    if (havetarget < 1){  drive.arcadeDrive(0, 0);  return ; }

    // now turn toward target:
    double steer = 0;
    if (x > 12){
      steer = 0.5;
    }
    else if (x < 3){
      steer = -0.5;
    }
    else if (x < 11 && x > 9){
      steer=0.4;
    }
    else if (x < 9 && x > 7){
      steer = 0.3;
    }
    else if (x > 3 && x < 5){
      steer = -0.3;
    }

    else if ( x > 5 && x < 7 ){
      target_found_score = target_found_score + 1 ;
      steer = 0;
    } else {
      target_found_score = 0 ;
    }
    //limeSteerControl.setNumber(steer) ;
    
    double straight = 0;
    if (area > 4){
      straight = -0.6;
    }
    else if (area > 3.3 && area < 4){
      straight = -0.5;
    }
    else if (area < 3.3 && area > 2.7 ){
      straight = -0.4;
    }
    else if (area < 1){
      straight = 0.6;
    }
    else if (area < 2 && area > 1){
      straight = 0.5;
    }
    else if (area > 2 && area < 2.3){
      straight = 0.4;
    }

    else if ( area > 2.3 && area < 2.7 ){
      target_found_distance_score = target_found_distance_score + 1 ;
      straight = 0;
    } else {
      target_found_distance_score = 0 ;
    }
    
    
    //limeSteerControl.setNumber(straight) ;
    drive.arcadeDrive(straight, steer);
    

    if (target_found_score > 40 && target_found_distance_score > 40){
      ready_to_fire = true ;
    }

  }
// -------------------------------------------------------------


/*
public void do_Colors(){

  Color detectedColor = color_sensor.getColor();
    String colorString;
    ColorMatchResult match = color_matcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget){
      colorString = "Blue";
    } else if (match.color == kGreenTarget){
        colorString = "Green";
      } else if (match.color == kRedTarget){
          colorString = "Red";
      } else if (match.color == kYellowTarget){
           colorString = "Yellow";
      } else {
          colorString = "No Color Yet";
      }

      SmartDashboard.putNumber("Red", detectedColor.red);
      SmartDashboard.putNumber("Green", detectedColor.green);
      SmartDashboard.putNumber("Blue", detectedColor.blue); 
      SmartDashboard.putNumber("Confidence", match.confidence);
      SmartDashboard.putString("Detected Color", colorString);
}
*/
  
void pid_playground() {
  //    pc = new PIDController(kP,kI,kD) ;
  double Kp = pidKpControl.getDouble(.03) ;
  double Kd = pidKdControl.getDouble(.05) ;
  //double Kp = .5 ;
  double Ki = 0 ; //double Kd = 0 ;
  SmartDashboard.putNumber("KP", Kp) ;
  SmartDashboard.putNumber("KD", Kd) ;
  //gyroPID = new PIDController(Kp,Ki,Kd) ;
  limepid = new PIDController(Kp, Ki, Kd);
  //drivepid = new PIDController(Kp, Ki, Kd);
  //tiltpid = new PIDController(Kp, Ki, Kd);
}

// -------------------------------------------------------------
public void shooter_wheels_reverse_unjam() {
  shooterWheel1.set(-1) ;
  shooterWheel2.set(1) ; 

}
  public void shooter_wheels_reverse_unjam_old() {
    // note what the speeds are at the beginning of unjam, so that we can set the motors back to that speed after unjam.
  old_shooter_speed_1 = shooterWheel1.get() ;
  old_shooter_speed_2 = shooterWheel2.get() ;
  if (old_shooter_speed_1 > 0) {
      shooterWheel1.set(-1) ;
  } else if (old_shooter_speed_1 < 0) {
    shooterWheel1.set(1) ; 
  }
  if (old_shooter_speed_2 > 0) {
    shooterWheel2.set(-1) ;
  } else if (old_shooter_speed_2 < 0) {
    shooterWheel2.set(1) ; 
  }
}
public void shooter_wheels_reverse_unjam_done() {
  shooterWheel1.set(0) ;
  shooterWheel2.set(0) ;
  //shooterWheel1.set(old_shooter_speed_1);
  //shooterWheel2.set(old_shooter_speed_2);
}
// -------------------------------------------------------------
public void shooting_unjam() {
  // note what the speeds are at the beginning of unjam, so that we can set the motors back to that speed after unjam.
  old_dial_speed = dialMotor.get() ;
  dialMotor.set(1) ;
  old_shooter_speed_1 = shooterWheel1.get() ;
  old_shooter_speed_2 = shooterWheel2.get() ;
  shooterWheel1.set(1) ; 
  shooterWheel2.set(-1) ; 

}
public void shooting_unjam_done() {
  dialMotor.set(old_dial_speed);
  shooter_wheels_reverse_unjam_done() ;
}


// -------------------------------------------------------------
public void do_mpose_init() {
  od = new DifferentialDriveOdometry( new Rotation2d()) ;
  var gyroAngle = Rotation2d.fromDegrees(-gyro.getAngle());
  mpose = od.update(gyroAngle, 
    talonLeftDrive.getSelectedSensorPosition()  , 
    talonRightDrive.getSelectedSensorPosition() 
    ) ;
}

// -------------------------------------------------------------
public void do_mpose_update() {
  var gyroAngle = Rotation2d.fromDegrees(-gyro.getAngle());
  mpose = od.update(gyroAngle, 
    talonLeftDrive.getSelectedSensorPosition()  , 
    talonRightDrive.getSelectedSensorPosition() 
    ) ;
    SmartDashboard.putString("MPOSE:", mpose.toString()) ;

}

public boolean do_mpose( double destX, double destY) {
  do_mpose_update() ;
  Translation2d t2d = mpose.getTranslation();
  //double theX = t2d.getX() ;
  //double theY = t2d.getY() ;
  //SmartDashboard.putNumber("THEX", theX) ;
 // double destX = -5800 ;
  //double destY = -14100 ;
  //destX = 10000 ;   destY = 0 ;
  double Xdiff = destX - t2d.getX() ;
  double Ydiff = destY - t2d.getY() ;
  double Tangle = Math.abs(Math.toDegrees(Math.atan(Xdiff/Ydiff))) ;
  double NTangle = 0 ;
  if (Xdiff >= 0 && Ydiff >= 0) {
    NTangle = 90 - Tangle ;
  } else if (Xdiff >= 0 &&  Ydiff <= 0){
    NTangle = -(90 - Tangle) ;
  } else if (Xdiff <= 0 && Ydiff <= 0){
    NTangle = - (Tangle + 90) ;
  } else if (Xdiff <= 0 && Ydiff >= 0){
    NTangle = 90 + Tangle ;
  } 
  
  double distance = Math.sqrt( (Xdiff * Xdiff) + (Ydiff * Ydiff)) ;
  SmartDashboard.putString("DIFFS:", "X:" + Math.round(Xdiff) 
    + " Y:" + Math.round(Ydiff) + " THETA:" + Math.round(Tangle) 
    + " NewTH:" + Math.round(NTangle) + " DISTANCE:" + Math.round(distance)
     );

  //if (joystick.getRawButton(10) == false) { return false ;}

    // turn to the angle, and wait until we're at it.
  if (mpose_turn_1_done == false) {
    mpose_turn_1_done = gyro_turn_to(-NTangle);
    return false ;
  }

  SmartDashboard.putString("MPOSE STATUS", "first turn finished.");
  

  if (distance < 2000) {
    SmartDashboard.putString("MPOSE STATUS", "arrived at: X:" + destX + " Y:" + destY);
    drive.arcadeDrive(0, 0);
    return true ;
  }

  drive_straight(-NTangle, .5);
  return false ;

//    if (joystick.getRawButton(9)) { drive.arcadeDrive(answer, 0); }

    
}

// -------------------------------------------------------------
// -------------------------------------------------------------

public void zero_everything() {
  tiltEncoder.reset();
  dialEncoder.reset() ;
  wheelElevatorEncoder.reset();
  zeroControl.setBoolean(false);
  talonLeftDrive.setSelectedSensorPosition(0,0,30);
  talonRightDrive.setSelectedSensorPosition(0,0,30);
}
// -------------------------------------------------------------


}
