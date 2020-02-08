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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.* ;
import edu.wpi.first.wpilibj.SpeedControllerGroup ;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.SpeedController ;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.can.* ;
import edu.wpi.first.wpilibj.AnalogInput ;
import java.util.Map;
import edu.wpi.first.wpilibj.controller.*;
import javax.print.attribute.standard.JobPrioritySupported;

import com.ctre.phoenix.motorcontrol.* ;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.Encoder;



public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final String blahblahblah = "blah auton";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();




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
  

  // CONTROLLERS:
  XboxController xbox = new XboxController(0);
  Joystick joystick = new Joystick(1);


  // ENCODERS:
  Encoder dialEncoder ;
  Encoder tiltEncoder ;

  // global'ish Variables used in routines below:
  Timer myClock = new Timer();
  double throttle; 
  double turn_throttle;
  DifferentialDrive drive ;

  /// DIAL
  boolean move_dial_one_notch = false ;
  double dial_notch_begin_encoder_value = 0 ;
  boolean dial_is_moving_one_notch = false ;
  AnalogInput beam1 = new AnalogInput(0) ;
  AnalogInput beam2 = new AnalogInput(1) ;
  boolean dial_by_sensor_is_on =false ;

  // TILT:
  String tiltGoal = "floor" ;
  double tiltSpeed = 100 ;

  //network tables:
  private NetworkTableEntry intakeControl;
  private NetworkTableEntry shooterControl;
  private NetworkTableEntry shooterControlLow;
  private NetworkTableEntry dialNotchControl ;

  private NetworkTableEntry tiltFloorControl ;
  private NetworkTableEntry tiltLowControl ;
  private NetworkTableEntry tiltHighControl ;

  private NetworkTableEntry beamIntakeControl ;


  @Override
  public void robotInit() {
    myClock.start();

    // define autonomous options:
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    m_chooser.addOption("blah", blahblahblah);
    SmartDashboard.putData("Auto choices", m_chooser);


    // motors:
    shooterWheel1 = new CANSparkMax(10, MotorType.kBrushless);
    shooterWheel2 = new CANSparkMax(11, MotorType.kBrushless);
    talonLeftDrive = new WPI_TalonSRX(1);
    talonRightDrive = new WPI_TalonSRX(2);
    leftDriveFollow = new WPI_VictorSPX(25);
    rightDriveFollow = new WPI_VictorSPX(24);
    dialMotor = new WPI_VictorSPX(21);
    tiltMotor = new WPI_VictorSPX(22);
    intakeMotor = new WPI_VictorSPX(23);

    setupTestButtons() ; // test mode buttons on shuffleboard.

    // encoders:
    tiltEncoder = new Encoder (0,1);
    dialEncoder = new Encoder (2,3) ;

    // driving:
    talonLeftDrive.configFactoryDefault() ;
    talonRightDrive.configFactoryDefault() ;
    leftDriveFollow.configFactoryDefault();  
    leftDriveFollow.follow(talonLeftDrive);
    rightDriveFollow.configFactoryDefault();  
    rightDriveFollow.follow(talonRightDrive);
    drive = new DifferentialDrive(talonLeftDrive,talonRightDrive);

    


  }


  @Override
  public void robotPeriodic() {
    all_values_to_dashboard();

  }


  @Override
  public void autonomousInit() {
      tiltEncoder.reset();
      dialEncoder.reset() ;

    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        SmartDashboard.putString("custome code:", "custom");
        break;

        case kDefaultAuto:
        default:
        // Put default auto code here
        SmartDashboard.putString("custome code:", "default");
        break;

        case blahblahblah:
        SmartDashboard.putString("custome code:", "blah");
        break ;

      }

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    normal_driving();
    // more stuff here.

  }

  // -------------------------------------------------------------
  // -------------------------------------------------------------
  // -------------------------------------------------------------

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

    // SHOOTER WHEELS
    if (intakeControl.getBoolean(false)){
      run_shooter_wheels_for_intake() ;
    } else if (shooterControl.getBoolean(false)){
      run_shooter_wheels_for_shooting_out_high() ;
    } else if (shooterControlLow.getBoolean(false)){
      run_shooter_wheels_for_shooting_out_low() ;
    } else {
      stop_shooter_wheels();
    }


    // DIAL 
    if (dialNotchControl.getBoolean(false))  {
      move_dial_one_notch = true ;
      dialNotchControl.setBoolean(false) ;
    }
    dial_intake_turning();
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
    } 
    do_tilting();


  }
  // -------------------------------------------------------------
  // -------------------------------------------------------------
  // TILT CONTROL
  public void do_tilting() {

    double height = tiltEncoder.getDistance();
    double low_goal = 250 ;
    double high_goal = 400 ;

    if (tiltGoal == "floor"){
      if (height < 25 && tiltMotor.get() == 0) { return ;}
      if (height < 25) {
        System.out.println("do_tilting, to floor: current tilt height: " + height + ", setting to zero.") ;
        tiltMotor.set(0);
        return ;
      }
      // means the speed is above 25.  lower it.
      tiltMotor.set(-.6);
      return ;
    }


    if (tiltGoal == "low") {
      if (  Math.abs( low_goal - height) < 30) {
        tiltMotor.set(0);
        return ;
      }
      if ( height < low_goal) {
        tiltMotor.set(.7) ;
        return ;
      }
      if ( height > low_goal) {
        tiltMotor.set(-.6);
        return ;
      }
    }
    
    if (tiltGoal == "high") {
      if (  Math.abs( high_goal - height) < 30) {
        tiltMotor.set(0);
        return ;
      }
      if ( height < high_goal) {
        tiltMotor.set(.7) ;
        return ;
      }
      if ( height > high_goal) {
        tiltMotor.set(-.6);
        return ;
      }
    }
  }


  // -------------------------------------------------------------
  //  dial around one ball at a time.
  public void dial_intake_turning() {
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
    if ( beam1.getAverageVoltage() < .1) {
      dialMotor.set(.6) ;
      return;
    }
    if (beam1.getAverageVoltage() > .1 && beam2.getAverageVoltage() > .1){
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
    shooterWheel1.set(.5);
    shooterWheel2.set(-.5) ;
  }
  public void run_shooter_wheels_for_shooting_out_high() {
    shooterWheel1.set(-1);
    shooterWheel2.set(1) ;
  }
  public void run_shooter_wheels_for_shooting_out_low() {
    shooterWheel1.set(-.3);
    shooterWheel2.set(.3) ;
  }
  // -------------------------------------------------------------
  // -------------------------------------------------------------


  //   -------------------------------------------------------------
  public void all_values_to_dashboard() {
    SmartDashboard.putNumber("TILT:",tiltEncoder.getDistance());
    SmartDashboard.putNumber("TILT-SPEED:",tiltEncoder.getRate());
    SmartDashboard.putNumber("DIAL:",dialEncoder.getDistance());
    SmartDashboard.putNumber("beam-1:", beam1.getAverageVoltage()) ;
    SmartDashboard.putNumber("beam-2:", beam2.getAverageVoltage()) ;

  }

  // -------------------------------------------------------------
  public void setupTestButtons() {
    intakeControl = Shuffleboard.getTab("controls")
    .add("intake in",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(6, 1)
    .withSize(3, 2)
    .getEntry();

    shooterControl = Shuffleboard.getTab("controls")
    .add("shooter wheels out fast",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(6, 3)
    .withSize(3, 2)
    .getEntry();

    shooterControlLow = Shuffleboard.getTab("controls")
    .add("shooter wheels out LOW",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(6, 5)
    .withSize(3, 2)
    .getEntry();

    dialNotchControl = Shuffleboard.getTab("controls")
    .add("move-dial one click",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(6, 7)
    .withSize(3, 2)
    .getEntry();

    tiltFloorControl = Shuffleboard.getTab("controls")
    .add("tilt-to-floor",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(10, 1)
    .withSize(3, 2)
    .getEntry();

    tiltLowControl = Shuffleboard.getTab("controls")
    .add("tilt-LOW",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(10, 3)
    .withSize(3, 2)
    .getEntry();

    tiltHighControl = Shuffleboard.getTab("controls")
    .add("tilt-HIGH",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(10, 5)
    .withSize(3, 2)
    .getEntry();

    beamIntakeControl = Shuffleboard.getTab("controls")
    .add("BEAM intake",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(15, 1)
    .withSize(3, 2)
    .getEntry();

    

    }

  // -------------------------------------------------------------
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
    drive.arcadeDrive(joyy * throttle , joyz * turn_throttle );
  }

// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------


}
