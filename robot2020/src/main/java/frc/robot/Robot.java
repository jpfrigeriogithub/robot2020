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
  WPI_VictorSPX wheelSpinnerMotor ;
  WPI_VictorSPX wheelElevatorMotor ;
  CANSparkMax winchMotor ;

  // CONTROLLERS:
  XboxController xbox = new XboxController(1);
  Joystick joystick = new Joystick(0);


  // ENCODERS:
  Encoder dialEncoder ;
  Encoder tiltEncoder ;
  Encoder wheelElevatorEncoder ;

  // global'ish Variables used in routines below:
  Timer myClock = new Timer();
  double throttle; 
  double turn_throttle;
  DifferentialDrive drive ;
  boolean tab_intake_mode_on = false ;
  boolean tab_low_dump_mode_on = false ;


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

  //network tables:
  private NetworkTableEntry intakeControl;
  private NetworkTableEntry shooterControl;
  private NetworkTableEntry shooterControlLow;
  private NetworkTableEntry dialNotchControl ;

  private NetworkTableEntry tiltFloorControl ;
  private NetworkTableEntry tiltLowControl ;
  private NetworkTableEntry tiltHighControl ;

  private NetworkTableEntry beamIntakeControl ;
  private NetworkTableEntry allIntakeControl ;
  private NetworkTableEntry lowDumpControl ;
  private NetworkTableEntry spinDialOutControl ;
 
  private NetworkTableEntry colorWheelSpinControl ;
  private NetworkTableEntry colorWheelElevatorDownControl; 
  private NetworkTableEntry colorWheelElevatorUpControl ;
  private NetworkTableEntry colorWheelElevatorAtWheelControl;

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
    talonLeftDrive = new WPI_TalonSRX(2);
    talonRightDrive = new WPI_TalonSRX(1);
    leftDriveFollow = new WPI_VictorSPX(25);
    rightDriveFollow = new WPI_VictorSPX(24);
    dialMotor = new WPI_VictorSPX(21);
    tiltMotor = new WPI_VictorSPX(22);
    intakeMotor = new WPI_VictorSPX(23);

    wheelElevatorMotor = new WPI_VictorSPX(20);
    wheelSpinnerMotor = new WPI_VictorSPX(26);
    winchMotor = new CANSparkMax(12,MotorType.kBrushless) ;

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
    controlTabStuff() ;
    normal_driving();
    // more stuff here.

  }

  // -------------------------------------------------------------
  // -------------------------------------------------------------
  public void intake_rollers_on() {
    intakeMotor.set(.4) ;
  }
  public void intake_rollers_off() {
    intakeMotor.set(0) ;
  }
  // -------------------------------------------------------------
  public void run_all_intake() {
    tiltGoal = "floor" ;
    do_tilting();
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
    do_tilting();
    if (current_tilt_location != "low") { return ;} // wait until tilt is correct.
    // tilt is now OK, fire.
    run_shooter_wheels_for_shooting_out_low() ;
    dialMotor.set(-1) ;
  }
  public void stop_low_dump () {
    dialMotor.set(0);
    stop_shooter_wheels();
  }
  
  // --------------------------------------
  public void do_wheel_height() {
    double wheel_up_goal = 800 ;
    double wheel_atwheel_goal = 750 ;
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
    if (spinDialOutControl.getBoolean(false) 
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
    /*  if I want to run JUST the beams and dial:
    if (beamIntakeControl.getBoolean(false)) {
      dial_by_sensor_is_on = true ;
      dial_by_sensor() ;
    } else if (dial_by_sensor_is_on == true) {
      dial_by_sensor_is_on = false ;
      dialMotor.set(0) ;
    }
    */


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


    // Color Wheel

  }


  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

    //controlTabStuff();

  }
  // -------------------------------------------------------------
  // -------------------------------------------------------------
  // TILT CONTROL
  public void do_tilting() {

    double tiltPosition = tiltEncoder.getDistance();
    // Robot starts HIGH, at 0.  increase to get to low, increase more to get to floor.
    double low_goal = 150 ;
    double floor_goal = 400 ;

    current_tilt_location = "";

    if (tiltGoal == "high"){
      if (tiltPosition < 30)  {
        // then we are AT high.  so stop doing anything.
        current_tilt_location = "high" ;
        tiltMotor.set(0);
        return ;
      }
      // need to raise it to get to high
      tiltMotor.set(-.7);
      return ;
    }

    if (tiltGoal == "low") {
      if (  Math.abs( low_goal - tiltPosition) < 30) {
        current_tilt_location = "low";
        tiltMotor.set(0);
        return ;
      }
      // it's too low, raise it up:
      if ( tiltPosition < low_goal) {
        tiltMotor.set(.6) ;
        return ;
      }
      if ( tiltPosition > low_goal) {
        tiltMotor.set(-.7);
        return ;
      }
    }
    

    if (tiltGoal == "floor") {
      if (  Math.abs( floor_goal - tiltPosition) < 30) {
        current_tilt_location = "floor";
        tiltMotor.set(0);
        return ;
      }
      if ( tiltPosition < floor_goal) {
        tiltMotor.set(.6) ;
        return ;
      }
      if ( tiltPosition > floor_goal) {
        tiltMotor.set(-.7);
        return ;
      }
    }
  }


  // -------------------------------------------------------------
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
    SmartDashboard.putNumber("color-wheel-elevator:", wheelElevatorEncoder.getDistance());

  }

  // -------------------------------------------------------------
  public void setupTestButtons() {
    intakeControl = Shuffleboard.getTab("controls")
    .add("spin intake roller IN",false)
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

    dialNotchControl = Shuffleboard.getTab("controls")
    .add("move-dial one click",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(1, 6)
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


    /*
    beamIntakeControl = Shuffleboard.getTab("controls")
    .add("BEAM intake",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(7, 0)
    .withSize(3, 1)
    .getEntry(); */

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
    drive.arcadeDrive(-joyy * throttle , joyz * turn_throttle );
  }

// -------------------------------------------------------------
// -------------------------------------------------------------
// -------------------------------------------------------------


}
