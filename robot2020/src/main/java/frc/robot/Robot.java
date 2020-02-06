/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
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

import java.util.Map;

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

  //network tables:
  private NetworkTableEntry intakeControl;
  private NetworkTableEntry shooterControl;
  private NetworkTableEntry shooterControlLow;
  private NetworkTableEntry dialNotchControl ;



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

    setupTestButtons() ;

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
  
  public void dosomething() {
    SmartDashboard.putString("sommething", "did something");
  }


  @Override
  public void robotPeriodic() {
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
    all_values_to_dashboard();
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        SmartDashboard.putString("custome code:", "custom");
        shooterWheel1.set(1) ;

        break;

        case kDefaultAuto:
        default:
        // Put default auto code here
        SmartDashboard.putString("custome code:", "default");
        shooterWheel1.set(0) ;
        shooterWheel2.set(0) ;

        break;

        case blahblahblah:
        shooterWheel1.set(.5) ;
        shooterWheel2.set(.5) ;
        SmartDashboard.putString("custome code:", "blah");
        break ;



      }

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    all_values_to_dashboard();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    all_values_to_dashboard();

    if (intakeControl.getBoolean(false)){
      run_shooter_wheels_for_intake() ;
    } else if (shooterControl.getBoolean(false)){
      run_shooter_wheels_for_shooting_out() ;
    } else if (shooterControlLow.getBoolean(false)){
      run_shooter_wheels_for_shooting_out_low() ;
    } else {
      stop_shooter_wheels();
    }

    if (dialNotchControl.getBoolean(false))  {
      move_dial_one_notch = true ;
      dialNotchControl.setBoolean(false) ;
    }
    dial_intake_turning();
  


  }

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

  public void stop_shooter_wheels(){
    shooterWheel1.set(0);
    shooterWheel2.set(0) ;
  }
  public void run_shooter_wheels_for_intake() {
    shooterWheel1.set(.5);
    shooterWheel2.set(-.5) ;
  }
  public void run_shooter_wheels_for_shooting_out() {
    shooterWheel1.set(-1);
    shooterWheel2.set(1) ;
  }
  public void run_shooter_wheels_for_shooting_out_low() {
    shooterWheel1.set(-.6);
    shooterWheel2.set(.6) ;
  }


  public void all_values_to_dashboard() {
    SmartDashboard.putNumber("TILT:",tiltEncoder.getDistance());
    SmartDashboard.putNumber("DIAL:",dialEncoder.getDistance());

  }

  public void setupTestButtons() {
    intakeControl = Shuffleboard.getTab("controls")
    .add("intake in",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(10, 1)
    .withSize(3, 2)
    .getEntry();

    shooterControl = Shuffleboard.getTab("controls")
    .add("shoot out",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(10, 3)
    .withSize(3, 2)
    .getEntry();

    shooterControlLow = Shuffleboard.getTab("controls")
    .add("shoot out LOW",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(10, 5)
    .withSize(3, 2)
    .getEntry();

    dialNotchControl = Shuffleboard.getTab("controls")
    .add("move-dial",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(10, 7)
    .withSize(3, 2)
    .getEntry();


    }



}
