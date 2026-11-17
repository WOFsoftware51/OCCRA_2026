// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final CANBus kCANBus = new CANBus("rio");

  private final TalonFX leftLeader = new TalonFX(1, kCANBus);
  private final TalonFX leftFollower = new TalonFX(2, kCANBus);
  private final TalonFX rightLeader = new TalonFX(3, kCANBus);
  private final TalonFX rightFollower = new TalonFX(4, kCANBus);
  private final TalonFX intake = new TalonFX(10, kCANBus);
  private final TalonFX transferBottom = new TalonFX(11, kCANBus);
  private final TalonFX transferTop = new TalonFX(12, kCANBus);
  private final TalonFX shooterBottom = new TalonFX(13, kCANBus);
  private final TalonFX shooterTop = new TalonFX(14, kCANBus);

  private final DutyCycleOut intakeRequest = new DutyCycleOut(0);
  private final DutyCycleOut leftOut = new DutyCycleOut(0);
  private final DutyCycleOut rightOut = new DutyCycleOut(0);

  private final XboxController driver = new XboxController(0);
  private final XboxController operator = new XboxController(1);

  private int printCount = 0;
  private static double kIntakeSpeed = 0.40;
  private static final Double kSpitSpeed = -0.70*0.25;
  private static final double kTopSpeed = 850;
  private static final double kBottomSpeed = 3200;
  private static double kTransferBottomSpeed = -1.0;
  private static final double kTransferTopSpeed = -1.0;
  private final DutyCycleOut topRequest = new DutyCycleOut(0);
  private final DutyCycleOut bottomRequest = new DutyCycleOut(0);
  private final DutyCycleOut transferBottomRequest = new DutyCycleOut(0);
  private final DutyCycleOut transferTopRequest = new DutyCycleOut(0);
  private final DutyCycleOut spitRequest = new DutyCycleOut(0);

  // Auto timing
  private final Timer driveTimer = new Timer();
  private boolean driving = false;
  private static final double kAutoSpinUpSeconds = 1.5;
  private static final double kAutoFeedSeconds = 1.0;
  private static final double kAutoDriveSeconds = 0.5;
  private static final double kAutoDriveSpeed = 0.3;

  private final Timer autoTimer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    /* Configure the devices */
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intake.getConfigurator().apply(intakeConfig);

    TalonFXConfiguration SpitConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intake.getConfigurator().apply(SpitConfig);

    var leftConfiguration = new TalonFXConfiguration();
    var rightConfiguration = new TalonFXConfiguration();
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    TalonFXConfiguration transferConfig = new TalonFXConfiguration();
    transferConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // transferConfig.MotorOutput.Inverted =
    // InvertedValue.CounterClockwise_Positive;
    transferBottom.getConfigurator().apply(transferConfig);
    transferTop.getConfigurator().apply(transferConfig);

    shooterTop.getConfigurator().apply(config);
    shooterBottom.getConfigurator().apply(config);

    /*
     * User can optionally change the configs or leave it alone to perform a factory
     * default
     */
    leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    //not mine ?
    leftLeader.getConfigurator().apply(leftConfiguration);
    leftFollower.getConfigurator().apply(leftConfiguration);
    rightLeader.getConfigurator().apply(rightConfiguration);
    rightFollower.getConfigurator().apply(rightConfiguration);

    /* Set up followers to follow leaders */
    leftFollower.setControl(new Follower(leftLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    rightFollower.setControl(new Follower(rightLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    //not mine
    leftLeader.setSafetyEnabled(true);
    rightLeader.setSafetyEnabled(true);
    //not mine
  }

  @Override
  public void robotPeriodic() {
    if (++printCount >= 10) {
      printCount = 0;
      System.out.println("Left out: " + leftLeader.get());
      System.out.println("Right out: " + rightLeader.get());
      System.out.println("Left Pos: " + leftLeader.getPosition());
      System.out.println("Right Pos: " + rightLeader.getPosition());
      //notmine
    }
  }

  @Override
  public void autonomousInit() {
    autoTimer.restart();
  }

  @Override
  public void autonomousPeriodic() {
    double t = autoTimer.get();
    double feedEnd = kAutoSpinUpSeconds + kAutoFeedSeconds;
    double driveEnd = feedEnd + kAutoDriveSeconds;
    boolean shootTwo = false;
    boolean intaking = false;


        // if(shootTwo = false){
        //   shootTwo = true;
        //   setTransfer(true);
        //   intake.set(0);
        //   leftLeader.set(0);
        //   rightLeader.set(0);
        // }else if(shootTwo = true){
        //   shootTwo = true;
        //   setTransfer(true);
        //   intake.set(0.4);
        //   leftLeader.set(0);
        //   rightLeader.set(0);
        // }else if(intake.getPosition().getValueAsDouble()>50){
        //   shootTwo = true;
        //   setTransfer(true);
        //   intake.set(0.4);
        //   leftLeader.set(0.2);
        //   rightLeader.set(0.2);
        // }else{
        //   shootTwo = false;
        //   setTransfer(false);
        //   intake.set(0.0);
        //   leftLeader.set(0);
        //   leftLeader.set(0.0);
        //   rightLeader.set(0.0);
        // }
        if(shootTwo = false){
          shootTwo = true;
          intake.set(0);
          setTransfer(false);
          leftLeader.set(0);
          rightLeader.set(0);
        }else if (shootTwo = true){
          shootTwo = true;
          intake.set(0);
          setTransfer(true);
          leftLeader.set(0);
          rightLeader.set(0);
        }else if(leftLeader.getPosition().getValueAsDouble()<12 && rightLeader.getPosition().getValueAsDouble()<12){
          leftLeader.set(0.1);
          rightLeader.set(0.1);
          shootTwo = true;
          intake.set(0.4);
          setTransfer(true);
        }else if(leftLeader.getPosition().getValueAsDouble()>12 && rightLeader.getPosition().getValueAsDouble()>12){
          leftLeader.set(0.0);
          rightLeader.set(0.0);
          intake.set(0.4);
          shootTwo = true;
        }else{
          shootTwo = false;
          intake.set(0);
          setTransfer(false);
          leftLeader.set(0);
          rightLeader.set(0);
        }
        // if (t < kAutoSpinUpSeconds) {
        //   shootTwo = true;
        //   setTransfer(false);
        //   leftOut.Output = 0.0;
        //   rightOut.Output = 0.0;
        // } else if (t < feedEnd) {
        //   shootTwo = true;
        //   setTransfer(true);
        //   leftOut.Output = 0.0;
        //   rightOut.Output = 0.0;
        // } else if (t < driveEnd) {
        //   // Drive forward for 2 seconds
        //   shootTwo = false;
        //   setTransfer(false);
        //   leftFollower.set(0.3);
        //   rightFollower.set(0.3);
        // } else {
        //   shootTwo = false;
        //   setTransfer(false);
        //   leftFollower.set(0.0);
        //   rightFollower.set(0.0);
        // }
      
        if(shooterTop.getVelocity().getValueAsDouble()*60.0 < kTopSpeed-300)
        {
          shooterTop.setControl(topRequest.withOutput(1));
        }
        else if(shooterTop.getVelocity().getValueAsDouble()*60.0 < kTopSpeed)
        {
          shooterTop.setControl(topRequest.withOutput(0.3));
        }
        else
        {
          shooterTop.setControl(topRequest.withOutput(0));
        }
    
    
         if(shooterBottom.getVelocity().getValueAsDouble()*60.0 < kBottomSpeed-300)
        {
          shooterBottom.setControl(bottomRequest.withOutput(1));
        }
        else if(shooterBottom.getVelocity().getValueAsDouble()*60.0 < kBottomSpeed)
        {
          shooterBottom.setControl(bottomRequest.withOutput(0.5));
        }
        else
        {
          shooterBottom.setControl(bottomRequest.withOutput(0));
        }
    
        // if(leftLeader.getPosition().getValueAsDouble()<12){//22
        //   leftLeader.set(0.1);
        // } else{
        //   leftLeader.set(0);
        //   intaking = false;
        // }
        // if(rightLeader.getPosition().getValueAsDouble()<12){ //18
        //   rightLeader.set(0.1);
        // } else{
        //   rightLeader.set(0);
        //   intaking = false;
        // }
    
      
      }
    
     
    
      @Override
  public void teleopInit() {
    setTransfer(false);
    driveForward(false);
  }

  private void driveForward(boolean on) {
    // if (on && !driving) {
    // driveTimer.restart(); // first loop of a new drive: start the clock
    // driving = true;
    // } else if (!on) {
    // driving = false;
    // }
    // leftOut.Output = 0.5;
    // rightOut.Output = 0.5;
    // if (on && driveTimer.get() < kAutoDriveSeconds) {

    // } else {
    // leftOut.Output = 0;
    // rightOut.Output = 0;
    // }
    leftLeader.setPosition(0);
    rightLeader.setPosition(0);
    shooterTop.setPosition(0);
    shooterBottom.setPosition(0);
    intake.setPosition(0);
  }

  private void setTransfer(boolean on) {
    transferBottom.setControl(transferBottomRequest.withOutput(on ? kTransferBottomSpeed : 0));
    transferTop.setControl(transferTopRequest.withOutput(on ? kTransferTopSpeed : 0));
  }

  @Override
  public void teleopPeriodic() {
    /* Get forward and rotational throttle from driver */
    /* invert the driver Y because forward Y is negative */
    double fwd = -driver.getLeftY();
    double rot = driver.getRightX();
    rot = rot * 0.3;
    fwd = fwd * 0.3;
    /* Set output to control frames */
    leftOut.Output = fwd + rot;
    rightOut.Output = fwd - rot;
    /* And set them to the motors */
    if (!driver.getAButton()) {
      leftLeader.setControl(leftOut);
      rightLeader.setControl(rightOut);
    }
    boolean shoot = operator.getRightBumperButton();
    boolean shootTwo = operator.getAButton();
    //change back to A button if it doesn't work
    boolean spit = operator.getLeftBumperButton();
    boolean humanLoad = operator.getLeftTriggerAxis() > 0.5;
    boolean intaking = operator.getRightTriggerAxis() > 0.5;
    boolean maxSpeed = false;

    // Intake
    double intakeOut = 0;
    double transferBottomOut = 0;
    double transferTopOut = 0;

    if(shooterTop.getVelocity().getValueAsDouble()*60.0 > kTopSpeed-100 & shooterBottom.getVelocity().getValueAsDouble()*60.0 > kBottomSpeed-100)
    {
      maxSpeed = true;
    }

    if (spit) {
      intakeOut = -kSpitSpeed;
    } else if (humanLoad) {
      intakeOut = -0.2;
    } else if (intaking) {
      intakeOut = kIntakeSpeed;
    } else if (shoot && maxSpeed) {
      intakeOut = kIntakeSpeed;
    } else {
      intakeOut = 0;
    }

    if(shootTwo && shooterTop.getVelocity().getValueAsDouble()*60.0 < kTopSpeed-300)
    {
      shooterTop.setControl(topRequest.withOutput(1));
    }
    else if(shootTwo && shooterTop.getVelocity().getValueAsDouble()*60.0 < kTopSpeed)
    {
      shooterTop.setControl(topRequest.withOutput(0.3));
    }
    else
    {
      shooterTop.setControl(topRequest.withOutput(0));
    }


     if(shootTwo && shooterBottom.getVelocity().getValueAsDouble()*60.0 < kBottomSpeed-200)
    {
      shooterBottom.setControl(bottomRequest.withOutput(1));
    }
    else if(shootTwo && shooterBottom.getVelocity().getValueAsDouble()*60.0 < kBottomSpeed)
    {
      shooterBottom.setControl(bottomRequest.withOutput(0.5));
    }
    else
    {
      shooterBottom.setControl(bottomRequest.withOutput(0));
    }


    // TransferBottom
    if (spit) {
      transferBottomOut = -kTransferBottomSpeed*0.25;
    } else if (humanLoad) {
      transferBottomOut = kTransferBottomSpeed;
    } else if (intaking) {
      transferBottomOut = kTransferBottomSpeed;
    } else if (shoot && maxSpeed) {
      transferBottomOut = kTransferBottomSpeed;
    } else {
      transferBottomOut = 0;
    }

    // TransferTop
    if (spit) {
      transferTopOut = -kTransferTopSpeed*0.25;
    } else if (humanLoad) {
      transferTopOut = kTransferTopSpeed;
    } else if (intaking) {
      transferTopOut = 0;
    } else if (shoot && maxSpeed) {
      transferTopOut = kTransferTopSpeed;
    } else {
      transferTopOut = 0;
    }

    intake.setControl(intakeRequest.withOutput(intakeOut));
    transferTop.setControl(transferTopRequest.withOutput(transferTopOut));
    transferBottom.setControl(transferBottomRequest.withOutput(transferBottomOut));

    leftLeader.getPosition().getValueAsDouble();
    rightLeader.getPosition().getValueAsDouble();

    shooterTop.getPosition().getValueAsDouble();
    shooterBottom.getPosition().getValueAsDouble();

    intake.getPosition().getValueAsDouble();
    
    SmartDashboard.putNumber("Shooter Top Velocity", shooterTop.getVelocity().getValueAsDouble()*60.0);
    SmartDashboard.putNumber("Shooter Bottom Velocity", shooterBottom.getVelocity().getValueAsDouble()*60.0);
    SmartDashboard.putBoolean("MaxSpeed", maxSpeed);
    SmartDashboard.putNumber("leftPosition", leftLeader.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("rightPosition", rightLeader.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("TopPosition", shooterTop.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("BottomPosition", shooterBottom.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("IntakePosition", intake.getPosition().getValueAsDouble());
    
  }
 

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    /* Zero out controls so we aren't just relying on the enable frame */
    leftOut.Output = 0;
    rightOut.Output = 0;
    leftLeader.setControl(leftOut);
    rightLeader.setControl(rightOut);
    shooterTop.setControl(topRequest.withOutput(0));
    shooterBottom.setControl(bottomRequest.withOutput(0));
    transferBottom.setControl(transferBottomRequest.withOutput(0));
    transferTop.setControl(transferTopRequest.withOutput(0));
    intake.setControl(intakeRequest.withOutput(0));
    intake.setControl(spitRequest.withOutput(0));
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
