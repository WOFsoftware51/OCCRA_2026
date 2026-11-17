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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

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

  // ---------------------------------------------------------------------------
  // Autonomous
  // ---------------------------------------------------------------------------
  // Drive distances are in drive-motor rotations (what leftLeader.getPosition()
  // reports). Measure on the field and tune these.
  private static final double kAutoSpinUpTimeout = 2.0; // max seconds to wait for shooter speed
  private static final double kAutoFeedSeconds = 1.0; // seconds to run the feed per shot
  private static final double kAutoDriveSpeed = 0.2; // duty cycle while driving
  private static final double kAutoTaxiRotations = 12.0; // distance to leave the start zone
  private static final double kAutoPickupRotations = 12.0; // distance to the second ball
  private static final double kAutoDriveTimeout = 4.0; // safety stop for any drive step

  /** The routines selectable from the dashboard ("Auto Routine" chooser). */
  private enum AutoRoutine {
    DO_NOTHING,
    DRIVE_FORWARD,
    SHOOT_PRELOAD,
    SHOOT_THEN_DRIVE,
    TWO_BALL
  }

  /**
   * One step of an auto routine. Called every loop with the seconds since the
   * step started; return true when the step is finished.
   */
  @FunctionalInterface
  private interface AutoStep {
    boolean run(double stepTime);
  }

  private final SendableChooser<AutoRoutine> autoChooser = new SendableChooser<>();
  private final Timer stepTimer = new Timer();
  private List<AutoStep> autoSteps = List.of();
  private int autoStepIndex = 0;

  // Outputs requested by the current auto step; applied once per loop.
  private double autoLeft = 0;
  private double autoRight = 0;
  private boolean autoShooter = false;
  private boolean autoIntake = false; // intake + bottom transfer (hold ball)
  private boolean autoFeed = false; // intake + both transfers (shoot ball)

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

    autoChooser.setDefaultOption("Shoot preload", AutoRoutine.SHOOT_PRELOAD);
    autoChooser.addOption("Do nothing", AutoRoutine.DO_NOTHING);
    autoChooser.addOption("Drive forward", AutoRoutine.DRIVE_FORWARD);
    autoChooser.addOption("Shoot then drive", AutoRoutine.SHOOT_THEN_DRIVE);
    autoChooser.addOption("Two ball", AutoRoutine.TWO_BALL);
    SmartDashboard.putData("Auto Routine", autoChooser);
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
    leftLeader.setPosition(0);
    rightLeader.setPosition(0);

    AutoRoutine routine = autoChooser.getSelected();
    autoSteps = buildRoutine(routine == null ? AutoRoutine.DO_NOTHING : routine);
    autoStepIndex = 0;
    stepTimer.restart();
    SmartDashboard.putString("Auto Selected", String.valueOf(routine));
  }

  @Override
  public void autonomousPeriodic() {
    // Everything defaults to off; the active step turns on what it needs.
    autoLeft = 0;
    autoRight = 0;
    autoShooter = false;
    autoIntake = false;
    autoFeed = false;

    if (autoStepIndex < autoSteps.size()) {
      boolean done = autoSteps.get(autoStepIndex).run(stepTimer.get());
      if (done) {
        autoStepIndex++;
        stepTimer.restart();
      }
    }

    applyAutoOutputs();
    SmartDashboard.putNumber("Auto Step", autoStepIndex);
  }

  private List<AutoStep> buildRoutine(AutoRoutine routine) {
    switch (routine) {
      case DRIVE_FORWARD:
        return List.of(
            driveTo(kAutoTaxiRotations, false, false));
      case SHOOT_PRELOAD:
        return List.of(
            spinUp(),
            feed());
      case SHOOT_THEN_DRIVE:
        return List.of(
            spinUp(),
            feed(),
            driveTo(kAutoTaxiRotations, false, false));
      case TWO_BALL:
        return List.of(
            spinUp(),
            feed(),
            driveTo(kAutoPickupRotations, true, false), // drive out intaking
            driveTo(0, true, true), // come back, spin shooter up on the way
            spinUp(),
            feed());
      case DO_NOTHING:
      default:
        return List.of();
    }
  }

  // ----- Auto step builders -----

  /** Spin the shooter until it is at speed (or the timeout passes). */
  private AutoStep spinUp() {
    return t -> {
      autoShooter = true;
      return shooterAtSpeed() || t >= kAutoSpinUpTimeout;
    };
  }

  /** Keep the shooter spinning and feed balls into it for kAutoFeedSeconds. */
  private AutoStep feed() {
    return t -> {
      autoShooter = true;
      autoFeed = true;
      return t >= kAutoFeedSeconds;
    };
  }

  /**
   * Drive straight until the average drive position reaches targetRotations
   * (measured from where auto started), or kAutoDriveTimeout passes.
   */
  private AutoStep driveTo(double targetRotations, boolean intaking, boolean shooterOn) {
    return t -> {
      double error = targetRotations - averageDrivePosition();
      autoIntake = intaking;
      autoShooter = shooterOn;
      if (Math.abs(error) < 0.25 || t >= kAutoDriveTimeout) {
        return true;
      }
      double speed = Math.copySign(kAutoDriveSpeed, error);
      autoLeft = speed;
      autoRight = speed;
      return false;
    };
  }

  /** Wait in place (everything off except the shooter if requested). */
  @SuppressWarnings("unused")
  private AutoStep waitSeconds(double seconds, boolean shooterOn) {
    return t -> {
      autoShooter = shooterOn;
      return t >= seconds;
    };
  }

  // ----- Auto helpers -----

  private double averageDrivePosition() {
    return (leftLeader.getPosition().getValueAsDouble()
        + rightLeader.getPosition().getValueAsDouble()) / 2.0;
  }

  private double shooterTopRpm() {
    return shooterTop.getVelocity().getValueAsDouble() * 60.0;
  }

  private double shooterBottomRpm() {
    return shooterBottom.getVelocity().getValueAsDouble() * 60.0;
  }

  private boolean shooterAtSpeed() {
    return shooterTopRpm() > kTopSpeed - 100 && shooterBottomRpm() > kBottomSpeed - 100;
  }

  /** Same bang-bang speed control the old auto used. */
  private void runShooter(boolean on) {
    double top = 0;
    double bottom = 0;
    if (on) {
      double topRpm = shooterTopRpm();
      if (topRpm < kTopSpeed - 300) {
        top = 1.0;
      } else if (topRpm < kTopSpeed) {
        top = 0.3;
      }
      double bottomRpm = shooterBottomRpm();
      if (bottomRpm < kBottomSpeed - 300) {
        bottom = 1.0;
      } else if (bottomRpm < kBottomSpeed) {
        bottom = 0.5;
      }
    }
    shooterTop.setControl(topRequest.withOutput(top));
    shooterBottom.setControl(bottomRequest.withOutput(bottom));
  }

  private void applyAutoOutputs() {
    leftLeader.setControl(leftOut.withOutput(autoLeft));
    rightLeader.setControl(rightOut.withOutput(autoRight));
    runShooter(autoShooter);

    double intakeOut = (autoIntake || autoFeed) ? kIntakeSpeed : 0;
    double bottomOut = (autoIntake || autoFeed) ? kTransferBottomSpeed : 0;
    double topOut = autoFeed ? kTransferTopSpeed : 0; // top only runs when shooting
    intake.setControl(intakeRequest.withOutput(intakeOut));
    transferBottom.setControl(transferBottomRequest.withOutput(bottomOut));
    transferTop.setControl(transferTopRequest.withOutput(topOut));
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
