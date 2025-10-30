// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Set;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Setpoints;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GroundConstants;
import frc.robot.Constants.OutakeConstants;
import frc.robot.Setpoints.Arm.GroundIntake;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.gearing.Sprocket;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.SensorConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.simulation.Sensor;

public class IntakeArmSubsystem extends SubsystemBase {


  // Vendor motor controller object
  private SparkMax m_motor = new SparkMax(CanIDs.IntakeArm, MotorType.kBrushless);



  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(GroundConstants.kP, GroundConstants.kI, GroundConstants.kD, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
  .withSimClosedLoopController(GroundConstants.ksimP, GroundConstants.ksimI, GroundConstants.ksimD, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
  // Feedforward Constants
  .withFeedforward(new ArmFeedforward(GroundConstants.kS, GroundConstants.kG, GroundConstants.kV))
  .withSimFeedforward(new ArmFeedforward(GroundConstants.ksimS, GroundConstants.ksimG, GroundConstants.ksimV))
  // Telemetry name and verbosity level
  .withTelemetry("IntakeArm", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to the gearbox attached to your motor.
  .withGearing(new MechanismGearing(new GearBox(GroundConstants.gearbox), new Sprocket(GroundConstants.sprocket)))
  // Motor properties to prevent over currenting.
  .withMotorInverted(false)
  .withIdleMode(MotorMode.BRAKE)
  .withStartingPosition(GroundConstants.kStartingPose)
  .withStatorCurrentLimit(GroundConstants.statorCurrentLimit)
  .withClosedLoopRampRate(Seconds.of(0.25))
  .withOpenLoopRampRate(Seconds.of(0.25))
  .withExternalEncoder(m_motor.getAbsoluteEncoder())
  .withExternalEncoderInverted(true)
  .withUseExternalFeedbackEncoder(true)
  ;
  
  //.withZeroOffset(Degrees.of(0));-same thing as ArmConfig.withHorizontalZero()
  
  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController = new SparkWrapper(m_motor, DCMotor.getNeoVortex(1), smcConfig);
 
  private ArmConfig armCfg = new ArmConfig(sparkSmartMotorController)
  // Soft limit is applied to the SmartMotorControllers PID
  .withSoftLimits(GroundConstants.softLimitMin, GroundConstants.softLimitMax)
  // Hard limit is applied to the simulation.
  .withHardLimit(GroundConstants.hardLimitMin, GroundConstants.hardLimitMax)

  // Length and mass of your arm for sim.
  .withLength(GroundConstants.armLength)
  .withMass(GroundConstants.armMass)
  // Telemetry name and verbosity for the arm.
  .withTelemetry("IntakeArm", TelemetryVerbosity.HIGH)
  
  .withHorizontalZero(GroundConstants.kHorizontalZero); 

  
  // Arm Mechanism
  private Arm m_Arm = new Arm(armCfg);

  /** Creates a new ExampleSubsystem. */
  public IntakeArmSubsystem() {
   sparkSmartMotorController.synchronizeRelativeEncoder(); 
  }
 

   /**
   * Set the angle of the arm.
   * @param angle Angle to go to.
   */
  public Command setAngle(double angle) {
    return m_Arm.setAngle(Degrees.of(angle));
    //.until(arm.isNear(angle, Degrees.of(OutakeConstants.kArmAllowableError)));
}

  /**
   * Move the arm up and down.
   * @param dutycycle [-1, 1] speed to set the arm too.
   */
  public Command set(double dutycycle) { return m_Arm.set(dutycycle);} //sparkMaxController.getDutyCycle();
  //DutyCycleEncoder m_encoderFR = new DutyCycleEncoder(0, 4.0, 2.0); 0-DIO channel 0

  /**
   * Run sysId on the {@link Arm}
   */
  public Command sysId() { 
    return m_Arm.sysId(Volts.of(4.5), Volts.of(0.5).per(Second), Seconds.of(4));
  }


  public boolean aroundAngle(double angle, double allowableError){
    return MathUtil.isNear(angle, m_Arm.getAngle().in(Degrees), allowableError);
  }
  
  public boolean aroundAngle(double angle){
    return MathUtil.isNear(angle, m_Arm.getAngle().in(Degrees), GroundConstants.kArmAllowableError.in(Degrees));
  }

  public boolean aroundGround(){ return aroundAngle(Setpoints.Arm.GroundIntake.intakeAngle);} //.until(()->intakeArmSubsystem.aroundAngle(0)));

  public boolean aroundPass(){ return aroundAngle(Setpoints.Arm.GroundIntake.passAngle);}

  public Command setGround() { 
    return setAngle(Setpoints.Arm.GroundIntake.intakeAngle);
  } // button triggers, intaking, stop, set pass, till around angle, pass

  public Command setPass() {
     return setAngle(Setpoints.Arm.GroundIntake.passAngle);
    } 

  public Command hold(){return setAngle(m_Arm.getAngle().in(Degrees));}

  public Command holdDefer(){
        return Commands.defer(()->hold(),Set.of(this));
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_Arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_Arm.simIterate();
  }

  public Angle getAngle() {
    return m_Arm.getAngle();
  }

}

/*
 * Things to be done
 * Tuning
 * LaserCan
 */