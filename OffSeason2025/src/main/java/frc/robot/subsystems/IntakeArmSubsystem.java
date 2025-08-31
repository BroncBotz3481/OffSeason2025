package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeArmSubsystem extends SubsystemBase
{

  private final SparkMax                   armMotor         = new SparkMax(1, MotorType.kBrushless);
  //  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig = new SmartMotorControllerTelemetryConfig()
//          .withMechanismPosition()
//          .withRotorPosition()
//          .withMechanismLowerLimit()
//          .withMechanismUpperLimit();
  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(15, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))//PID?
      .withSoftLimit(Degrees.of(-5), Degrees.of(125))
      .withGearing(gearing(gearbox(1, 35)))//Gearbox?
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
//      .withSpecificTelemetry("ArmMotor", motorTelemetryConfig)
      .withStatorCurrentLimit(Amps.of(40))
//      .withVoltageCompensation(Volts.of(12))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ArmFeedforward(0, 0, 0, 0))//Feedforward?
      .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController       motor            = new SparkWrapper(armMotor,
                                                                               DCMotor.getNeo550(1),
                                                                               motorConfig);
  private final MechanismPositionConfig    robotToMechanism = new MechanismPositionConfig()
      .withMaxRobotHeight(Inches.of(35.5))
      .withMaxRobotLength(Inches.of(28))//without the bumper->28"
      .withRelativePosition(new Translation3d(Meters.of(0.06078), Meters.of(0.17947), Meters.of(0.1083)));
      //x/y/z-according to onshape's axis, x is distance of front and back;z is the height

  private       ArmConfig m_config = new ArmConfig(motor)
      .withLength(Inches.of(15.59))
      .withHardLimit(Degrees.of(-10), Degrees.of(150))//need update
      .withTelemetry("Arm", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(8))
      .withStartingPosition(Degrees.of(0))//match with the default cmd in robotcontainer
      .withHorizontalZero(Degrees.of(0));
    //.withMechanismPositionConfig(robotToMechanism);
  private final Arm       arm      = new Arm(m_config);

  public IntakeArmSubsystem()
  {
  }

  public void periodic()
  {
    arm.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    arm.simIterate();
  }

  public Command armCmd(double dutycycle)
  {
    return arm.set(dutycycle);
  }

  public Command sysId()
  {
    return arm.sysId(Volts.of(5), Volts.of(2).per(Second), Second.of(30));
  }

  public Command setAngle(Angle angle)
  {
    return arm.setAngle(angle);
  }

  
}