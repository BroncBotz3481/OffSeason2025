package frc.robot.systems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.Constants.GroundConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.OutakeArmSubsystem;
import frc.robot.subsystems.OutakeRollerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import yams.mechanisms.config.SensorConfig;
import yams.motorcontrollers.simulation.Sensor;
import yams.motorcontrollers.simulation.SensorData;

;

public class LoadingSystem
{


  private IntakeArmSubsystem    m_intake;
  private OutakeArmSubsystem    m_outake;
  private ElevatorSubsystem     m_elevator;
  private SwerveSubsystem       m_swerve;
  private IntakeRollerSubsystem m_intakeRoller;
  private OutakeRollerSubsystem m_outakeRoller;
  private SwerveInputStream     m_swerveInputStream;
  private TargetingSystem       m_targetSystem;
  private LaserCan              m_endEffectorLaserCAN       = new LaserCan(19);
  private Sensor                m_endEffectorLaserCanSensor = new SensorConfig("EndEffectorLaserCAN")
      .withField("EndEffectorLaserCan", () -> m_endEffectorLaserCAN.getMeasurement().distance_mm, 1)
      .getSensor();
  private Sensor                m_intakeSensor;

  Angle loadingAngle        = Degrees.of(-5);
  Angle intakeTransferAngle = Degrees.of(95);
  Angle outakeTransferAngle = Degrees.of(105);
  boolean hasCoral = false;

  public LoadingSystem(IntakeArmSubsystem intake, ElevatorSubsystem elevator, SwerveSubsystem swerve,
                       OutakeArmSubsystem outake, IntakeRollerSubsystem intakeRoller,
                       OutakeRollerSubsystem outakeRoller,
                       TargetingSystem targeting, SwerveInputStream driveStream)
  {
    m_intake = intake;
    m_elevator = elevator;
    m_swerve = swerve;
    m_outake = outake;
    m_intakeRoller = intakeRoller;
    m_targetSystem = targeting;
    m_outakeRoller = outakeRoller;
    m_swerveInputStream = driveStream;
    m_intakeSensor = new SensorConfig("IntakeRoller")
        .withField("Current", () -> intakeRoller.getCurrent().in(Amps), 0.0)
        .getSensor();
    //When the coral intake sucks in coral and current drops, sets isLoaded true for 0.5
    new Trigger(()->m_intakeSensor.getAsDouble("Current")>= IntakeConstants.kCurrentLoaded).debounce(0.5).onTrue(Commands.runOnce(()->hasCoral=true));
    new Trigger(()->m_intakeRoller.getDutycycle() > 0.0).onTrue(Commands.runOnce(()->hasCoral=false));

    new Trigger(()->m_intake.aroundAngle(intakeTransferAngle.in(Degrees)) && m_outake.aroundAngle(outakeTransferAngle.in(Degrees)) && hasCoral)
        .onTrue(coralTransfer());
    if (Robot.isSimulation())
    {setupSimulation();}
  }

  


  public void setupSimulation()
  {
    new Trigger(() -> m_intake.getAngle().lte(Degrees.of(-5)))
        .onTrue(Commands.runOnce(() -> m_intakeSensor.getField("Current").set(SensorData.convert(30.0))));
    new Trigger(() -> m_intake.getAngle().isNear(intakeTransferAngle, Degrees.of(1)) &&
                      m_outake.getAngle().isNear(outakeTransferAngle, Degrees.of(1)) &&
                      m_intakeRoller.getDutycycle() > 0.0)
        .onTrue(Commands.runOnce(() -> {
          m_intakeSensor.getField("Current").set(SensorData.convert(0));
          m_endEffectorLaserCanSensor.getField("EndEffectorLaserCan").set(SensorData.convert(IntakeConstants.kLaserSenseDistancemm));
        }));
  }


  public Command coralLoad()
  {

    return m_intake.setGround().repeatedly().alongWith(m_intakeRoller.in().repeatedly())
                   .until(() ->m_intakeSensor.getAsDouble("Current") >= IntakeConstants.kCurrentLoaded);

  }

  public Command coralTransfer()
  {
    return m_intakeRoller.out().alongWith(m_outakeRoller.in())
                         .until(()->m_endEffectorLaserCanSensor.getAsDouble("EndEffectorLaserCan") >= IntakeConstants.kLaserSenseDistancemm);
  }

  public Command coralLoadAuto()
  {

    return null;

  }


  public Command coralLock()
  {
    // Set arm to target angle, elev target height
    return null;
  }



}
