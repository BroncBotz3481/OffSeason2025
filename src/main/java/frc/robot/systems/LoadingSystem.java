package frc.robot.systems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
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
  private LoadingSystem         m_loadingSystem;
  private TargetingSystem       m_targetSystem;
  private LaserCan              m_endEffectorLaserCAN       = new LaserCan(19);
  private Sensor                m_endEffectorLaserCanSensor = new SensorConfig("EndEffectorLaserCAN")
      .withField("EndEffectorLaserCan", () -> m_endEffectorLaserCAN.getMeasurement().distance_mm, 1)
      .getSensor();
  private Sensor                m_intakeSensor;


  public LoadingSystem(IntakeArmSubsystem intake, ElevatorSubsystem elevator, SwerveSubsystem swerve,
                       OutakeArmSubsystem outake, IntakeRollerSubsystem intakeRoller,
                       OutakeRollerSubsystem outakeRoller,
                       LoadingSystem loading, TargetingSystem targeting, SwerveInputStream driveStream)
  {
    m_intake = intake;
    m_elevator = elevator;
    m_swerve = swerve;
    m_outake = outake;
    m_loadingSystem = loading;
    m_intakeRoller = intakeRoller;
    m_targetSystem = targeting;
    m_outakeRoller = outakeRoller;
    m_swerveInputStream = driveStream;
    m_intakeSensor = new SensorConfig("IntakeRoller")
        .withField("Current", () -> intakeRoller.getCurrent().in(Amps), 0.0)
        .getSensor();
    if (Robot.isSimulation())
    {setupSimulation();}
  }

  public void setupSimulation()
  {
    Angle loadingAngle        = Degrees.of(-5);
    Angle intakeTransferAngle = Degrees.of(95);
    Angle outakeTransferAngle = Degrees.of(105);
    new Trigger(() -> m_intake.getAngle().lte(Degrees.of(-5)))
        .onTrue(Commands.runOnce(() -> m_intakeSensor.getField("Current").set(SensorData.convert(40.0))));
    new Trigger(() -> m_intake.getAngle().isNear(intakeTransferAngle, Degrees.of(1)) &&
                      m_outake.getAngle().isNear(outakeTransferAngle, Degrees.of(1)) &&
                      m_intakeRoller.getDutycycle() < 0.0)
        .onTrue(Commands.runOnce(() -> {
          m_intakeSensor.getField("Current").set(SensorData.convert(0));
          m_endEffectorLaserCanSensor.getField("EndEffectorLaserCan").set(SensorData.convert(5));
        }));
  }


  public Command coralLoad()
  {

    return m_intake.setGround().repeatedly().alongWith(m_intakeRoller.in().repeatedly());

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

//   private DigitalInput dio = new DigitalInput(0); // Standard DIO
//   private final Sensor laser = new SensorConfig("CoralDetectorBeamBreak") // Name of the sensor 
//   .withField("Beam", m_laserCan::getMeasurement().distance_mm,233 ) // Add a Field to the sensor named "Beam" whose value is dio.get() and defaults to false
//   .withSimulatedValue("beam",(() ->m_outake.aroundAngle(Setpoints.Arm.OuttakeArm.passAngle, OutakeConstants.kArmAllowableError)), 118) // Change "Beam" field to true when the arm is near 40deg +- 2deg
//   .getSensor(); // Get the sensor.

// //double c = m_LaserCan.getMeasurement().distance_mm;
// public boolean sensorDistance(){
//   return laser.getAsDouble("beam") >= Setpoints.Arm.OuttakeArm.sensorDistanceThreshold; //MAKE SURE THERE IS NOTHING INFRONT OF THE SENSOR
//}


}
