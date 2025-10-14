package frc.robot.systems;

import static edu.wpi.first.units.Units.Degrees;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OutakeConstants;
import frc.robot.Setpoints;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.OutakeArmSubsystem;
import frc.robot.subsystems.OutakeRollerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import yams.mechanisms.config.SensorConfig;
import yams.motorcontrollers.simulation.Sensor;

;

public class LoadingSystem
{

 
  private IntakeArmSubsystem    m_intake;
  private OutakeArmSubsystem m_outake;
  private ElevatorSubsystem    m_elevator;
  private SwerveSubsystem      m_swerve;
  private IntakeRollerSubsystem m_intakeRoller;
  private OutakeRollerSubsystem m_outakeRoller;
  private SwerveInputStream    m_swerveInputStream;
  private LoadingSystem        m_loadingSystem;
  private TargetingSystem      m_targetSystem;



  public LoadingSystem(IntakeArmSubsystem intake, ElevatorSubsystem elevator, SwerveSubsystem swerve,
                      OutakeArmSubsystem outake, IntakeRollerSubsystem intakeRoller, OutakeRollerSubsystem outakeRoller, 
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
  }

  private Canandcolor m_canAndColor = new Canandcolor(19);

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

  private DigitalInput dio = new DigitalInput(0); 
  private final Sensor coralSensor = new SensorConfig("CoralCanAndCOlor") 
  .withField("Proximity", m_canAndColor::getProximity, 0.0) 
  .withSimulatedValue("Proximity",(() ->m_outake.aroundAngle(Setpoints.Arm.OuttakeArm.passAngle, OutakeConstants.kArmAllowableError)), 1.0) // Change "Beam" field to true when the arm is near 40deg +- 2deg
  .getSensor(); // Get the sensor.


public boolean sensorDistance(){
  return coralSensor.getAsDouble("Proximity") >= Setpoints.Arm.OuttakeArm.sensorDistanceThreshold; //MAKE SURE THERE IS NOTHING INFRONT OF THE SENSOR
}


 
}
