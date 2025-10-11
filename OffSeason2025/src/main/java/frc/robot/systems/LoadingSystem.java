package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Setpoints;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.OutakeArmSubsystem;
import frc.robot.subsystems.OutakeRollerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

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


 
}
