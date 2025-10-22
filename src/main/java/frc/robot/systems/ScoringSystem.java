package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.OutakeArmSubsystem;
import frc.robot.subsystems.OutakeRollerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import static edu.wpi.first.units.Units.Seconds;


public class ScoringSystem
{

  private IntakeArmSubsystem    m_intake;
  private OutakeArmSubsystem m_outake;
  private ElevatorSubsystem    m_elevator;
  private SwerveSubsystem      m_swerve;
  private IntakeRollerSubsystem m_intakeRoller;
  private OutakeRollerSubsystem m_outakeRoller;
  private SwerveInputStream m_swerveInputStream;
  private LoadingSystem        m_loadingSystem;
  private TargetingSystem      m_targetSystem;



  public ScoringSystem(IntakeArmSubsystem intake, ElevatorSubsystem elevator, SwerveSubsystem swerve,
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

  ///  Score Coral Command for PathPlanner that does not move the swerve drive at all keeping the pathplanner auto
  /// intact.
  public Command scoreCoralAuto()
  {
    return null;
  }

  public Command restArmsSafe()
  {
    return null;
  }

  public Trigger inScoringPosition()
  {
      return m_elevator.atCoralHeight(m_targetSystem).and(m_outake.atCoralAngle(m_targetSystem));
  }

  public Command applyTargetingState()
  {
      // Repeatedly set the elevator and outtake arm at the targetting system state.
      return m_elevator.getCoralCommand(m_targetSystem)
              .alongWith(m_outake.getCoralCommand(m_targetSystem));
  }

  public Command outtake()
  {
      return m_outakeRoller.out().withTimeout(Seconds.of(1.5));
  }

  public Command safeState()
  {
      return m_outake.pass().alongWith(m_elevator.toMin());
  }


  public Command scoreCoral()
  {
    // Arm down, elevator down, drive backwards x in
    return m_swerve.stopDrivingCommand()
      .andThen(applyTargetingState())
      .until(inScoringPosition())
      .andThen(m_targetSystem.driveToCoralTarget(m_swerve).deadlineFor(applyTargetingState()))
      .andThen(outtake().deadlineFor(applyTargetingState()))
      .andThen(m_swerve.driveBackwards().deadlineFor(safeState()));
  }



}

//                     WALDO
///                     ( ) /-----\
///                  |||||||||
//                   ((o)-(o))
//                    |  W  |
//                    --| |--
