package frc.robot.systems;

import static edu.wpi.first.units.Units.Amps;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
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

public class LoadingSystem {


    private IntakeArmSubsystem m_intake;
    private OutakeArmSubsystem m_outake;
    private ElevatorSubsystem m_elevator;
    private SwerveSubsystem m_swerve;
    private IntakeRollerSubsystem m_intakeRoller;
    private OutakeRollerSubsystem m_outakeRoller;
    private SwerveInputStream m_swerveInputStream;
    private TargetingSystem m_targetSystem;
    private LaserCan m_endEffectorLaserCAN = new LaserCan(19); // TODO: Needs to be moved to Constants
    private Sensor m_endEffectorLaserCanSensor = new SensorConfig("EndEffectorLaserCAN")
            .withField("EndEffectorLaserCan", () -> m_endEffectorLaserCAN.getMeasurement().distance_mm, 1)
            .getSensor();
    private Sensor m_intakeSensor;
    private boolean hasCoral;


    public LoadingSystem(IntakeArmSubsystem intake, ElevatorSubsystem elevator, SwerveSubsystem swerve,
                         OutakeArmSubsystem outake, IntakeRollerSubsystem intakeRoller,
                         OutakeRollerSubsystem outakeRoller,
                         TargetingSystem targeting, SwerveInputStream driveStream) {
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
        hasCoral = false;

        setupAutoTransfer();

        if (Robot.isSimulation()) {
            setupSimulation();
        }
    }
    /// ----------------- Sensor Data ----------------- ///

    private SensorData getIntakeCurrent()
    {
        return m_intakeSensor.getField("Current");
    }

    private SensorData getOuttakeCoralDistance()
    {
        return m_endEffectorLaserCanSensor.getField("EndEffectorLaserCan");
    }

    /// ----------------- Sensor Triggers/Events ----------------- ///

    private boolean readyToTransfer() {
        return m_intake.aroundPass() && m_outake.aroundPass() && hasCoral;
    }

    private boolean intakeHasCoral() {
        return getIntakeCurrent().getAsDouble() >= IntakeConstants.kCurrentLoaded;
    }

    public boolean outtakeHasCoral() {
        return getOuttakeCoralDistance().getAsInt() >= IntakeConstants.kLaserSenseDistancemm;
    }

    /// ----------------- Sensor Data Simulation ----------------- ///

    private Command simulateCoralIntake()
    {
        return Commands.runOnce(()->getIntakeCurrent().set(SensorData.convert(80.0)));
    }

    private Command simulateCoralPass()
    {
        return Commands.runOnce(()->{
            getOuttakeCoralDistance().set(SensorData.convert(IntakeConstants.kLaserSenseDistancemm));
            getIntakeCurrent().set(SensorData.convert(IntakeConstants.kCurrentLoaded - 20)); // Not realistic at all.
        });
    }

    public void setupSimulation() {
        new Trigger(m_intake::aroundGround)
                .onTrue(simulateCoralIntake());
        new Trigger(() -> m_intake.aroundPass() && m_outake.aroundPass() && m_intakeRoller.outtaking())
                .onTrue(simulateCoralPass());
    }

    /// ----------------- System Commands ----------------- ///

    /**
     * Sets up the auto transfer of a coral based on sensor data.
     */
    private void setupAutoTransfer()
    {
        new Trigger(this::intakeHasCoral)
                .debounce(0.5) // Only valid if true for 500ms or half a second
                .onTrue(Commands.runOnce(() -> hasCoral = true)); // Set this.hasCoral to true.
        new Trigger(m_intakeRoller::outtaking).onTrue(Commands.runOnce(() -> hasCoral = false)); // Set this.hasCoral to false when intake starts outtaking

        new Trigger(this::readyToTransfer).onTrue(coralTransfer()); // <-- Automatically transfer the coral when ready to transfer
    }

    public Command coralLoad() {

        return m_intake.setGround().alongWith(m_intakeRoller.in(), m_outake.pass())
                .until(this::intakeHasCoral);

    }

    public Command coralTransfer() {
        return m_intakeRoller.out().alongWith(m_outakeRoller.in())
                .until(this::outtakeHasCoral);
    }

    public Command coralLoadAuto() {

        return m_intake.setPass().alongWith(m_outake.pass(), m_elevator.pass())
                .until(()-> readyToTransfer()).andThen(coralTransfer());

    }


    public Command coralLock() {
        // Set arm to target angle, elev target height
        return null;
    }


}
