package frc.robot.subsystems;


import java.util.function.BooleanSupplier;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs;


public class OutakeRollerSubsystem extends SubsystemBase
{

  
  public static final double kWristMomentOfInertia = 0.00032; // kg * m^2
  
    private final SparkMax m_rollerMotor = new SparkMax(CanIDs.OuttakeRoller, MotorType.kBrushless);
  
    private final DCMotor m_rollerMotorGearbox = DCMotor.getNEO(1);
  
    private final FlywheelSim m_rollerSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(
        m_rollerMotorGearbox,
        kWristMomentOfInertia,
        1), m_rollerMotorGearbox, 1.0 / 4096.0);
  
    private final SparkMaxSim m_rollerMotorSim = new SparkMaxSim(m_rollerMotor, m_rollerMotorGearbox);
  
  
    public OutakeRollerSubsystem()
    {
      SparkMaxConfig config = new SparkMaxConfig();
      config
            .inverted(true)
            .smartCurrentLimit(40);
            config.idleMode(IdleMode.kBrake);
      m_rollerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command) done
      //       in the constructor or in the robot coordination class, such as RobotContainer.
      //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
      //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }
  
    @Override
    public void simulationPeriodic()
    {
      // In this method, we update our simulation of what our arm is doing
      // First, we set our "inputs" (voltages)
      m_rollerSim.setInput(m_rollerMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
  
      // Next, we update it. The standard loop time is 20ms.
      m_rollerSim.update(0.02);
  
      // Finally, we set our simulated encoder's readings and simulated battery voltage
      //m_encoderSim.setDistance(m_coralArmSim.getAngleRads());
  
      m_rollerMotorSim.iterate(m_rollerSim.getAngularVelocityRPM(),
                               RoboRioSim.getVInVoltage(),
                               // Simulated battery voltage, in Volts
                               0.02);
  
  
    }
  
  
    public Command setAlgaeIntakeRoller(double speed)
    {
      return run(() -> {
        m_rollerMotor.set(speed);
      });
    }
  
    public Command out()
    {
      return setAlgaeIntakeRoller(1);
  }

  public Command in()
  {
    return setAlgaeIntakeRoller(-1);
  }




}