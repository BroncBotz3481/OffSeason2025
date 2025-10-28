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
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

import java.lang.management.MemoryType;
import java.util.Map;
import java.util.Set;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Setpoints;
import frc.robot.Setpoints.Arm.OuttakeArm;
import frc.robot.Setpoints.Elevator.Coral;
import frc.robot.systems.TargetingSystem;
import frc.robot.systems.TargetingSystem.ReefBranchLevel;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GroundConstants;
import frc.robot.Constants.OutakeConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.gearing.Sprocket;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class OutakeArmSubsystem extends SubsystemBase {
    private SparkMax spark = new SparkMax(CanIDs.OutakeArm, MotorType.kBrushless);

    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            // Feedback Constants (PID Constants)
            .withClosedLoopController(OutakeConstants.kP, OutakeConstants.kI, OutakeConstants.kD, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
            .withSimClosedLoopController(OutakeConstants.kPSim, OutakeConstants.kISim, OutakeConstants.kDsim, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
            // Feedforward Constants
            .withFeedforward(new ArmFeedforward(0, 0, 0))
            .withSimFeedforward(new ArmFeedforward(0, 0, 0))
            // Telemetry name and verbosity level
            .withTelemetry("OutakeArm", TelemetryVerbosity.HIGH)
            // Gearing from the motor rotor to final shaft.
            // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to the gearbox attached to your motor.
            .withGearing(new MechanismGearing(new GearBox(OutakeConstants.kgearbox)))
            // Motor properties to prevent over currenting.
            .withMotorInverted(false)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40))
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withExternalEncoder(spark.getAbsoluteEncoder())
            .withExternalEncoderInverted(true)
            .withUseExternalFeedbackEncoder(true)
            .withZeroOffset(Degrees.of(0));


    private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(CanIDs.OutakeArm), smcConfig);

    private ArmConfig armCfg = new ArmConfig(sparkSmartMotorController)
            // Soft limit is applied to the SmartMotorControllers PID
            .withSoftLimits(OutakeConstants.softLimitMin, OutakeConstants.softLimitMax)
            // Hard limit is applied to the simulation.
            .withHardLimit(OutakeConstants.hardLimitMin, OutakeConstants.hardLimitMax)
            // Starting position is where your arm starts
            .withStartingPosition(OutakeConstants.kStartingPose)
            // Length and mass of your arm for sim.
            .withLength(Meters.of(0.3471418))
            .withMass(Pounds.of(6))
            // Telemetry name and verbosity for the arm.
            .withTelemetry("OutakeArm", TelemetryVerbosity.HIGH);

    // Arm Mechanism
    private Arm arm = new Arm(armCfg);

    /**
     * Creates a new ExampleSubsystem.
     */
    public OutakeArmSubsystem() {
        //sparkSmartMotorController.synchronizeRelativeEncoder();
    }



    public Command set(double dutycycle) {
        return arm.set(dutycycle);
    }

    public Command sysId() {
        return arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
    }

    // public Command hold() {
    //     return setAngle(arm.getAngle().in(Degrees)).repeatedly();
    // }

    private double angleHold = 0;
    public Command hold(boolean sync)
    {
      return startRun(() -> {
        if (sync)
        {
            sparkSmartMotorController.synchronizeRelativeEncoder();
        }
        angleHold = MathUtil.clamp(getAngle().in(Degrees),
                                   GroundConstants.softLimitMin.in(Degrees) +
                                   GroundConstants.kArmAllowableError.in(Degrees),
                                   GroundConstants.softLimitMax.in(Degrees) -
                                   GroundConstants.kArmAllowableError.in(Degrees));
        m_pidController.reset(arm.getAngle());
      }, () ->       arm.setAngle(Degrees.of(MathUtil.clamp(angleHold, GroundConstants.softLimitMin.in(Degrees),GroundConstants.softLimitMax.in(Degrees)))));
    
    }
  
    public Command hold()
    {
      return hold(true);
    }
    
    public Command holdDefer(){
        return Commands.defer(()->hold(),Set.of(this));
    }

    public Trigger isLoaded() {
        return null;
    }

    /**
     * Gets the height of the elevator and compares it to the given height with the given tolerance.
     *
     * @param height         Height in meters
     * @param allowableError Tolerance in meters.
     * @return Within that tolerance.
     */
    public boolean aroundAngle(double angle, double allowableError) {
        return MathUtil.isNear(angle, arm.getAngle().in(Degrees), allowableError);
    }

    /**
     * Gets the height of the elevator and compares it to the given height with the given tolerance.
     *
     * @param height Height in meters
     * @return Within that tolerance.
     */
    public boolean aroundAngle(double angle) {
        return aroundAngle(angle, Constants.OutakeConstants.kArmAllowableError);
    }
    
 /**
   * Set the angle of the arm.
   * @param angle Angle to go to.
   */
  public Command setAngle(double angle) {
    return arm.setAngle(Degrees.of(angle)).until(()->aroundAngle(angle));
}

    public Command L1() {
        return setAngle(Setpoints.Arm.OuttakeArm.L1);
    }

    public Command L2() {
        return setAngle(Setpoints.Arm.OuttakeArm.L2);
    }

    public Command L3() {
        return setAngle(Setpoints.Arm.OuttakeArm.L3);
    }

    public Command L4() {
        return setAngle(Setpoints.Arm.OuttakeArm.L4);
    }

    public Command pass() {
        return setAngle(Setpoints.Arm.OuttakeArm.passAngle);
    }

    public Command getCoralCommand(TargetingSystem targetingSystem) {
        return Commands.select(Map.of(ReefBranchLevel.L1, L1(),
                        ReefBranchLevel.L2, L2(),
                        ReefBranchLevel.L3, L3(),
                        ReefBranchLevel.L4, L4()),
                targetingSystem::getTargetBranchLevel);
    }

    public Trigger atCoralAngle(TargetingSystem targetingSystem) {
        return new Trigger(() -> {
            switch (targetingSystem.getTargetBranchLevel()) {
                case L2 -> {
                    return aroundAngle(Coral.L2);
                }
                case L3 -> {
                    return aroundAngle(Coral.L3);
                }
                case L1 -> {
                    return aroundAngle(Coral.L1);
                }
                case L4 -> {
                    return aroundAngle(Coral.L4);
                }
            }
            return false;
        });

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
        arm.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        arm.simIterate();
    }

    public Angle getAngle() {
        return arm.getAngle();
    }

    public boolean aroundPass() {
        return aroundAngle(OuttakeArm.passAngle);
    }
}
