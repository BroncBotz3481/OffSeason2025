// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0.5;
  }
  public static final double              maxSpeed      = 7; //meters per second

public final class CanIDs{
  public static final int ElevatorMain = 13;
  public static final int ElevatorFollower = 14;
  public static final int IntakeArm = 15;
  public static final int OutakeArm = 16; 
  public static final int IntakeRoller = 17;
  public static final int OuttakeRoller= 18;
}

public final class ElevatorConstants {

  
  
  public static final double drumRadius = 0.015875;
  public static final Distance mechanismCircumference = Meters.of(2 * Math.PI * drumRadius);

  public static final double kP = 1.6 / (2 * Math.PI); // convert radians to rotations
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double ksimP = 1.6 / (2 * Math.PI);
  public static final double ksimI = 0;
  public static final double ksimD = 0;

  public static final double kS = 0;
  public static final double kG = 0.13;
  public static final double kV = 0;
  public static final double ksimS = 0;
  public static final double ksimG = 0;
  public static final double ksimV = 0;

  public static final int gearbox = 15;
  public static final Current statorCurrentLimit = Amps.of(40);
  
  public static final Distance startingHeight = Meters.of(0);
  public static final Distance hardLimitMin = Meters.of(0);
  public static final Distance hardLimitMax = Meters.of(2.0658);
  public static final Mass mass = Pounds.of(28);
  public static final Distance absoluteOffset = Meters.of(0);
 public static final double   kElevatorAllowableError   = RobotBase.isSimulation() ? 
                                                          Units.inchesToMeters(1)
                                                          : 0.07;
  


  
}
public static class GroundConstants {
  public static final double kP = 100;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double ksimP = 100;
  public static final double ksimI = 0;
  public static final double ksimD = 0;

  public static final double kS = 0;
  public static final double kG = 0;
  public static final double kV = 0;
  public static final double ksimS = 0;
  public static final double ksimG = 0;
  public static final double ksimV = 0;

  public static final int gearbox = 28;
  public static final Current statorCurrentLimit = Amps.of(40);

  public static final Angle softLimitMin = Degrees.of(5);
  public static final Angle softLimitMax = Degrees.of(150);
  public static final Angle hardLimitMin = Degrees.of(5);
  public static final Angle hardLimitMax = Degrees.of(150);
  public static final Distance armLength = Meters.of(0.3511296);
  public static final Mass armMass =  Pounds.of(8);
  
  public static final Angle startingPosition = Degrees.of(150); // setting position of relative encoder
  public static final Angle kHorizontalZero = Degrees.of(15);// Parallel to the ground at 15deg - setting position of absolute
  public static final Angle kArmAllowableError = Degrees.of(RobotBase.isSimulation() ? 0.01 : 4);
}
public static class OutakeConstants {
  
  public static final double kArmAllowableError = RobotBase.isSimulation() ? 0.01 : 4;
  public static final Angle kArmHP = Degrees.of(145);
  public static final Angle kHorizontalZero = Degrees.of(15); // Parallel to the ground at 15deg

  
}


public static class IntakeConstants{
  public static final double kRollerSpeed = 0.8;
  public static double kCurrentLoaded = 50;
}
  
}
