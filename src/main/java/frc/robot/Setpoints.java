package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class Setpoints
{

  public static class Wrist {
    public static final double rest = 0.60;
    public static final double active = 0.35;
  }

  public static class Elevator
  {

    public static class Coral
    {
      
      public static double L1 = Inches.of(11.5).in(Meters); 
      public static double L2 = Inches.of(24.5).in(Meters);
      public static double L3 = Inches.of(38).in(Meters);
      public static double L4 = Inches.of(66.5).in(Meters);
      public static double HP = 1.04;//Guesses
    }

  }

  public static class Arm
  {

    public static class OuttakeArm
    {

    public static double passAngle = 45;// TOtAl GuesS
      public static double L1 = -45;
      public static double L2 = -35;
      public static double L3 = -35;
      public static double L4 = -15;
    public static double sensorDistanceThreshold = 0.8;
    }
    public static class GroundIntake{
      public static double intakeAngle = 179;
      public static double passAngle = 50;
    }

  }

  public static class AutoScoring
  {
    public static class Processor {
      public static final Transform2d offset = new Transform2d(Inches.of(24).in(Meters),
                                                               Inches.of(0).in(Meters),
                                                               Rotation2d.fromDegrees(0));
    public static Pose2d centerFace;
    }
    public static class Reef 
    {
// x + front ->, y + left 
      public static final Transform2d coralOffset = new Transform2d(Inches.of(28).in(Meters),
                                                                    Inches.of(7.5).in(Meters),
                                                                    Rotation2d.fromDegrees(180));
      public static final Transform2d     algaeOffset = new Transform2d(Inches.of(24).in(Meters),
                                                                    Inches.of(-14).in(Meters),
                                                                    Rotation2d.fromDegrees(180));
    }

    public static class HumanPlayer
    {

      public static class Left
      {

        public static final Transform2d offset = new Transform2d(Inches.of(24).in(Meters),
                                                                 Inches.of(0).in(Meters),
                                                                 Rotation2d.fromDegrees(0));
      }

      public static class Right
      {

        public static final Transform2d offset = new Transform2d(Inches.of(24).in(Meters),
                                                                 Inches.of(0).in(Meters),
                                                                 Rotation2d.fromDegrees(0));
      }
    }
  }


}
