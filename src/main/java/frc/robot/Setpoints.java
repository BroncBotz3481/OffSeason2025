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

      public static double L1 = 0;
      public static double L2 = 0.014;
      public static double L3 = 0.014;
      public static double L4 = 0.47;
      public static double HP = 0;
    }

  }

  public static class Arm
  {

    public static class OuttakeArm
    {

    public static double passAngle = 0;// TOtAl GuesS
      public static double L1 = 100;
      public static double L2 = 120;
      public static double L3 = 120;
      public static double L4 = 180;
    public static double sensorDistanceThreshold = 0.8;
    }
    public static class GroundIntake{
      public static double intakeAngle = 0.0;
      public static double passAngle = 120;
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
