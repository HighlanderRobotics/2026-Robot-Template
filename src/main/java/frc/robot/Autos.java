// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot.RobotType;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Autos {
  private final AutoFactory factory;

  // Declare triggers
  // mehhhhhhh
  private static boolean autoPreScore;
  private static boolean autoScore;
  private static boolean autoIntakeCoral;

  // private static boolean autoIntakeAlgae;

  @AutoLogOutput(key = "Superstructure/Auto Pre Score Request")
  public static Trigger autoPreScoreReq =
      new Trigger(() -> autoPreScore).and(DriverStation::isAutonomous);

  @AutoLogOutput(key = "Superstructure/Auto Score Request")
  public static Trigger autoScoreReq =
      new Trigger(() -> autoScore).and(DriverStation::isAutonomous);

  @AutoLogOutput(key = "Superstructure/Auto Coral Intake Request")
  public static Trigger autoIntakeCoralReq =
      new Trigger(() -> autoIntakeCoral).and(DriverStation::isAutonomous);

  public enum PathEndType {
    PLACEHOLDER;
  }

  public enum Path {
    PLACEHOLDER("placeholder", "placeholder", PathEndType.PLACEHOLDER);

    private final String start;
    private final String end;
    private final PathEndType type;

    private Path(String start, String end, PathEndType type) {
      this.start = start;
      this.end = end;
      this.type = type;
    }

    public AutoTrajectory getTrajectory(AutoRoutine routine) {
      // AutoRoutine docs say that this "creates" a new trajectory, but the factory does check if
      // it's already present
      return routine.trajectory(start + "to" + end);
    }
  }

  public Autos(SwerveSubsystem swerve) {
    factory =
        new AutoFactory(
            swerve::getPose,
            swerve::resetPose,
            swerve.choreoDriveController(),
            true,
            swerve,
            (traj, edge) -> {
              if (Robot.ROBOT_TYPE != RobotType.REAL)
                Logger.recordOutput(
                    "Choreo/Active Traj",
                    DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get().equals(Alliance.Blue)
                        ? traj.getPoses()
                        : traj.flipped().getPoses());
            });
  }

  // TODO write leave auto
  public Command leaveAuto() {
    final AutoRoutine routine = factory.newRoutine("Leave Auto");
    Path[] paths = {};

    Command autoCommand = Commands.none();

    for (Path path : paths) {
      autoCommand =
          autoCommand.andThen(
              Commands.print("Running path: " + path.toString()).andThen(runPath(path, routine)));
    }
    return routine.cmd();
  }

  public Command runPath(Path path, AutoRoutine routine) {
    PathEndType type = path.type;
    switch (type) {
      default: // this should never happen
        return Commands.none();
    }
  }

  public Command setAutoScoreReqTrue() {
    return Commands.runOnce(
        () -> {
          autoScore = true;
        });
  }

  public Command setAutoPreScoreReqTrue() {
    return Commands.runOnce(() -> autoPreScore = true);
  }

  public Command setAutoScoreReqFalse() {
    return Commands.runOnce(
        () -> {
          autoScore = false;
          autoPreScore = false;
        });
  }
}
