// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.camera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;
import frc.robot.Robot.RobotType;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.Tracer;
import java.util.NoSuchElementException;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public class Camera {

  // The intrinsics and distortion coefficients are only actually used for sim
  public record CameraConstants(
      String name,
      Transform3d robotToCamera,
      Matrix<N3, N3> intrinsicsMatrix,
      Matrix<N8, N1> distCoeffs) {}

  // These standard deviations are used for scaling how much we "trust" a pose estimate
  // Standard deviations are a way of measuring how far away something is from the mean. A dataset
  // with a very high standard deviation is very spread out, versus a dataset with a low standard
  // deviation has most of its data clustered around the mean
  // In other words, the higher the standard deviation, the greater the noise
  // Noisy estimates are less reliable, so we can increase or decrease these standard deviation
  // values associated with a pose estimate to reflect how reliable we think an estimate is
  public static final Matrix<N3, N1> visionPointBlankDevs =
      new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[] {0.6, 0.6, 0.5});

  // infinite devs are used to show that we really don't trust this estimate
  public static final Matrix<N3, N1> infiniteDevs =
      new Matrix<N3, N1>(
          Nat.N3(),
          Nat.N1(),
          new double[] {
            Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY
          });
  public static final double distanceFactor = 3.0;

  private final CameraIO io;
  private final CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();
  private final PhotonPoseEstimator estimator =
      new PhotonPoseEstimator(
          SwerveSubsystem.SWERVE_CONSTANTS.getFieldTagLayout(),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          null);

  // set up error alerts
  private final Alert futureVisionData;
  private final Alert disconnectedAlert;

  private Pose3d pose = Pose3d.kZero;

  public Camera(CameraIO io) {
    this.io = io;
    // Tells the estimator what the transformation is between the camera and the robot
    estimator.setRobotToCameraTransform(io.getCameraConstants().robotToCamera);
    // io.updateInputs(inputs);
    futureVisionData =
        new Alert(getName() + " Vision Data Coming from ✨The Future✨", AlertType.kError);
    disconnectedAlert = new Alert(getName() + " Disconnected!", AlertType.kError);
  }

  // MUST CALL FROM SUBSYSTEM! NOT PART OF COMMAND SCHEDULER
  public void periodic() {
    Tracer.trace("Update inputs", this::updateInputs);
    Tracer.trace("Process april tag inputs", this::processApriltagInputs);

    // TODO potentially later track nt disconnected vs completely disconnected like 6328 does?
    disconnectedAlert.set(!inputs.connected);
  }

  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void processApriltagInputs() {
    Logger.processInputs("Apriltag Vision/" + io.getName(), inputs);
  }

  public Optional<EstimatedRobotPose> update(PhotonPipelineResult result) {

    // if we don't see any tags, don't return anything
    if (result.getTargets().size() < 1) {
      return Optional.empty();
    }
    if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput(
          "Vision/" + io.getName() + "/Best Distance",
          result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm());
    Optional<EstimatedRobotPose> estPose = estimator.update(result);
    return estPose;
  }

  public void setSimPose(Optional<EstimatedRobotPose> simEst, boolean newResult) {
    this.io.setSimPose(simEst, newResult);
  }

  public String getName() {
    return io.getName();
  }

  public static Matrix<N3, N1> findVisionMeasurementStdDevs(EstimatedRobotPose estimation) {
    double sumDistance = 0;
    for (PhotonTrackedTarget target : estimation.targetsUsed) {
      Transform3d t3d = target.getBestCameraToTarget();
      sumDistance +=
          Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
    }
    double avgDistance = sumDistance / estimation.targetsUsed.size();

    Matrix<N3, N1> deviation =
        visionPointBlankDevs.times(Math.max(avgDistance, 0.0) * distanceFactor);
    if (estimation.targetsUsed.size() == 1) {
      deviation = deviation.times(3);
    }
    if (estimation.targetsUsed.size() == 1 && estimation.targetsUsed.get(0).poseAmbiguity > 0.15) {
      return infiniteDevs;
    }
    // Reject if estimated pose is in the air or ground
    if (Math.abs(estimation.estimatedPose.getZ()) > 0.125) {
      return infiniteDevs;
    }
    // TAG_COUNT_DEVIATION_PARAMS
    //     .get(
    //         MathUtil.clamp(
    //             estimation.targetsUsed.size() - 1, 0, TAG_COUNT_DEVIATION_PARAMS.size() - 1))
    //     .computeDeviation(avgDistance);
    return deviation;
  }

  public void updateCamera(SwerveDrivePoseEstimator swerveEstimator) {
    boolean hasFutureData = false;
    try {
      if (!inputs.stale) {
        Optional<EstimatedRobotPose> estPose =
            Tracer.trace("Update Camera", () -> update(inputs.result));
        Pose3d visionPose = estPose.get().estimatedPose;
        pose = visionPose;
        // Sets the pose on the sim field
        setSimPose(estPose, !inputs.stale);

        // if (Robot.ROBOT_TYPE != RobotType.REAL)
        Logger.recordOutput("Vision/" + getName() + "/Pose3d", visionPose);
        // if (Robot.ROBOT_TYPE != RobotType.REAL)
        Logger.recordOutput("Vision/" + getName() + "/Pose2d", visionPose.toPose2d());
        final Matrix<N3, N1> deviations = findVisionMeasurementStdDevs(estPose.get());
        // if (Robot.ROBOT_TYPE != RobotType.REAL)
        Logger.recordOutput("Vision/" + getName() + "/Deviations", deviations.getData());

        Tracer.trace(
            "Add Measurement From " + getName(),
            () -> {
              swerveEstimator.addVisionMeasurement(
                  visionPose.toPose2d(),
                  inputs.result.metadata.captureTimestampMicros / 1.0e6,
                  deviations.times(DriverStation.isAutonomous() ? 2.0 : 1.0));
              // the sussifier
            });

        hasFutureData |= inputs.result.metadata.captureTimestampMicros > RobotController.getTime();
        // if (Robot.ROBOT_TYPE != RobotType.REAL)
        Logger.recordOutput("Vision/" + getName() + "/Invalid Pose Result", "Good Update");

        Tracer.trace(
            "Log Tag Poses",
            () -> {
              Pose3d[] targetPose3ds = new Pose3d[inputs.result.targets.size()];
              for (int j = 0; j < inputs.result.targets.size(); j++) {
                targetPose3ds[j] =
                    SwerveSubsystem.SWERVE_CONSTANTS
                        .getFieldTagLayout()
                        .getTagPose(inputs.result.targets.get(j).getFiducialId())
                        .get();
              }
              // if (Robot.ROBOT_TYPE != RobotType.REAL)
              Logger.recordOutput("Vision/" + getName() + "/Target Poses", targetPose3ds);
            });

      } else {
        // if (Robot.ROBOT_TYPE != RobotType.REAL)
        Logger.recordOutput("Vision/" + getName() + "/Invalid Pose Result", "Stale");
      }
    } catch (NoSuchElementException e) {
      // if (Robot.ROBOT_TYPE != RobotType.REAL)
      Logger.recordOutput("Vision/" + getName() + "/Invalid Pose Result", "Bad Estimate");
    }
    futureVisionData.set(hasFutureData);
  }

  public CameraConstants getCameraConstants() {
    return io.getCameraConstants();
  }

  public Pose3d getPose() {
    return pose;
  }
}
