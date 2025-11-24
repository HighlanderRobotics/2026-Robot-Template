// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.components.camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.components.camera.Camera.CameraConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** Add your docs here. */
public class CameraIOSim implements CameraIO {

  private final CameraConstants constants;
  private final VisionSystemSim sim;
  private final PhotonCamera camera;
  private final PhotonCameraSim simCamera;

  public final Supplier<Pose3d> poseSupplier;

  public CameraIOSim(CameraConstants constants, Supplier<Pose3d> poseSupplier) {
    this.sim = new VisionSystemSim(constants.name());
    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(1080, 960, constants.intrinsicsMatrix(), constants.distCoeffs());
    cameraProp.setCalibError(0.0, 0.0);
    cameraProp.setFPS(50.0);
    cameraProp.setAvgLatencyMs(30.0);
    cameraProp.setLatencyStdDevMs(5.0);
    this.camera = new PhotonCamera(constants.name());
    this.simCamera = new PhotonCameraSim(camera, cameraProp);
    simCamera.enableDrawWireframe(true);
    simCamera.setMaxSightRange(7.0);
    this.constants = constants;
    sim.addCamera(simCamera, constants.robotToCamera());

    try {
      final var field = SwerveSubsystem.SWERVE_CONSTANTS.getFieldTagLayout();
      field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      sim.addAprilTags(field);
    } catch (Exception e) {
      e.printStackTrace();
    }

    this.poseSupplier = poseSupplier;
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {
    // should always be connected in sim
    inputs.connected = true;
    sim.update(poseSupplier.get());
    var results = camera.getAllUnreadResults();
    if (results.size() > 0) {
      inputs.result = results.get(results.size() - 1);
      inputs.stale = false;
    } else {
      // else leave stale data
      inputs.stale = true;
    }
  }

  @Override
  public void setSimPose(Optional<EstimatedRobotPose> simEst, boolean newResult) {
    simEst.ifPresentOrElse(
        est ->
            sim.getDebugField().getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()),
        () -> {
          if (newResult) sim.getDebugField().getObject("VisionEstimation").setPoses();
        });
  }

  @Override
  public String getName() {
    return constants.name();
  }

  @Override
  public CameraConstants getCameraConstants() {
    return constants;
  }
}
