package com.team1701.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;

import com.team1701.lib.cameras.PhotonCameraWrapper;
import com.team1701.robot.Constants;
import com.team1701.robot.Robot;
import com.team1701.robot.estimation.PoseEstimator;
import com.team1701.robot.subsystems.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class Vision extends Subsystem {
    public static Vision mInstance = null;

    private final ArrayList<PhotonCameraWrapper> mCameras = new ArrayList<PhotonCameraWrapper>();
    private AprilTagFieldLayout mAprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    private Optional<VisionSystemSim> mVisionSim = Optional.empty();

    public static synchronized Vision getInstance() {
        if (mInstance == null) {
            mInstance = new Vision();
        }

        return mInstance;
    }

    private Vision() {
        Supplier<AprilTagFieldLayout> fieldLayoutSupplier = () -> mAprilTagFieldLayout;
        Supplier<Pose3d> robotPoseSupplier =
                () -> new Pose3d(PoseEstimator.getInstance().get());

        mCameras.add(new PhotonCameraWrapper(
                Constants.Vision.kFrontLeftCameraName,
                Constants.Vision.kRobotToFrontLeftCamPose,
                Constants.Vision.kPoseStrategy,
                fieldLayoutSupplier,
                robotPoseSupplier));
        mCameras.add(new PhotonCameraWrapper(
                Constants.Vision.kBackLeftCameraName,
                Constants.Vision.kRobotToBackLeftCamPose,
                Constants.Vision.kPoseStrategy,
                fieldLayoutSupplier,
                robotPoseSupplier));
        mCameras.add(new PhotonCameraWrapper(
                Constants.Vision.kFrontRightCameraName,
                Constants.Vision.kRobotToFrontRightCamPose,
                Constants.Vision.kPoseStrategy,
                fieldLayoutSupplier,
                robotPoseSupplier));
        mCameras.add(new PhotonCameraWrapper(
                Constants.Vision.kBackRightCameraName,
                Constants.Vision.kRobotToBackRightCamPose,
                Constants.Vision.kPoseStrategy,
                fieldLayoutSupplier,
                robotPoseSupplier));

        if (Robot.isSimulation()) {
            var visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(AprilTagFields.kDefaultField.loadAprilTagLayoutField());

            var cameraProperties = new SimCameraProperties();
            cameraProperties.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProperties.setCalibError(0.35, 0.10);
            cameraProperties.setFPS(15);
            cameraProperties.setAvgLatencyMs(50);
            cameraProperties.setLatencyStdDevMs(15);

            mCameras.forEach(camera -> camera.addToVisionSim(visionSim, cameraProperties));

            mVisionSim = Optional.of(visionSim);
        }

        mCameras.forEach(camera -> {
            camera.addEstimatedPoseConsumer(estimation -> PoseEstimator.getInstance()
                    .addVisionMeasurement(estimation.estimatedPose.toPose2d(), estimation.timestampSeconds));
            camera.addTargetFilter(target -> target.getPoseAmbiguity() < Constants.Vision.kMaxPoseAmbiguity);
        });
    }

    @Override
    public void readPeriodicInputs() {
        mVisionSim.ifPresent(sim -> sim.update(PoseEstimator.getInstance().get()));
        mCameras.forEach(PhotonCameraWrapper::update);
    }

    @Override
    public void writePeriodicOutputs() {}

    @Override
    public void stop() {}

    @Override
    public void outputTelemetry() {
        mCameras.forEach(PhotonCameraWrapper::outputTelemetry);
        mVisionSim.ifPresent(
                sim -> Logger.recordOutput("Vision/SimPose", sim.getDebugField().getRobotPose()));
    }
}
