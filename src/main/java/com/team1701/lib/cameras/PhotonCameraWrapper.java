package com.team1701.lib.cameras;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Predicate;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonCameraWrapper {
    private final PhotonCamera mCamera;
    private final PhotonCameraInputs mCameraInputs;
    private final PhotonPoseEstimator mPoseEstimator;
    private final Transform3d mRobotToCamPose;
    private final Supplier<AprilTagFieldLayout> mFieldLayoutSupplier;
    private final Supplier<Pose3d> mRobotPoseSupplier;
    private final ArrayList<Consumer<EstimatedRobotPose>> mEstimatedPoseConsumers = new ArrayList<>();
    private final ArrayList<Predicate<PhotonTrackedTarget>> mTargetFilters = new ArrayList<>();
    private final ArrayList<Predicate<Pose3d>> mPoseFilters = new ArrayList<>();

    private Pose3d mLastUnfilteredPose = new Pose3d();
    private Pose3d mLastFilteredPose = new Pose3d();

    public PhotonCameraWrapper(
            String cameraName,
            Transform3d robotToCamPose,
            PoseStrategy poseStrategy,
            Supplier<AprilTagFieldLayout> fieldLayoutSupplier,
            Supplier<Pose3d> robotPoseSupplier) {
        mCamera = new PhotonCamera(cameraName);
        mCameraInputs = new PhotonCameraInputs();
        mPoseEstimator = new PhotonPoseEstimator(fieldLayoutSupplier.get(), poseStrategy, mCamera, robotToCamPose);
        mPoseEstimator.setMultiTagFallbackStrategy(poseStrategy);
        mRobotToCamPose = robotToCamPose;
        mFieldLayoutSupplier = fieldLayoutSupplier;
        mRobotPoseSupplier = robotPoseSupplier;
    }

    public void update() {
        mCameraInputs.isConnected = mCamera.isConnected();
        mCameraInputs.pipelineResult = mCamera.getLatestResult();
        Logger.processInputs("Camera/" + mCamera.getName(), mCameraInputs);

        var pipelineResult = mCameraInputs.pipelineResult;
        if (!pipelineResult.hasTargets()) {
            return;
        }

        mPoseEstimator.setFieldTags(mFieldLayoutSupplier.get());

        var filteredPipelineResult = filterTargets(pipelineResult);
        var estimatedRobotPose = mPoseEstimator.update(filteredPipelineResult);
        if (estimatedRobotPose.isEmpty()) {
            return;
        }

        mLastUnfilteredPose = estimatedRobotPose.get().estimatedPose;

        if (!mPoseFilters.stream().allMatch(filter -> filter.test(estimatedRobotPose.get().estimatedPose))) {
            return;
        }

        mLastFilteredPose = estimatedRobotPose.get().estimatedPose;
        mEstimatedPoseConsumers.forEach(consumer -> consumer.accept(estimatedRobotPose.get()));
    }

    private PhotonPipelineResult filterTargets(PhotonPipelineResult pipelineResult) {

        var filteredTargets = pipelineResult.getTargets().stream()
                .filter(target -> mTargetFilters.stream().allMatch(filter -> filter.test(target)))
                .toList();
        var filteredPipelineResult = new PhotonPipelineResult(pipelineResult.getLatencyMillis(), filteredTargets);
        filteredPipelineResult.setTimestampSeconds(pipelineResult.getTimestampSeconds());
        return filteredPipelineResult;
    }

    public void addEstimatedPoseConsumer(Consumer<EstimatedRobotPose> consumer) {
        mEstimatedPoseConsumers.add(consumer);
    }

    public void addTargetFilter(Predicate<PhotonTrackedTarget> filter) {
        mTargetFilters.add(filter);
    }

    public void addPoseFilter(Predicate<Pose3d> filter) {
        mPoseFilters.add(filter);
    }

    public void addToVisionSim(VisionSystemSim visionSim, SimCameraProperties cameraProperties) {
        var cameraSim = new PhotonCameraSim(mCamera, cameraProperties);
        visionSim.addCamera(cameraSim, mRobotToCamPose);
    }

    public void outputTelemetry() {
        var cameraNamespace = "Camera/" + mCamera.getName();
        Logger.recordOutput(cameraNamespace + "/UnfilteredRobotPose", mLastUnfilteredPose);
        Logger.recordOutput(cameraNamespace + "/FilteredRobotPose", mLastFilteredPose);

        var robotPose = mRobotPoseSupplier.get();
        var targetPoses = mCameraInputs.pipelineResult.targets.stream()
                .map(target -> robotPose
                        .plus(mRobotToCamPose)
                        .plus(target.getBestCameraToTarget())
                        .toPose2d())
                .toArray(Pose2d[]::new);
        Logger.recordOutput(cameraNamespace + "/TargetPoses", targetPoses);
    }
}
