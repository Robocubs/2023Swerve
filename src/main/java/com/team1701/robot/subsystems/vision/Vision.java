package com.team1701.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.ConcurrentModificationException;
import java.util.List;
import java.util.Optional;
import java.util.stream.DoubleStream;

import com.team1701.lib.util.LoggingUtil;
import com.team1701.robot.Constants;
import com.team1701.robot.estimation.PoseEstimator;
import com.team1701.robot.subsystems.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class Vision extends Subsystem {

    public static Vision mInstance = null;
    private ArrayList<VisionCamera> mCameras = new ArrayList<VisionCamera>();
    private AprilTagFieldLayout mAprilTagFieldLayout;
    private boolean mLoadedTagFieldLayout = false;

    public static synchronized Vision getInstance() {
        if (mInstance == null) {
            mInstance = new Vision();
        }

        return mInstance;
    }

    private Vision() {
        mCameras.add(new VisionCamera(
                Constants.kFrontRightCameraName,
                mAprilTagFieldLayout,
                Constants.kRobotToFrontRightCamPose,
                Constants.kCameraPoseStrategy));
        mCameras.add(new VisionCamera(
                Constants.kBackLeftCameraName,
                mAprilTagFieldLayout,
                Constants.kRobotToBackLeftCamPose,
                Constants.kCameraPoseStrategy));
        mCameras.add(new VisionCamera(
                Constants.kFrontRightCameraName,
                mAprilTagFieldLayout,
                Constants.kRobotToFrontRightCamPose,
                Constants.kCameraPoseStrategy));
        mCameras.add(new VisionCamera(
                Constants.kBackRightCameraName,
                mAprilTagFieldLayout,
                Constants.kRobotToBackRightCamPose,
                Constants.kCameraPoseStrategy));
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(
            PhotonPoseEstimator poseEstimator, PhotonPipelineResult pipelineResult) {
        if (!mLoadedTagFieldLayout) return Optional.empty();
        poseEstimator.setReferencePose(PoseEstimator.getInstance().getCurrentPose());
        return poseEstimator.update(pipelineResult);
    }

    @Override
    public void readPeriodicInputs() {
        mCameras.forEach(c -> c.addVisionMeasurementWithConstraints(PoseEstimator.getInstance()));
    }

    @Override
    public void writePeriodicOutputs() {}

    @Override
    public void stop() {}

    @Override
    public void outputTelemetry() {
        mCameras.forEach(VisionCamera::outputTelemetry);
    }

    private static enum FieldChoice {
        COMPETITION,
        UDJ
    }

    public static enum LimelightMode {
        PIECE_DETECTION,
        UNKNOWN
    }

    private class VisionCamera {
        private PhotonCamera mCamera;
        private PhotonPoseEstimator mPhotonPoseEstimator;
        private PhotonPipelineResult mPipelineResult = new PhotonPipelineResult();
        public double mPreviousCamPipelineTimestamp = 0;
        private double mLastOutputTimestampSeconds = 0;
        private String mCameraName;
        private Pose2d mRobotVisionPose = new Pose2d();
        private CameraInputs mCameraInputs;

        public VisionCamera(
                String cameraName,
                AprilTagFieldLayout initialFieldLayout,
                Transform3d robotToCamPose,
                PoseStrategy poseStrategy) {
            mCameraName = cameraName;
            mCameraInputs = new CameraInputs(new PhotonCamera(cameraName));
            mPhotonPoseEstimator = new PhotonPoseEstimator(initialFieldLayout, poseStrategy, mCamera, robotToCamPose);
            mPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
        }

        public void addVisionMeasurementWithConstraints(PoseEstimator poseEstimator) {
            var estimatedCamGlobalPose =
                    Vision.getInstance().getEstimatedGlobalPose(getPhotonPoseEstimator(), getLatestPipelineResult());
            var currentTimestamp = getTimestampSeconds();
            if (currentTimestamp != mPreviousCamPipelineTimestamp && hasTargets()) {
                mPreviousCamPipelineTimestamp = currentTimestamp;
                try {
                    if (estimatedCamGlobalPose.isPresent()) {
                        mRobotVisionPose =
                                estimatedCamGlobalPose.get().estimatedPose.toPose2d();
                        if (mRobotVisionPose.getX() < 4 || mRobotVisionPose.getX() > 12.5) {
                            poseEstimator.addVisionMeasurement(mRobotVisionPose, currentTimestamp);
                        }
                    }
                } catch (ConcurrentModificationException e) {
                    return;
                }
            }
        }

        public PhotonPoseEstimator getPhotonPoseEstimator() {
            return mPhotonPoseEstimator;
        }

        public boolean hasTargets() {
            return mPipelineResult.hasTargets();
        }

        public double getTimestampSeconds() {
            return mPipelineResult.getTimestampSeconds();
        }

        public PhotonPipelineResult getLatestPipelineResult() {
            Logger.getInstance().processInputs("Camera/" + mCameraName, mCameraInputs);
            var photonPipelineResult = mCameraInputs.pipelineResult;

            if (!photonPipelineResult.hasTargets()) return photonPipelineResult;

            var filteredTargets = photonPipelineResult.getTargets().stream()
                    .filter(target -> target.getPoseAmbiguity() <= Constants.kCameraMaxPoseAmbiguity)
                    .toList();
            var filteredPipelineResult =
                    new PhotonPipelineResult(photonPipelineResult.getLatencyMillis(), filteredTargets);
            filteredPipelineResult.setTimestampSeconds(photonPipelineResult.getTimestampSeconds());
            mPipelineResult = filteredPipelineResult;
            return filteredPipelineResult;
        }

        public void outputTelemetry() {
            var logger = Logger.getInstance();
            var cameraNamespace = "Camera/" + mCameraName;
            if (getTimestampSeconds() != mLastOutputTimestampSeconds) {
                logger.recordOutput(cameraNamespace + "/SeesAprilTag", hasTargets());
                logger.recordOutput(cameraNamespace + "/RobotPose", mRobotVisionPose);

                mLastOutputTimestampSeconds = getTimestampSeconds();
                List<Pose3d> trackedTagPositions = new ArrayList<Pose3d>();
                /* for (PhotonTrackedTarget target : mPipelineResult.targets) {
                    var fiducialID = target.getFiducialId();
                    if (fiducialID != -1) {
                        trackedTagPositions.add(
                                getLoadedLayout().getTagPose(fiducialID).get());
                    }
                }
                */

                logger.recordOutput(
                        cameraNamespace + "/AprilTagPoses",
                        trackedTagPositions.toArray(new Pose3d[trackedTagPositions.size()]));
            }
        }
    }

    private class CameraInputs implements LoggableInputs {
        private PhotonCamera mCamera;
        private PhotonPipelineResult pipelineResult;
        private boolean isConnected;

        public CameraInputs(PhotonCamera camera) {
            mCamera = camera;
        }

        @Override
        public void toLog(LogTable table) {
            isConnected = mCamera.isConnected();
            table.put("IsConnected", isConnected);

            pipelineResult = mCamera.getLatestResult();
            var targets = pipelineResult.targets;
            var targetCount = targets.size();

            table.put("Timestamp", pipelineResult.getTimestampSeconds());
            table.put("Latency", pipelineResult.getLatencyMillis());
            table.put("TargetCount", targets.size());

            for (int i = 0; i < targetCount; i++) {
                var targetNamespace = "Target/" + i + "/";
                var target = targets.get(i);

                table.put(targetNamespace + "Yaw", target.getYaw());
                table.put(targetNamespace + "Pitch", target.getPitch());
                table.put(targetNamespace + "Skew", target.getSkew());
                table.put(targetNamespace + "Area", target.getArea());
                table.put(targetNamespace + "FiducialID", target.getFiducialId());
                table.put(targetNamespace + "PoseAmbiguity", target.getPoseAmbiguity());

                LoggingUtil.put(table, targetNamespace + "Pose", target.getBestCameraToTarget());
                LoggingUtil.put(table, targetNamespace + "AltPose", target.getAlternateCameraToTarget());

                var minAreaRectCorners = target.getMinAreaRectCorners().stream()
                        .flatMapToDouble(c -> DoubleStream.of(c.x, c.y))
                        .toArray();

                table.put(targetNamespace + "MinAreaRectCorners", minAreaRectCorners);

                var detectedCorners = target.getDetectedCorners().stream()
                        .flatMapToDouble(c -> DoubleStream.of(c.x, c.y))
                        .toArray();

                table.put(targetNamespace + "DetectedCorners", detectedCorners);
            }
        }

        @Override
        public void fromLog(LogTable table) {
            var timestamp = table.getDouble("Timestamp", 0);
            var latency = (int) table.getInteger("Latency", 0);
            var targetCount = (int) table.getInteger("TargetCount", 0);
            var targets = new ArrayList<PhotonTrackedTarget>(targetCount);

            for (int i = 0; i < targetCount; i++) {
                var targetNamespace = "Target/" + i + "/";

                var minAreaRectCornerCords =
                        table.getDoubleArray(targetNamespace + "MinAreaRectCorners", new double[0]);
                var minAreaRectCorners = new ArrayList<TargetCorner>(4);
                for (int j = 0; j < minAreaRectCornerCords.length / 2; j++) {
                    minAreaRectCorners.add(
                            j, new TargetCorner(minAreaRectCornerCords[j * 2], minAreaRectCornerCords[j * 2 + 1]));
                }

                var detectedCornerCords = table.getDoubleArray(targetNamespace + "DetectedCorners", new double[0]);
                var detectedCorners = new ArrayList<TargetCorner>(detectedCornerCords.length);
                for (int j = 0; j < detectedCornerCords.length / 2; j++) {
                    detectedCorners.add(new TargetCorner(detectedCornerCords[j * 2], detectedCornerCords[j * 2 + 1]));
                }

                var legacyFiducialID = table.getInteger(targetNamespace + "Fiducial ID", 0);

                var trackedTarget = new PhotonTrackedTarget(
                        table.getDouble(targetNamespace + "Yaw", 0),
                        table.getDouble(targetNamespace + "Pitch", 0),
                        table.getDouble(targetNamespace + "Area", 0),
                        table.getDouble(targetNamespace + "Skew", 0),
                        (int) table.getInteger(targetNamespace + "FiducialID", legacyFiducialID),
                        LoggingUtil.getTransform3d(table, targetNamespace + "Pose"),
                        LoggingUtil.getTransform3d(table, targetNamespace + "AltPose"),
                        table.getDouble(targetNamespace + "PoseAmbiguity", 0),
                        minAreaRectCorners,
                        detectedCorners);

                targets.add(trackedTarget);
            }

            pipelineResult = new PhotonPipelineResult(latency, targets);
            pipelineResult.setTimestampSeconds(timestamp);

            isConnected = table.getBoolean("IsConnected", false);
        }
    }
}
