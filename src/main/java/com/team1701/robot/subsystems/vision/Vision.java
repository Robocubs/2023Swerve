package com.team1701.robot.subsystems.vision;

import java.util.ArrayList;

import com.team1701.lib.cameras.PhotonCameraWrapper;
import com.team1701.robot.Constants;
import com.team1701.robot.estimation.PoseEstimator;
import com.team1701.robot.subsystems.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class Vision extends Subsystem {
    public static Vision mInstance = null;

    private final ArrayList<PhotonCameraWrapper> mCameras = new ArrayList<PhotonCameraWrapper>();
    private AprilTagFieldLayout mAprilTagFieldLayout;

    public static synchronized Vision getInstance() {
        if (mInstance == null) {
            mInstance = new Vision();
        }

        return mInstance;
    }

    private Vision() {
        mAprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        mCameras.add(new PhotonCameraWrapper(
                Constants.Vision.kFrontLeftCameraName,
                Constants.Vision.kRobotToFrontLeftCamPose,
                Constants.Vision.kPoseStrategy,
                () -> mAprilTagFieldLayout));
        mCameras.add(new PhotonCameraWrapper(
                Constants.Vision.kBackLeftCameraName,
                Constants.Vision.kRobotToBackLeftCamPose,
                Constants.Vision.kPoseStrategy,
                () -> mAprilTagFieldLayout));
        mCameras.add(new PhotonCameraWrapper(
                Constants.Vision.kFrontRightCameraName,
                Constants.Vision.kRobotToFrontRightCamPose,
                Constants.Vision.kPoseStrategy,
                () -> mAprilTagFieldLayout));
        mCameras.add(new PhotonCameraWrapper(
                Constants.Vision.kBackRightCameraName,
                Constants.Vision.kRobotToBackRightCamPose,
                Constants.Vision.kPoseStrategy,
                () -> mAprilTagFieldLayout));

        mCameras.forEach(camera -> {
            camera.addEstimatedPoseConsumer(estimation -> PoseEstimator.getInstance()
                    .addVisionMeasurement(estimation.estimatedPose.toPose2d(), estimation.timestampSeconds));
            camera.addTargetFilter(target -> target.getPoseAmbiguity() < Constants.Vision.kMaxPoseAmbiguity);
        });
    }

    @Override
    public void readPeriodicInputs() {
        mCameras.forEach(PhotonCameraWrapper::update);
    }

    @Override
    public void writePeriodicOutputs() {}

    @Override
    public void stop() {}

    @Override
    public void outputTelemetry() {
        mCameras.forEach(PhotonCameraWrapper::outputTelemetry);
    }
}
