package com.team1701.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;

import com.team1701.lib.cameras.AprilTagCamera;
import com.team1701.lib.cameras.AprilTagCameraIO;
import com.team1701.robot.Constants;
import com.team1701.robot.Robot;
import com.team1701.robot.estimation.PoseEstimator;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class Vision extends SubsystemBase {
    private final PoseEstimator mPoseEstimator = PoseEstimator.getInstance();
    private final ArrayList<AprilTagCamera> mCameras = new ArrayList<AprilTagCamera>();
    private AprilTagFieldLayout mAprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    private Optional<VisionSystemSim> mVisionSim = Optional.empty();

    public Vision(
            AprilTagCameraIO cameraIOFrontLeft,
            AprilTagCameraIO cameraIOFrontRight,
            AprilTagCameraIO cameraIOBackLeft,
            AprilTagCameraIO cameraIOBackRight) {
        Supplier<AprilTagFieldLayout> fieldLayoutSupplier = () -> mAprilTagFieldLayout;

        mCameras.add(new AprilTagCamera(
                Constants.Vision.kFrontLeftCameraName,
                cameraIOFrontLeft,
                Constants.Vision.kRobotToFrontLeftCamPose,
                Constants.Vision.kPoseStrategy,
                fieldLayoutSupplier,
                mPoseEstimator::getPose3d));
        mCameras.add(new AprilTagCamera(
                Constants.Vision.kFrontRightCameraName,
                cameraIOFrontRight,
                Constants.Vision.kRobotToFrontRightCamPose,
                Constants.Vision.kPoseStrategy,
                fieldLayoutSupplier,
                mPoseEstimator::getPose3d));
        mCameras.add(new AprilTagCamera(
                Constants.Vision.kBackLeftCameraName,
                cameraIOBackLeft,
                Constants.Vision.kRobotToBackLeftCamPose,
                Constants.Vision.kPoseStrategy,
                fieldLayoutSupplier,
                mPoseEstimator::getPose3d));
        mCameras.add(new AprilTagCamera(
                Constants.Vision.kBackRightCameraName,
                cameraIOBackRight,
                Constants.Vision.kRobotToBackRightCamPose,
                Constants.Vision.kPoseStrategy,
                fieldLayoutSupplier,
                mPoseEstimator::getPose3d));

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
    public void periodic() {
        mCameras.forEach(AprilTagCamera::periodic);
    }

    @Override
    public void simulationPeriodic() {
        if (mVisionSim.isEmpty()) {
            return;
        }

        mVisionSim.get().update(mPoseEstimator.getPose2d());
        Logger.recordOutput("Vision/SimPose", mVisionSim.get().getDebugField().getRobotPose());
    }
}
