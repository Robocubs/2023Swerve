package com.team1701.robot;

import java.util.Optional;
import java.util.stream.Stream;

import com.team1701.lib.cameras.PhotonCameraIO;
import com.team1701.lib.cameras.PhotonCameraIOPhotonCamera;
import com.team1701.lib.drivers.encoders.EncoderIO;
import com.team1701.lib.drivers.encoders.EncoderIOAnalog;
import com.team1701.lib.drivers.gyros.GyroIO;
import com.team1701.lib.drivers.gyros.GyroIOPigeon2;
import com.team1701.lib.drivers.gyros.GyroIOSim;
import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.robot.Configuration.Mode;
import com.team1701.robot.commands.AutonomousCommands;
import com.team1701.robot.commands.DriveWithJoysticks;
import com.team1701.robot.estimation.PoseEstimator;
import com.team1701.robot.subsystems.drive.Drive;
import com.team1701.robot.subsystems.drive.DriveMotorFactory;
import com.team1701.robot.subsystems.drive.SwerveModuleIO;
import com.team1701.robot.subsystems.vision.Vision;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    protected final Drive drive;
    protected final Vision vision;
    private final CommandXboxController mDriverController = new CommandXboxController(0);
    private final LoggedDashboardChooser<Command> autonomousModeChooser = new LoggedDashboardChooser<>("Auto Mode");

    public RobotContainer() {
        Optional<Drive> drive = Optional.empty();
        Optional<Vision> vision = Optional.empty();

        if (Configuration.getMode() != Mode.REPLAY) {
            switch (Configuration.getRobot()) {
                case SWERVE_BOT:
                    drive = Optional.of(Drive.build(new GyroIOPigeon2(10), new SwerveModuleIO[] {
                        new SwerveModuleIO(
                                DriveMotorFactory.createDriveMotorIOSparkMax(10),
                                DriveMotorFactory.createSteerMotorIOSparkMax(11),
                                new EncoderIOAnalog(0)),
                        new SwerveModuleIO(
                                DriveMotorFactory.createDriveMotorIOSparkMax(12),
                                DriveMotorFactory.createSteerMotorIOSparkMax(13),
                                new EncoderIOAnalog(1)),
                        new SwerveModuleIO(
                                DriveMotorFactory.createDriveMotorIOSparkMax(16),
                                DriveMotorFactory.createSteerMotorIOSparkMax(17),
                                new EncoderIOAnalog(3)),
                        new SwerveModuleIO(
                                DriveMotorFactory.createDriveMotorIOSparkMax(14),
                                DriveMotorFactory.createSteerMotorIOSparkMax(15),
                                new EncoderIOAnalog(2)),
                    }));
                    break;
                case SIMULATION_BOT:
                    var gyroIO = new GyroIOSim(
                            () -> PoseEstimator.getInstance().getPose2d().getRotation());
                    var simDrive = Drive.build(
                            gyroIO,
                            Stream.generate(() -> SwerveModuleIO.createSim(DCMotor.getKrakenX60(1), DCMotor.getNEO(1)))
                                    .limit(Constants.Drive.kNumModules)
                                    .toArray(SwerveModuleIO[]::new));
                    gyroIO.setYawSupplier(() -> simDrive.getVelocity(), Constants.kLoopPeriodSeconds);
                    drive = Optional.of(simDrive);
                    break;
                default:
                    break;
            }

            vision = Optional.of(Vision.build(
                    new PhotonCameraIOPhotonCamera(Constants.Vision.kFrontLeftCameraName),
                    new PhotonCameraIOPhotonCamera(Constants.Vision.kFrontRightCameraName),
                    new PhotonCameraIOPhotonCamera(Constants.Vision.kBackLeftCameraName),
                    new PhotonCameraIOPhotonCamera(Constants.Vision.kBackRightCameraName)));
        }

        this.drive = drive.orElseGet(() -> Drive.build(
                new GyroIO() {},
                Stream.generate(() -> new SwerveModuleIO(new MotorIO() {}, new MotorIO() {}, new EncoderIO() {}))
                        .limit(Constants.Drive.kNumModules)
                        .toArray(SwerveModuleIO[]::new)));

        this.vision = vision.orElseGet(() -> Vision.build(
                new PhotonCameraIO() {}, new PhotonCameraIO() {}, new PhotonCameraIO() {}, new PhotonCameraIO() {}));

        setupControllerBindings();
        setupAutonomousChoices();
    }

    private void setupControllerBindings() {
        drive.setDefaultCommand(new DriveWithJoysticks(
                drive,
                () -> -mDriverController.getLeftY(),
                () -> -mDriverController.getLeftX(),
                () -> -mDriverController.getRightX(),
                () -> mDriverController.leftBumper().getAsBoolean()
                        ? Constants.Drive.kSlowKinematicLimits
                        : Constants.Drive.kFastKinematicLimits));
        mDriverController.x().onTrue(Commands.runOnce(() -> drive.zeroGyroscope()));
    }

    public void setupAutonomousChoices() {
        var commands = new AutonomousCommands(drive);
        autonomousModeChooser.addDefaultOption("Demo", commands.demo());
    }

    public Optional<Command> getAutonomousCommand() {
        return Optional.ofNullable(autonomousModeChooser.get());
    }
}
