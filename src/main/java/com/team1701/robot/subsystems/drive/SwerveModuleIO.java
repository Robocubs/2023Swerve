package com.team1701.robot.subsystems.drive;

import com.team1701.lib.drivers.encoders.EncoderIO;
import com.team1701.lib.drivers.encoders.EncoderIOSim;
import com.team1701.lib.drivers.motors.MotorIO;
import com.team1701.lib.drivers.motors.MotorIOSim;
import com.team1701.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class SwerveModuleIO {
    public final MotorIO driveMotorIO;
    public final MotorIO steerMotorIO;
    public final EncoderIO steerEncoderIO;

    public static SwerveModuleIO createSim(DCMotor driveMotor, DCMotor steerMotor) {
        var driveMotorIO =
                new MotorIOSim(driveMotor, Constants.Drive.kDriveReduction, 0.025, Constants.kLoopPeriodSeconds);
        var steerMotorIO =
                new MotorIOSim(steerMotor, Constants.Drive.kSteerReduction, 0.004, Constants.kLoopPeriodSeconds);
        steerMotorIO.enableContinuousInput(0, 2 * Math.PI);
        var encoderOffset = new Rotation2d(Math.random() * 2 * Math.PI);
        var encoderIO = new EncoderIOSim(() -> steerMotorIO
                .getPosition()
                .times(Constants.Drive.kSteerReduction)
                .plus(encoderOffset));
        return new SwerveModuleIO(driveMotorIO, steerMotorIO, encoderIO);
    }

    public SwerveModuleIO(MotorIO driveMotorIO, MotorIO steerMotorIO, EncoderIO steerEncoderIO) {
        this.driveMotorIO = driveMotorIO;
        this.steerMotorIO = steerMotorIO;
        this.steerEncoderIO = steerEncoderIO;
    }
}
