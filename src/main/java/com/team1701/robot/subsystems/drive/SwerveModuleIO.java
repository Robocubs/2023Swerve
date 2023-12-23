package com.team1701.robot.subsystems.drive;

import com.team1701.lib.drivers.encoders.EncoderIO;
import com.team1701.lib.drivers.motors.MotorIO;

public class SwerveModuleIO {
    public final MotorIO driveMotorIO;
    public final MotorIO steerMotorIO;
    public final EncoderIO steerEncoderIO;

    public SwerveModuleIO(MotorIO driveMotorIO, MotorIO steerMotorIO, EncoderIO steerEncoderIO) {
        this.driveMotorIO = driveMotorIO;
        this.steerMotorIO = steerMotorIO;
        this.steerEncoderIO = steerEncoderIO;
    }
}
