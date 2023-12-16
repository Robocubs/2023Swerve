package com.team1701.robot.subsystems.drive;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.team1701.lib.drivers.motors.MotorIOSparkMax;
import com.team1701.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;

public final class MotorFactory {
    public static MotorIOSparkMax createDriveMotorIOSparkMax(int deviceId, boolean isInverted) {
        var motor = new CANSparkMax(deviceId, MotorType.kBrushless);
        var encoder = motor.getEncoder();
        var controller = motor.getPIDController();

        Consumer<REVLibError> onError =
                (error) -> DriverStation.reportWarning("Failed to configure Spark Max " + deviceId, false);

        motor.setCANTimeout(200);

        // TODO: decide when to burn values
        configureWithRetry(() -> motor.restoreFactoryDefaults(), onError);

        configureWithRetry(() -> motor.setSmartCurrentLimit(80), onError);
        configureWithRetry(() -> motor.enableVoltageCompensation(12), onError);

        configureWithRetry(() -> encoder.setPosition(0), onError);
        configureWithRetry(() -> encoder.setMeasurementPeriod(10), onError);
        configureWithRetry(() -> encoder.setAverageDepth(2), onError);

        configureWithRetry(() -> controller.setP(Constants.Drive.kDriveKp.get()), onError);
        configureWithRetry(() -> controller.setD(Constants.Drive.kDriveKd.get()), onError);
        configureWithRetry(() -> controller.setFF(Constants.Drive.kDriveKf.get()), onError);

        configureWithRetry(() -> motor.burnFlash(), onError);

        motor.setInverted(isInverted);
        motor.setCANTimeout(0);

        return new MotorIOSparkMax(motor);
    }

    public static MotorIOSparkMax createSteerMotorIOSparkMax(int deviceId, boolean isInverted) {
        var motor = new CANSparkMax(deviceId, MotorType.kBrushless);
        var encoder = motor.getEncoder();
        var controller = motor.getPIDController();

        Consumer<REVLibError> onError =
                (error) -> DriverStation.reportWarning("Failed to configure Spark Max " + deviceId, false);

        motor.setCANTimeout(200);

        // TODO: decide when to burn values
        configureWithRetry(() -> motor.restoreFactoryDefaults(), onError);

        configureWithRetry(() -> motor.setSmartCurrentLimit(30), onError);
        configureWithRetry(() -> motor.enableVoltageCompensation(12.0), onError);

        configureWithRetry(() -> encoder.setPosition(0), onError);
        configureWithRetry(() -> encoder.setMeasurementPeriod(10), onError);
        configureWithRetry(() -> encoder.setAverageDepth(2), onError);

        configureWithRetry(() -> controller.setP(Constants.Drive.kSteerKp.get()), onError);
        configureWithRetry(() -> controller.setD(Constants.Drive.kSteerKd.get()), onError);

        configureWithRetry(() -> controller.setPositionPIDWrappingEnabled(true), onError);
        configureWithRetry(() -> controller.setPositionPIDWrappingMinInput(0), onError);
        configureWithRetry(
                () -> controller.setPositionPIDWrappingMaxInput(1.0 / Constants.Drive.kSteerReduction), onError);

        configureWithRetry(() -> motor.burnFlash(), onError);

        motor.setInverted(isInverted);
        motor.setCANTimeout(0);

        return new MotorIOSparkMax(motor);
    }

    private static void configureWithRetry(Supplier<REVLibError> config, Consumer<REVLibError> onFailure) {
        REVLibError error = REVLibError.kUnknown;
        for (var i = 0; i < 4; i++) {
            error = config.get();
            if (error == REVLibError.kOk) {
                return;
            }
        }

        onFailure.accept(error);
    }
}
