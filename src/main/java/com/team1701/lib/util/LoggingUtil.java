package com.team1701.lib.util;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

public class LoggingUtil {
    public static void put(LogTable table, String key, Rotation2d rotation) {
        table.put(key, rotation.getRadians());
    }

    public static Rotation2d getRotation2d(LogTable table, String key) {
        return getRotation2d(table, key, GeometryUtil.kRotationIdentity);
    }

    public static Rotation2d getRotation2d(LogTable table, String key, Rotation2d defaultValue) {
        return new Rotation2d(table.get(key, defaultValue.getRadians()));
    }

    public static void put(LogTable table, String prefix, Transform3d transform) {
        put(table, prefix + "/Translation", transform.getTranslation());
        put(table, prefix + "/Rotation", transform.getRotation());
    }

    public static Transform3d getTransform3d(LogTable table, String prefix) {
        return getTransform3d(table, prefix, GeometryUtil.kTransform3dIdentity);
    }

    public static Transform3d getTransform3d(LogTable table, String prefix, Transform3d defaultValue) {
        return new Transform3d(
                getTranslation3d(table, prefix + "/Translation", defaultValue.getTranslation()),
                getRotation3d(table, prefix + "/Rotation", defaultValue.getRotation()));
    }

    public static void put(LogTable table, String prefix, Translation3d translation) {
        table.put(prefix + "/X", translation.getX());
        table.put(prefix + "/Y", translation.getY());
        table.put(prefix + "/Z", translation.getZ());
    }

    public static Translation3d getTranslation3d(LogTable table, String prefix) {
        return getTranslation3d(table, prefix, GeometryUtil.kTranslation3dIdentity);
    }

    public static Translation3d getTranslation3d(LogTable table, String prefix, Translation3d defaultValue) {
        return new Translation3d(
                table.get(prefix + "/X", defaultValue.getX()),
                table.get(prefix + "/Y", defaultValue.getY()),
                table.get(prefix + "/Z", defaultValue.getZ()));
    }

    public static void put(LogTable table, String prefix, Rotation3d rotation) {
        var quaternion = rotation.getQuaternion();
        table.put(prefix + "/W", quaternion.getW());
        table.put(prefix + "/X", quaternion.getX());
        table.put(prefix + "/Y", quaternion.getY());
        table.put(prefix + "/Z", quaternion.getZ());
    }

    public static Rotation3d getRotation3d(LogTable table, String prefix) {
        return getRotation3d(table, prefix, GeometryUtil.kRotation3dIdentity);
    }

    public static Rotation3d getRotation3d(LogTable table, String prefix, Rotation3d defaultValue) {
        var defaultQuaternion = defaultValue.getQuaternion();
        return new Rotation3d(new Quaternion(
                table.get(prefix + "/W", defaultQuaternion.getW()),
                table.get(prefix + "/X", defaultQuaternion.getX()),
                table.get(prefix + "/Y", defaultQuaternion.getY()),
                table.get(prefix + "/Z", defaultQuaternion.getZ())));
    }

    public static void recordOutput(String prefix, Translation2d translation) {
        Logger.recordOutput(prefix + "/X", translation.getX());
        Logger.recordOutput(prefix + "/Y", translation.getY());
    }

    public static void recordOutput(String prefix, ChassisSpeeds chassisSpeeds) {
        Logger.recordOutput(prefix + "/Vx", chassisSpeeds.vxMetersPerSecond);
        Logger.recordOutput(prefix + "/Vy", chassisSpeeds.vyMetersPerSecond);
        Logger.recordOutput(prefix + "/AngularVelocity", chassisSpeeds.omegaRadiansPerSecond);
    }
}
