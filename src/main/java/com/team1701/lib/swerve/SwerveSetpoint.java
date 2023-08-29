package com.team1701.lib.swerve;

import java.util.Arrays;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveSetpoint {
    public ChassisSpeeds mChassisSpeeds;
    public SwerveModuleState[] mModuleStates;

    public SwerveSetpoint(int moduleCount) {
        this.mChassisSpeeds = new ChassisSpeeds();
        this.mModuleStates = new SwerveModuleState[moduleCount];
        Arrays.setAll(mModuleStates, i -> new SwerveModuleState());
    }

    public SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] initialStates) {
        this.mChassisSpeeds = chassisSpeeds;
        this.mModuleStates = initialStates;
    }

    @Override
    public String toString() {
        var ret = mChassisSpeeds.toString() + "\n";
        for (var i = 0; i < mModuleStates.length; ++i) {
            ret += "  " + mModuleStates[i].toString() + "\n";
        }
        return ret;
    }
}
