package com.team1701.lib.drivers.encoders;

import com.team1701.lib.util.SignalSamplingThread;
import org.littletonrobotics.junction.AutoLog;

public interface EncoderIO {
    @AutoLog
    public static class EncoderInputs {
        public double positionRadians;
    }

    public default void updateInputs(EncoderInputs inputs) {}

    public default void enableYawSampling(SignalSamplingThread samplingThread) {}
}
