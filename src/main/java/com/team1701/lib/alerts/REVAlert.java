package com.team1701.lib.alerts;

import com.revrobotics.REVLibError;

public class REVAlert {
    private final Alert mAlert;
    private final String mMessageFormat;

    public REVAlert(Object device, int deviceId) {
        this(device.getClass(), deviceId);
    }

    public <T> REVAlert(Class<T> deviceClass, int deviceId) {
        mMessageFormat =
                "[" + deviceClass.getSimpleName() + "] Failed to configure device " + deviceId + " with error %s";
        mAlert = Alert.error(String.format(mMessageFormat, REVLibError.kUnknown));
    }

    public void enable(REVLibError error) {
        mAlert.setMessage(String.format(mMessageFormat, error));
        mAlert.enable();
    }

    public void disable() {
        mAlert.disable();
    }
}
