package com.team1701.robot;

import com.team1701.lib.drivers.CubJoystick;
import com.team1701.lib.util.DeadZone;

public class ControllerManager {
    private static ControllerManager mInstance;
    private CubJoystick mDriverJoystick;
    private CubJoystick mOperatorJoystick;

    public static final int kXBOXButtonX = 3;
    public static final int kXBOXButtonA = 1;
    public static final int kXBOXButtonB = 2;
    public static final int kXBOXButtonY = 4;
    public static final int kXBOXBumperLeft = 5;
    public static final int kXBOXBumperRight = 6;
    public static final int kXBOXMediaLeft = 7;
    public static final int kXBOXMediaRight = 8;
    public static final int kXBOXButtonLeftJoystick = 9;
    public static final int kXBOXButtonRightJoystick = 10;

    public static final int kXBOXTriggerRight = 3;
    public static final int kXBOXTriggerLeft = 2;
    public static final int kXBOXJoystickLeftXAxis = 0;
    public static final int kXBOXJoystickLeftYAxis = 1;
    public static final int kXBOXJoystickRightXAxis = 4;
    public static final int kXBOXJoystickRightYAxis = 5;

    public static final int kLogitechJoystickLeftXAxis = 0;
    public static final int kLogitechJoystickLeftYAxis = 1;
    public static final int kLogitechJoystickRightXAxis = 2;
    public static final int kLogitechJoystickRightYAxis = 3;

    public static ControllerManager getInstance() {
        if (mInstance == null) {
            mInstance = new ControllerManager();
        }

        return mInstance;
    }

    private ControllerManager() {
        mDriverJoystick = new CubJoystick(0);
        mDriverJoystick.setXChannel(kXBOXJoystickLeftXAxis);
        mDriverJoystick.setYChannel(kXBOXJoystickLeftYAxis);
        mDriverJoystick.setZChannel(kXBOXJoystickRightXAxis);
        mDriverJoystick.setXDeadZone(new DeadZone(Constants.Controls.kDriverXDeadZone));
        mDriverJoystick.setYDeadZone(new DeadZone(Constants.Controls.kDriverYDeadZone));

        mOperatorJoystick = new CubJoystick(1);
        mOperatorJoystick.setYChannel(kXBOXJoystickLeftYAxis);
        mOperatorJoystick.setXChannel(kXBOXJoystickRightXAxis);
    }

    public CubJoystick getDriverJoystick() {
        return mDriverJoystick;
    }

    public CubJoystick getOperatorJoystick() {
        return mOperatorJoystick;
    }

    public CubJoystick getJoystick(JoystickSlot slot) {
        switch (slot) {
            case DRIVER:
                return mDriverJoystick;
            case OPERATOR:
                return mOperatorJoystick;
            default:
                throw new UnsupportedOperationException(slot + " not supported.");
        }
    }

    public void onButtonPressed(JoystickMapping mapping, Runnable handler) {
        getJoystick(mapping.getSlot()).onButtonPressed(mapping.getButton(), handler);
    }

    public void onButtonReleased(JoystickMapping mapping, Runnable handler) {
        getJoystick(mapping.getSlot()).onButtonReleased(mapping.getButton(), handler);
    }

    public boolean getRawButton(JoystickMapping mapping) {
        return getJoystick(mapping.getSlot()).getRawButton(mapping.getButton());
    }

    public void invokeHandlers() {
        mDriverJoystick.invokeHandlers();
        mOperatorJoystick.invokeHandlers();
    }

    public void resetHandlers() {
        mDriverJoystick.resetHandlers();
        mOperatorJoystick.resetHandlers();
    }

    public static class JoystickMapping {
        private final JoystickSlot mSlot;
        private final int mButton;

        public JoystickMapping(JoystickSlot slot, int button) {
            mSlot = slot;
            mButton = button;
        }

        public JoystickSlot getSlot() {
            return mSlot;
        }

        public int getButton() {
            return mButton;
        }
    }

    public enum JoystickSlot {
        DRIVER,
        OPERATOR
    }
}
