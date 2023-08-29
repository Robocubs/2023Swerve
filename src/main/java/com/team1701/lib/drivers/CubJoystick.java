package com.team1701.lib.drivers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Predicate;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import com.team1701.lib.util.DeadZone;
import com.team1701.lib.util.LatchedBoolean;
import edu.wpi.first.wpilibj.Joystick;

public class CubJoystick extends Joystick {
    private static final double kDefaultDeadZone = 0.16;
    private static final int kMaxJoystickAxes = 12;
    private Map<Integer, List<Runnable>> mButtonPressedHandlers = new HashMap<Integer, List<Runnable>>();
    private Map<Integer, List<Runnable>> mButtonReleasedHandlers = new HashMap<Integer, List<Runnable>>();
    private Map<Integer, List<Runnable>> mAxisPressedHandlers = new HashMap<Integer, List<Runnable>>();
    private Map<Integer, List<Runnable>> mAxisReleasedHandlers = new HashMap<Integer, List<Runnable>>();
    private Map<Integer, LatchedBoolean> mAxisPressedLatches = new HashMap<Integer, LatchedBoolean>();
    private Map<Integer, LatchedBoolean> mAxisReleasedLatches = new HashMap<Integer, LatchedBoolean>();
    private Map<Integer, DeadZone> mDeadZones;

    public CubJoystick(int port) {
        super(port);

        var defaultDeadZone = new DeadZone(kDefaultDeadZone);
        mDeadZones = IntStream.range(0, kMaxJoystickAxes)
                .boxed()
                .collect(Collectors.toMap(axis -> axis, axis -> defaultDeadZone));
    }

    public double getXWithDeadZone() {
        return getRawAxisWithDeadZone(getXChannel());
    }

    public double getYWithDeadZone() {
        return getRawAxisWithDeadZone(getYChannel());
    }

    public double getZWithDeadZone() {
        return getRawAxisWithDeadZone(getZChannel());
    }

    public double getRawAxisWithDeadZone(int axis) {
        var deadZone = mDeadZones.get(axis);
        return deadZone.apply(getRawAxis(axis));
    }

    public void setXDeadZone(DeadZone deadZone) {
        setDeadZone(getXChannel(), deadZone);
    }

    public void setYDeadZone(DeadZone deadZone) {
        setDeadZone(getYChannel(), deadZone);
    }

    public void setZDeadZone(DeadZone deadZone) {
        setDeadZone(getZChannel(), deadZone);
    }

    public void setDeadZone(int axis, DeadZone deadZone) {
        mDeadZones.put(axis, deadZone);
    }

    public void onButtonPressed(int button, Runnable handler) {
        mButtonPressedHandlers.putIfAbsent(button, new ArrayList<Runnable>());
        mButtonPressedHandlers.get(button).add(handler);
    }

    public void onButtonReleased(int button, Runnable handler) {
        mButtonReleasedHandlers.putIfAbsent(button, new ArrayList<Runnable>());
        mButtonReleasedHandlers.get(button).add(handler);
    }

    public void onXPressed(Runnable handler) {
        onAxisPressed(getXChannel(), handler);
    }

    public void onXReleased(Runnable handler) {
        onAxisReleased(getXChannel(), handler);
    }

    public void onYPressed(Runnable handler) {
        onAxisPressed(getYChannel(), handler);
    }

    public void onYReleased(Runnable handler) {
        onAxisReleased(getYChannel(), handler);
    }

    public void onZPressed(Runnable handler) {
        onAxisPressed(getZChannel(), handler);
    }

    public void onZReleased(Runnable handler) {
        onAxisReleased(getZChannel(), handler);
    }

    public void onAxisPressed(int axis, Runnable handler) {
        mAxisPressedHandlers.putIfAbsent(axis, new ArrayList<Runnable>());
        mAxisPressedLatches.putIfAbsent(axis, new LatchedBoolean());
        mAxisPressedHandlers.get(axis).add(handler);
    }

    public void onAxisReleased(int axis, Runnable handler) {
        mAxisReleasedHandlers.putIfAbsent(axis, new ArrayList<Runnable>());
        mAxisReleasedLatches.putIfAbsent(axis, new LatchedBoolean());
        mAxisReleasedHandlers.get(axis).add(handler);
    }

    private boolean getRawAxisPressed(int axis) {
        var value = getRawAxisWithDeadZone(axis);
        var latch = mAxisPressedLatches.get(axis);
        return latch.update(value != 0);
    }

    private boolean getRawAxisReleased(int axis) {
        var value = getRawAxisWithDeadZone(axis);
        var latch = mAxisReleasedLatches.get(axis);
        return latch.update(value == 0);
    }

    public void invokeHandlers() {
        invokeHandlersMatchingFilter(mButtonPressedHandlers, this::getRawButtonPressed);
        invokeHandlersMatchingFilter(mButtonReleasedHandlers, this::getRawButtonReleased);
        invokeHandlersMatchingFilter(mAxisPressedHandlers, this::getRawAxisPressed);
        invokeHandlersMatchingFilter(mAxisReleasedHandlers, this::getRawAxisReleased);
    }

    private void invokeHandlersMatchingFilter(Map<Integer, List<Runnable>> handlers, Predicate<Integer> predicate) {
        handlers.entrySet().stream()
                .filter(entry -> predicate.test(entry.getKey()))
                .flatMap(entry -> entry.getValue().stream())
                .forEach(Runnable::run);
    }

    public void resetHandlers() {
        mButtonPressedHandlers.keySet().forEach(this::getRawButtonPressed);
        mButtonReleasedHandlers.keySet().forEach(this::getRawButtonReleased);
        mAxisPressedHandlers.keySet().forEach(this::getRawAxisPressed);
        mAxisReleasedHandlers.keySet().forEach(this::getRawAxisReleased);
    }
}
