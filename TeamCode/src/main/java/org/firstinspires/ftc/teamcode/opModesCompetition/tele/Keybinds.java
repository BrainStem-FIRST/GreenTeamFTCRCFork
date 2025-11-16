package org.firstinspires.ftc.teamcode.opModesCompetition.tele;

import org.firstinspires.ftc.teamcode.utils.generalOpModes.GamepadTracker;

import java.util.HashMap;
import java.util.function.Predicate;

public class Keybinds {
    public enum D1Trigger {
        TOGGLE_INTAKE,
        MANUAL_OUTTAKE,
        AUTO_AIM,
        TRACK_SHOOTER_SPEED,
        START_SHOOTING,
        CONTINUE_SHOOTING,
        TRACK_POSE_PREDICT
    }
    public enum D2Trigger {
        INCREASE_SHOOTER_SPEED_STATE,
        DECREASE_SHOOTER_SPEED_STATE,
        TOGGLE_PASSIVE_INTAKE,
        SHOOT_NEAR,
        SHOOT_FAR,
        APPLY_PINPOINT_RESET_POWERS,
        RESET_PINPOINT,
        TOGGLE_PARK_FOR_SHOOT,
        ENTER_PARK_MODE,
        RAISE_PARK
    }

    public final GamepadTracker g1, g2;
    private final HashMap<D1Trigger, Predicate<GamepadTracker>> gt1Triggers;
    private final HashMap<D2Trigger, Predicate<GamepadTracker>> gt2Triggers;


    public Keybinds(GamepadTracker g1, GamepadTracker g2) {
        this.g1 = g1;
        this.g2 = g2;
        gt1Triggers = new HashMap<>();
        gt2Triggers = new HashMap<>();

        gt1Triggers.put(D1Trigger.TOGGLE_INTAKE, GamepadTracker::isRTClicked);
        gt1Triggers.put(D1Trigger.MANUAL_OUTTAKE, GamepadTracker::isLBPressed);
        gt1Triggers.put(D1Trigger.AUTO_AIM, GamepadTracker::isLTPressed);
        gt1Triggers.put(D1Trigger.TRACK_SHOOTER_SPEED, GamepadTracker::isRBClicked);
        gt1Triggers.put(D1Trigger.START_SHOOTING, GamepadTracker::isRBClicked);
        gt1Triggers.put(D1Trigger.CONTINUE_SHOOTING, GamepadTracker::isRBPressed);
        gt1Triggers.put(D1Trigger.TRACK_POSE_PREDICT, GamepadTracker::isBackButtonPressed);

        gt2Triggers.put(D2Trigger.INCREASE_SHOOTER_SPEED_STATE, GamepadTracker::isYClicked);
        gt2Triggers.put(D2Trigger.DECREASE_SHOOTER_SPEED_STATE, GamepadTracker::isAClicked);
        gt2Triggers.put(D2Trigger.TOGGLE_PASSIVE_INTAKE, GamepadTracker::isBClicked);
        gt2Triggers.put(D2Trigger.SHOOT_FAR, GamepadTracker::isDpadUpClicked);
        gt2Triggers.put(D2Trigger.SHOOT_NEAR, GamepadTracker::isDpadDownClicked);
        gt2Triggers.put(D2Trigger.APPLY_PINPOINT_RESET_POWERS, GamepadTracker::isRTPressed);
        gt2Triggers.put(D2Trigger.RESET_PINPOINT, GamepadTracker::isXClicked);
        gt2Triggers.put(D2Trigger.TOGGLE_PARK_FOR_SHOOT, GamepadTracker::isXClicked);
        gt2Triggers.put(D2Trigger.ENTER_PARK_MODE, GamepadTracker::isLTClicked);
        gt2Triggers.put(D2Trigger.RAISE_PARK, GamepadTracker::isLTClicked);
    }


    public boolean check(D1Trigger trigger) {
        if (g1 == null)
            return false;
        if (gt1Triggers.get(trigger) == null)
            throw new IllegalStateException("d1 trigger " + trigger + " is not defined in Keybinds");
        return gt1Triggers.get(trigger).test(g1);
    }

    public boolean check(D2Trigger trigger) {
        if (g2 == null)
            return false;
        if (gt2Triggers.get(trigger) == null)
            throw new IllegalStateException("d2 trigger " + trigger + " is not defined in Keybinds");
        return gt2Triggers.get(trigger).test(g2);
    }
}