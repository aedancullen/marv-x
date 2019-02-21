package org.firstinspires.ftc.teamcode;

public class MarvConstantsV2 {

    public static int QUADPACER_TPU = 150;

    public static int AP_COUNTS_TO_STABLE = 5;
    public static double AP_NAV_UNITS_TO_STABLE = 1;
    public static double AP_ORIENT_UNITS_TO_STABLE = 0.05;

    public static int EXPANDO_HORIZ_UP = 1900;
    public static int EXPANDO_HORIZ_SAFE = 340;

    public static double HORIZ_LIFT_UP_NEUTRAL = .5;
    public static double HORIZ_LIFT_UP_WAITING = .90;
    public static double HORIZ_LIFT_UP_DUMPING = .965;
    public static double HORIZ_LIFT_DOWN = 0.14;

    public static double VERT_SWING_CENTER = 0.49;
    public static double VERT_SWING_LEFT = 0.49; /*VERT_SWING_CENTER + .42 / 4.0;*/
    public static double VERT_SWING_RIGHT = 0.49; /*VERT_SWING_CENTER - .42 / 4.0;*/

    public static double VERT_LATCH_LOCKED = 0.02;
    public static double VERT_LATCH_OPEN = 0.5;

    public static double VERT_LIFT_UP = 0.88;
    public static double VERT_LIFT_DOWN = 0.115;
    public static double VERT_LIFT_SAFE = 0.3;

    public static int VERT_LIFT_TOSWING_MILLIS = 500;

    public static int VERT_LIFT_TOMINUSG_MILLIS = 1000;
    public static double VERT_LIFT_MINUSG = VERT_LIFT_DOWN + (0.9 * (VERT_LIFT_UP - VERT_LIFT_DOWN));

    public static int EXPANDO_VERT_UP = 3100; //3200
    public static int EXPANDO_VERT_DOWN = 0;
    public static int EXPANDO_VERT_SAFE = 2800 /*- 140*/;

    public static int EXPANDO_VERT_2IN = 275;

    public static int EXPANDO_VERT_BOXFREE = 1000;

    public static double EXPANDO_VERT_TOUP_SPEED = 1.0;
    public static double EXPANDO_VERT_TODOWN_SPEED = 0.5;
    public static double EXPANDO_VERT_TOSAFE_SPEED = 0.5;
    public static int VERT_SPIN_TODROP_MILLIS = 500;

    public static double VERT_SPIN_NEUTRAL = 0.500;
    public static double VERT_SPIN_R2BACK = 0.85;
    public static double VERT_SPIN_L2BACK = 0.15;
    public static double VERT_SPIN_R2BACK_FARTHER = 1;
    public static double VERT_SPIN_R2BACK_LESSER = 0.61;
    public static double VERT_SPIN_L2BACK_FARTHER = 0;
    public static double VERT_SPIN_L2BACK_LESSER = 0.39;

}
