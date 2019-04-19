
package org.firstinspires.ftc.teamcode;

public class MarvConstantsV3 {

    public static int QUADPACER_TPU = 150;
    public static int QUADPACER_SUBSTEPS = 1;
    public static double[] QUADPACER_POS = new double[] {0, -1.6, 0};

    public static int AP_COUNTS_TO_STABLE = 5;
    public static double AP_NAV_UNITS_TO_STABLE = 0.7;
    public static double AP_ORIENT_UNITS_TO_STABLE = 0.025;


    public static int EXPANDO_DIAG_UP_FAR_POSITION = 1225;
    public static int EXPANDO_DIAG_UP_MID_POSITION = 1025;
    public static int EXPANDO_DIAG_UP_NEAR_POSITION = 1025;

    public static int EXPANDO_DIAG_SAFE_POSITION = 200;

    public static double EXPANDO_DIAG_UP_POWER = 1.0;
    public static double EXPANDO_DIAG_SAFE_POWER = 0.75;
    public static double EXPANDO_DIAG_STALIN_POWER = 0.25;

    public static long EXPANDO_DIAG_STALIN_TIME = 500;
    public static long EXPANDO_DIAG_DROP_TIME = 1250;
    public static long EXPANDO_DIAG_UNDROP_TIME = 250;

    public static long EXPANDO_DIAG_PAUSE_TIME = 200;


    public static double DROP_ANGLE_FLAT = 0.5-0.05;
    public static double DROP_ANGLE_NEAR = 0.75-0.05;
    public static double DROP_ANGLE_FAR = 0.8-0.05;
    public static double DROP_ANGLE_MID = 0.8-0.05;

    public static double SWOP_ANGLE_NORMAL = 0.492;
    public static double SWOP_ANGLE_SWOPPED = 0.65;

    public static int EXPANDO_HORIZ_UP = 1800;
    public static int EXPANDO_HORIZ_DOWN = 115;
    public static int EXPANDO_HORIZ_SAFE = 385; // 370
    public static int EXPANDO_HORIZ_FLYING_LIMIT = (int)(EXPANDO_HORIZ_UP*0.66);

    public static double HORIZ_LIFT_UP_NEUTRAL = .38;
    public static double HORIZ_LIFT_UP_WAITING = .68;
    public static double HORIZ_LIFT_UP_DUMPING = .88; // .90
    public static double HORIZ_LIFT_DOWN = 0.14;

    // expando vert

    public static int EXPANDO_VERT_STOP = 7300;
    public static int EXPANDO_VERT_EXTRA = 250;

    // UC stuff

    public static long UC_HORIZLIFT_TOUP_MS = 600;
    public static long UC_HORIZLIFT_TOWAIT_MS = 500;
    public static long UC_HORIZLIFT_TODUMP_MS = 0;

    public static double UC_HORIZSPIN_EJECT = 0.85;
    public static double UC_HORIZSPIN_INTAKE = 0.85;
    public static double UC_HORIZSPIN_TRANSFER = 0.85;
    public static double UC_HORIZSPIN_HOLD = 0.15;

    public static int UC_EXPANDOHORIZ_BUF = 50;


    // new expando-horiz state machine - timings in millis

    public static long EHSM_UP = 400;
    public static long EHSM_TRANSFER = 400;

    // auto

    public static int AUTO_MARKER;
    public static int AUTO_SAMPLE_NEAR;
    public static int AUTO_SAMPLE_MID;
    public static int AUTO_SAMPLE_FAR;
    public static int AUTO_SAMPLE_MORE;
}
