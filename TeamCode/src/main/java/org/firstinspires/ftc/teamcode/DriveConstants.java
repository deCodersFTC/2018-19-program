/**
 * Copyright 2018 deCoders FTC
 */

package org.firstinspires.ftc.teamcode;

public class DriveConstants {
    private static final DriveConstants ourInstance = new DriveConstants();

    public static DriveConstants getInstance() {
        return ourInstance;
    }

    public static double ZERO_HEIGHT = 5.11;
    public static double HOOK_OFFSET = 0.25;
    public static int ARM_LIFT_OFFSET = 5;
    public static int MOVE_TIME = 3;
    public static int BACKUP_TIME = 5;

    public static String HOOK_DEVICE_NAME = "Hook";
    public static String HEIGHT_SENSOR_NAME = "Height";

    public static String FRONT_SENOR_NAME = "FrontD";
    public static String REAR_SENSOR_NAME = "RearD";
    public static String RIGHT_SENSOR_NAME = "RightD";

    public static String LEFT_FRONT_WHEEL_NAME = "LeftDriveFront";
    public static String RIGHT_FRONT_WHEEL_NAME = "RightDriveFront";
    public static String LEFT_REAR_WHEEL_NAME = "LeftDriveBack";
    public static String RIGHT_REAR_WHEEL_NAME = "RightDriveBack";

    private DriveConstants() {
    }
}
