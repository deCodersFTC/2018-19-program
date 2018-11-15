/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;


import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbot
{
    /* Public OpMode members. */
    public DcMotor  LeftDriveFront = null;
    public DcMotor  RightDriveFront = null;
    public DcMotor  LeftDriveBack = null;
    public DcMotor  RightDriveBack = null;

    public DcMotor  lift = null;

    public DistanceSensor heightSensor = null;
    public DistanceSensor frontSensor = null;
    public DistanceSensor rearSensor = null;
    public DistanceSensor rightSensor = null;

    public ColorSensor sensorColor = null;
    public DistanceSensor sensorDistance = null;

    BNO055IMU imu;

    public static double ZERO_HEIGHT = 5.11;
    public static double HOOK_OFFSET = 0.25;
    public static int ARM_LIFT_OFFSET = 5;
    public static int MOVE_TIME = 3;
    public static int BACKUP_TIME = 5;




    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, DcMotor LeftDriveFront, DcMotor  RightDriveFront, DcMotor  LeftDriveBack, DcMotor  RightDriveBack, DcMotor lift, DistanceSensor heightSensor, ColorSensor sensorColor, DistanceSensor sensorDistance, BNO055IMU imu) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //Init Wheels
        LeftDriveFront  = hwMap.get(DcMotor.class, "LeftDriveFront");
        RightDriveFront = hwMap.get(DcMotor.class, "RightDriveFront");
        LeftDriveBack    = hwMap.get(DcMotor.class, "LeftDriveBack");
        RightDriveBack  = hwMap.get(DcMotor.class, "RightDriveBack");
        LeftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        RightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        LeftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        RightDriveBack.setDirection(DcMotor.Direction.REVERSE);
        LeftDriveFront.setPower(0);
        RightDriveFront.setPower(0);
        LeftDriveBack.setPower(0);
        RightDriveBack.setPower(0);
        LeftDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Init Lift
        lift = hwMap.get(DcMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotor.Direction.FORWARD);
        //Init DistanceSensors
        heightSensor=hwMap.get(DistanceSensor.class, "Height");
        //Init ColorSensor
        sensorColor = hwMap.get(ColorSensor.class, "Color");
        sensorDistance = hwMap.get(DistanceSensor.class, "Color");
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;






    }
}


