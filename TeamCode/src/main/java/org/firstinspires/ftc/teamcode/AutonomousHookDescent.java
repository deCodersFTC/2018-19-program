/* Copyright (c) 2017 FIRST. All rights reserved.

 * Written by deCoders Robotics Team
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */



@Autonomous(name="Hook Descend Code", group="Linear Opmode")
//@Disabled
public class AutonomousHookDescent extends LinearOpMode implements Runnable {

    // thread to run the autonomous code
    private Thread auto;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lift = null;
    private DcMotor LeftDriveFront = null;
    private DcMotor RightDriveFront = null;
    private DcMotor LeftDriveBack = null;
    private DcMotor RightDriveBack = null;

    public boolean bob = false;

    DistanceSensor heightSensor;

    public AutonomousHookDescent() {
        auto = new Thread(this);
    }

    public void run() {
        autonomousOperation();
    }


    private void autonomousOperation() {
        // keep extending arm till the all 4 wheels touch the floor


        while (heightSensor.getDistance(DistanceUnit.INCH) > DriveConstants.ZERO_HEIGHT){
            lift.setPower(1.0);
            //descending robot
        }
        // robot has landed.
        if (Thread.interrupted()) {
            return;
        }

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // keep moving arm up further for additional ARM_LIFT_OFFSE_TIME
        lift.setTargetPosition(DriveConstants.ARM_LIFT_OFFSET);

        if (Thread.interrupted()) {
            return;
        }

        // move the robot to the side to extricate the arm from the
        // lander for MOVE_TIME seconds
        for (long stop=System.nanoTime()+TimeUnit.SECONDS.toNanos(DriveConstants.MOVE_TIME);stop>System.nanoTime();) {
            LeftDriveFront.setPower(1.0);
            RightDriveFront.setPower(-1.0);
            LeftDriveBack.setPower(-1.0);
            RightDriveBack.setPower(1.0);
            //uses the mecanum wheels to move robot sideways and out of the hook for MOVE_TIME seconds
        }

        while (heightSensor.getDistance(DistanceUnit.INCH) > DriveConstants.ZERO_HEIGHT){
            lift.setPower(-1.0);
            //shrinks the arm back down
        }

        for (long stop=System.nanoTime()+TimeUnit.SECONDS.toNanos(DriveConstants.BACKUP_TIME);stop>System.nanoTime();) {
            LeftDriveFront.setPower(-1.0);
            RightDriveFront.setPower(-1.0);
            LeftDriveBack.setPower(-1.0);
            RightDriveBack.setPower(-1.0);
            //reverses the robot and backes it away from the tower for 5 seconds
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Distance from Ground", heightSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();

    }


    @Override
    public void runOpMode() {

        bob = true;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Lift initialization
        lift = hardwareMap.get(DcMotor.class, DriveConstants.HOOK_DEVICE_NAME);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotor.Direction.FORWARD);

        // Heght sensor
        heightSensor = hardwareMap.get(DistanceSensor.class, DriveConstants.HEIGHT_SENSOR_NAME);

        // wheel initialization
        LeftDriveFront = hardwareMap.get(DcMotor.class, DriveConstants.LEFT_FRONT_WHEEL_NAME);
        RightDriveFront = hardwareMap.get(DcMotor.class, DriveConstants.RIGHT_FRONT_WHEEL_NAME);
        LeftDriveBack = hardwareMap.get(DcMotor.class, DriveConstants.LEFT_REAR_WHEEL_NAME);
        RightDriveBack = hardwareMap.get(DcMotor.class, DriveConstants.RIGHT_REAR_WHEEL_NAME);


        LeftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        RightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        LeftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        RightDriveBack.setDirection(DcMotor.Direction.REVERSE);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP
        // or the timer expires)

        // start the autonomous operation
        auto.start();
        while (opModeIsActive()) {
        }

        // stop the robot operation.
        auto.interrupt();
        lift.setPower(0);

    }
}
