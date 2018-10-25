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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="FourWheelDrive", group="deCoders Prototypes")
public class FourWheelDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftDriveFront = null;
    private DcMotor RightDriveFront = null;
    private DcMotor LeftDriveBack = null;
    private DcMotor RightDriveBack = null;

    private DcMotor lift = null;
    DigitalChannel digitalTouch;

    private DcMotor.Direction direction = DcMotor.Direction.FORWARD;
    private boolean atTop = false, atBottom = false;
    private boolean vPadEnabled = true;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        lift  = hardwareMap.get(DcMotor.class, "lift");
        lift.setDirection(direction);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LeftDriveFront  = hardwareMap.get(DcMotor.class, "LeftDriveFront");
        RightDriveFront = hardwareMap.get(DcMotor.class, "RightDriveFront");
        LeftDriveBack  = hardwareMap.get(DcMotor.class, "LeftDriveBack");
        RightDriveBack = hardwareMap.get(DcMotor.class, "RightDriveBack");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        LeftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        RightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        LeftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        RightDriveBack.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("B a=", gamepad1.a);
            telemetry.addData("B y=", gamepad1.y);
            telemetry.addData("Direction=", direction);
            telemetry.addData("vpad=", vPadEnabled);

            double liftPower;

            // if the magSensor is on, find direction and set atTop or atBottom.
            if (digitalTouch.getState() == false) {
                telemetry.addData("Magnet is", digitalTouch.getState());
                telemetry.addData("vPad is", vPadEnabled);
                telemetry.addData("Limit", "hit");
                if (atTop == false && atBottom == false) {
                    if (direction == DcMotor.Direction.REVERSE) {
                        atTop = true;
                        direction = DcMotor.Direction.FORWARD;
                        lift.setPower(1);
                    } else {
                        atBottom = true;
                        direction = DcMotor.Direction.REVERSE;
                        lift.setPower(-1);
                    }
                    vPadEnabled = false;
                }
            } else {
                telemetry.addData("Magnet is", digitalTouch.getState());
                if(atTop || atBottom) {
                    lift.setPower(0);
                    atTop = atBottom = false;
                    telemetry.addData("Limit", "clear");
                }

                if(vPadEnabled == true) {
                    if (gamepad1.a == true) {
                        direction = DcMotor.Direction.REVERSE;
                        lift.setPower(-1);
                    } else if (gamepad1.y == true) {
                        direction = DcMotor.Direction.FORWARD;
                        lift.setPower(1);
                    } else {
                        lift.setPower(0);
                    }
                } else {
                    // when the hand is removed, then reenable the pad
                    if(gamepad1.a == false && gamepad1.y == false) {
                        vPadEnabled = true;
                    }
                }
            }

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            //double slidePower;


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = -Rangeclip(drive + turn, -1.0, 1.0) ;
            rightPower   = -Range.clip(drive - turn, -1.0, 1.0) ;
            //slidePower = -gamepad1.left_stick_x;
            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            //  leftPower  = -gamepad1.left_stick_y ;
            //  rightPower = -gamepad1.left_stick_y ;


            // Send calculated power to wheels
            LeftDriveFront.setPower(leftPower);
            RightDriveFront.setPower(rightPower);
            LeftDriveBack.setPower(leftPower);
            RightDriveBack.setPower(rightPower);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();

            idle();
        }
    }
}