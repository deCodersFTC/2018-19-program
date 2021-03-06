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

//package org.firstinspires.ftc.robotcontroller.external.samples;
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

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

@TeleOp(name="IntakeMechanism", group="Linear Opmode")
//@Disabled
public class IntakeMechanism extends LinearOpMode {

    // Declare OpMode members.
    private static final int TARGET_POSITION_COLLECT = 0;
    private static final int TARGET_POSITION_DEPOSIT = 1;
    private static final int INTAKE_LIFT_COLLECT_POSITION = -200;
    private static final int INTAKE_LIFT_DEPOSIT_POSITION = -1000;

    private ElapsedTime runtime = new ElapsedTime();
    private CRServo intakeSpin = null;
    private DcMotor intakeLift = null;
    double intakeLiftSpeed;
    //int intakeLiftCollectPosition = 0;
    int intakeTiltCollectPosition = 0;
    int intakeLiftDepositPosition = 0;
    //    int intakeLiftCurrentPosition;
    private DcMotor LeftDriveFront = null;
    private DcMotor RightDriveFront = null;
    private DcMotor LeftDriveBack = null;
    private DcMotor RightDriveBack = null;
    // initiating drive motors

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        intakeSpin = hardwareMap.get(CRServo.class, "intakeSpin");
        intakeLift = hardwareMap.get(DcMotor.class, "intakeLift");

        LeftDriveFront  = hardwareMap.get(DcMotor.class, "LeftDriveFront");
        RightDriveFront = hardwareMap.get(DcMotor.class, "RightDriveFront");
        LeftDriveBack  = hardwareMap.get(DcMotor.class, "LeftDriveBack");
        RightDriveBack = hardwareMap.get(DcMotor.class, "RightDriveBack");

        LeftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        RightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        LeftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        RightDriveBack.setDirection(DcMotor.Direction.REVERSE);

        intakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("Status", "Initialized");
        telemetry.addData("intakeLift Collect Position", INTAKE_LIFT_COLLECT_POSITION);
        telemetry.addData("intakeLift Deposit Position", INTAKE_LIFT_DEPOSIT_POSITION);
        telemetry.addData("intakeLift Current Position", intakeLift.getCurrentPosition());
        telemetry.update();

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //spin.setDirection(CRServo.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double LeftFrontPower;
            double RightFrontPower;
            double LeftBackPower;
            double RightBackPower;

            if (gamepad2.a == true) {
                intakeSpin.setPower(1.0);
            }else if (gamepad2.y == true){
                intakeSpin.setPower(-1.0);
            }else {
                intakeSpin.setPower(0.0);
            }

            // if (gamepad1.a == true) {
            //     intakeLiftSpeed = -0.75;
            //     positionIntakeMechanism(TARGET_POSITION_DEPOSIT);
            // } else if (gamepad1.y){
            //     intakeLiftSpeed = 0.75;
            //     positionIntakeMechanism(TARGET_POSITION_COLLECT);
            // }

            if (gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0) {
                intakeLiftSpeed = gamepad2.right_trigger - gamepad2.left_trigger;
                if (intakeLiftSpeed >= 0.35) {
                    intakeLiftSpeed = 0.35;
                }else if (intakeLiftSpeed <= -0.65) {
                    intakeLiftSpeed = -0.65;
                }
                intakeLift.setPower(intakeLiftSpeed);
            }else{
                intakeLift.setPower(0.0);
            }

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = gamepad1.left_stick_y;
            double turn  = -gamepad1.right_stick_x;
            double slide = gamepad1.left_stick_x;

            //Constructing
            LeftFrontPower = drive + slide + turn;
            RightFrontPower = drive - slide - turn;
            LeftBackPower = drive - slide + turn;
            RightBackPower = drive + slide - turn;

            if(gamepad1.right_stick_x != 0 && (gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0)){
                LeftFrontPower /= 2;
                RightFrontPower /= 2;
                LeftBackPower /= 2;
                RightBackPower /= 2;
            }

            // Send calculated power to wheels
            LeftDriveFront.setPower(LeftFrontPower);
            RightDriveFront.setPower(RightFrontPower);
            LeftDriveBack.setPower(LeftBackPower);
            RightDriveBack.setPower(RightBackPower);

            //Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Written by", "Intake SubTeam");
            telemetry.addData("intakeSpin Motor Speed", intakeSpin.getPower());
            telemetry.addData("intakeLift Motor Speed", intakeLift.getPower());
            telemetry.addData("intakeLift.getCurrentPosition", intakeLift.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }

    public void positionIntakeMechanism(int targetPosition) {
        telemetry.addData("intakeLift Current Mode", intakeLift.getMode());
        DcMotor.RunMode previousMode = intakeLift.getMode();
        telemetry.addData("intakeLift Current Position", intakeLift.getCurrentPosition());
        intakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (targetPosition == TARGET_POSITION_COLLECT) {
            intakeLift.setTargetPosition(INTAKE_LIFT_COLLECT_POSITION);
        }else if (targetPosition == TARGET_POSITION_DEPOSIT) {
            intakeLift.setTargetPosition(INTAKE_LIFT_DEPOSIT_POSITION);
        }
        intakeLift.setPower(intakeLiftSpeed);

        telemetry.addData("Intake Target Position", targetPosition);
        telemetry.addData("intakeLiftSpeed", intakeLiftSpeed);
        telemetry.addData("intakeLift Current Position", intakeLift.getCurrentPosition());

        telemetry.update();
        intakeLift.setMode(previousMode);
    }
}