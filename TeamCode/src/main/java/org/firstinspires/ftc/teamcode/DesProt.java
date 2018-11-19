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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;
/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="DesProt", group="Iterative Opmode")
@Disabled
public class DesProt extends OpMode {
    public DcMotor  LeftDriveFront = null;
    public DcMotor  RightDriveFront = null;
    public DcMotor  LeftDriveBack = null;
    public DcMotor  RightDriveBack = null;

    public DcMotor  lift = null;

    public DistanceSensor heightSensor = null;
    public DistanceSensor frontSensor = null;
//    public DistanceSensor rearSensor = null;
//    public DistanceSensor rightSensor = null;

    public ColorSensor sensorColor = null;
    public DistanceSensor sensorDistance = null;

    BNO055IMU imu;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    //Encoder Variable
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1;
    static final double     TURN_SPEED              = 1;
    static final double     SLIDE_SPEED             = 1;



    @Override
    public void init() {
        //InitWheels
        LeftDriveFront  = hardwareMap.get(DcMotor.class, "LeftDriveFront");
        RightDriveFront = hardwareMap.get(DcMotor.class, "RightDriveFront");
        LeftDriveBack    = hardwareMap.get(DcMotor.class, "LeftDriveBack");
        RightDriveBack  = hardwareMap.get(DcMotor.class, "RightDriveBack");
        LeftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        RightDriveFront.setDirection(DcMotor.Direction.FORWARDS);
        LeftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        RightDriveBack.setDirection(DcMotor.Direction.FORWARDS);

        LeftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        LeftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //InitLift
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotor.Direction.FORWARD);
        //Init DistanceSensors
        heightSensor=hardwareMap.get(DistanceSensor.class, "Height");
        //frontSensor = hardwareMap.get(DistanceSensor.class, "FrontD");
//        rearSensor = hardwareMap.get(DistanceSensor.class, "RearD");
//        rightSensor = hardwareMap.get(DistanceSensor.class, "RightD");
        //Init ColorSensor
        sensorColor = hardwareMap.get(ColorSensor.class, "Color");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "Color");
        telemetry.addData("Status", "Initialized");
        }

    public void stopAllMotors(){
    LeftDriveBack.setPower(0);
    RightDriveBack.setPower(0);
    LeftDriveFront.setPower(0);
    RightDriveFront.setPower(0);
    lift.setPower(0);
    }


    public void encoderDrive(double speed, double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (loopActive) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = LeftDriveFront.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newRightFrontTarget = RightDriveFront.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = LeftDriveBack.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = RightDriveBack.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);


            LeftDriveFront.setTargetPosition(newLeftFrontTarget);
            RightDriveFront.setTargetPosition(newRightFrontTarget);
            LeftDriveBack.setTargetPosition(newLeftBackTarget);
            RightDriveBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
             LeftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             RightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             LeftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             RightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
             LeftDriveBack.setPower(Math.abs(speed));
             LeftDriveFront.setPower(Math.abs(speed));
             RightDriveBack.setPower(Math.abs(speed));
             RightDriveFront.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (loopActive && (runtime.seconds() < timeoutS) && ( LeftDriveBack.isBusy() &&  RightDriveBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d: %7d", newLeftBackTarget,  newLeftFrontTarget, newRightBackTarget, newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", LeftDriveFront.getCurrentPosition(), LeftDriveBack.getCurrentPosition(), RightDriveBack.getCurrentPosition(), RightDriveFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            stopAllMotors();
            //Set to RUN_USING_ENCODER
            LeftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void Forwards(double distance){
        encoderDrive(DRIVE_SPEED,distance,distance,distance,distance, 5);
    }
    public void Backwards(double distance){
        encoderDrive(DRIVE_SPEED, -distance, -distance, -distance, -distance, 5);
    }
    public void TurnRight(double a){
        double degrees = a * 12/90;
        encoderDrive(TURN_SPEED, degrees, -degrees, degrees,- degrees,5);
    }
    public void TurnLeft(double a){
        double degrees = a * 12/90;
        encoderDrive(TURN_SPEED, -degrees, degrees, -degrees, degrees, 5);
    }
    public void slideRight(double distance){
        encoderDrive(SLIDE_SPEED,distance,-distance,-distance,distance,5);
    }
    public void slideLeft(double distance){
        encoderDrive(SLIDE_SPEED,-distance,distance,distance,-distance,5);
    }

    @Override
    public void init_loop() {
        stopAllMotors();
        telemetry.addData("Status","Waiting for User ");
        telemetry.update();

    }

    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    public boolean loopActive;
    @Override
    public void loop() {
        loopActive = true;
        //Testing Encoders----> To Test on Next Date at Playing Field
//        Forwards(36);
//        Backwards(36);
//        TurnLeft(90);
//        TurnRight(90);
//        slideLeft(36);
//        slideRight(36);

        //Actual Autonomous
        //Slide hook out
        Forwards(1);
        //Turn away from lander
        TurnLeft(90);
        //Move away from lander
        Forwards(18);
        //Move so robot is facing right mineral
        slideLeft(18);
        //Move closer to mineral
        Forwards(9);


        int whileScanning = 0;
        boolean ifGold;
        boolean redOn = false;
        boolean greenOn = false;
        boolean blueOn = false;
        ifGold = false;
        while(ifGold == false){
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("RedOn", redOn);
            telemetry.addData("GreenOn", greenOn);
            telemetry.addData("BlueOn", blueOn);
            telemetry.update();

            if((sensorColor.red()^2) =>525 && ((sensorColor).red()^2) <= 725){
                redOn = true;
            }
            if((sensorColor.green()^2)=>224 && (sensorColor.green()^2)<=424){
                greenOn = true;
            }
            if((sensorColor.blue()^2)=>476 && (sensorColor.blue()^2<=676){
                blueOn = true;
            }


            if (blueOn && redOn && greenOn){
                Forwards(10);
                Backwards(10);
                ifGold=true;
                whileScanning++;
            }
            else{
                slideRight(2.8);
                whileScanning++;
            }

        }
        if(whileScanning == 1 && ifGold==true){
            telemetry.addData("Gold Position", "1");
            telemetry.update();
        }
        if(whileScanning == 2 && ifGold== true){
            slideLeft(16.4);
            telemetry.addData("Gold Position", "2");
            telemetry.update();
        }
        if(whileScanning == 3 && ifGold==true){
            slideLeft(33.6);
            telemetry.addData("Gold Position", "3");
            telemetry.update();
        }

        TurnLeft(35);
        Forwards(10);
        TurnLeft(35);
        Forwards(70);
        //Drop team Marker into Depot

        Backwards(108);
        //Reached Crater
        telemetry.addData("Reached","Crater");
        telemetry.update();        










    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
     
    public void stop(){
        stopAllMotors();
        loopActive = false;
    }

}
