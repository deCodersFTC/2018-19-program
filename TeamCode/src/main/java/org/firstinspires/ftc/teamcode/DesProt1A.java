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

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import java.util.Locale;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="DesProt1A", group="Linear Opmode")

public class DesProt1A extends LinearOpMode {


    // Declare OpMode members.
    public DcMotor  LeftDriveFront;
    public DcMotor  RightDriveFront;
    public DcMotor  LeftDriveBack;
    public DcMotor  RightDriveBack;

    public DcMotor  lift;
    private DistanceSensor heightSensor = null;


    public DistanceSensor frontSensor = null;

    Orientation angles;

    BNO055IMU imu;
    double origAngle;


    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.6;
    static final double     SLIDE_SPEED             = 0.6;




    public void orientationChecker(double angleTarget){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double newAngle;
        double difference;
        int checknum = 0;
        newAngle = angles.firstAngle;
        difference = newAngle-angleTarget;

        while(checknum<=3 && difference!=0){

            TurnRight(difference);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            newAngle = angles.firstAngle;
            difference = newAngle-angleTarget;
            telemetry.addData("NewAngle", newAngle);
            telemetry.addData("Difference", difference);
            telemetry.addData("Original Angle", origAngle);
            telemetry.addData("AngleTarget", angleTarget);
            telemetry.update();
            telemetry.clear();
            checknum++;
        }
    }



    public void stopAllMotors(){
        LeftDriveBack.setPower(0);
        RightDriveBack.setPower(0);
        LeftDriveFront.setPower(0);
        RightDriveFront.setPower(0);
        lift.setPower(0);
    }

    public void descent(double speed, double descentInches){
        int newDescentTarget;

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newDescentTarget = lift.getCurrentPosition() + (int)(descentInches * COUNTS_PER_INCH);


            lift.setTargetPosition(newDescentTarget);

            // Turn On RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            lift.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void encoderDrive(double speed, double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

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
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && ( LeftDriveBack.isBusy() &&  RightDriveBack.isBusy())) {


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

    public void Backwards(double distance){
        encoderDrive(DRIVE_SPEED,distance,distance,distance,distance, 5);
    }
    public void Forwards(double distance){
        encoderDrive(DRIVE_SPEED, -distance, -distance, -distance, -distance, 5);
    }
    public void TurnLeft(double a){
        double degrees = a * 24/90;
        encoderDrive(TURN_SPEED, degrees, -degrees, degrees,- degrees,5);
    }
    public void TurnRight(double a){
        double degrees = a * 24/90;
        encoderDrive(TURN_SPEED, -degrees, degrees, -degrees, degrees, 5);
    }
    public void slideLeft(double distance){
        encoderDrive(SLIDE_SPEED,-distance,distance,distance,-distance,5);
    }
    public void slideRight(double distance){
        encoderDrive(SLIDE_SPEED,distance,-distance,-distance,distance,5);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void runOpMode() {

        //InitWheels

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        LeftDriveFront  = hardwareMap.get(DcMotor.class, "LeftDriveFront");
        RightDriveFront = hardwareMap.get(DcMotor.class, "RightDriveFront");
        LeftDriveBack    = hardwareMap.get(DcMotor.class, "LeftDriveBack");
        RightDriveBack  = hardwareMap.get(DcMotor.class, "RightDriveBack");

        LeftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        RightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        LeftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        RightDriveBack.setDirection(DcMotor.Direction.REVERSE);

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
        heightSensor = hardwareMap.get(DistanceSensor.class, "Height");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotor.Direction.FORWARD);
        //Init DistanceSensors


        LeftDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stopAllMotors();
        telemetry.addData("Status","Waiting for User ");
        telemetry.update();
        waitForStart();
        runtime.reset();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        origAngle = angles.firstAngle;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);



            if(heightSensor.getDistance(DistanceUnit.INCH) > 4){
                lift.setPower(1.0);

            }
            else{
                lift.setPower(0);
                slideLeft(0.1);
            }


            Forwards(3);
            /*e Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import java.util.Locale;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
            @Autonomous(name="DesProt1A", group="Linear Opmode")

            public class DesProt1A extends LinearOpMode {


                // Declare OpMode members.
                public DcMotor  LeftDriveFront;
                public DcMotor  RightDriveFront;
                public DcMotor  LeftDriveBack;
                public DcMotor  RightDriveBack;

                public DcMotor  lift;
                private DistanceSensor heightSensor = null;


                public DistanceSensor frontSensor = null;

                Orientation angles;

                BNO055IMU imu;
                double origAngle;


                private ElapsedTime runtime = new ElapsedTime();

                static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
                static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
                static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
                static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
                static final double     DRIVE_SPEED             = 0.6;
                static final double     TURN_SPEED              = 0.6;
                static final double     SLIDE_SPEED             = 0.6;




                public void orientationChecker(double angleTarget){
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    double newAngle;
                    double difference;
                    int checknum = 0;
                    newAngle = angles.firstAngle;
                    difference = newAngle-angleTarget;

                    while(checknum<=3 && difference!=0){

                        TurnRight(difference);
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        newAngle = angles.firstAngle;
                        difference = newAngle-angleTarget;
                        telemetry.addData("NewAngle", newAngle);
                        telemetry.addData("Difference", difference);
                        telemetry.addData("Original Angle", origAngle);
                        telemetry.addData("AngleTarget", angleTarget);
                        telemetry.update();
                        telemetry.clear();
                        checknum++;
                    }
                }



                public void stopAllMotors(){
                    LeftDriveBack.setPower(0);
                    RightDriveBack.setPower(0);
                    LeftDriveFront.setPower(0);
                    RightDriveFront.setPower(0);
                    lift.setPower(0);
                }

                public void descent(double speed, double descentInches){
                    int newDescentTarget;

                    if (opModeIsActive()) {

                        // Determine new target position, and pass to motor controller
                        newDescentTarget = lift.getCurrentPosition() + (int)(descentInches * COUNTS_PER_INCH);


                        lift.setTargetPosition(newDescentTarget);

                        // Turn On RUN_TO_POSITION
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        // reset the timeout time and start motion.
                        runtime.reset();
                        lift.setPower(Math.abs(speed));

                        // keep looping while we are still active, and there is time left, and both motors are running.
                        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                        // its target position, the motion will stop.  This is "safer" in the event that the robot will
                        // always end the motion as soon as possible.
                        // However, if you require that BOTH motors have finished their moves before the robot continues
                        // onto the next step, use (isBusy() || isBusy()) in the loop test.

                        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                }


                public void encoderDrive(double speed, double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches, double timeoutS) {
                    int newLeftFrontTarget;
                    int newRightFrontTarget;
                    int newLeftBackTarget;
                    int newRightBackTarget;

                    // Ensure that the opmode is still active
                    if (opModeIsActive()) {

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
                        while (opModeIsActive() && (runtime.seconds() < timeoutS) && ( LeftDriveBack.isBusy() &&  RightDriveBack.isBusy())) {


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

                public void Backwards(double distance){
                    encoderDrive(DRIVE_SPEED,distance,distance,distance,distance, 5);
                }
                public void Forwards(double distance){
                    encoderDrive(DRIVE_SPEED, -distance, -distance, -distance, -distance, 5);
                }
                public void TurnLeft(double a){
                    double degrees = a * 24/90;
                    encoderDrive(TURN_SPEED, degrees, -degrees, degrees,- degrees,5);
                }
                public void TurnRight(double a){
                    double degrees = a * 24/90;
                    encoderDrive(TURN_SPEED, -degrees, degrees, -degrees, degrees, 5);
                }
                public void slideLeft(double distance){
                    encoderDrive(SLIDE_SPEED,-distance,distance,distance,-distance,5);
                }
                public void slideRight(double distance){
                    encoderDrive(SLIDE_SPEED,distance,-distance,-distance,distance,5);
                }

                String formatAngle(AngleUnit angleUnit, double angle) {
                    return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
                }

                String formatDegrees(double degrees){
                    return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
                }

                public void runOpMode() {

                    //InitWheels

                    imu = hardwareMap.get(BNO055IMU.class, "imu");
                    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                    parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
                    parameters.loggingEnabled = true;
                    parameters.loggingTag = "IMU";
                    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
                    imu.initialize(parameters);

                    LeftDriveFront  = hardwareMap.get(DcMotor.class, "LeftDriveFront");
                    RightDriveFront = hardwareMap.get(DcMotor.class, "RightDriveFront");
                    LeftDriveBack    = hardwareMap.get(DcMotor.class, "LeftDriveBack");
                    RightDriveBack  = hardwareMap.get(DcMotor.class, "RightDriveBack");

                    LeftDriveFront.setDirection(DcMotor.Direction.FORWARD);
                    RightDriveFront.setDirection(DcMotor.Direction.REVERSE);
                    LeftDriveBack.setDirection(DcMotor.Direction.FORWARD);
                    RightDriveBack.setDirection(DcMotor.Direction.REVERSE);

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
                    heightSensor = hardwareMap.get(DistanceSensor.class, "Height");
                    lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    lift.setDirection(DcMotor.Direction.FORWARD);
                    //Init DistanceSensors


                    LeftDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    RightDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    LeftDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    RightDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    stopAllMotors();
                    telemetry.addData("Status","Waiting for User ");
                    telemetry.update();
                    waitForStart();
                    runtime.reset();
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    origAngle = angles.firstAngle;
                    // run until the end of the match (driver presses STOP)
                    while (opModeIsActive()) {
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);



                        if(heightSensor.getDistance(DistanceUnit.INCH) > 4){
                            lift.setPower(1.0);

                        }
                        else{
                            lift.setPower(0);
                            slideLeft(0.1);
                        }


                        Forwards(3);
                        TurnLeft(90);
                        orientationChecker(90);
                        Forwards(15);
                        slideLeft(20.5);
                        Forwards(8);
                        Backwards(6);
                        TurnLeft(90);
                        orientationChecker(180);
                        Forwards(20);
                        TurnLeft(45);
                        orientationChecker(-135);
                        slideRight(9);
                        Forwards(55);
                        Backwards(300);
                        break;
                    }
                    stopAllMotors();

                }

            }
            TurnLeft(90);
            orientationChecker(90);
            Forwards(15);
            slideLeft(20.5);
            Forwards(8);
            Backwards(6);
            TurnLeft(90);
            orientationChecker(180);
            Forwards(20);
            TurnLeft(45);
            orientationChecker(-135);
            slideRight(9);
            Forwards(55);
            Backwards(300);
            break;
        }
        stopAllMotors();

    }

}
