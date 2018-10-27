package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.concurrent.TimeUnit;



@TeleOp(name="HookDriveWithTriggers", group="Linear Opmode")
//@Disabled
public class HookDriveWithTriggers extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftDriveFront = null;
    private DcMotor RightDriveFront = null;
    private DcMotor LeftDriveBack = null;
    private DcMotor RightDriveBack = null;
    private DcMotor lift = null;
    DigitalChannel digitalTouch;

    private DcMotorSimple.Direction direction = DcMotor.Direction.FORWARD;
    private boolean atTop = false, atBottom = false;
    private boolean vPadEnabled = true;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
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
        lift  = hardwareMap.get(DcMotor.class, "lift");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        lift.setDirection(direction);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry

            double LeftFrontPower;
            double RightFrontPower;
            double RightBackPower;
            double LeftBackPower;





            // Wait for the game to start (driver presses PLAY)

            double drive = gamepad1.left_stick_y;
            double slide = gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;
            // if the magSensor is on, find direction and set atTop or atBottom.

            LeftFrontPower = drive + slide + turn;
            RightFrontPower = drive - slide - turn;
            LeftBackPower = drive - slide + turn;
            RightBackPower = drive + slide - turn;

            if(turn != 0 && (drive != 0 || slide != 0)){
                LeftDriveFront.setPower(LeftFrontPower/2);
                LeftDriveBack.setPower(LeftBackPower/2);
                RightDriveFront.setPower(RightFrontPower/2);
                RightDriveBack.setPower(RightBackPower/2);

            }
            else{
                LeftDriveFront.setPower(LeftFrontPower);
                LeftDriveBack.setPower(LeftBackPower);
                RightDriveFront.setPower(RightFrontPower);
                RightDriveBack.setPower(RightBackPower);
            }

            telemetry.addData("Status", "Runtime: " + runtime.toString());
            telemetry.addData("Motors","Left Front Power: (%.2f), Right Front Power: (%.2f), Left Back Power: (%.2f), Right Back Power: (%.2f)", LeftDriveFront.getPower(), RightDriveFront.getPower(), LeftDriveBack.getPower(), RightDriveBack.getPower());
            telemetry.addData("Written by", "Arul and Samarth :)");
            telemetry.update();


            telemetry.addData("B down=", gamepad1.left_trigger);
            telemetry.addData("B up=", gamepad1.right_trigger);
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
                if (atTop || atBottom) {
                    lift.setPower(0);
                    atTop = atBottom = false;
                    telemetry.addData("Limit", "clear");
                }

                if (vPadEnabled == true) {
                    lift.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                } else {
                    // when the hand is removed, then reenable the pad
                    if (gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0) {
                        vPadEnabled = true;
                    }
                }
            }
        }

        //telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motor Power", lift.getPower());
        telemetry.addData("Written by", "Tejas and Sharad");
        telemetry.update();
    }
}

