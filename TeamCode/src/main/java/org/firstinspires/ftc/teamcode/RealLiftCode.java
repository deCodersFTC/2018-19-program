/* Copyright (c) 2017 FIRST. All rights reserved.
 * deCoders 14368
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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@TeleOp(name="RealLiftCode", group="Linear Opmode")
// @Disabled
public class RealLiftCode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lift = null;
    DigitalChannel digitalTouch;

    private DcMotorSimple.Direction direction = DcMotor.Direction.FORWARD;
    private boolean atTop = false, atBottom = false;
    private boolean vPadEnabled = true;

    @Override
    public void runOpMode() {
        telemetry.addData("Stat/* Copyright (c) 2017 FIRST. All rights reserved.\n" +
                " * deCoders 14368\n" +
                " *\n" +
                " * Redistribution and use in source and binary forms, with or without modification,\n" +
                " * are permitted (subject to the limitations in the disclaimer below) provided that\n" +
                " * the following conditions are met:\n" +
                " *\n" +
                " * Redistributions of source code must retain the above copyright notice, this list\n" +
                " * of conditions and the following disclaimer.\n" +
                " *\n" +
                " * Redistributions in binary form must reproduce the above copyright notice, this\n" +
                " * list of conditions and the following disclaimer in the documentation and/or\n" +
                " * other materials provided with the distribution.\n" +
                " *\n" +
                " * Neither the name of FIRST nor the names of its contributors may be used to endorse or\n" +
                " * promote products derived from this software without specific prior written permission.\n" +
                " *\n" +
                " * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS\n" +
                " * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS\n" +
                " * \"AS IS\" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,\n" +
                " * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE\n" +
                " * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE\n" +
                " * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL\n" +
                " * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR\n" +
                " * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER\n" +
                " * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,\n" +
                " * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE\n" +
                " * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.\n" +
                " */\n" +
                "\n" +
                "package org.firstinspires.ftc.teamcode;\n" +
                "\n" +
                "import com.qualcomm.robotcore.eventloop.opmode.Disabled;\n" +
                "import com.qualcomm.robotcore.hardware.DcMotorSimple;\n" +
                "import com.qualcomm.robotcore.hardware.DigitalChannel;\n" +
                "import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;\n" +
                "import com.qualcomm.robotcore.eventloop.opmode.TeleOp;\n" +
                "import com.qualcomm.robotcore.hardware.DcMotor;\n" +
                "import com.qualcomm.robotcore.util.ElapsedTime;\n" +
                "import com.qualcomm.robotcore.util.Range;\n" +
                "\n" +
                "\n" +
                "/**\n" +
                " * This file contains an minimal example of a Linear \"OpMode\". An OpMode is a 'program' that runs in either\n" +
                " * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu\n" +
                " * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode\n" +
                " * class is instantiated on the Robot Controller and executed.\n" +
                " *\n" +
                " * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot\n" +
                " * It includes all the skeletal structure that all linear OpModes contain.\n" +
                " *\n" +
                " * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.\n" +
                " * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list\n" +
                " */\n" +
                "\n" +
                "@TeleOp(name=\"RealLiftCode\", group=\"Linear Opmode\")\n" +
                "// @Disabled\n" +
                "public class RealLiftCode extends LinearOpMode {\n" +
                "\n" +
                "    // Declare OpMode members.\n" +
                "    private ElapsedTime runtime = new ElapsedTime();\n" +
                "    private DcMotor lift = null;\n" +
                "    DigitalChannel digitalTouch;\n" +
                "\n" +
                "    private DcMotorSimple.Direction direction = DcMotor.Direction.FORWARD;\n" +
                "    private boolean atTop = false, atBottom = false;\n" +
                "    private boolean vPadEnabled = true;\n" +
                "\n" +
                "    @Override\n" +
                "    public void runOpMode() {\n" +
                "        telemetry.addData(\"Status\", \"Initialized\");\n" +
                "        telemetry.update();\n" +
                "\n" +
                "        digitalTouch = hardwareMap.get(DigitalChannel.class, \"sensor_digital\");\n" +
                "        digitalTouch.setMode(DigitalChannel.Mode.INPUT);\n" +
                "\n" +
                "\n" +
                "        // Initialize the hardware variables. Note that the strings used here as parameters\n" +
                "        // to 'get' must correspond to the names assigned during the robot configuration\n" +
                "        // step (using the FTC Robot Controller app on the phone).\n" +
                "        lift  = hardwareMap.get(DcMotor.class, \"lift\");\n" +
                "\n" +
                "        // Most robots need the motor on one side to be reversed to drive forward\n" +
                "        // Reverse the motor that runs backwards when connected directly to the battery\n" +
                "        lift.setDirection(direction);\n" +
                "        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);\n" +
                "\n" +
                "        // Wait for the game to start (driver presses PLAY)\n" +
                "        waitForStart();\n" +
                "        runtime.reset();\n" +
                "\n" +
                "        // run until the end of the match (driver presses STOP)\n" +
                "        while (opModeIsActive()) {\n" +
                "            telemetry.addData(\"B a=\", gamepad1.a);\n" +
                "            telemetry.addData(\"B y=\", gamepad1.y);\n" +
                "            telemetry.addData(\"Direction=\", direction);\n" +
                "            telemetry.addData(\"vpad=\", vPadEnabled);\n" +
                "\n" +
                "            double liftPower;\n" +
                "\n" +
                "            // if the magSensor is on, find direction and set atTop or atBottom.\n" +
                "            if (digitalTouch.getState() == false) {\n" +
                "                telemetry.addData(\"Magnet is\", digitalTouch.getState());\n" +
                "                telemetry.addData(\"vPad is\", vPadEnabled);\n" +
                "                telemetry.addData(\"Limit\", \"hit\");\n" +
                "                if (atTop == false && atBottom == false) {\n" +
                "                    if (direction == DcMotor.Direction.REVERSE) {\n" +
                "                        atTop = true;\n" +
                "                        direction = DcMotor.Direction.FORWARD;\n" +
                "                        lift.setPower(1);\n" +
                "                    } else {\n" +
                "                        atBottom = true;\n" +
                "                        direction = DcMotor.Direction.REVERSE;\n" +
                "                        lift.setPower(-1);\n" +
                "                    }\n" +
                "                    vPadEnabled = false;\n" +
                "                }\n" +
                "            } else {\n" +
                "                telemetry.addData(\"Magnet is\", digitalTouch.getState());\n" +
                "                if(atTop || atBottom) {\n" +
                "                    lift.setPower(0);\n" +
                "                    atTop = atBottom = false;\n" +
                "                    telemetry.addData(\"Limit\", \"clear\");\n" +
                "                }\n" +
                "\n" +
                "                if(vPadEnabled == true) {\n" +
                "                    if (gamepad1.a == true) {\n" +
                "                        direction = DcMotor.Direction.REVERSE;\n" +
                "                        lift.setPower(-1);\n" +
                "                    } else if (gamepad1.y == true) {\n" +
                "                        direction = DcMotor.Direction.FORWARD;\n" +
                "                        lift.setPower(1);\n" +
                "                    } else {\n" +
                "                        lift.setPower(0);\n" +
                "                    }\n" +
                "                } else {\n" +
                "                    // when the hand is removed, then reenable the pad\n" +
                "                    if(gamepad1.a == false && gamepad1.y == false) {\n" +
                "                        vPadEnabled = true;\n" +
                "                    }\n" +
                "                }\n" +
                "            }\n" +
                "\n" +
                "            //telemetry.addData(\"Status\", \"Run Time: \" + runtime.toString());\n" +
                "            telemetry.addData(\"Motor Power\", lift.getPower());\n" +
                "            telemetry.addData(\"Written by\", \"Tejas and Sharad\");\n" +
                "            telemetry.update();\n" +
                "        }\n" +
                "    }\n" +
                "}\nus", "Initialized");
        telemetry.update();

        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
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

            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motor Power", lift.getPower());
            telemetry.addData("Written by", "Tejas and Sharad");
            telemetry.update();
        }
    }
}
