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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="ExampleBot: Teleop", group="ExampleBot")
//@Disabled
public class ExampleBotLinearTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareExampleBot robot          = new HardwareExampleBot();

    @Override
    public void runOpMode() {
        double x,y,r;
        boolean intake;
        boolean chng = false;
        float shooter;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.initForTele(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Starting");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            y = -gamepad1.left_stick_y;
            x =  gamepad1.left_stick_x;
            r =  gamepad1.right_stick_x;
            intake = gamepad1.left_bumper;
            shooter = gamepad1.right_trigger;

            y = x*(Math.sin(-Math.PI/4)) + y*(Math.cos(-Math.PI/4));
            x = x*(Math.cos(-Math.PI/4)) - y*(Math.sin(-Math.PI/4));

            //Starts moves shooter if trigger is pressed
            if(shooter > .1) {
                robot.shooter.setPower(1);
            }
            else {
                robot.shooter.setPower(0);
            }

            //Checks if disk is in shooter using distance sensor
            if(robot.dist.getDistance(DistanceUnit.CM) > 5) {
                //If not advances conveyor
                robot.con1.setPower(1);
                robot.con2.setPower(1);
            }
            else {
                //otherwise stops conveyor
                robot.con1.setPower(0);
                robot.con2.setPower(0);
            }

            if(intake && !chng) {
                if(robot.intake.getPower() == 0) {
                    robot.intake.setPower(1);
                }
                else {
                    robot.intake.setPower(0);
                }
                chng = true;
            }
            else if(!intake) {
                chng = false;
            }


            if(Math.abs(r) > .1) {
                robot.frontRightDrive.setPower(-r);
                robot.backRightDrive.setPower(-r);

                robot.frontLeftDrive.setPower(r);
                robot.backRightDrive.setPower(r);
            }
            else {
                // Drive linearly if no rotation is occurring
                robot.frontLeftDrive.setPower(x);
                robot.frontRightDrive.setPower(y);
                robot.backLeftDrive.setPower(y);
                robot.backRightDrive.setPower(x);
            }

            // Send telemetry message to signify robot running;
            telemetry.addData("x",  "%.2f", x);
            telemetry.addData("y",  "%.2f", y);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
