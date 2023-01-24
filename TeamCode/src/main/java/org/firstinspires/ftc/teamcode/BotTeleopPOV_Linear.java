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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="BetaBot23: Teleop POV", group="BetaBot23")
public class BotTeleopPOV_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware2223Bot robot           = new Hardware2223Bot();
    //private boolean isOpen = false;

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Waiting");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            drivetrain();
            //intake();
            linearSlide();
            telemetry.update();
            sleep(50);
        }
    }

    private void linearSlide() {
        if(gamepad1.right_trigger != 0) {
            robot.linearSlide.setPower(gamepad1.right_trigger);
        }
        else if(gamepad1.left_trigger != 0) {
            robot.linearSlide.setPower(-gamepad1.left_trigger);
        }
        else {
            robot.linearSlide.setPower(0);
            robot.linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /*private void intake() {
        if(gamepad1.right_trigger >= 1 && isOpen) {
            closeClaw();
        }
        else if(gamepad1.right_trigger >= 1 && !isOpen) {
            openClaw();
        }
    }

    private void openClaw() {
        telemetry.addData("Say", "Opening Claw");
        isOpen = true;

    }

    private void closeClaw() {
        telemetry.addData("Say", "Closing Claw");
        isOpen = false;
        
    }*/

    private void drivetrain() {
         double y  = -gamepad1.left_stick_y;
         double x  = gamepad1.left_stick_x * 1.1;
         double rx = gamepad1.right_stick_x;

         double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
         double frontLeftDrivePower  = (y+x+rx)/denom;
         double rearLeftDrivePower   = (y-x+rx)/denom;
         double frontRightDrivePower = (y-x-rx)/denom;
         double rearRightDrivePower  = (y+x-rx)/denom;

         robot.frontLeftDrive.setPower(frontLeftDrivePower);
         robot.rearLeftDrive.setPower(rearLeftDrivePower);
         robot.frontRightDrive.setPower(frontRightDrivePower);
         robot.rearRightDrive.setPower(rearRightDrivePower);
    }
}
