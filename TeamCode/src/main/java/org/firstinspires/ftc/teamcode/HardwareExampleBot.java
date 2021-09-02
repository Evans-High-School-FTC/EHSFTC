package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.hardware.motors.RevRoboticsUltraPlanetaryHdHexMotor;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 */
public class HardwareExampleBot
{
    /* Public OpMode members. */
    public DcMotor  frontLeftDrive   = null;
    public DcMotor  frontRightDrive  = null;
    public DcMotor  backLeftDrive    = null;
    public DcMotor  backRightDrive   = null;
    public DcMotor  intake           = null;
    public DistanceSensor dist       = null;
    public DcMotor  con1             = null;
    public DcMotor  con2             = null;
    public DcMotor  shooter          = null;

    //public Servo    leftClaw    = null;
    //public Servo    rightClaw   = null;

    //public static final double MID_SERVO       =  0.5 ;
    //public static final double ARM_UP_POWER    =  0.45 ;
    //public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    private String FLD,FRD,BLD,BRD,INT,DIS,conA,conB,SHT;

    /* Constructor */
    public HardwareExampleBot(){
        FLD = "fld";
        FRD = "frd";
        BLD = "bld";
        BRD = "brd";
        INT = "int";
        DIS = "dis";
        conA= "cn1";
        conB= "cn2";
        SHT = "sht";
    }

    /* Initialize standard Hardware interfaces for remote control*/
    public void initForTele(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftDrive  = hwMap.get(DcMotor.class, FLD);
        frontRightDrive = hwMap.get(DcMotor.class, FRD);
        backLeftDrive   = hwMap.get(DcMotor.class, BLD);
        backRightDrive  = hwMap.get(DcMotor.class, BRD);
        intake          = hwMap.get(DcMotor.class, INT);
        dist            = hwMap.get(DistanceSensor.class, DIS);
        con1            = hwMap.get(DcMotor.class, conA);
        con2            = hwMap.get(DcMotor.class, conB);
        shooter         = hwMap.get(DcMotor.class, SHT);

        // Set motor directions THIS IS A MECANUM DRIVE
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        con1.setDirection(DcMotorSimple.Direction.FORWARD);
        con2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motor types (Optional)
        frontLeftDrive.setMotorType(MotorConfigurationType.getMotorType(RevRoboticsUltraPlanetaryHdHexMotor.class));
        frontRightDrive.setMotorType(MotorConfigurationType.getMotorType(RevRoboticsUltraPlanetaryHdHexMotor.class));
        backLeftDrive.setMotorType(MotorConfigurationType.getMotorType(RevRoboticsUltraPlanetaryHdHexMotor.class));
        backRightDrive.setMotorType(MotorConfigurationType.getMotorType(RevRoboticsUltraPlanetaryHdHexMotor.class));
        intake.setMotorType(MotorConfigurationType.getMotorType(RevRoboticsUltraPlanetaryHdHexMotor.class));
        con1.setMotorType(MotorConfigurationType.getMotorType(RevRoboticsCoreHexMotor.class));
        con2.setMotorType(MotorConfigurationType.getMotorType(RevRoboticsCoreHexMotor.class));
        shooter.setMotorType(MotorConfigurationType.getMotorType(RevRoboticsUltraPlanetaryHdHexMotor.class));


        // Set all motors to zero power
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        intake.setPower(0);
        con1.setPower(0);
        con2.setPower(0);
        shooter.setPower(0);

        //Disables harsh stops
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        con1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        con2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to run without encoders.
        // Use encoders for motors that require precision/automatic control
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        con1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        con1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        //leftClaw  = hwMap.get(Servo.class, "left_hand");
        //rightClaw = hwMap.get(Servo.class, "right_hand");
        //leftClaw.setPosition(MID_SERVO);
        //rightClaw.setPosition(MID_SERVO);
    }

    /* Initialize standard Hardware for autonomous control interfaces */
    public void initForAuto(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftDrive  = hwMap.get(DcMotor.class, FLD);
        frontRightDrive = hwMap.get(DcMotor.class, FRD);
        backLeftDrive   = hwMap.get(DcMotor.class, BLD);
        backRightDrive  = hwMap.get(DcMotor.class, BRD);
        intake          = hwMap.get(DcMotor.class, INT);
        con1            = hwMap.get(DcMotor.class, conA);
        con2            = hwMap.get(DcMotor.class, conB);
        shooter         = hwMap.get(DcMotor.class, SHT);

        // Set motor directions THIS IS A MECANUM DRIVE
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        con1.setDirection(DcMotorSimple.Direction.FORWARD);
        con2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motor types (Optional)
        frontLeftDrive.setMotorType(MotorConfigurationType.getMotorType(RevRoboticsUltraPlanetaryHdHexMotor.class));
        frontRightDrive.setMotorType(MotorConfigurationType.getMotorType(RevRoboticsUltraPlanetaryHdHexMotor.class));
        backLeftDrive.setMotorType(MotorConfigurationType.getMotorType(RevRoboticsUltraPlanetaryHdHexMotor.class));
        backRightDrive.setMotorType(MotorConfigurationType.getMotorType(RevRoboticsUltraPlanetaryHdHexMotor.class));
        intake.setMotorType(MotorConfigurationType.getMotorType(RevRoboticsUltraPlanetaryHdHexMotor.class));
        con1.setMotorType(MotorConfigurationType.getMotorType(RevRoboticsCoreHexMotor.class));
        con2.setMotorType(MotorConfigurationType.getMotorType(RevRoboticsCoreHexMotor.class));
        shooter.setMotorType(MotorConfigurationType.getMotorType(RevRoboticsUltraPlanetaryHdHexMotor.class));

        // Set all motors to zero power
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        intake.setPower(0);
        con1.setPower(0);
        con2.setPower(0);
        shooter.setPower(0);

        //Automatic Breaking
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        con1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        con2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to run without encoders.
        // Use encoders for motors that require precision/automatic control
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        con1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        con1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        //leftClaw  = hwMap.get(Servo.class, "left_hand");
        //rightClaw = hwMap.get(Servo.class, "right_hand");
        //leftClaw.setPosition(MID_SERVO);
        //rightClaw.setPosition(MID_SERVO);
    }
 }

