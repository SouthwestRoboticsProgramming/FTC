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
//package SkyStone;
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;
import java.lang.Thread;


import org.firstinspires.ftc.teamcode.HardwareMecanum;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Rack and pinion: Rack and Pinion Motor:   "rack_and_pinion"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareMecanum
{
    
    /* Public OpMode members. */
    public DcMotor  leftFrontDrive   = null;
    public DcMotor  rightFrontDrive  = null;
    public DcMotor  leftBackDrive   = null;
    public DcMotor  rightBackDrive  = null;
    public DcMotor  shooter        = null;
    public DcMotor  wobbleMotor    = null;
    public DcMotor  intake         = null;
    public DcMotor  intake2         = null;
    //public DcMotor  leftArm     = null;
    // public DcMotor  rack_and_pinion  = null;
     //public Servo    leftServo    = null;
     public Servo    flicker   = null;
     public Servo    wobble1   = null;
     public Servo    wobble2   = null;
     //public Servo    clawServo = null;
     //public Servo    armServo      =null;
    //  public Servo    markerDropper =null;
    // public DcMotor  mineral_grabber = null;
    // public Servo    mineral_dropper = null;
    

    // public static final double MID_SERVO       =  0.5 ;
    // public static final double ARM_UP_POWER    =  0.45 ;
    // public static final double ARM_DOWN_POWER  = -0.45 ;
    

    // // ColorSensor sensorColor;
    // public DistanceSensor sensorDistance;
    // public ColorSensor sensorColor;
    // public ColorSensor sensorColor2;
    
    

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMecanum(){

    }
    
    public void ResetEncoders(){
        //mineral_grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontDrive  = hwMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive  = hwMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hwMap.get(DcMotor.class, "rightBackDrive");
        shooter = hwMap.get(DcMotor.class, "shooter");
        wobbleMotor = hwMap.get(DcMotor.class, "wobbleMotor");
        intake = hwMap.get(DcMotor.class, "intake");
        intake2 = hwMap.get(DcMotor.class, "intake2");
        //leftArm    = hwMap.get(DcMotor.class, "left_arm");
        //rack_and_pinion = hwMap.get(DcMotor.class, "rack_and_pinion");
        //leftServo = hwMap.get(Servo.class, "leftServo");
        //mineral_grabber = hwMap.get(DcMotor.class, "mineral_grabber");
        flicker = hwMap.get(Servo.class, "flicker");
        wobble1 = hwMap.get(Servo.class, "wobble1");
        wobble2 = hwMap.get(Servo.class, "wobble2");
        //clawServo = hwMap.get(Servo.class, "clawServo");
        //armServo = hwMap.get(Servo.class, "armServo");
        // grabberMoverServo = hwMap.get(Servo.class, "grabberMoverServo");
        // grabberGetterServo = hwMap.get(Servo.class, "grabberGetterServo");


        // get a reference to the distance sensor that shares the same name.
        // sensorDistance = hwMap.get(DistanceSensor.class, "sensor_range");
        // sensorColor=hwMap.get(ColorSensor.class, "sensor_color_distance");
        // sensorColor2=hwMap.get(ColorSensor.class, "color_distance_sensor");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        shooter.setDirection(DcMotor.Direction.FORWARD);
        wobbleMotor.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake2.setDirection(DcMotor.Direction.FORWARD);
     
        // rack_and_pinion.setDirection(DcMotor.Direction.FORWARD);
        // mineral_grabber.setDirection(DcMotor.Direction.FORWARD);
        // //leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // mineral_grabber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // rack_and_pinion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        shooter.setPower(0);
        wobbleMotor.setPower(0);
        intake.setPower(0);
        intake2.setPower(0);
        //leftArm.setPower(0);
        // rack_and_pinion.setPower(0);
        //leftServo.setPosition(1);
        // mineral_grabber.setPower(0);
         flicker.setPosition(0);
         wobble1.setPosition(1);
         wobble2.setPosition(0);
         //clawServo.setPosition(1);
         //armServo.setPosition(1);
        //  grabberMoverServo.setPosition(1);
        //  grabberGetterServo.setPosition(1);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // rack_and_pinion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // mineral_grabber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        //leftClaw  = hwMap.get(Servo.class, "left_hand");
        //rightClaw = hwMap.get(Servo.class, "right_hand");
        //leftClaw.setPosition(0.6);
        //rightClaw.setPosition(0.6);
    }

}
