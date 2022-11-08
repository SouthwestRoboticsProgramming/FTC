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
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop", group="Pushbot")

public class Teleop extends OpMode{

    /* Declare OpMode members. */
    HardwareMecanum robot       = new HardwareMecanum(); // use the class created to define a Pushbot's hardware
       
    //double          leftServoOffset  = 0.0 ;                  // Servo mid position
    //final double    leftServo_SPEED  = 0.2 ; 
    double          flickerOffset  = 0.0 ;                  // Servo mid position
    final double    flicker_SPEED  = 0.2 ;
    int             upWobbleTarget = 0;
    int             downWobbleTarget = 0;
    //double          clawServoOffset  = 0.0 ;                  // Servo mid position
    //final double    clawServo_SPEED  = 0.2 ; 
    //double          armServoOffset    = 0.0;
    //final double    armServo_SPEED    = 0.2;
    // double          grabberMoverServoOffset  = 0.0 ;                  // Servo mid position
    // final double    grabberMoverServo_SPEED  = 0.2 ; 
    // double          grabberGetterServoOffset  = 0.0 ;                  // Servo mid position
    // final double    grabberGetterServo_SPEED  = 0.2 ; 
    // could also use HardwarePushbotMatrix class.
   
    // int Up_Limit = 0;
    // int Up_Position = 57;
    // int Down_Position = 110;
    // int Down_Limit = 150;
    
    // int Position_Error = 3;
    
    // int defaultPosition = Up_Limit;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        
         robot.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);//change to BRAKE
         robot.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);//change to BRAKErobot.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);//change to BRAKE
         robot.leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);//change to BRAKErobot.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);//change to BRAKE
         robot.rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);//change to BRAKE
         robot.shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        
        //robot.mineral_grabber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "We Don't Talk About That");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        upWobbleTarget = robot.wobbleMotor.getCurrentPosition() + 350;
        downWobbleTarget = robot.wobbleMotor.getCurrentPosition() + 0; //-35, +35
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double leftFront;
        double rightFront;
        double leftBack;
        double rightBack;
        double leftTrigger;
        double rightTrigger;
        double shooterSpeed;
        double wobbleSpeed;
        double intakeSpeed;
        
       
       // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        leftFront = +gamepad1.left_stick_y;
        leftBack = +gamepad1.left_stick_y;
        rightFront = +gamepad1.right_stick_y;
        rightBack = +gamepad1.right_stick_y;
        rightTrigger = +gamepad1.right_trigger;
        leftTrigger = +gamepad1.left_trigger;
        
        
        shooterSpeed = 0;
        wobbleSpeed = 0;
        intakeSpeed = 0;
        
        if (leftTrigger>0){
            robot.leftFrontDrive.setPower(-leftTrigger);
            robot.rightFrontDrive.setPower(leftTrigger);
            robot.leftBackDrive.setPower(leftTrigger);
            robot.rightBackDrive.setPower(-leftTrigger);
        }
        
        else if (rightTrigger>0){
            robot.leftFrontDrive.setPower(rightTrigger);
            robot.rightFrontDrive.setPower(-rightTrigger);
            robot.leftBackDrive.setPower(-rightTrigger);
            robot.rightBackDrive.setPower(rightTrigger);
        }
        else {

        robot.leftFrontDrive.setPower(leftFront);
        robot.rightFrontDrive.setPower(rightFront);
        robot.leftBackDrive.setPower(leftBack);
        robot.rightBackDrive.setPower(rightBack);   
        }
        
        //robot.shooter.setPower(shooterSpeed);

        // Use gamepad left & right bumpers to open and close the claw
         if (gamepad1.right_bumper){
              robot.intake.setPower(1);
              robot.intake2.setPower(-1);
         }
         else if
          (gamepad1.left_bumper){
             robot.intake.setPower(-1);
             robot.intake2.setPower(1);
         } else{ 
             robot.intake.setPower(0);
             robot.intake2.setPower(0);
             
         } 
         
         if (gamepad1.y){ 
             robot.flicker.setPosition(.3);
         } else{ 
             robot.flicker.setPosition(0);
         
          } 
      if (gamepad1.b){ 
            robot.shooter.setPower(.555); //.54, .6, .59, .56
         } 
         else{ 
            robot.shooter.setPower(0);
         
         }
         
         // a= servos
         // x == motor
           if (gamepad1.a){ 
             robot.wobble1.setPosition(.6);//.65
             robot.wobble2.setPosition(1);
         } else{ 
              robot.wobble1.setPosition(1);
              robot.wobble2.setPosition(0);
         
     }
     
            if (gamepad1.x){
                robot.wobbleMotor.setTargetPosition(upWobbleTarget);
                robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.wobbleMotor.setPower(0.35);
            }
            else{
                robot.wobbleMotor.setTargetPosition(downWobbleTarget);
                robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.wobbleMotor.setPower(0.35);
            }
             
            
            
            // Set Target and Turn On RUN_TO_POSITION
            
            
            //mineral_dropperOffset-=mineral_dropper_SPEED;
        // Move both servos to new position.  Assume servos are mirror image of each other.
        // mineral_dropperOffset = Range.clip(mineral_dropperOffset, 0.5 ,-0.5);
        // robot.mineral_dropper.setPosition(robot.MID_SERVO + mineral_dropperOffset);
        // robot.mineral_dropper.setPosition(robot.MID_SERVO - mineral_dropperOffset);

        // Use gamepad buttons to move the arm up (Y) and down (A)
        // if (gamepad2.y)
        // {
        //     //robot.leftArm.setPower(robot.ARM_UP_POWER);
        //     //robot.rack_and_pinion.setPower(.5);
            
        // }
        // else
        // if
        // (gamepad2.a)
        // {
        //     //robot.leftArm.setPower(robot.ARM_DOWN_POWER);
        //     robot.rack_and_pinion.setPower(-.5);
        // }
        // else
         {
        //     //robot.leftArm.setPower(0.0);
        //     robot.rack_and_pinion.setPower(0.0);
}
        // Send telemetry message to signify robot running;
        // telemetry.addData("mineral_dropper",  "Offset = %.2f", mineral_dropperOffset);
        // telemetry.addData("mineral_grabber",  "%.2f", left);
        // telemetry.addData("mineral_grabber", "%.2f", right);
        
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
