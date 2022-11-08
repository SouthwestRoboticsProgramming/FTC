package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name="Pushbot: lightning mcween", group="Pushbot")

public class TeleOp2 extends OpMode{

    /* Declare OpMode members. */
    HardwareRobot robot       = new HardwareRobot(); // use the class created to define a Pushbot's hardware
    double          clawOffset  = 0.5 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;

        robot.leftDriver.setPower(left);
        robot.rightDriver.setPower(right);

        // Use gamepad left & right Triggers to open and close the claw
        if (gamepad1.a) {
            clawOffset += CLAW_SPEED;
             robot.leftClaw.setPosition(.25);
        if (gamepad1.x)
            robot.leftClaw.setPosition(.25);
        else if (gamepad1.y)
        robot.rightClaw.setPosition(0);
       
       
        }else if(gamepad1.right_trigger>.9){
            
           
            robot.leftClaw.setPosition(.55);
        }
        // Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset,-0.5,0.5);
        //robot.leftClaw.setPosition(robot.MID_SERVO + 0);
        //robot.rightClaw.setPosition(robot.MID_SERVO - 0);

        // Use gamepad buttons to move the arm up (Y) and down (A)
        if (gamepad1.right_bumper){
            robot.leftgear.setPower(robot.ARM_DOWN_POWER);
        }
        else if (gamepad1.left_trigger>.9){
            robot.leftgear.setPower(-1000);
            robot.rightgear.setPower(1000);
            robot.centergear.setPower(-1000);
        }
        else{
            robot.leftgear.setPower(0.0);
            robot.rightgear.setPower(0.0);
            robot.centergear.setPower(0.0);
        }
 // Use gamepad buttons to move the flipper up (X) and down (B)
     if (gamepad1.left_bumper){
         robot.flipper.setPosition(-.1);
     }
     else{
         robot.flipper.setPosition(0.5);
         
         
//use gamepad button to move the goal
     if (gamepad1.x){
         robot.wobble.setPosition(.9);
        
     }
     else{
         robot.wobble.setPosition(.25);
     }
     }
        // Send telemetry message to signify robot running;
        telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
    }
      
      
     
     
     
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
