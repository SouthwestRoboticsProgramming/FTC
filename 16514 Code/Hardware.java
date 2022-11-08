package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
 * Motor channel:  Left  drive motor:        "left_driver"
 * Motor channel:  Right drive motor:        "right_driver"
 * Motor channel:  Manipulator drive motor:  "left_gear"
 * Motor channel:  Manipulator drive motor   "right_gear"
 * Motor channel:  Manipulator drive motor   "center_gear"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 * Servo channel:  Servo to flip capstone: "flipper"
 */
class HardwareRobot
{
    /* Public OpMode members. */
    public DcMotor  leftDriver   = null;
    public DcMotor  rightDriver  = null;
    public DcMotor  leftgear     = null;
    public DcMotor  rightgear    = null;
    public DcMotor  centergear    = null;
    public Servo    leftClaw    = null;
    public Servo    rightClaw   = null;
    public Servo    flipper  =null;
    public Servo    wobble =null;

    public static final double MID_SERVO       =  0.5  ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareRobot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDriver  = hwMap.get(DcMotor.class, "leftDriver");
        rightDriver = hwMap.get(DcMotor.class, "rightDriver");
        leftgear   = hwMap.get(DcMotor.class, "leftgear");
        rightgear  = hwMap.get(DcMotor.class, "rightgear");
        centergear  = hwMap.get(DcMotor.class, "centergear");
        rightgear.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftgear.setDirection(DcMotor.Direction.REVERSE);// Set to REVERSE if using Andymark motors
        centergear.setDirection(DcMotor.Direction.FORWARD);// Set to REVERSE if using Andymark motors
        
        
        leftDriver.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDriver.setDirection(DcMotor.Direction.REVERSE);// Set to REVERSE if using AndyMark motors
        leftDriver.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriver.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set all motors to zero power
        leftDriver.setPower(0);
        rightDriver.setPower(0);
        leftgear.setPower(0);
        rightgear.setPower(0);
        centergear.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDriver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftgear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightgear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centergear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        leftClaw  = hwMap.get(Servo.class, "leftClaw");
        rightClaw = hwMap.get(Servo.class, "rightClaw");
        leftClaw.setPosition(.25);
        rightClaw.setPosition(.25);
        flipper = hwMap.get(Servo.class, "flipper") ;
        flipper.setPosition(.5);
        wobble =hwMap.get(Servo.class, "wobble");
        wobble.setPosition(.5);
    }
 }
