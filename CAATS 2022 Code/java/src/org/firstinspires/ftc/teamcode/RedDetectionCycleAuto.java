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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import java.lang.annotation.Target;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;


/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="RedDetectionCycleAuto", group="Pushbot")

public class RedDetectionCycleAuto extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMecanum         robot   = new HardwareMecanum();   // Use a Pushbot's hardware
    BNO055IMU               imu     = null;                       // Additional Gyro device

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.85;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.55;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.01;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF            = 0.001;     // Larger is more responsive, but also less stable
    private ElapsedTime     runtime = new ElapsedTime();
        int             bottomTarget = 0;
        int             middleTarget = 0;
        int             topTarget = 0;
        int             positionTop = 0;
        int             positionZero = 0;
        int             rotateTarget = 0;
        int             centerTarget = 0;

    private DistanceSensor leftSensorRange;
    private DistanceSensor rightSensorRange;
    private DigitalChannel touch;
    private DigitalChannel redLED1;
    private DigitalChannel greenLED1;
    private DigitalChannel redLED2;
    private DigitalChannel greenLED2;
    private DigitalChannel redLED3;
    private DigitalChannel greenLED3;
    private DigitalChannel redLED4;
    private DigitalChannel greenLED4;
    public ColorSensor    color;

    
    //     private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    // private static final String LABEL_FIRST_ELEMENT = "Quad";
    // private static final String LABEL_SECOND_ELEMENT = "Single";
    
    // int             upWobbleTarget = 0;
    // int             downWobbleTarget = 0;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    // private static final String VUFORIA_KEY =
    //         "AZpKr3f/////AAABmaUYHPich0WJvMYAYv7wG4Y7mMlvn0/4QyVXJfSph+10mj5ijldPoBRezSZhZiecccQ0W+Db6T9xpuWxBWSJ0o5vZT6npaaOChxoI5ROiYT3c5QTi8NZTy6FReo/mwW5QfTqayNZtOvgr4nau0TghOUKrCB66xh22XEMIJ2QIeBmr+fjYFltRx/DKD0IqzYhAN6wlNRWez2Wvf1cGvdctZFuEvzJESdwJ0/OPUHbPWF0Ph2BnePb7ytdgbGXWcv8N/mguyvoj1t8nw1r+KAlMozEbpbOE6cQtPQ01gc/PRtXVGnBYXoKwyAPV9fyio7BpiNaCxaPESn/p04GbIhyM1KsOqZVZKJwqWnpDKoL0mrD";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    //private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    //private TFObjectDetector tfod;

    
    @Override
    public void runOpMode() {
        
        leftSensorRange = hardwareMap.get(DistanceSensor.class, "distanceSensor1");
        rightSensorRange = hardwareMap.get(DistanceSensor.class, "distanceSensor2");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)leftSensorRange;
        Rev2mDistanceSensor sensorTimeOfFlight2 = (Rev2mDistanceSensor)rightSensorRange;
        //int ringsDetected = 0;
        
        // upWobbleTarget = robot.wobbleMotor.getCurrentPosition() + 75;
        // downWobbleTarget = robot.wobbleMotor.getCurrentPosition() + 5;
        
        // initVuforia();
        // initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        // if (tfod != null) {
        //     tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.               `
            //tfod.setZoom(3, 1.78);

        

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
         
        robot.init(hardwareMap);

         // Set up the parameters with which we will use our IMU.
         
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        redLED1 = hardwareMap.get(DigitalChannel.class, "red1");
        greenLED1 = hardwareMap.get(DigitalChannel.class, "green1");
        redLED2 = hardwareMap.get(DigitalChannel.class, "red2");
        greenLED2 = hardwareMap.get(DigitalChannel.class, "green2");
        redLED3 = hardwareMap.get(DigitalChannel.class, "red3");
        greenLED3 = hardwareMap.get(DigitalChannel.class, "green3");
        redLED4 = hardwareMap.get(DigitalChannel.class, "red4");
        greenLED4 = hardwareMap.get(DigitalChannel.class, "green4");
        touch = hardwareMap.get(DigitalChannel.class, "touch");
        
        
        redLED1.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED1.setMode(DigitalChannel.Mode.OUTPUT);
        redLED2.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED2.setMode(DigitalChannel.Mode.OUTPUT);
        redLED3.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED3.setMode(DigitalChannel.Mode.OUTPUT);
        redLED4.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED4.setMode(DigitalChannel.Mode.OUTPUT);


        telemetry.addData(">", "Robot Params.");    //
        telemetry.update();
       
         
        // Retrieve and initialize the IMU. Ensure the robot it stationary and calibrate the gyro.
        imu = hardwareMap.get(BNO055IMU.class, "imu");
       
       
        telemetry.addData(">", "Robot imu get.");    //
        telemetry.update();
       
       
        imu.initialize(parameters);
       
       
        telemetry.addData(">", "Imu Ready.");    //
        telemetry.update();
       

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        // telemetry.addData(">", "Calibrating Gyro");    //
        // telemetry.update();

         //gyro.calibrate();

        // // make sure the gyro is calibrated before continuing
        //  while (!isStopRequested() && gyro.isCalibrating())  {
        //      sleep(50);
        //      idle();
        //  }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            
            telemetry.addData("deviceName",leftSensorRange.getDeviceName() );
            telemetry.addData("range", String.format("%.01f cm", leftSensorRange.getDistance(DistanceUnit.CM)));
            //telemetry.addData("range", String.format("%.01f in", leftSensorRange.getDistance(DistanceUnit.INCH)));
            
            telemetry.addData("deviceName",rightSensorRange.getDeviceName() );
            telemetry.addData("range", String.format("%.01f cm", rightSensorRange.getDistance(DistanceUnit.CM)));
            //telemetry.addData("range", String.format("%.01f in", rightSensorRange.getDistance(DistanceUnit.INCH)));

            // Rev2mDistanceSensor specific methods.
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            //telemetry.addData(">", "Robot Heading = %f", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();
            
        //         if (tfod != null) {
        //             // getUpdatedRecognitions() will return null if no new information is available since
        //             // the last time that call was made.
        //             List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        //             if (updatedRecognitions != null) {
        //               telemetry.addData("# Object Detected", updatedRecognitions.size());
        //               // step through the list of recognitions and display boundary info.
        //               int i = 0;
                     
        //                 telemetry.addData(">", "zero rings");
        //                 ringsDetected = 0;
                      
                      
        //               for (Recognition recognition : updatedRecognitions) {
        //                 telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
        //                 telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
        //                         recognition.getLeft(), recognition.getTop());
        //                 telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
        //                         recognition.getRight(), recognition.getBottom());
                                
        //                         if(recognition.getBottom()-recognition.getTop() > 50) {
        //                             telemetry.addData(">", "four rings");
        //                             ringsDetected = 4;
        //                         } else {
        //                             telemetry.addData(">", "one ring");
        //                             ringsDetected = 1;
                                
        //                         }
        //               }
                      
                      
                        
        //               } 
        //               telemetry.update();
         
        //     }
        //         else{
        //                 telemetry.addData(">", "zero rings");
        //                 ringsDetected = 0;
                        
        //             }    
            
        }
        // if (tfod != null) {
        //     tfod.shutdown();
        // }
        //gyro.resetZAxisIntegrator();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        bottomTarget = robot.lift.getCurrentPosition() + 300;
        middleTarget = robot.lift.getCurrentPosition() + 610; 
        topTarget = robot.lift.getCurrentPosition() + 925;
        positionTop = robot.lift.getCurrentPosition() + 1100;
        positionZero = robot.lift.getCurrentPosition() + 0;
        rotateTarget = robot.baseRotate.getCurrentPosition() - 80;
        centerTarget = robot.baseRotate.getCurrentPosition() + 0;
        
        //Do Stuff Here!!!
        //Do stuff here!!!
        
        if (robot.distanceSensor1.getDistance(DistanceUnit.CM) < 100 ) {
            gyroDrive(DRIVE_SPEED, -12, 0, 4);
            gyroHold(TURN_SPEED, 90, 1.5);
            gyroDrive(DRIVE_SPEED, -20, 90, 1);
            gyroHold(TURN_SPEED, 0, 1.3);
            robot.lift.setTargetPosition(middleTarget);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(0.2);
            sleep(1750);
            gyroDrive(0.7, 8, 95, 1.5); 
            robot.intake.setPower(-1);
            sleep(1000);
            robot.intake.setPower(0);
            robot.lift.setPower(0);
            gyroDrive(DRIVE_SPEED, -7, 95, 3);
            gyroHold(TURN_SPEED, -90, 1.5);
            robot.lift.setTargetPosition(bottomTarget);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(0.2);
            sleep(1500);
            gyroDrive(DRIVE_SPEED, -75, -90, 3);
            robot.lift.setTargetPosition(positionZero);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(0.2);
            sleep(1000);
            robot.lift.setPower(0);
            gyroStrafe(DRIVE_SPEED, 10, -90, 1);
            
            
        }    
        else if (robot.distanceSensor2.getDistance(DistanceUnit.CM) < 100 ) {
            gyroDrive(DRIVE_SPEED, -12, 0, 4);
            gyroHold(TURN_SPEED, 90, 1.5);
            gyroDrive(DRIVE_SPEED, -20, 90, 1);
            gyroHold(TURN_SPEED, 0, 1.5);
            robot.lift.setTargetPosition(bottomTarget);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(0.4);
            sleep(500);
            gyroDrive(0.7, 7.5, 95, 1); 
            robot.intake.setPower(-1);
            sleep(1000);
            robot.intake.setPower(0);
            robot.lift.setPower(0);
            gyroDrive(DRIVE_SPEED, -6, 95, 3);
            gyroHold(TURN_SPEED, 90, 1.5);
            robot.lift.setTargetPosition(positionZero);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(0.4);
            sleep(1500);
            robot.lift.setPower(0);
            gyroDrive(DRIVE_SPEED, 20, 90, 1);
            gyroStrafe(DRIVE_SPEED, 25, 90, 2);
            

            // robot.lift.setTargetPosition(bottomTarget);
            // robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // robot.lift.setPower(0.2);
            // sleep(1500);
            gyroDrive(DRIVE_SPEED, 30, 90, 3);
            robot.intake.setPower(1);
            gyroDrive(0.6, 15, 90, 3);
            gyroStrafe(DRIVE_SPEED, 1, 90, 0.5);

            while (((DistanceSensor) robot.color).getDistance(DistanceUnit.CM)>2){
        
                gyroDrive(0.6, 4, 90, 1);
                sleep(250);
                gyroDrive(0.6, -2, 90, 1);
            }
            
            if (((DistanceSensor) robot.color).getDistance(DistanceUnit.CM)<2){
            redLED1.setState(false);
            greenLED1.setState(true);
            redLED2.setState(false);
            greenLED2.setState(true);
            redLED3.setState(false);
            greenLED3.setState(true);
            redLED4.setState(false);
            greenLED4.setState(true);
            gyroStrafe(DRIVE_SPEED, 5, 90, 1);
            robot.lift.setTargetPosition(topTarget);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(0.4);
            gyroDrive(DRIVE_SPEED, -60, 90, 4);
            robot.intake.setPower(0);
            gyroStrafe(DRIVE_SPEED, -10, 90, 1);
            
            robot.baseRotate.setTargetPosition(-rotateTarget);
            robot.baseRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.baseRotate.setPower(-0.3);
            sleep(2500);
            gyroStrafe(DRIVE_SPEED, -24, 0, 2);
            robot.intake.setPower(-1);
            sleep(1250);
            robot.intake.setPower(0);
            robot.baseRotate.setTargetPosition(centerTarget);
            robot.baseRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.baseRotate.setPower(0.4);
            sleep(750);
            robot.lift.setTargetPosition(bottomTarget);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(0.4);
            gyroStrafe(DRIVE_SPEED, 8, 90, 1);
            robot.lift.setPower(0);
            sleep(1000);
            gyroDrive(DRIVE_SPEED, 65, 90, 3);
            robot.lift.setTargetPosition(positionZero);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(0.4);
            sleep(500);
            
            }
            
            
            
            // gyroStrafe(DRIVE_SPEED, 46, 95, 2);
            // gyroStrafe(0.8, 8.5, 95, 2);
            // robot.spinner.setPower(-0.55); 
            // sleep(2400);
            // robot.spinner.setPower(0);
            // gyroStrafe(DRIVE_SPEED, -25, 90, 4);
            // gyroDrive(DRIVE_SPEED, -6, 90, 1);
            // robot.lift.setTargetPosition(positionTop);
            // robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // robot.lift.setPower(0.2);
            // sleep(1000);
        
        }
        
        else {
            gyroStrafe(DRIVE_SPEED, -2, 0, 1);
            gyroDrive(DRIVE_SPEED, -36, 0, 2.5);
            //gyroDrive(DRIVE_SPEED, 8, 0, 1);
            robot.lift.setTargetPosition(topTarget);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(0.2);
            sleep(2500);
            robot.baseRotate.setTargetPosition(rotateTarget);
            robot.baseRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.baseRotate.setPower(-0.3);
            sleep(2500);
            gyroStrafe(DRIVE_SPEED, 4, 0, 1);
            robot.intake.setPower(-1);
            sleep(1250);
            robot.intake.setPower(0);
            // gyroStrafe(DRIVE_SPEED, -15, 0, 2);
            robot.baseRotate.setTargetPosition(centerTarget);
            robot.baseRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.baseRotate.setPower(0.3);
            sleep(750);
            robot.lift.setTargetPosition(positionZero);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(0.2);
            sleep(2500);
            robot.lift.setPower(0);
            sleep(1000);
            gyroDrive(DRIVE_SPEED, 22, 0, 1);
            gyroHold(TURN_SPEED, -90, 1.25);
            robot.lift.setTargetPosition(bottomTarget);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(0.2);
            sleep(1500);
            gyroDrive(DRIVE_SPEED, -60, -90, 4);
            // gyroHold(TURN_SPEED, 0, 1);
            // gyroStrafe(DRIVE_SPEED, 17, -95, 2);
            // gyroDrive(.8, -28, 0, 1.5);
            // robot.spinner.setPower(0.55); //.55
            // sleep(2500);
            // robot.spinner.setPower(0);
            // gyroDrive(DRIVE_SPEED, 19.5, -90, 4);
            // gyroStrafe(DRIVE_SPEED, 5, -90, 1);
            // gyroHold(TURN_SPEED, 0, 0.25);
            robot.lift.setTargetPosition(positionZero);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(0.2);
            sleep(1000);
            gyroStrafe(DRIVE_SPEED, 10, -90, 1);
        }
        //Getting into Storage Place
        // gyroDrive(DRIVE_SPEED, -16, 0, 4); //-0.25
        // gyroHold(TURN_SPEED, 90, 1);
        // gyroDrive(DRIVE_SPEED, -16, 90, 4);
        // gyroStrafe(DRIVE_SPEED, 14, 90, 4);
        // robot.spinner.setPower(-0.55); //.55
        // sleep(2400);
        // robot.spinner.setPower(0);
        // gyroStrafe(DRIVE_SPEED, -21, 90, 4);
        
        
        //gyroDrive(DRIVE_SPEED, 125, 90, 6);
        //gyroStrafe(DRIVE_SPEED, -5, -90, 0.5);
        //gyroDrive(DRIVE_SPEED, 50, -90, 5);
        //gyroTurn(TURN_SPEED, 130);     //gyroDrive(leftFrontDrive, -50, 0, 50);
         //robot.leftFrontDrive.setTargetPosition(-50);
         //robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // sleep(1000);
        //gyroHold(TURN_SPEED, 130, 1.5);
        //robot.spinner.setPower(-0.55);
        //sleep(1000);
        
        //robot.wobble1.setPosition(0);
        //robot.wobble2.setPosition(1);
        // sleep(500);
        // robot.wobbleMotor.setTargetPosition(upWobbleTarget);
        // robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // robot.wobbleMotor.setPower(0.3);
        //robot.wobbleMotor.setPower(0);
        
        // sleep(1000);
        // gyroDrive(DRIVE_SPEED, 22.0, 0.0, 2.0);
        // gyroStrafe(DRIVE_SPEED, 13, 0.0, 1.5); //17
        // gyroDrive(DRIVE_SPEED, 31, 0, 2); //33
        // robot.shooter.setPower(.529); //.535!!! .533 .538 .530
        // gyroHold(TURN_SPEED, 0.0, 2.0);
        // //robot.wobbleMotor.setPower(0.75);
        // sleep(1000);
        // //robot.wobbleMotor.setPower(0);
        // robot.flicker.setPosition(.3);
        // sleep(1000);
        // robot.flicker.setPosition(0);
        // sleep(1000);
        // robot.flicker.setPosition(.3);
        // sleep(1000);
        // robot.flicker.setPosition(0);
        // sleep(1000);
        // robot.flicker.setPosition(.3);
        // sleep(1000);
        // robot.flicker.setPosition(0);
        // sleep(1000);
        // robot.shooter.setPower(0);
        // //gyroDrive(DRIVE_SPEED, 12, 0, 1);
        
        
        // if (ringsDetected == 0) {
            
        //     gyroStrafe(DRIVE_SPEED, -10, 0.0, 1.0);
        //     gyroDrive(DRIVE_SPEED, 17, 0.0, 1); 
        //     robot.wobbleMotor.setTargetPosition(downWobbleTarget);
        //     robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //     robot.wobbleMotor.setPower(0.35);
        //     sleep(1000);
        //     robot.wobble1.setPosition(1);
        //     robot.wobble2.setPosition(0);
        //     sleep(250);
        //     gyroStrafe(DRIVE_SPEED, -45, 0, 3);
        //     gyroHold(TURN_SPEED, 0.0, 0.25);
        //     gyroDrive(DRIVE_SPEED, -49.8, 0, 3.5); //55, 44.3, 50.4
        //     gyroHold(TURN_SPEED, 0.0, 0.25);
        //     gyroStrafe(DRIVE_SPEED, 13, 0, 1); //12
        //     sleep(500);
        //     gyroHold(TURN_SPEED, 0.0, 0.75);
        //     robot.wobble1.setPosition(0);
        //     robot.wobble2.setPosition(1);
        //     sleep(500);
        //     robot.wobbleMotor.setTargetPosition(upWobbleTarget);
        //     robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //     robot.wobbleMotor.setPower(0.35);
        //     gyroDrive(DRIVE_SPEED, 61, 0, 3);
        //     gyroStrafe(DRIVE_SPEED, 30, 0, 3);
        //     gyroHold(TURN_SPEED, 0.0, 0.25);
        //     robot.wobbleMotor.setTargetPosition(downWobbleTarget);
        //     robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //     robot.wobbleMotor.setPower(0.35);
        //     sleep(1000);
        //     robot.wobble1.setPosition(1);
        //     robot.wobble2.setPosition(0);
        //     gyroStrafe(DRIVE_SPEED, -15, 0, 0.5);
            
        // }
        
        // else if (ringsDetected == 1){
            
        //     gyroDrive(DRIVE_SPEED, 38, 0.0, 3); //line was swapped, 45, 38
        //     gyroStrafe(DRIVE_SPEED, -34, 0.0, 3.0); //line was swapped, 38, 40
        //     gyroHold(TURN_SPEED, 0.0, 0.5);
        //     robot.wobbleMotor.setTargetPosition(downWobbleTarget);
        //     robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //     robot.wobbleMotor.setPower(0.3); //.35
        //     sleep(1000);
        //     robot.wobble1.setPosition(1);
        //     robot.wobble2.setPosition(0);
        //     sleep(500);
        //     gyroStrafe(DRIVE_SPEED, -16, 0, 1);
        //     gyroDrive(DRIVE_SPEED, -78, 0, 4); //81.5, 75, 78, 76.5
        //     gyroHold(TURN_SPEED, 0, .25);
        //     gyroStrafe(DRIVE_SPEED, 14, 0, 1);
        //     sleep(500);
        //     gyroHold(TURN_SPEED, 0.0, 0.25);
        //     robot.wobble1.setPosition(0);
        //     robot.wobble2.setPosition(1);
        //     sleep(500);
        //     robot.wobbleMotor.setTargetPosition(upWobbleTarget);
        //     robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //     robot.wobbleMotor.setPower(0.3); //.35
        //     sleep(250);
        //     gyroDrive(DRIVE_SPEED, 80, 0, 4); //80, 78
        //     //gyroStrafe(DRIVE_SPEED, 8, 0, 1); //10
        //     robot.wobbleMotor.setTargetPosition(downWobbleTarget);
        //     robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //     robot.wobbleMotor.setPower(0.35);
        //     sleep(1000);
        //     robot.wobble1.setPosition(1);
        //     robot.wobble2.setPosition(0);
        //     sleep(250);
        //     //gyroStrafe(DRIVE_SPEED, -5, 0, 1);
        //     gyroDrive(DRIVE_SPEED, -20, 0, 3);
            
            
        // }
        // else{
        //     gyroStrafe(DRIVE_SPEED, -10, 0.0, 1.0); //11
        //     gyroDrive(DRIVE_SPEED, 61, 0.0, 3);
        //     robot.wobbleMotor.setTargetPosition(downWobbleTarget);
        //     robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //     robot.wobbleMotor.setPower(0.35);
        //     sleep(1000);
        //     robot.wobble1.setPosition(1);
        //     robot.wobble2.setPosition(0);
        //     sleep(500);
        //     gyroStrafe(DRIVE_SPEED, -39, 0.0, 2.0); //40
        //     gyroHold(TURN_SPEED, 0.0, 0.5);
        //     gyroDrive(DRIVE_SPEED, -99, 0, 5); //98.75 94.2, 97.5
        //     gyroHold(TURN_SPEED, 0.0, 0.5);
        //     gyroStrafe(DRIVE_SPEED, 13.5, 0, 1); //12.5, 13
        //     robot.wobble1.setPosition(0);
        //     robot.wobble2.setPosition(1);
        //     gyroHold(TURN_SPEED, 0.0, 0.5);
        //     sleep(500);
        //     robot.wobbleMotor.setTargetPosition(upWobbleTarget);
        //     robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //     robot.wobbleMotor.setPower(0.35);
        //     gyroDrive(DRIVE_SPEED, 102, 0, 5); //100
        //     gyroStrafe(DRIVE_SPEED, 33, 0, 2); //32
        //     gyroHold(TURN_SPEED, 0.0, 0.5);
        //     robot.wobbleMotor.setTargetPosition(downWobbleTarget);
        //     robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //     robot.wobbleMotor.setPower(0.35);
        //     sleep(750);
        //     robot.wobble1.setPosition(1);
        //     robot.wobble2.setPosition(0);
        //     sleep(500);
        //     gyroDrive(DRIVE_SPEED, -40, 0, 3);//42
        // }
    
        // Drive FWD 48 inches
        // gyroTurn( TURN_SPEED, -45.0);         // Turn  CCW to -45 Degrees
        // gyroHold( TURN_SPEED, -45.0, 0.5);    // Hold -45 Deg heading for a 1/2 second
        // gyroDrive(DRIVE_SPEED, 12.0, -45.0);  // Drive FWD 12 inches at 45 degrees
        // gyroTurn( TURN_SPEED,  45.0);         // Turn  CW  to  45 Degrees
        // gyroHold( TURN_SPEED,  45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second
        // gyroTurn( TURN_SPEED,   0.0);         // Turn  CW  to   0 Degrees
        // gyroHold( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for a 1 second
        // gyroDrive(DRIVE_SPEED,-48.0, 0.0);    // Drive REV 48 inches

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


   /**
    *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
    * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from current heading.
    */
    
    public void gyroDrive ( double speed,
                            double distance,
                            double angle,
                            double timeout) {

        int     newLeftFrontTarget;
        int     newRightFrontTarget;
        int     newLeftBackTarget;
        int     newRightBackTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftFrontSpeed;
        double  rightFrontSpeed;
        double  leftBackSpeed;
        double  rightBackSpeed;
        //double  spinnerSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + moveCounts; //-, plunged in wrong way? 
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + moveCounts;
            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() + moveCounts;
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);

            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftFrontDrive.setPower(speed);
            robot.rightFrontDrive.setPower(speed);
            robot.leftBackDrive.setPower(speed);
            robot.rightBackDrive.setPower(speed);
            //robot.spinner.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            runtime.reset();
            while (opModeIsActive() &&
                   (robot.leftFrontDrive.isBusy() && robot.rightFrontDrive.isBusy()) &&
                (robot.leftBackDrive.isBusy() && robot.rightBackDrive.isBusy())
                && runtime.seconds()<timeout)  {


                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftFrontSpeed = speed - steer;
                rightFrontSpeed =  speed + steer;
                leftBackSpeed =  speed - steer;
                rightBackSpeed = speed + steer;
               
                // Normalize speeds if either one exceeds +/- 1.0;
               max = Math.max (
                    Math.max(Math.abs(leftFrontSpeed), Math.abs(rightFrontSpeed)),
                    Math.max(Math.abs(leftBackSpeed), Math.abs(rightBackSpeed))
                );
                if (max > 1.0)
                {
                    leftFrontSpeed /= max;
                    rightFrontSpeed /= max;
                    leftBackSpeed /= max;
                    rightBackSpeed /= max;
                }

                robot.leftFrontDrive.setPower(leftFrontSpeed);
                robot.rightFrontDrive.setPower(rightFrontSpeed);
                robot.leftBackDrive.setPower(leftBackSpeed);
                robot.rightBackDrive.setPower(rightBackSpeed);
                //robot.spinner.setPower(spinnerSpeed);
                
                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      newLeftFrontTarget,  newRightFrontTarget,  newLeftBackTarget,  newRightBackTarget);
                telemetry.addData("ActualFront",  "%7d:%7d",      robot.leftFrontDrive.getCurrentPosition(),
                                                             robot.rightFrontDrive.getCurrentPosition());
                telemetry.addData("ActualBack",  "%7d:%7d",                                            
                                                             robot.leftBackDrive.getCurrentPosition(),
                                                             robot.rightBackDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f:%5.2f:%5.2f",  leftFrontSpeed, rightFrontSpeed, leftBackSpeed, rightBackSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.leftBackDrive.setPower(0);
            robot.rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }



public void gyroStrafe ( double speed,
                            double distance,
                            double angle,
                            double timeout) {

        int     newLeftFrontTarget;
        int     newRightFrontTarget;
        int     newLeftBackTarget;
        int     newRightBackTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftFrontSpeed;
        double  rightFrontSpeed;
        double  leftBackSpeed;
        double  rightBackSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + moveCounts;
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() - moveCounts;
            newLeftBackTarget = robot.leftBackDrive.getCurrentPosition() - moveCounts;
            newRightBackTarget = robot.rightBackDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            robot.leftBackDrive.setTargetPosition(newLeftBackTarget);
            robot.rightBackDrive.setTargetPosition(newRightBackTarget);

            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftFrontDrive.setPower(speed);
            robot.rightFrontDrive.setPower(-speed);
            robot.leftBackDrive.setPower(-speed);
            robot.rightBackDrive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            runtime.reset();
            while (opModeIsActive() &&
                   (robot.leftFrontDrive.isBusy() && robot.rightFrontDrive.isBusy()) &&
                (robot.leftBackDrive.isBusy() && robot.rightBackDrive.isBusy())
                && runtime.seconds()<timeout)  {


                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftFrontSpeed = speed - steer;
                rightFrontSpeed = - speed + steer;
                leftBackSpeed = - speed - steer;
                rightBackSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
               max = Math.max (
                    Math.max(Math.abs(leftFrontSpeed), Math.abs(rightFrontSpeed)),
                    Math.max(Math.abs(leftBackSpeed), Math.abs(rightBackSpeed))
                );
                if (max > 1.0)
                {
                    leftFrontSpeed /= max;
                    rightFrontSpeed /= max;
                    leftBackSpeed /= max;
                    rightBackSpeed /= max;
                }

               robot.leftFrontDrive.setPower(leftFrontSpeed);
                robot.rightFrontDrive.setPower(rightFrontSpeed);
                robot.leftBackDrive.setPower(leftBackSpeed);
                robot.rightBackDrive.setPower(rightBackSpeed);
               
                if (Math.abs(newLeftFrontTarget-robot.leftFrontDrive.getCurrentPosition())<25){
                    robot.leftFrontDrive.setPower(0);
                }
               
                if (Math.abs(newRightFrontTarget-robot.rightFrontDrive.getCurrentPosition())<25){
                    robot.rightFrontDrive.setPower(0);
                }

                if (Math.abs(newLeftBackTarget-robot.leftBackDrive.getCurrentPosition())<25){
                    robot.leftBackDrive.setPower(0);
                }
               
                 if (Math.abs(newRightBackTarget-robot.rightBackDrive.getCurrentPosition())<25){
                    robot.rightBackDrive.setPower(0);
                }


                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("ActualFront",  "%7d:%7d",      robot.leftFrontDrive.getCurrentPosition(),
                                                             robot.rightFrontDrive.getCurrentPosition());
                telemetry.addData("ActualBack",  "%7d:%7d",                                            
                                                             robot.leftBackDrive.getCurrentPosition(),
                                                             robot.rightBackDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f:%5.2f:%5.2f",  leftFrontSpeed, rightFrontSpeed, leftBackSpeed, rightBackSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.leftBackDrive.setPower(0);
            robot.rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
   
    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle, double timeout) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;
        double  spinnerSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftFrontDrive.setPower(leftSpeed);
        robot.rightFrontDrive.setPower(rightSpeed);
        robot.leftBackDrive.setPower(leftSpeed);
        robot.rightBackDrive.setPower(rightSpeed);
        //robot.spinner.setPower(spinnerSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

// private void initVuforia() {
//         /*
//          * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//          */
//         VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

//         parameters.vuforiaLicenseKey = VUFORIA_KEY;
//         parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

//         //  Instantiate the Vuforia engine
//         vuforia = ClassFactory.getInstance().createVuforia(parameters);

//         // Loading trackables is not necessary for the TensorFlow Object Detection engine.
//     }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    // private void initTfod() {
    //     int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
    //         "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    //     TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
    //   //tfodParameters.minResultConfidence = 0.8f; //0.8f
    //   tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
    //   tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    // }

}

