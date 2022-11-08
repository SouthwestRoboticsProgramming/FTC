telemetry.update();

// gyro.calibrate();

// // make sure the gyro is calibrated before continuing
// while (!isStopRequested() && gyro.isCalibrating())  {
//     sleep(50);
//     idle();
// }

telemetry.addData(">", "Robot Ready.");    //
telemetry.update();

robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

// Wait for the game to start (Display Gyro value), and reset gyro before we move..
while (!isStarted()) {
    //telemetry.addData(">", "Robot Heading = %f", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
    //telemetry.update();
    
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
              telemetry.addData("# Object Detected", updatedRecognitions.size());
              // step through the list of recognitions and display boundary info.
              int i = 0;
             
                telemetry.addData(">", "zero rings");
                ringsDetected = 0;
              
              
              for (Recognition recognition : updatedRecognitions) {
                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
                        
                        if(recognition.getBottom()-recognition.getTop() > 50) {
                            telemetry.addData(">", "four rings");
                            ringsDetected = 4;
                        } else {
                            telemetry.addData(">", "one ring");
                            ringsDetected = 1;
                        
                        }
              }
              
              
                
              } 
               telemetry.update();
 
    }
        else{
                telemetry.addData(">", "zero rings");
                ringsDetected = 0;
                
            }    
    
}
if (tfod != null) {
    tfod.shutdown();
}
//gyro.resetZAxisIntegrator();

// Step through each leg of the path,
// Note: Reverse movement is obtained by setting a negative distance (not speed)
// Put a hold after each turn

 upWobbleTarget = robot.wobbleMotor.getCurrentPosition() + 350;
downWobbleTarget = robot.wobbleMotor.getCurrentPosition() + 5; //+5

//Do Stuff Here!!!
//Do stuff here!!!

// robot.wobble1.setPosition(0);
// robot.wobble2.setPosition(1);
// sleep(500);
// robot.wobbleMotor.setTargetPosition(upWobbleTarget);
// robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
// robot.wobbleMotor.setPower(0.35);
// //robot.wobbleMotor.setPower(0);robot.wobble1.setPosition(0);
// //robot.wobble2.setPosition(1);
// // sleep(500);
// // robot.wobbleMotor.setTargetPosition(upWobbleTarget);
// // robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
// // robot.wobbleMotor.setPower(0.35);
// //robot.wobbleMotor.setPower(0);
// sleep(1000);
// gyroDrive(DRIVE_SPEED, 54.0, 0.0, 2.0); //22
// // gyroStrafe(DRIVE_SPEED, 12, 0.0, 1.0);
// // gyroDrive(DRIVE_SPEED, 32, 0, 2);
// gyroStrafe(DRIVE_SPEED, -14.5, 0, 1.5); //18, 20
// robot.shooter.setPower(.56); //.474-.52
// gyroHold(TURN_SPEED, 0.0, 1.75);
// sleep(1000);
// robot.flicker.setPosition(.3);
// sleep(500);
// robot.flicker.setPosition(0);
// sleep(1000);
// robot.shooter.setPower(.546);
// gyroStrafe(DRIVE_SPEED, -9.5, 0, 1);
// gyroHold(TURN_SPEED, 0, 0.5);
// sleep(500);
// robot.flicker.setPosition(.3);
// sleep(500);
// robot.flicker.setPosition(0);
// sleep(1000);
// robot.shooter.setPower(.546);
// gyroStrafe(DRIVE_SPEED, -8, 0, 1);
// gyroHold(TURN_SPEED, 0, .5);
// sleep(500);
// robot.flicker.setPosition(.3);
// sleep(500);
// robot.flicker.setPosition(0);
// sleep(500);
// robot.shooter.setPower(0);
// //gyroDrive(DRIVE_SPEED, 12, 0, 1);

if (ringsDetected == 0) {

     robot.wobble1.setPosition(0);
    robot.wobble2.setPosition(1);
    sleep(500);
    robot.wobbleMotor.setTargetPosition(upWobbleTarget);
    robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.wobbleMotor.setPower(0.35);
    //robot.wobbleMotor.setPower(0);robot.wobble1.setPosition(0);
    //robot.wobble2.setPosition(1);
    // sleep(500);
    // robot.wobbleMotor.setTargetPosition(upWobbleTarget);
    // robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // robot.wobbleMotor.setPower(0.35);
    //robot.wobbleMotor.setPower(0);
    sleep(1000);
    gyroDrive(DRIVE_SPEED, 54.0, 0.0, 2.0); //22
    // gyroStrafe(DRIVE_SPEED, 12, 0.0, 1.0);
    // gyroDrive(DRIVE_SPEED, 32, 0, 2);
    gyroStrafe(DRIVE_SPEED, -10.5, 0, 1.5); //18, 20, 14.5, 13.5, 13, 12.25
    robot.shooter.setPower(.545); //.474-.52, .56
    gyroHold(TURN_SPEED, 0.0, 1.75);
    sleep(1000);
    robot.flicker.setPosition(.3);
    sleep(500);
    robot.flicker.setPosition(0);
    sleep(1000);
    robot.shooter.setPower(.545); //.56, .55
    gyroStrafe(DRIVE_SPEED, -6.5, 0, 1); //7.5, 6.5
    gyroHold(TURN_SPEED, 0, 0.5);
    sleep(500);
    robot.flicker.setPosition(.3);
    sleep(500);
    robot.flicker.setPosition(0);
    sleep(1000);
    robot.shooter.setPower(.54); //.55, .54, .5425
    gyroStrafe(DRIVE_SPEED, -8.5, 0, 1); //9, 9.5
    gyroHold(TURN_SPEED, 0, .5);
    sleep(500);
    robot.flicker.setPosition(.3);
    sleep(500);
    robot.flicker.setPosition(0);
    sleep(500);
    robot.shooter.setPower(0);
    //gyroDrive(DRIVE_SPEED, 12, 0, 1);

    gyroDrive(DRIVE_SPEED, 21, 0, 2);  
    gyroStrafe(DRIVE_SPEED, 32, 0, 2); //34
    gyroHold(TURN_SPEED, 0, 1);
    robot.wobbleMotor.setTargetPosition(downWobbleTarget);
    robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.wobbleMotor.setPower(0.35);
    sleep(1000);
    robot.wobble1.setPosition(1);
    robot.wobble2.setPosition(0);
    sleep(250);
    gyroStrafe(DRIVE_SPEED, -57, 0, 3); //-49
    gyroHold(TURN_SPEED, 0.0, 0.25);
    gyroDrive(DRIVE_SPEED, -41.5, 0, 3); //43.5, 41, 41.65, 40, 38, 36.65, 40.5
    gyroHold(TURN_SPEED, 0.0, 0.25);
    gyroStrafe(DRIVE_SPEED, 14, 0, 1); //12
    sleep(500);
    gyroHold(TURN_SPEED, 0.0, 0.75);
    robot.wobble1.setPosition(0);
    robot.wobble2.setPosition(1);
    sleep(350);
     robot.wobbleMotor.setTargetPosition(upWobbleTarget);
    robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.wobbleMotor.setPower(0.35);
    sleep(1000);
    gyroDrive(DRIVE_SPEED, 60, 0, 3); //62
    gyroHold(TURN_SPEED, 0.0, 0.5);
    gyroStrafe(DRIVE_SPEED, 27, 0, 2.25); //29, 27
    gyroHold(TURN_SPEED, 0.0, 0.5);
    robot.wobbleMotor.setTargetPosition(downWobbleTarget);
    robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.wobbleMotor.setPower(0.35);
    sleep(1000);
    robot.wobble1.setPosition(1);
    robot.wobble2.setPosition(0);
    sleep(250);
    gyroStrafe(DRIVE_SPEED, -7, 0, 1);

}

else if (ringsDetected == 1){

   robot.wobble1.setPosition(0);
    robot.wobble2.setPosition(1);
    sleep(500);
    robot.wobbleMotor.setTargetPosition(upWobbleTarget);
    robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.wobbleMotor.setPower(0.35);
    //robot.wobbleMotor.setPower(0);robot.wobble1.setPosition(0);
    //robot.wobble2.setPosition(1);
    // sleep(500);
    // robot.wobbleMotor.setTargetPosition(upWobbleTarget);
    // robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // robot.wobbleMotor.setPower(0.35);
    //robot.wobbleMotor.setPower(0);
    sleep(1000);
    gyroDrive(DRIVE_SPEED, 54.0, 0.0, 2.0); //22
    // gyroStrafe(DRIVE_SPEED, 12, 0.0, 1.0);
    // gyroDrive(DRIVE_SPEED, 32, 0, 2);
    gyroStrafe(DRIVE_SPEED, -10.5, 0, 1.5); //18, 20, 14.5, 13.5, 13, 12.25
    robot.shooter.setPower(.545); //.474-.52, .56
    gyroHold(TURN_SPEED, 0.0, 1.75);
    sleep(1000);
    robot.flicker.setPosition(.3);
    sleep(500);
    robot.flicker.setPosition(0);
    sleep(1000);
    robot.shooter.setPower(.545); //.56, .55
    gyroStrafe(DRIVE_SPEED, -6.5, 0, 1); //7.5, 6.5
    gyroHold(TURN_SPEED, 0, 0.5);
    sleep(500);
    robot.flicker.setPosition(.3);
    sleep(500);
    robot.flicker.setPosition(0);
    sleep(1000);
    robot.shooter.setPower(.54); //.55, .54, .5425
    gyroStrafe(DRIVE_SPEED, -8.5, 0, 1); //9, 9.5
    gyroHold(TURN_SPEED, 0, .5);
    sleep(500);
    robot.flicker.setPosition(.3);
    sleep(500);
    robot.flicker.setPosition(0);
    sleep(500);
    robot.shooter.setPower(0);
    //gyroDrive(DRIVE_SPEED, 12, 0, 1);

    //  robot.wobble1.setPosition(0);
    // robot.wobble2.setPosition(1);
    // sleep(500);
    // robot.wobbleMotor.setTargetPosition(upWobbleTarget);
    // robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // robot.wobbleMotor.setPower(0.35);
    // //robot.wobbleMotor.setPower(0);robot.wobble1.setPosition(0);
    // //robot.wobble2.setPosition(1);
    // // sleep(500);
    // // robot.wobbleMotor.setTargetPosition(upWobbleTarget);
    // // robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // // robot.wobbleMotor.setPower(0.35);
    // //robot.wobbleMotor.setPower(0);
    // sleep(1000);
    // gyroDrive(DRIVE_SPEED, 54.0, 0.0, 2.0); //22
    // // gyroStrafe(DRIVE_SPEED, 12, 0.0, 1.0);
    // // gyroDrive(DRIVE_SPEED, 32, 0, 2);
    // gyroStrafe(DRIVE_SPEED, -14.5, 0, 1.5); //18, 20
    // robot.shooter.setPower(.56); //.474-.52
    // gyroHold(TURN_SPEED, 0.0, 1.75);
    // sleep(1000);
    // robot.flicker.setPosition(.3);
    // sleep(500);
    // robot.flicker.setPosition(0);
    // sleep(1000);
    // robot.shooter.setPower(.56);
    // gyroStrafe(DRIVE_SPEED, -7.5, 0, 1);
    // gyroHold(TURN_SPEED, 0, 0.5);
    // sleep(500);
    // robot.flicker.setPosition(.3);
    // sleep(500);
    // robot.flicker.setPosition(0);
    // sleep(1000);
    // robot.shooter.setPower(.55);
    // gyroStrafe(DRIVE_SPEED, -9, 0, 1);
    // gyroHold(TURN_SPEED, 0, .5);
    // sleep(500);
    // robot.flicker.setPosition(.3);
    // sleep(500);
    // robot.flicker.setPosition(0);
    // sleep(500);
    // robot.shooter.setPower(0);

    gyroDrive(DRIVE_SPEED, 44, 0.0, 3.0);
    //gyroStrafe(DRIVE_SPEED, 7, 0.0, 1.5); //Sometimes commented out
    gyroStrafe(DRIVE_SPEED, -5, 0.0, 0.5);
    gyroHold(TURN_SPEED, 0.0, 0.5);
    robot.wobbleMotor.setTargetPosition(downWobbleTarget);
    robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.wobbleMotor.setPower(0.35);
    sleep(1000);
    robot.wobble1.setPosition(1);
    robot.wobble2.setPosition(0);
    sleep(500);
    gyroStrafe(DRIVE_SPEED, -18.75, 0, 1); //17, 20
    gyroDrive(DRIVE_SPEED, -74.75, 0, 4);//79.5, 79.8, 74, 74.5, 73.75
    gyroHold(TURN_SPEED, 0.0, 0.5);
    gyroStrafe(DRIVE_SPEED, 14, 0, 1);
    gyroHold(TURN_SPEED, 0.0, 0.5);
    robot.wobble1.setPosition(0);
    robot.wobble2.setPosition(1);
    sleep(250);
     robot.wobbleMotor.setTargetPosition(upWobbleTarget);
    robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.wobbleMotor.setPower(0.35);
    sleep(1000);
    gyroDrive(DRIVE_SPEED, 80, 0, 3);
    //gyroHold(TURN_SPEED, 0.0, 0.5);
    gyroStrafe(DRIVE_SPEED, -7, 0, .65); //7, 8.5
    gyroHold(TURN_SPEED, 0.0, 0.5);
    robot.wobbleMotor.setTargetPosition(downWobbleTarget);
    robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.wobbleMotor.setPower(0.35);
    sleep(1000);
    robot.wobble1.setPosition(1);
    robot.wobble2.setPosition(0);
    sleep(250);
    gyroStrafe(DRIVE_SPEED, -7, 0, 1);
    gyroDrive(DRIVE_SPEED, -20, 0, 2);
    
}

else {
    
    robot.wobble1.setPosition(0);
robot.wobble2.setPosition(1);
sleep(500);
robot.wobbleMotor.setTargetPosition(upWobbleTarget);
robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
robot.wobbleMotor.setPower(0.35);
//robot.wobbleMotor.setPower(0);

sleep(1000);
gyroDrive(DRIVE_SPEED, 20.0, 0.0, 1.5);
gyroStrafe(DRIVE_SPEED, 12, 0.0, 1.0);
gyroDrive(DRIVE_SPEED, 36, 0, 2);
robot.shooter.setPower(.55); //.55, .58, .565, .56
gyroHold(TURN_SPEED, 0.0, 1.0);
//robot.wobbleMotor.setPower(0.75);
sleep(1000);
//robot.wobbleMotor.setPower(0);
robot.flicker.setPosition(.3);
sleep(1000);
robot.flicker.setPosition(0);
sleep(1000);
robot.flicker.setPosition(.3);
sleep(1000);
robot.flicker.setPosition(0);
sleep(1000);
robot.flicker.setPosition(.3);
sleep(1000);
robot.flicker.setPosition(0);
sleep(1000);
robot.shooter.setPower(0);
//gyroDrive(DRIVE_SPEED, 12, 0, 1);
    
    gyroStrafe(DRIVE_SPEED, -15, 0.0, 2.0);
    gyroDrive(DRIVE_SPEED, 58, 0.0, 3); //58, 56
    robot.wobbleMotor.setTargetPosition(downWobbleTarget);
    robot.wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.wobbleMotor.setPower(0.35);
    sleep(750);
    robot.wobble1.setPosition(1);
    robot.wobble2.setPosition(0);
    sleep(500);
    gyroStrafe(DRIVE_SPEED, -6, 0.0, 1.0);
    robot.intake.setPower(1);     
    robot.intake2.setPower(-1);
    gyroHold(TURN_SPEED, 0, .5);
    gyroDrive(DRIVE_SPEED, -55, 0, 3.5); //71
    gyroDrive(.8, -16, 0, 1.5); //sometimes commented out
    //sleep(250);
    gyroDrive(DRIVE_SPEED, 5, 0, .5);
     gyroHold(TURN_SPEED, 0, 0.5);
    gyroDrive(DRIVE_SPEED, -10, 0, 1);
    gyroDrive(DRIVE_SPEED, 5, 0, 1);
    // sleep(1000);
    gyroHold(TURN_SPEED, 0, 1.75);
    //gyroStrafe(DRIVE_SPEED, 20, 0, 1.5); //24, 22
//     gyroDrive(DRIVE_SPEED, 11, 0, .75);
     robot.intake.setPower(0);
     robot.intake2.setPower(0);

     gyroDrive(DRIVE_SPEED, 5, 0, .75);
     
     robot.shooter.setPower(.52); //.525 for Emily's house .535 for Zoe's house, .58
     gyroHold(TURN_SPEED, 0.0, 1.0);
    //robot.wobbleMotor.setPower(0.75);
    //sleep(1000);
    //robot.wobbleMotor.setPower(0);
     robot.flicker.setPosition(.3);
     sleep(1000);
    robot.flicker.setPosition(0);
    sleep(1000);
    robot.shooter.setPower(.56);
    gyroTurn(TURN_SPEED, 5);
    //gyroHold(TURN_SPEED, 0.0, 0.5);
    robot.flicker.setPosition(.3);
    sleep(1000);
    robot.flicker.setPosition(0);
    sleep(1000);
    // robot.flicker.setPosition(.3);
    // sleep(1000); 
    // robot.flicker.setPosition(0);
    // sleep(1000);
     robot.shooter.setPower(0);
    gyroDrive(DRIVE_SPEED, 27, 0, 1);
}
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

// Ensure that the opmode is still active
if (opModeIsActive()) {

    // Determine new target position, and pass to motor controller
    moveCounts = (int)(distance * COUNTS_PER_INCH);
    newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + moveCounts;
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


    // start motion.
    speed = Range.clip(Math.abs(speed), 0.0, 1.0);
    robot.leftFrontDrive.setPower(speed);
    robot.rightFrontDrive.setPower(speed);
    robot.leftBackDrive.setPower(speed);
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
public void gyroTurn (  double speed, double angle) {

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

private void initVuforia() {
/*
 * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
 */
VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

parameters.vuforiaLicenseKey = VUFORIA_KEY;
parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

//  Instantiate the Vuforia engine
vuforia = ClassFactory.getInstance().createVuforia(parameters);

// Loading trackables is not necessary for the TensorFlow Object Detection engine.
}

/**
* Initialize the TensorFlow Object Detection engine.
*/
private void initTfod() {
int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
tfodParameters.minResultConfidence = 0.8f;
tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
}

}
