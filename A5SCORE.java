package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;




        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.firstinspires.ftc.teamcode.vision.SleeveDetection;
        import org.openftc.easyopencv.OpenCvCamera;
        import org.openftc.easyopencv.OpenCvCameraFactory;
        import org.openftc.easyopencv.OpenCvCameraRotation;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.CRServo;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import org.firstinspires.ftc.teamcode.vision. SleeveDetection.ParkingPosition;

@Autonomous(name = "A5SCORE")
public class A5SCORE extends LinearOpMode {

    /* Declare OpMode members. */
    //Movement
    DcMotor frontLeft, frontRight, backLeft, backRight;
    //Game-Related
    DcMotor lift, lift2;
    CRServo claw1, claw2;

    private ElapsedTime     runtime = new ElapsedTime();


    // values for base
    static final double     COUNTS_PER_MOTOR_REV    = 537.7;    // eg: TETRIX Motor Encoder //2150.8
    static final double     DRIVE_GEAR_REDUCTION    = 1;        // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953;     // For figuring circumference

    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;

    public static double STRAFE_INCHES = 12.0;

    //values for arm
    static final double     COUNTS_PER_LIFT_MOTOR_REV    = 384.5;  // eg: TETRIX Motor Encoder //2150.8
    static final double     ARM_GEAR_REDUCTION    = 0.3;        // This is < 1.0 if geared UP
    static final double     SPROCKET_DIAMETER_INCHES   = 0.8;     // For figuring circumference
    static final double     LIFT_PER_INCH         = (COUNTS_PER_LIFT_MOTOR_REV * ARM_GEAR_REDUCTION) / (SPROCKET_DIAMETER_INCHES * 3.1415);

    SleeveDetection sleeveDetection;
    OpenCvCamera camera;
    private volatile ParkingPosition position = ParkingPosition.LEFT;

    String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {
            position = sleeveDetection.getPosition();
            telemetry.addData("ROTATION: ", position);
            telemetry.update();
        }

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");


        lift = hardwareMap.dcMotor.get("lift");
        lift2 = hardwareMap.dcMotor.get("lift2");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw1 = hardwareMap.crservo.get("claw1");
        claw2 = hardwareMap.crservo.get("claw2");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d ",
                lift.getCurrentPosition(),
                frontRight.getCurrentPosition(),
                backLeft.getCurrentPosition(),
                backRight.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        position = sleeveDetection.getPosition();
        //  telemetry.addData("position");
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)(good note, 10/10)

        //Go Forward
        telemetry.addData("Running", "Encoder Test...");
        telemetry.update();


     /*
      claw1.setPower(-.5);
      claw2.setPower(.4);
       encoderDrive(DRIVE_SPEED, 2, 2, 5);
    */
        claw1.setPower(-.5);
        claw2.setPower(.4);

  //      encoderDrive(DRIVE_SPEED, 4, 4, 5);

        claw1.setPower(.5);
        claw2.setPower(-.4);
        sleep(1000);
   //     encoderDrive(DRIVE_SPEED,-6,-6,1.0);

        encoderDriveStrafe(DRIVE_SPEED,16.5,-16.5,2.0); // neg pos
        liftEncoderDrive(DRIVE_SPEED,-33,5.0);
        encoderDrive(DRIVE_SPEED, 3, 3, 5);
        liftEncoderDrive(DRIVE_SPEED, 30, 2.0);
        claw1.setPower(-.5);
        claw2.setPower(.4);
        encoderDrive(DRIVE_SPEED, -3, -3, 1.0);
        claw1.setPower(.5);
        claw2.setPower(-.4);
        encoderDriveStrafe(DRIVE_SPEED, -13,13,1.0); // pos neg QQQQQQQQQ
        sleep(1000);
        encoderDrive(DRIVE_SPEED, 23, 23, 5.0);
        sleep(1000);




//

        if(position == SleeveDetection.ParkingPosition.RIGHT){
            STRAFE_INCHES = 25;
            //telemetry.addData(STRAFE_INCHES);
        }
        else if(position == SleeveDetection.ParkingPosition.LEFT){
            STRAFE_INCHES = -28;
            // telemetry.addData(STRAFE_INCHES);
        }
        else{
            STRAFE_INCHES = 0;
            //telemetry.addData(STRAFE_INCHES);
        }
        encoderDriveStrafe(DRIVE_SPEED, STRAFE_INCHES, -STRAFE_INCHES,5);
        // encoderDrive(DRIVE_SPEED, 20, 20,2.0);



        telemetry.addData("Path", "Complete");
        telemetry.update();


    }


    public void liftEncoderDrive(double speed,
                                 double inches,
                                 double timeoutS) {
        int newliftTarget;
        int newlift2Target;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newliftTarget = lift.getCurrentPosition() + (int) (inches * LIFT_PER_INCH);
            newlift2Target = lift2.getCurrentPosition() - (int) (inches * LIFT_PER_INCH);
            lift.setTargetPosition(newliftTarget);
            lift2.setTargetPosition(newlift2Target);
            // Turn On RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            lift.setPower(Math.abs(speed));
            lift2.setPower(Math.abs(-speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (lift.isBusy())) {// frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d ", newliftTarget);//newBackLeftTarget, newFrontRightTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at %7d  ",
                        lift.getCurrentPosition());
                // backLeft.getCurrentPosition(),
                // frontRight.getCurrentPosition(),
                //  backRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            lift.setPower(0);
            lift2.setPower(0);

            // Turn off RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move
        }
    }
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newBackRightTarget = backRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            frontLeft.setTargetPosition(newFrontLeftTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backRight.setTargetPosition(newBackRightTarget);
            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d : %7d: %7d ", newFrontLeftTarget, newBackLeftTarget, newFrontRightTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d: %7d ",
                        frontLeft.getCurrentPosition(),
                        backLeft.getCurrentPosition(),
                        frontRight.getCurrentPosition(),
                        backRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    public void encoderDriveStrafe(double speed,
                                   double leftInches, double rightInches,
                                   double timeoutS) {

        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeft.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBackRightTarget = backRight.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH);
            frontLeft.setTargetPosition(newFrontLeftTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backRight.setTargetPosition(newBackRightTarget);
            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(-speed));
            frontRight.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(-speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newBackLeftTarget, newFrontRightTarget, newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        frontLeft.getCurrentPosition(),
                        backLeft.getCurrentPosition(),
                        frontRight.getCurrentPosition(),
                        backRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
