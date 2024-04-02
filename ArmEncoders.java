
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "ArmEncoders")

public class ArmEncoders extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor lift;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor frontRight;
    DcMotor backLeft;
    CRServo claw1;
    CRServo claw2;


     ElapsedTime runtime = new ElapsedTime();

    static final double ARM_COUNTS_PER_MOTOR_REV = 384.5;    // eg: TETRIX Motor Encoder
    static final double ARM_DRIVE_GEAR_REDUCTION = 0.3;     // No External Gearing.
    static final double ARM_WHEEL_DIAMETER_INCHES = 0.8;     // For figuring circumference
    static final double ARM_COUNTS_PER_INCH = (ARM_COUNTS_PER_MOTOR_REV * ARM_DRIVE_GEAR_REDUCTION) /
            (ARM_WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.4;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        lift = hardwareMap.get(DcMotor.class, "lift");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        claw1 = hardwareMap.crservo.get("claw1");
        claw2 = hardwareMap.crservo.get("claw2");


        lift.setDirection(DcMotor.Direction.REVERSE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                lift.getCurrentPosition(),
                telemetry.update());

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        liftEncoderDrive(DRIVE_SPEED,  5,  15);




        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }


    public void liftEncoderDrive(double speed,
                                 double inches,
                                 double timeoutS) {
        int newLiftTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLiftTarget = lift.getCurrentPosition() + (int) (inches * ARM_COUNTS_PER_INCH);

            lift.setTargetPosition(newLiftTarget);

            // Turn On RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            lift.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() && frontRight.isBusy()) && (backLeft.isBusy() && backRight.isBusy())) {


                // Display it for the driver.
                telemetry.addData("Running to", " %7d ", newLiftTarget);
                //telemetry.addData("Currently at",  " at %7d :%7d" :%7d :%7d",
                // frontLeft.getCurrentPosition(), frontRight.getCurrentPosition();
                //backLeft.getCurrentPosition(), backRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            lift.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move.
        }
    }

}
