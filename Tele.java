package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;

    @TeleOp (name = "mainTeleop")
    public class Tele extends OpMode {
        DcMotor frontLeft;
        DcMotor frontRight;
        DcMotor backLeft;
        DcMotor backRight;
        DcMotor lift;
        DcMotor lift2;
        CRServo claw1;
        CRServo claw2;

        @Override
        public void init() {
            frontLeft = hardwareMap.dcMotor.get("frontLeft");
            backRight = hardwareMap.dcMotor.get("backRight");
            frontRight = hardwareMap.dcMotor.get("frontRight");
            backLeft = hardwareMap.dcMotor.get("backLeft");
            lift = hardwareMap.dcMotor.get("lift");
            claw1 = hardwareMap.crservo.get("claw1");
            claw2 = hardwareMap.crservo.get("claw2");
            lift2 = hardwareMap.dcMotor.get("lift2");
        }

        @Override
        public void loop() {
            telemetry.addData("claw1", claw1.getPower());
            telemetry.addData("claw2", claw2.getPower());
            telemetry.addData("frontLeft", frontLeft.getPower());
            telemetry.addData("frontRight", frontRight.getPower());
            telemetry.addData("backLeft", backLeft.getPower());
            telemetry.addData("backRight", backRight.getPower());
            telemetry.update();

            //Movement (P1) - Going Backwards / Forwards / Turning
            if (Math.abs(gamepad1.left_stick_y) > .1 / 2 || Math.abs(gamepad1.right_stick_y) > .1 / 2) {
                frontLeft.setPower(gamepad1.left_stick_y);  // pos
                backLeft.setPower(gamepad1.left_stick_y);     // pos
                frontRight.setPower(-gamepad1.right_stick_y); // neg
                backRight.setPower(-gamepad1.right_stick_y);   // neg
            }
            //Strafing
            else if (gamepad1.left_trigger > .1) {
                frontLeft.setPower(gamepad1.left_trigger);
                backLeft.setPower(-gamepad1.left_trigger); // neg
                frontRight.setPower(gamepad1.left_trigger);
                backRight.setPower(-gamepad1.left_trigger); // neg
            } else if (gamepad1.right_trigger > .1) {
                frontLeft.setPower(-gamepad1.right_trigger); // neg
                backLeft.setPower(gamepad1.right_trigger);
                frontRight.setPower(-gamepad1.right_trigger); // neg
                backRight.setPower(gamepad1.right_trigger);

            }

            // slow controls
         else if (gamepad1.dpad_left) {
                    frontLeft.setPower(.25);
                    backLeft.setPower(-.25);
                    frontRight.setPower(.25);
                    backRight.setPower(-.25);
                } else if (gamepad1.dpad_right) {
                    frontLeft.setPower(-.25);
                    backLeft.setPower(.25);
                    frontRight.setPower(-.25);
                    backRight.setPower(.25);
                } else if (gamepad1.dpad_down) {
                    frontLeft.setPower(.25);
                    backLeft.setPower(.25);
                    frontRight.setPower(-.25);
                    backRight.setPower(-.25);
                } else if (gamepad1.dpad_up) {
                    frontLeft.setPower(-.25);
                    backLeft.setPower(-.25);
                    frontRight.setPower(.25);
                    backRight.setPower(.25);
                }


             else {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);

            }

            //Game Related (P2)
            if (Math.abs(gamepad2.left_trigger) > .1) {
                lift.setPower(gamepad2.left_trigger);
                lift2.setPower(-gamepad2.left_trigger);
            } else if (Math.abs(gamepad2.right_trigger) > .1) {
                lift.setPower(-gamepad2.right_trigger);
                lift2.setPower(gamepad2.right_trigger);
            }
                else if (gamepad2.dpad_up) {
                    lift.setPower(-.35);
                    lift2.setPower(.35);
            } else {
                lift.setPower(0);
                lift2.setPower(0);
            }

            //Two Simultaneous Carousels

            if (gamepad2.left_bumper) {
                claw1.setPower(.5);
                claw2.setPower(-.4);


            } else if (gamepad2.right_bumper) {
                claw1.setPower(-.5);
                claw2.setPower(.4);
            } else {
                claw1.setPower(0);
                claw2.setPower(0);

            }
        }
    }





