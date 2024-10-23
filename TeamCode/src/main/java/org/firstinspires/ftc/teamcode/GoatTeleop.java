class GoatsTeleop
{
    public static void main(String[] args) {
        package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

        @TeleOp(name = "Drive10 (Blocks to Java)")
        public class Drive10 extends LinearOpMode {

            private Servo servo0;
            private Servo servo1;
            private Servo servo2;
            private Servo servo3;
            private DcMotor motor0;
            private DcMotor motor1;
            private DcMotor motor2;
            private DcMotor motor3;

            int Motor0Forward;
            double MotorPower;
            int Motor1Forward;
            int Motor2Forward;
            int Motor3Forward;

            /**
             * This function is executed when this OpMode is selected from the Driver Station.
             */
            @Override
            public void runOpMode() {
                float RightStickX;
                float LeftStickX;
                float LeftStickY;
                float GP2RightStickY;
                int Motor4Forward;
                int Motor5Forward;

                servo0 = hardwareMap.get(Servo.class, "servo 0");
                servo1 = hardwareMap.get(Servo.class, "servo 1");
                servo2 = hardwareMap.get(Servo.class, "servo 2");
                servo3 = hardwareMap.get(Servo.class, "servo 3");
                motor0 = hardwareMap.get(DcMotor.class, "motor 0");
                motor1 = hardwareMap.get(DcMotor.class, "motor 1");
                motor2 = hardwareMap.get(DcMotor.class, "motor 2");
                motor3 = hardwareMap.get(DcMotor.class, "motor 3");

                waitForStart();
                while (!gamepad1.start) {
                }
                servo0.scaleRange(0, 1);
                servo1.scaleRange(0, 1);
                servo2.scaleRange(0, 1);
                servo3.scaleRange(0, 1);
                while (opModeIsActive()) {
                    GP2RightStickY = gamepad2.right_stick_y;
                    RightStickX = gamepad1.right_stick_x;
                    LeftStickX = gamepad1.left_stick_x;
                    LeftStickY = gamepad1.left_stick_y;
                    if (gamepad1.left_bumper) {
                        servo0.setPosition(1);
                    } else {
                        servo0.setPosition(0.3);
                    }
                    if (gamepad1.right_bumper) {
                        servo1.setPosition(0.3);
                    } else {
                        servo1.setPosition(0.74);
                    }
                    if (gamepad1.x) {
                        Motor4Forward = 1;
                        Motor5Forward = 0;
                        MotorPower = 1;
                        sleep(4000);
                    }
                    if (LeftStickY == -1) {
                        while (LeftStickY == -1) {
                            RightStickX = gamepad1.right_stick_x;
                            LeftStickX = gamepad1.left_stick_x;
                            LeftStickY = gamepad1.left_stick_y;
                            GP2RightStickY = gamepad2.right_stick_y;
                            Motor0Forward = 1;
                            Motor1Forward = 0;
                            Motor2Forward = 1;
                            Motor3Forward = 0;
                            MotorPower = 0.8;
                            Run_motors();
                        }
                    } else {
                    }
                    if (LeftStickY == 1) {
                        while (LeftStickY == 1) {
                            RightStickX = gamepad1.right_stick_x;
                            LeftStickX = gamepad1.left_stick_x;
                            LeftStickY = gamepad1.left_stick_y;
                            GP2RightStickY = gamepad2.right_stick_y;
                            Motor0Forward = 0;
                            Motor1Forward = 1;
                            Motor2Forward = 0;
                            Motor3Forward = 1;
                            MotorPower = 0.6;
                            Run_motors();
                        }
                    } else {
                    }
                    if (LeftStickX == 1) {
                        while (LeftStickX == 1) {
                            RightStickX = gamepad1.right_stick_x;
                            LeftStickX = gamepad1.left_stick_x;
                            LeftStickY = gamepad1.left_stick_y;
                            GP2RightStickY = gamepad2.right_stick_y;
                            Motor0Forward = 0;
                            Motor1Forward = 0;
                            Motor2Forward = 1;
                            Motor3Forward = 1;
                            MotorPower = 0.7;
                            Run_motors();
                        }
                    } else {
                    }
                    if (LeftStickX == -1) {
                        while (LeftStickX == -1) {
                            RightStickX = gamepad1.right_stick_x;
                            LeftStickX = gamepad1.left_stick_x;
                            LeftStickY = gamepad1.left_stick_y;
                            GP2RightStickY = gamepad2.right_stick_y;
                            Motor0Forward = 1;
                            Motor1Forward = 1;
                            Motor2Forward = 0;
                            Motor3Forward = 0;
                            MotorPower = 0.7;
                            Run_motors();
                        }
                    } else {
                    }
                    if (RightStickX == 1) {
                        while (RightStickX == 1) {
                            RightStickX = gamepad1.right_stick_x;
                            LeftStickX = gamepad1.left_stick_x;
                            LeftStickY = gamepad1.left_stick_y;
                            GP2RightStickY = gamepad2.right_stick_y;
                            Motor0Forward = 0;
                            Motor1Forward = 0;
                            Motor2Forward = 0;
                            Motor3Forward = 0;
                            MotorPower = 0.5;
                            Run_motors();
                        }
                    } else {
                    }
                    if (RightStickX == -1) {
                        while (RightStickX == -1) {
                            RightStickX = gamepad1.right_stick_x;
                            LeftStickX = gamepad1.left_stick_x;
                            LeftStickY = gamepad1.left_stick_y;
                            GP2RightStickY = gamepad2.right_stick_y;
                            Motor0Forward = 1;
                            Motor1Forward = 1;
                            Motor2Forward = 1;
                            Motor3Forward = 1;
                            MotorPower = 0.5;
                            Run_motors();
                        }
                    } else {
                    }
                }
            }

            /**
             * Describe this function...
             */
            private void Run_motors() {
                RobotLog.ii("DbgLog", "ogala bogala");
                if (Motor0Forward == 1) {
                    motor0.setDirection(DcMotor.Direction.REVERSE);
                } else {
                    motor0.setDirection(DcMotor.Direction.FORWARD);
                }
                if (Motor1Forward == 1) {
                    motor1.setDirection(DcMotor.Direction.REVERSE);
                } else {
                    motor1.setDirection(DcMotor.Direction.FORWARD);
                }
                if (Motor2Forward == 1) {
                    motor2.setDirection(DcMotor.Direction.REVERSE);
                } else {
                    motor2.setDirection(DcMotor.Direction.FORWARD);
                }
                if (Motor3Forward == 1) {
                    motor3.setDirection(DcMotor.Direction.REVERSE);
                } else {
                    motor3.setDirection(DcMotor.Direction.FORWARD);
                }
                motor0.setPower(MotorPower);
                motor1.setPower(MotorPower);
                motor2.setPower(MotorPower);
                motor3.setPower(MotorPower);
            }
        }
    }
}