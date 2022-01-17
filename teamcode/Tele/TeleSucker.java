//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
////import org.checkerframework.checker.initialization.qual.Initialized;
//
////@Disabled
//@TeleOp
//public class TeleOpSucker extends OpMode {
//
//    DcMotor leftMotor  ;
//    DcMotor rightMotor  ;
//    DcMotor duckMotor  ;
//    DcMotor armLeft;
//    DcMotor armRight;
//
//    CRServo contServoRight;
//    CRServo contServoLeft;
//
//    @Override
//    public void init() {
//        String initMessage = "Initialized";
//        telemetry.addData("Status", initMessage);  //Status= Initialized
//
//        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
//        armRight = hardwareMap.get(DcMotor.class, "armRight");
//        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
//        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
//        duckMotor = hardwareMap.get(DcMotor.class, "duckMotor");
//
//
//        contServoRight = hardwareMap.crservo.get("contServoRight");
//        contServoLeft = hardwareMap.crservo.get("contServoLeft");
//
//        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        duckMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        armLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//
//    }
//
//    @Override
//    public void loop() {
//
//        double larmpower = -gamepad2.left_trigger;
//        double rarmpower = +gamepad2.right_trigger;
//
//
//        double leftPower = -gamepad1.left_stick_y;
//        double rightPower = +gamepad1.right_stick_y;
//        double duckPower = -gamepad1.right_trigger;
//
//        double contPowerLeft, contPowerRight;
//
//        double avePower =(leftPower + rightPower) / 2;
//        boolean isFast = avePower > 0.75;
//
//        if (gamepad2.right_bumper){
//
//
//            contPowerRight = -1.0;
//            contPowerLeft = 1.0;
//        }
//        else if(gamepad2.left_bumper){
//
//            contPowerLeft = -1.0;
//            contPowerRight = 1.0;
//        }
//        else {
//            contPowerRight = 0.0;
//            contPowerLeft = 0.0;
//        }
//
//
//        if(isFast) {
//            telemetry.addData("Seed Of Robot", avePower);
//        }
//        else {
//            telemetry.addData("Robot Is Fast", false);
//        }
//
//
//
//        armLeft.setPower(larmpower);
//        armRight.setPower(rarmpower);
//        leftMotor.setPower(leftPower);
//        rightMotor.setPower(rightPower);
//        duckMotor.setPower(duckPower);
//
//        contServoRight.setPower(contPowerRight);
//        contServoLeft.setPower(contPowerLeft);
//    }
//
//
//
//}