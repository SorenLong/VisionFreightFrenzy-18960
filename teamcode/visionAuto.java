
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

// This is a modded version of the skystone EasyOpenCv example

@Autonomous
public class visionAuto extends LinearOpMode
{

     //Declaration of all hardware. Here I am using an arm intake with passive continuous servos,
    //change up to whatever suits your team(s) needs.

    OpenCvWebcam webcam;
    DuckDetection pipeline;
    BNO055IMU imu;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor armLeft;
    DcMotor armRight;
    DcMotor duckMotor;
    private static final double TICKS_PER_INCH = 651.897 * (25.0/20.0);
    CRServo contServoRight;
    CRServo contServoLeft;


    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new DuckDetection();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                //sets resolution, the supported resolutions are 320x240, 640x480, 1280x720, 1920x1080
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

                //called if camera wont open

            }
        });

        contServoRight = hardwareMap.crservo.get("contServoRight");
        contServoLeft = hardwareMap.crservo.get("contServoLeft");

        leftMotor  = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        armRight = hardwareMap.get(DcMotor.class, "armRight");
        duckMotor = hardwareMap.get(DcMotor.class, "duckMotor");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        duckMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);


        waitForStart();

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while(opModeIsActive() && elapsedTime.time(TimeUnit.SECONDS) < 3) {
            telemetry.addData("analysis", pipeline.getAnalysis());
            telemetry.update();
        }

        if(pipeline.getAnalysis() == DuckDetection.DuckPosition.LEFT){



        }

        else if(pipeline.getAnalysis() == DuckDetection.DuckPosition.CENTER) {


            duckMotor.setPower(0.4);
            sleep(1000);
        }

        else if(pipeline.getAnalysis() == DuckDetection.DuckPosition.RIGHT) {

            driveStraightAt35Percent(850);
            armLift(2400);
            intake(3000);

            armLeft.setPower(-0.60);
            armRight.setPower(-0.60);
            sleep(2000);

            contServoRight.setPower(0);
            contServoLeft.setPower(0);
            sleep(800);

            turn(-77);

            leftMotor.setPower(-.5);
            rightMotor.setPower(-.5);
            sleep(3000);

            leftMotor.setPower(0);
            rightMotor.setPower(0);
            sleep(2500);

            duckMotor.setPower(-0.5);
            sleep(2500);
            duckMotor.setPower(0);
            sleep(2700);

            rightMotor.setPower(0.4);
            leftMotor.setPower(0.4);
            sleep(1000);

            rightMotor.setPower(0);
            leftMotor.setPower(0);
            sleep(1000);


            leftMotor.setPower(1);
            rightMotor.setPower(1);
            sleep(2000);

        }

        else {
            telemetry.addData("So, i cant see where it is", pipeline.getAnalysis());
            telemetry.update();
        }

    }


    public void intake(double clawin){

        contServoLeft.setPower(1);
        contServoRight.setPower(-1);

    }

    public void armLift(double armLRDistance){

        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setDirection(DcMotorSimple.Direction.REVERSE);
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double alDistance = armLeft.getCurrentPosition();
        double arDistance = armRight.getCurrentPosition();

        double sign = Math.signum(armLRDistance);

        while(opModeIsActive() && ((alDistance + arDistance) / 2) < armLRDistance) {
            double diff = alDistance - arDistance;

            double leftarmPower = 0.50;
            double rightarmPower = 0.50;

            if (diff > 0) leftarmPower = 0.25;
            else rightarmPower = 0.5;

            armLeft.setPower(leftarmPower * sign);
            armRight.setPower(rightarmPower * sign);

            alDistance = armLeft.getCurrentPosition();
            arDistance = armRight.getCurrentPosition();

            telemetry.addData("arm left distance", alDistance);
            telemetry.addData("arm right distance", arDistance);
            telemetry.update();
        }

        armLeft.setPower(0);
        armRight.setPower(0);

    }

    public void driveStraight(double distance) {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double lDistance = -leftMotor.getCurrentPosition();
        double rDistance = -rightMotor.getCurrentPosition();

        double sign = Math.signum(distance);

        while(opModeIsActive() && Math.abs((lDistance + rDistance) / 2) < Math.abs(distance)) {
            double diff = Math.abs(lDistance) - Math.abs(rDistance);

            double leftPower = 0.75;
            double rightPower = 0.75;

            if (diff > 0) leftPower = 0.5;
            else rightPower = 0.5;

            leftMotor.setPower(leftPower * sign);
            rightMotor.setPower(rightPower * sign);

            lDistance = -leftMotor.getCurrentPosition();
            rDistance = -rightMotor.getCurrentPosition();


            telemetry.addData("left distance", lDistance);
            telemetry.addData("right distance", rDistance);
            telemetry.addData("angle", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void driveStraightAt35Percent(double distance) {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double lDistance = -leftMotor.getCurrentPosition();
        double rDistance = -rightMotor.getCurrentPosition();

        double sign = Math.signum(distance);

        while(opModeIsActive() && Math.abs((lDistance + rDistance) / 2) < Math.abs(distance)) {
            double diff = Math.abs(lDistance) - Math.abs(rDistance);

            double leftPower = 0.35;
            double rightPower = 0.35;

            if (diff > 0) leftPower = 0.35;
            else rightPower = 0.35;

            leftMotor.setPower(leftPower * sign);
            rightMotor.setPower(rightPower * sign);

            lDistance = -leftMotor.getCurrentPosition();
            rDistance = -rightMotor.getCurrentPosition();


            telemetry.addData("left distance", lDistance);
            telemetry.addData("right distance", rDistance);
            telemetry.addData("angle", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void turn(double desiredAngle) {
        double angle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        double sign = Math.signum(desiredAngle - angle);
//        double kP = (desiredAngle-angle) * 0.02;
        while(opModeIsActive() && Math.signum(desiredAngle - angle) == sign) {
//            kP = (desiredAngle-angle) * 0.01675;
//            kP = (desiredAngle-angle) * 0.019;
            leftMotor.setPower(-0.4 /* * kP */* sign);
            rightMotor.setPower(0.4 /* * kP */* sign);
            angle = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            telemetry.addData("angle", angle);
            telemetry.addData("desiredAngle", desiredAngle);
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public static class DuckDetection extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum DuckPosition
        {
            LEFT,
            CENTER,
            RIGHT
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(180,177);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(258,215);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(395,259);
        static final int REGION_WIDTH = 20;
        static final int REGION_HEIGHT = 20;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;

        private volatile DuckPosition position = DuckPosition.LEFT;

        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame)
        {

            inputToCb(firstFrame);


            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {

            inputToCb(input);


            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    BLUE,
                    2);

            Imgproc.rectangle(
                    input,
                    region2_pointA,
                    region2_pointB,
                    BLUE,
                    2);


            Imgproc.rectangle(
                    input,
                    region3_pointA,
                    region3_pointB,
                    BLUE,
                    2);



            int minOneTwo = Math.min(avg1, avg2);
            int min = Math.min(minOneTwo, avg3);


            if(min == avg1)
            {
                position = DuckPosition.LEFT;


                Imgproc.rectangle(
                        input,
                        region1_pointA,
                        region1_pointB,
                        GREEN,
                        -1);
            }
            else if(min == avg2)
            {
                position = DuckPosition.CENTER;


                Imgproc.rectangle(
                        input,
                        region2_pointA,
                        region2_pointB,
                        GREEN,
                        -1);
            }
            else if(min == avg3)
            {
                position = DuckPosition.RIGHT;


                Imgproc.rectangle(
                        input,
                        region3_pointA,
                        region3_pointB,
                        GREEN,
                        -1);
            }

            return input;
        }


        public DuckPosition getAnalysis()
        {
            return position;
        }
    }
}