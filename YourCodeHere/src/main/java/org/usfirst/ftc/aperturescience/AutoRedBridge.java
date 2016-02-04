package org.usfirst.ftc.aperturescience;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.swerverobotics.library.ClassFactory;
import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.EulerAngles;
import org.swerverobotics.library.interfaces.IBNO055IMU;
import org.swerverobotics.library.interfaces.Position;
import org.swerverobotics.library.interfaces.Velocity;

/**
 * Created by Aperture Science on 10/14/15.
 */

//@org.swerverobotics.library.interfaces.Autonomous
public class AutoRedBridge extends SynchronousOpMode {

    private DcMotor motorR;
    private DcMotor motorL;
    private DcMotor sweeper;
    private Servo wrist;
    final static int ENCODER_CPR = 1440;
    final static double GEAR_RATIO = 1;
    final static double WHEEL_DIAMETER = 4.9;
    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    IBNO055IMU imu;
    ElapsedTime elapsed    = new ElapsedTime();
    IBNO055IMU.Parameters   parameters = new IBNO055IMU.Parameters();

    EulerAngles angles;
    Position position;

    @Override
    protected void main() throws InterruptedException {   //Combined setup and run methods.
        motorR = hardwareMap.dcMotor.get("motorR");
        motorL = hardwareMap.dcMotor.get("motorL");
        motorL.setDirection(DcMotor.Direction.REVERSE);
        motorL.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorR.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        sweeper = hardwareMap.dcMotor.get("sweeper");
        sweeper.setDirection(DcMotor.Direction.REVERSE);
        wrist = hardwareMap.servo.get("wrist");

        parameters.angleUnit      = IBNO055IMU.ANGLEUNIT.DEGREES;
        parameters.accelUnit      = IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.loggingTag     = "BNO055";
        imu = ClassFactory.createAdaFruitBNO055IMU(hardwareMap.i2cDevice.get("imu"), parameters);

        // Enable reporting of position using the naive integrator
        imu.startAccelerationIntegration(new Position(), new Velocity());

        waitForStart();

        //new IMUThread().start();

        while (opModeIsActive()) {
            //drive(0.2, 25);

            wrist.setPosition(.25);
            sweeper.setPower(-.5);
            drive(.3, 12);
            Thread.sleep(200);
            turnGyro(-45);
            Thread.sleep(200);
            drive(.4, 42);
            Thread.sleep(200);
            turnGyro(90);
            Thread.sleep(200);
            driveBack(.3, 26);
            break;

            //Thread.sleep(2000);
        }

    }

    private class IMUThread extends Thread {
        public void run() {
            while(true) {
                angles = imu.getAngularOrientation();
                position = imu.getPosition();
                //Thread.sleep(10);
            }
        }
    }

    public int inchesToRotations(double distance) {  //Same two methods.
        double rotations = distance/CIRCUMFERENCE;
        return (int)(ENCODER_CPR * rotations * GEAR_RATIO);
    }

    public void drive(double power, double distance) throws InterruptedException {
        int n = inchesToRotations(distance);
        int start = motorR.getCurrentPosition();

        motorL.setPower(power);
        motorR.setPower(power);

        while (motorR.getCurrentPosition() < (start + n)) {
            //System.out.println("=========" + motorR.getCurrentPosition() + (", " + (start+n)));
            Thread.sleep(10);
        }

        motorL.setPower(0.0);
        motorR.setPower(0.0);
    }

    public void driveBack(double power, double distance) throws InterruptedException {
        int n = inchesToRotations(distance);
        int start = motorR.getCurrentPosition();

        motorL.setPower(-power);
        motorR.setPower(-power);

        while (motorR.getCurrentPosition() > (start - n)) {
            //System.out.println("=========" + motorR.getCurrentPosition() + (", " + (start+n)));
            Thread.sleep(10);
        }

        motorL.setPower(0.0);
        motorR.setPower(0.0);
    }

    public void turn(double power, double degree){

        int n = degreesToRotations(degree);
        int start = motorR.getCurrentPosition();

        double leftPower = power;
        double rightPower = power;

        if (degree > 0.0){
            leftPower *= -1;
        }
        else if (degree < 0.0){
            rightPower *= -1;
        }

        motorR.setPower(rightPower);
        motorL.setPower(leftPower);

        if(degree < 0) while (motorR.getCurrentPosition() > (start - n)) {}
        else while (motorR.getCurrentPosition() < (start + n)) {}

        motorR.setPower(0.0);
        motorL.setPower(0.0);

    }
    public int degreesToRotations(double dgr){
        return 800;
    }

    private double getHeading() {
        double heading = imu.getAngularOrientation().heading;

        if (heading > 180){
            heading -= 360;
        }

        return heading;
    }

    double zero = 0;
    private void setZero() {
        zero = getHeading();
    }
    private double getZero() {
        return zero;
    }

    public void turnGyro(double angle) throws InterruptedException {

        double heading = getHeading();
        telemetry.addData("heading", heading);

        setZero();
        zero = getZero();
        telemetry.addData("zero", zero);


        if (angle > 0){
            angle -= 11;

            motorL.setPower(0.25);
            motorR.setPower(-0.25);

            int cnt = 0;
            while (getHeading() - getZero() < angle){
                Thread.sleep(5);
                ++cnt;
            }

            motorL.setPower(0.0);
            motorR.setPower(0.0);
            System.out.println("COUNT: ======" + cnt);

        }

        else{
            angle += 11;
            motorR.setPower(0.25);
            motorL.setPower(-0.25);
            while (getHeading() - getZero() > angle){

                Thread.sleep(5);
            }
            motorL.setPower(0.0);
            motorR.setPower(0.0);
        }


    }

}