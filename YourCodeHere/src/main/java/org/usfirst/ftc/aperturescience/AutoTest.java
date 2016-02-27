package org.usfirst.ftc.aperturescience;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.ClassFactory;
import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Disabled;
import org.swerverobotics.library.interfaces.EulerAngles;
import org.swerverobotics.library.interfaces.IBNO055IMU;
import org.swerverobotics.library.interfaces.Position;
import org.swerverobotics.library.interfaces.Velocity;

/**
 * Nasic 7.2 - JimmyCentral (Autonomous)
 *
 * @author FTC 5064 Aperture Science
 */
@org.swerverobotics.library.interfaces.Autonomous
@Disabled
public class AutoTest extends SynchronousOpMode {

    // Hardware
    private DcMotor arm;
    private DcMotor turntable;
    private DcMotor motorR;
    private DcMotor motorL;
    private DcMotor sweeper;
    private Servo leftShield;
    private Servo rightShield;
    private Servo backShield;
    private Servo wrist;
    private TouchSensor armLimit;
    private TouchSensor turnLimit;

    /* bucket constants */
    private final double WDOWN = 0.53;
    private final double WUP = 0.0;

    /* shield constants */
    private final double LSDOWN = 0.01;
    private final double LSUP = 0.57;
    private final double RSDOWN = 0.6;
    private final double RSUP = 0.04;
    private final double BSDOWN = 0.81;
    private final double BSUP = 0.35;

    /* motor constants */
    private final int ENCODER_CPR = 1440;
    private final double GEAR_RATIO = 1;
    private final double WHEEL_DIAMETER = 4.9;
    private final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    private final double RANGE = 20;
    private final double GAIN = .1;

    /* gryo/magnometer */
    private IBNO055IMU imu;
    private IBNO055IMU.Parameters parameters = new IBNO055IMU.Parameters();

    @Override
    protected void main() throws InterruptedException {

        // motors
        motorR = hardwareMap.dcMotor.get("motorR");
        motorL = hardwareMap.dcMotor.get("motorL");
        arm = hardwareMap.dcMotor.get("arm");
        turntable = hardwareMap.dcMotor.get("turntable");
        sweeper = hardwareMap.dcMotor.get("sweeper");

        // servos
        leftShield = hardwareMap.servo.get("leftShield");
        rightShield = hardwareMap.servo.get("rightShield");
        backShield = hardwareMap.servo.get("backShield");
        wrist = hardwareMap.servo.get("wrist");

        // touch sensors
        armLimit = hardwareMap.touchSensor.get("armSwitch");
        turnLimit = hardwareMap.touchSensor.get("ttSwitch");

        // initial servo positions
        leftShield.setPosition(LSDOWN);
        wrist.setPosition(WUP);
        rightShield.setPosition(RSDOWN);
        backShield.setPosition(BSDOWN);

        // run certain motors using encoders
        motorL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorL.setDirection(DcMotor.Direction.REVERSE);
        motorR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        arm.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        arm.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        turntable.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        turntable.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        sweeper.setDirection(DcMotor.Direction.REVERSE);

        // setup the imu
        parameters.angleUnit = IBNO055IMU.ANGLEUNIT.DEGREES;
        parameters.accelUnit = IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.loggingTag = "BNO055";
        imu = ClassFactory.createAdaFruitBNO055IMU(hardwareMap.i2cDevice.get("gyro"), parameters);

        // enable reporting of position using the naive integrator
        imu.startAccelerationIntegration(new Position(), new Velocity());

        // make sure the IMU is working properly
        byte value = imu.getSystemStatus();
        telemetry.addData("imu", "system status = " + value);
        if (value == 0x5) {
            telemetry.addData("imu", "system is ready");
        } else {
            telemetry.addData("imu", "RESTART NEEDED");
        }
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            /*
            // drive back 35 inches
            driveBack(.3, 35);
            Thread.sleep(500);
            */

            // turn left 45 degrees
            turnGyro(-45);
            Thread.sleep(200);

            /*
            // drive back 31 inches
            driveBack(.4, 31);
            Thread.sleep(500);

            // turn left 45 degrees
            turnGyro(-45);
            Thread.sleep(200);

            // drive back 15 inches
            driveBack(.3, 15);
            Thread.sleep(200);

            // drop the climbers
            autoArm();
            */

            // error checking
            Thread.sleep(1000);
            telemetry.addData("Final Heading", getHeading());
            telemetry.update();

            // we're done!
            break;
        }

    }

    private final double AUTO_ARM_MAX = 4400;
    private final double NORMAL_ARM_SPEED = 0.4;
    private final double ARMPOS_MOVE_BUCKET = 1500;
    private final double ARMPOS_MOVE_BUCKET_RANGE = 4000;

    public void autoArm() throws InterruptedException {
        // bring the arm up
        arm.setPower(NORMAL_ARM_SPEED);
        double armPos = arm.getCurrentPosition();
        while (arm.getCurrentPosition() < AUTO_ARM_MAX) {
            armPos = arm.getCurrentPosition();
            double wristPos = Range.clip(((armPos - ARMPOS_MOVE_BUCKET) / ARMPOS_MOVE_BUCKET_RANGE), 0, 1);
            wrist.setPosition(wristPos);
        }
        arm.setPower(0);

        // drop the bucket and shimmy
        wrist.setPosition(0.1);
        Thread.sleep(200);
        for (int i = 0; i < 20; i++) {
            wrist.setPosition(0.13);
            Thread.sleep(100);
            wrist.setPosition(0.1);
            Thread.sleep(100);
        }
        wrist.setPosition(WDOWN);
        Thread.sleep(500);
        wrist.setPosition(0.1);
        Thread.sleep(1000);
        wrist.setPosition(WDOWN);
        Thread.sleep(500);

        // bring the arm down
        while (armLimit.isPressed() && armPos > -100) {
            armPos = arm.getCurrentPosition();
            double wristPos = Range.clip(((armPos - ARMPOS_MOVE_BUCKET) / ARMPOS_MOVE_BUCKET_RANGE), 0, 1);
            wrist.setPosition(wristPos);
            arm.setPower(-Range.clip(armPos / 200, 0.2, 1));
        }
        arm.setPower(0);

        // bring the bucket up
        wrist.setPosition(WUP);
    }

    /* convert inches (distance) to rotations (motor) */
    public int inchesToRotations(double distance) {
        double rotations = distance/CIRCUMFERENCE;
        return (int)(ENCODER_CPR * rotations * GEAR_RATIO);
    }

    /* drive forward some distance -- using proportional control */
    public void drive(double power, double distance) throws InterruptedException {
        int n = inchesToRotations(distance);
        int start = motorR.getCurrentPosition();
        double pointing = getHeading();

        motorL.setPower(power);
        motorR.setPower(power);

        while (motorR.getCurrentPosition() < (start + n)) {
            Thread.sleep(10);
            double error = pointing - getHeading();
            double fix = (error / RANGE) * GAIN;
            motorL.setPower(power + fix);
            motorR.setPower(power - fix);
            /*
            telemetry.addData("fix ", fix);
            telemetry.addData("error ",error);
            telemetry.update();
            */
        }
        motorL.setPower(0.0);
        motorR.setPower(0.0);
    }

    public void old_drive(double power, double distance) throws InterruptedException {
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

    /* drive backwards -- using proportional control */
    public void temp_driveBack(double power, double distance) throws InterruptedException {
        int n = inchesToRotations(distance);
        int start = motorR.getCurrentPosition();
        double pointing = getHeading();

        motorL.setPower(-power);
        motorR.setPower(-power);

        while (motorR.getCurrentPosition() > (start - n)) {
            Thread.sleep(10);
            double error = pointing - getHeading();
            double fix = (error / RANGE) * GAIN;
            motorL.setPower(-(power + fix));
            motorR.setPower(-(power - fix));
            /*
            telemetry.addData("fix ", fix);
            telemetry.addData("error ",error);
            telemetry.update();
            */
        }
        motorL.setPower(0.0);
        motorR.setPower(0.0);
    }

    /* ask the imu for the current heading */
    private double getHeading() {
        double heading = imu.getAngularOrientation().heading;

        if (heading > 180){
            heading -= 360;
        }

        /* debugging
        telemetry.addData("heading", heading);
        telemetry.update();
        */

        return heading;
    }

    double zero = 0;

    /* use the current heading as our new 0 angle */
    private void setZero() {
        zero = getHeading();
    }

    public void turnGyro(double angle) throws InterruptedException {

        setZero();

        if (angle > 0) {    // right turn
            angle -= 10;     // lag

            motorL.setPower(-0.25);
            motorR.setPower(0.25);

            while (getHeading() - zero < angle){
                Thread.sleep(5);
            }
            motorL.setPower(0.0);
            motorR.setPower(0.0);

        } else {            // left turn
            angle += 10;     // lag

            motorL.setPower(0.25);
            motorR.setPower(-0.25);

            int cnt = 0;
            double heading;
            while ((heading = getHeading()) - zero > angle){
                telemetry.addData("left turn " + cnt, heading + ", " + zero + ", " + angle
                        + ", " + imu.getSystemStatus() + ", " + imu.getSystemError());
                telemetry.update();
                cnt++;
                Thread.sleep(10);
            }
            motorL.setPower(0.0);
            motorR.setPower(0.0);

        }

        telemetry.addData("turnGyro", "angle = " + angle);
        telemetry.addData("zero", "zero = " + zero);
        telemetry.addData("heading", getHeading());
        telemetry.update();
    }
}