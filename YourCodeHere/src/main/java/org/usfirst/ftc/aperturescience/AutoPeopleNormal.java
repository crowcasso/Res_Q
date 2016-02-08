package org.usfirst.ftc.aperturescience;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.ClassFactory;
import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.IBNO055IMU;
import org.swerverobotics.library.interfaces.Position;
import org.swerverobotics.library.interfaces.Velocity;

/**
 * AutoPeopleNormal (Autonomous)
 *
 * This is the main code for autonomous. Override main()
 * to add run specific instructions.
 *
 * @author FTC 5064 Aperture Science
 */
public class AutoPeopleNormal extends SynchronousOpMode {

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
    private Servo redArm;
    private TouchSensor armLimit;
    private TouchSensor turnLimit;
    private ColorSensor colorSensor;

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
    private final int ENCODER_CPR = 1120;
    private final double GEAR_RATIO = 1;
    private final double WHEEL_DIAMETER = 4.9;
    private final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    private final double RANGE = 20;
    private final double GAIN = .1;

    /* red arm constants */
    private final double RED_UPOUT = 0.45;
    private final double RED_TUCKED = 0.03;

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
        redArm = hardwareMap.servo.get("redArm");

        // touch sensors
        armLimit = hardwareMap.touchSensor.get("armSwitch");
        turnLimit = hardwareMap.touchSensor.get("ttSwitch");

        // initial servo positions
        leftShield.setPosition(LSDOWN);
        wrist.setPosition(WUP);
        rightShield.setPosition(RSDOWN);
        backShield.setPosition(BSDOWN);
        redArm.setPosition(RED_TUCKED);

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

        // it was suggested to do imu set first on i2c channel
        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        waitForStart();

        // alliance specific code moved to new classes
    }

    /* move the arm so its out of the way */
    public void setRedArm() {
        redArm.setPosition(RED_UPOUT);
    }


    /* code for moving the main arm and bucket to drop climbers */
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
            double wristPos = Range.clip(((armPos - ARMPOS_MOVE_BUCKET) / ARMPOS_MOVE_BUCKET_RANGE), 0.2, 1);
            wrist.setPosition(wristPos);
        }
        arm.setPower(0);

        // drop the bucket and shimmy
        wrist.setPosition(0.1);
        Thread.sleep(200);
        for (int i = 0; i < 20; i++) {
            wrist.setPosition(0.15);
            Thread.sleep(100);
            wrist.setPosition(0.12);
            Thread.sleep(100);
        }
        Thread.sleep(500);

        // bring the arm down
        while (armLimit.isPressed() && armPos > -50) {
            armPos = arm.getCurrentPosition();
            double wristPos = Range.clip(((armPos - ARMPOS_MOVE_BUCKET) / ARMPOS_MOVE_BUCKET_RANGE), 0.2, 1);
            wrist.setPosition(wristPos);
            arm.setPower(-Range.clip(armPos / 200.0, 0.2, 1));
            Thread.sleep(20);  // do we need this?
            System.out.println("armPos: " + armPos + ", arm power: " + -Range.clip(armPos / 200, 0.2, 1));
        }

        for (int stops = 1; stops <= 5; stops++) {
            arm.setPower(0);
            Thread.sleep(200);
            System.out.println("Stop # " + stops);
        }

        System.out.println("armPos: " + arm.getCurrentPosition() + ", armLimit: " + !armLimit.isPressed());

        // bring the bucket up
        wrist.setPosition(WUP);
    }

    /* convert inches (distance) to rotations (motor) */
    public int inchesToRotations(double distance) {
        double rotations = distance/CIRCUMFERENCE;
        return (int)(ENCODER_CPR * rotations * GEAR_RATIO);
    }

    /* drive forward some distance -- using proportional control */
    public void drive_proportinal(double power, double distance) throws InterruptedException {
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
        }
        motorL.setPower(0.0);
        motorR.setPower(0.0);
    }

    /* drive until we see red */
    public boolean driveToRed(double power, double distance) throws InterruptedException {
        int n = inchesToRotations(distance);
        int start = motorR.getCurrentPosition();

        motorL.setPower(power);
        motorR.setPower(power);

        boolean foundRed = false;

        while (!(foundRed = isRed()) && motorR.getCurrentPosition() < (start + n)) {
            Thread.sleep(10);
        }
        motorL.setPower(0.0);
        motorR.setPower(0.0);

        return foundRed;
    }

    /* drive until we see blue */
    public boolean driveToBlue(double power, double distance) throws InterruptedException {
        int n = inchesToRotations(distance);
        int start = motorR.getCurrentPosition();

        motorL.setPower(power);
        motorR.setPower(power);

        boolean foundBlue = false;

        while (!(foundBlue = isBlue()) && motorR.getCurrentPosition() < (start + n)) {
            Thread.sleep(10);
        }
        motorL.setPower(0.0);
        motorR.setPower(0.0);

        return foundBlue;
    }

    /*  drive forward without error correction */
    public void drive(double power, double distance) throws InterruptedException {
        int n = inchesToRotations(distance);
        int start = motorR.getCurrentPosition();

        motorL.setPower(power);
        motorR.setPower(power);

        while (motorR.getCurrentPosition() < (start + n)) {
            Thread.sleep(10);
        }

        motorL.setPower(0.0);
        motorR.setPower(0.0);
    }

    /* drive back without error correction */
    public void driveBack(double power, double distance) throws InterruptedException {
        int n = inchesToRotations(distance);
        int start = motorR.getCurrentPosition();

        motorL.setPower(-power);
        motorR.setPower(-power);

        while (motorR.getCurrentPosition() > (start - n)) {
            Thread.sleep(10);
        }

        motorL.setPower(0.0);
        motorR.setPower(0.0);
    }

    /* drive back until we see white */
    public boolean driveBackToWhite(double power, double distance) throws InterruptedException {
        int n = inchesToRotations(distance);
        int start = motorR.getCurrentPosition();
        double red = colorSensor.red();
        double blue = colorSensor.blue();
        double green = colorSensor.green();

        motorL.setPower(-power);
        motorR.setPower(-power);

        boolean foundWhite = false;

        while (!(foundWhite = isWhite()) && motorR.getCurrentPosition() > (start - n)) {
            Thread.sleep(10);
        }

        motorL.setPower(0.0);
        motorR.setPower(0.0);
        colorSensor.enableLed(true);
        telemetry.addData("RED",colorSensor.red());
        telemetry.addData("BLUE",colorSensor.blue());
        telemetry.addData("GREEN",colorSensor.green());
        telemetry.addData("ARGB", colorSensor.argb());
        telemetry.update();

        return foundWhite;
    }

    /* drive back until we see red or white */
    public boolean driveBackToRedOrWhite(double power, double distance) throws InterruptedException {
        int n = inchesToRotations(distance);
        int start = motorR.getCurrentPosition();
        double red = colorSensor.red();
        double blue = colorSensor.blue();
        double green = colorSensor.green();

        motorL.setPower(-power);
        motorR.setPower(-power);

        boolean foundRed = false;
        boolean foundWhite = false;

        while (!(foundRed = isRed()) && !(foundWhite = isWhite()) && motorR.getCurrentPosition() > (start - n)) {
            Thread.sleep(10);
        }

        motorL.setPower(0.0);
        motorR.setPower(0.0);
        colorSensor.enableLed(true);
        telemetry.addData("RED",colorSensor.red());
        telemetry.addData("BLUE",colorSensor.blue());
        telemetry.addData("GREEN",colorSensor.green());
        telemetry.addData("ARGB", colorSensor.argb());
        telemetry.update();

        return foundRed && foundWhite;
    }

    /* drive back until we see blue or white */
    public boolean driveBackToBlueOrWhite(double power, double distance) throws InterruptedException {
        int n = inchesToRotations(distance);
        int start = motorR.getCurrentPosition();
        double red = colorSensor.red();
        double blue = colorSensor.blue();
        double green = colorSensor.green();

        motorL.setPower(-power);
        motorR.setPower(-power);

        boolean foundBlue = false;
        boolean foundWhite = false;

        while (!(foundBlue = isRed()) && !(foundWhite = isWhite()) && motorR.getCurrentPosition() > (start - n)) {
            Thread.sleep(10);
        }

        motorL.setPower(0.0);
        motorR.setPower(0.0);
        colorSensor.enableLed(true);
        telemetry.addData("RED",colorSensor.red());
        telemetry.addData("BLUE",colorSensor.blue());
        telemetry.addData("GREEN",colorSensor.green());
        telemetry.addData("ARGB", colorSensor.argb());
        telemetry.update();

        return foundBlue && foundWhite;
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
        }
        motorL.setPower(0.0);
        motorR.setPower(0.0);
    }

    /* ask the imu for the current heading */
    public double getHeading() {
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

    /* use the imu to accurately turn */
    public void turnGyro(double angle) throws InterruptedException {

        setZero();

        if (angle > 0) {    // right turn
            angle -= 6;     // lag

            motorL.setPower(-0.10);
            motorR.setPower(0.10);

            while (getHeading() - zero < angle){
                Thread.sleep(5);
            }
            motorL.setPower(0.0);
            motorR.setPower(0.0);

        } else {            // left turn
            angle += 6;     // lag

            motorL.setPower(0.10);
            motorR.setPower(-0.10);

            while (getHeading() - zero > angle){
                Thread.sleep(5);
            }
            motorL.setPower(0.0);
            motorR.setPower(0.0);

        }

        /* Debugging */
        telemetry.addData("turnGyro", "angle = " + angle);
        telemetry.addData("zero", "zero = " + zero);
        telemetry.addData("heading", getHeading());
        telemetry.update();

    }

    /* is the color sensor seeing white? */
    public boolean isWhite() {
        if (colorSensor.red() > 6.0 && colorSensor.blue() > 6.0 && colorSensor.green() > 6.0){
            return true;
        }
        return false;
    }

    /* is the color sensor seeing red? */
    public boolean isRed() {
        if (colorSensor.red() > 4.0 && colorSensor.blue() < 2.0 && colorSensor.green() < 2.0){
            return true;
        }
        return false;
    }

    /* is the color sensor seeing blue? */
    public boolean isBlue() {
        if (colorSensor.red() < 1.0 && colorSensor.blue() > 3.0 && colorSensor.green() < 1.0) {
            return true;
        }
        return false;
    }
}