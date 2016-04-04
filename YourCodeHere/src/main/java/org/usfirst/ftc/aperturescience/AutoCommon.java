package org.usfirst.ftc.aperturescience;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.ClassFactory;
import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.IBNO055IMU;
import org.swerverobotics.library.interfaces.Position;
import org.swerverobotics.library.interfaces.Velocity;

/**
 * AutoCommon (Autonomous)
 *
 * This is the main code for autonomous. Override main()
 * to add run specific instructions.
 *
 * @author FTC 5064 Aperture Science
 */
public class AutoCommon extends SynchronousOpMode {

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
    private Servo leftTape;
    private Servo rightTape;
    private Servo redArm;
    private Servo blueArm;
    private TouchSensor armLimit;
    private TouchSensor turnLimit;
    private ColorSensor colorSensor;
    private UltrasonicSensor ultra;
    private GyroSensor gyro;

    /* bucket constants */
    private final double WDOWN = 0.47;
    private final double WUP = 0.01;

    /* shield constants */
    private final double LSDOWN = 0.01;
    private final double LSUP = 0.57;
    private final double RSDOWN = 0.6;
    private final double RSUP = 0.04;
    private final double BSDOWN = 0.42;
    private final double BSUP = 0.84;

    /* motor constants */
    private final int ENCODER_CPR = 1120;
    private final double GEAR_RATIO = 1;
    private final double WHEEL_DIAMETER = 4.9;
    private final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    private final double RANGE = 20;
    private final double GAIN = .1;

    /* red arm constants */
    private final double RED_UP = 1.0;
    private final double RED_DOWN = 0.11;

    /* blue arm constants */
    private final double BLUE_UP = 0.0;
    private final double BLUE_DOWN = 0.8;

    /* tape constants */
    private final double LTAPE_UP = 0.85;
    private final double RTAPE_UP = 0.15;

    /* gryo/magnometer */
    /*
    private IBNO055IMU imu;
    private IBNO055IMU.Parameters parameters = new IBNO055IMU.Parameters();*/

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
        blueArm = hardwareMap.servo.get("blueArm");
        leftTape = hardwareMap.servo.get("leftTape");
        rightTape = hardwareMap.servo.get("rightTape");

        // touch sensors
        armLimit = hardwareMap.touchSensor.get("armSwitch");
        turnLimit = hardwareMap.touchSensor.get("ttSwitch");

        // Range finder
        ultra = hardwareMap.ultrasonicSensor.get("ultra");

        //Gyroscope
        gyro = hardwareMap.gyroSensor.get("gyro");

        // initial servo positions
        leftShield.setPosition(LSDOWN);
        wrist.setPosition(WUP);
        rightShield.setPosition(RSDOWN);
        backShield.setPosition(BSDOWN);
        redArm.setPosition(RED_UP);
        blueArm.setPosition(BLUE_UP);
        leftTape.setPosition(LTAPE_UP);
        rightTape.setPosition(RTAPE_UP);

        // run certain motors using encoders
        motorL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorL.setDirection(DcMotor.Direction.REVERSE);
        motorR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        arm.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        arm.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        turntable.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        turntable.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        sweeper.setDirection(DcMotor.Direction.REVERSE);

        /*telemetry.addData("imu", "starting up the imu ...");
        telemetry.update();

        // setup the imu
        parameters.angleUnit = IBNO055IMU.ANGLEUNIT.DEGREES;
        parameters.accelUnit = IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.loggingTag = "BNO055";
        imu = ClassFactory.createAdaFruitBNO055IMU(hardwareMap.i2cDevice.get("imu"), parameters);

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
        telemetry.update();*/

        // it was suggested to do imu set first on i2c channel
        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        telemetry.addData("Gyro:","Ready to go!");
        telemetry.update();

        waitForStart();

        // alliance specific code moved to new classes
    }

    /* pull left tape out to clear the arm */
    public void setTapes() {
        leftTape.setPosition(LTAPE_UP - .1);
        rightTape.setPosition(RTAPE_UP);
    }


    /* code for moving the main arm and bucket to drop climbers */
    private final double AUTO_ARM_MAX = 6975;
    private final double AUTO_ARM_VERT = 4500;
    private final double NORMAL_ARM_SPEED = 0.75;
    private final double ARMPOS_MOVE_BUCKET = 1500;

    private final double ARMPOS_MOVE_BUCKET_RANGE = 4000;
    private boolean doBucket = true;

    private double armPos2 = 0.0;
    public void autoArmUp() throws InterruptedException {
        // bring the arm up
        arm.setPower(NORMAL_ARM_SPEED);
        armPos2 = arm.getCurrentPosition();
        while (arm.getCurrentPosition() < AUTO_ARM_MAX) {
            armPos2 = arm.getCurrentPosition();
            double wristPos = Range.clip(((armPos2 - ARMPOS_MOVE_BUCKET) / ARMPOS_MOVE_BUCKET_RANGE), 0.2, 1);//--------
            if(arm.getCurrentPosition() > (AUTO_ARM_MAX - 2000)) {
            doBucket = false;
            wrist.setPosition(0.7);
             } else {
             wrist.setPosition(wristPos);
             }
        }
        arm.setPower(0);

        Thread.sleep(500);

        // drop the bucket
        doBucket = true;
    }

    public void autoArmDown() throws InterruptedException {
        // bring the arm down
        int loopCount = 0;
        while (armLimit.isPressed() && armPos2 >= 0) {
            armPos2 = arm.getCurrentPosition();
            double wristPos = Range.clip(((armPos2 - ARMPOS_MOVE_BUCKET) / ARMPOS_MOVE_BUCKET_RANGE), 0.2, 1);
            if (armPos2 < ARMPOS_MOVE_BUCKET) {
                wristPos = WUP;
            }
            wrist.setPosition(wristPos);
            double armPower = -Range.clip(armPos2 / 200.0, 0.1, 0.75); //FIXME
            arm.setPower(armPower);
            Thread.sleep(20);  // do we need this?
            loopCount++;
            System.out.println("armPos: " + armPos2 + ", arm power: " + armPower);
        }

        System.out.println("loopCount: " + loopCount);
        System.out.println("arm is down, arm.isBusy: " + arm.isBusy());

        arm.setPower(0);

        System.out.println("armPos: " + arm.getCurrentPosition()
                + ", armLimit: " + !armLimit.isPressed()
                + ", arm.isBusy: " + arm.isBusy());

        /* extra stops -- just in case */
        for (int stops = 1; stops <= 5; stops++) {
            arm.setPower(0);
            Thread.sleep(100);
            System.out.println("Stop # " + stops);
        }

        // bring the bucket up
        wrist.setPosition(WUP);
    }

    public void autoArmVert() throws InterruptedException {
        // bring the arm vertical
        int loopCount = 0;
        while (armLimit.isPressed() && armPos2 >= AUTO_ARM_VERT) {
            armPos2 = arm.getCurrentPosition();
            double wristPos = Range.clip(((armPos2 - ARMPOS_MOVE_BUCKET) / ARMPOS_MOVE_BUCKET_RANGE), 0.2, 1);
            if (armPos2 < ARMPOS_MOVE_BUCKET) {
                wristPos = WUP;
            }
            wrist.setPosition(wristPos);
            arm.setPower(-Range.clip(armPos2 / 200.0, 0.05, 0.75)); //FIXME
            Thread.sleep(20);  // do we need this?
            loopCount++;
            System.out.println("armPos: " + armPos2 + ", arm power: " + -Range.clip(armPos2 / 200, 0.05, 1));
        }

        System.out.println("loopCount: " + loopCount);
        System.out.println("arm is down, arm.isBusy: " + arm.isBusy());

        arm.setPower(0);

        System.out.println("armPos: " + arm.getCurrentPosition()
                + ", armLimit: " + !armLimit.isPressed()
                + ", arm.isBusy: " + arm.isBusy());

        /* extra stops -- just in case */
        for (int stops = 1; stops <= 5; stops++) {
            arm.setPower(0);
            Thread.sleep(100);
            System.out.println("Stop # " + stops);
        }

        // bring the bucket up
        wrist.setPosition(WUP);
    }

    public void sweeperOn() {
        sweeper.setPower(-1.0);
    }

    public void sweeperOff() {
        sweeper.setPower(0.0);
    }

    /* convert inches (distance) to rotations (motor) */
    public int inchesToRotations(double distance) {
        double rotations = distance/CIRCUMFERENCE;
        return (int)(ENCODER_CPR * rotations * GEAR_RATIO);
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

    /* drive back using ultrasonic */
    public double driveToDistance(double power, double distance, double toWall) throws InterruptedException {
        int n = inchesToRotations(distance);
        int start = motorR.getCurrentPosition();

        for (int i = 0; i < 5; i++) {
            getDistance();
            Thread.sleep(10);
        }

        double diff = 0.0;
        while (Math.abs(diff = (getDistance() - toWall)) > 4) {
            if (diff < 0) {
                motorL.setPower(power);
                motorR.setPower(power);
            } else {
                motorL.setPower(-power);
                motorR.setPower(-power);
            }
        }

        motorL.setPower(0.0);
        motorR.setPower(0.0);

        return currentDistance;
    }

    /* drive back until we see white */
    public boolean driveBackToWhite(double power, double distance) throws InterruptedException {
        int n = inchesToRotations(distance);
        int start = motorR.getCurrentPosition();
        double red = colorSensor.red();
        double blue = colorSensor.blue();
        double green = colorSensor.green();
        double motorCount = 0;
        double motorPower = -power;

        /*if(start > n * 0.8) {
            motorPower = -power / 2;
            motorL.setPower(motorPower);
            motorR.setPower(motorPower);
        } else {
            motorPower = -power;
            motorL.setPower(motorPower);
            motorR.setPower(motorPower);
        }*/

        motorL.setPower(motorPower);
        motorR.setPower(motorPower);

        boolean foundWhite = false;

        while (!(foundWhite = isWhite()) && (motorR.getCurrentPosition() > (start - n))) {
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



    /* ask the gyro for the current heading */
    public double getHeading() {
        //double heading = imu.getAngularOrientation().heading;
        double heading = gyro.getHeading();

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
            angle -= 13;     // lag

            motorL.setPower(-0.30);
            motorR.setPower(0.30);

            while (getHeading() - zero < angle){
                Thread.sleep(5);
            }
            motorL.setPower(0.0);
            motorR.setPower(0.0);

        } else {            // left turn
            angle += 13;     // lag

            motorL.setPower(0.30);
            motorR.setPower(-0.30);

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
    public void turnGyroSlow(double angle) throws InterruptedException {

        setZero();

        if (angle > 0) {    // right turn
            angle -= 5;     // lag

            motorL.setPower(-0.10);
            motorR.setPower(0.10);

            while (getHeading() - zero < angle){
                Thread.sleep(5);
            }
            motorL.setPower(0.0);
            motorR.setPower(0.0);

        } else {            // left turn
            angle += 5;     // lag

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


    private double currentDistance = 50.0;
    private long distanceTime = 0;
    private final double FILTER_FACTOR = 0.2;
    private final int MAX_READINGS = 5;

    /** return the distance from the ultrasonic sensor */
    public double getDistance() {

        // grab a bunch of readings and average them into one
        double newDistance = 0.0;
        int validCount = 0;
        for (int i = 0; i < MAX_READINGS; i++) {
            double level = ultra.getUltrasonicLevel();
            // throw out any invalid readings
            if (level > 0.0 && level < 255.0) {
                newDistance += level;
                validCount++;
            }
        }

        // make sure we got at least one good reading
        if (validCount != 0) {
            newDistance = newDistance / validCount;
            currentDistance = newDistance;
            //currentDistance = (currentDistance * (1.0 - FILTER_FACTOR)) + (newDistance * FILTER_FACTOR);
            System.out.println("currentDistance: " + currentDistance + " using " + validCount + " readings");
        }

        return currentDistance;
    }

    /* is the color sensor seeing white? */
    public boolean isWhite() {
        if (colorSensor.red() >= 6.0 && colorSensor.blue() >= 6.0 && colorSensor.green() >= 6.0){
            return true;
        }
        return false;
    }


    /*******************************/
    /**       UNUSED CODE         **/
    /*******************************/

    /* bring arm up ... shimmy ... bring arm down */
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

        // shimmy
        for (int i = 0; i < 25; i++) {
            wrist.setPosition(0.15);
            Thread.sleep(100);
            wrist.setPosition(0.1);
            Thread.sleep(100);
        }
        Thread.sleep(500);

        // bring the arm down
        while (armLimit.isPressed() && armPos > -50) {
            armPos = arm.getCurrentPosition();
            double wristPos = Range.clip(((armPos - ARMPOS_MOVE_BUCKET) / ARMPOS_MOVE_BUCKET_RANGE), 0.2, 1);
            wrist.setPosition(wristPos);
            arm.setPower(-Range.clip(armPos / 200.0, 0.2, 0.75)); //FIXME
            Thread.sleep(20);  // do we need this?
            System.out.println("armPos: " + armPos + ", arm power: " + -Range.clip(armPos / 200, 0.2, 1));
        }

        System.out.println("arm is down, arm.isBusy: " + arm.isBusy());

        arm.setPower(0);

        System.out.println("armPos: " + arm.getCurrentPosition()
                + ", armLimit: " + !armLimit.isPressed()
                + ", arm.isBusy: " + arm.isBusy());

        /* extra stops -- just in case */
        for (int stops = 1; stops <= 5; stops++) {
            arm.setPower(0);
            Thread.sleep(100);
            System.out.println("Stop # " + stops);
        }

        // bring the bucket up
        wrist.setPosition(WUP);
    }

    /* bring arm up ... bring arm down */
    public void autoArmNoJimmy() throws InterruptedException {
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
        wrist.setPosition(0);
        Thread.sleep(2000);

        // bring the arm down
        int loopCount = 0;
        while (armLimit.isPressed() && armPos >= 0) {
            armPos = arm.getCurrentPosition();
            double wristPos = Range.clip(((armPos - ARMPOS_MOVE_BUCKET) / ARMPOS_MOVE_BUCKET_RANGE), 0.2, 1);
            wrist.setPosition(wristPos);
            arm.setPower(-Range.clip(armPos / 200.0, 0.1, 0.75)); //FIXME
            Thread.sleep(20);  // do we need this?
            loopCount++;
            //System.out.println("armPos: " + armPos + ", arm power: " + -Range.clip(armPos / 200, 0.2, 1));
        }

        System.out.println("loopCount: " + loopCount);
        System.out.println("arm is down, arm.isBusy: " + arm.isBusy());

        arm.setPower(0);

        System.out.println("armPos: " + arm.getCurrentPosition()
                + ", armLimit: " + !armLimit.isPressed()
                + ", arm.isBusy: " + arm.isBusy());

        /* extra stops -- just in case */
        for (int stops = 1; stops <= 5; stops++) {
            arm.setPower(0);
            Thread.sleep(100);
            System.out.println("Stop # " + stops);
        }

        // bring the bucket up
        wrist.setPosition(WUP);
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

    /*  drive forward without error correction */
    public void driveAcc(double distance, double rampDistance, double finalPower, double initPower ) throws InterruptedException {
        int totalDist = inchesToRotations(distance);
        int rampDist = inchesToRotations(rampDistance);
        int start = motorR.getCurrentPosition();
        finalPower = Range.clip(finalPower, .2, 0.75);
        initPower = Range.clip(initPower, .2, 0.75);
        if(initPower > finalPower) initPower = finalPower;
        if(2*rampDist > totalDist) rampDist = totalDist / 2;

        motorR.setPower(initPower);
        motorL.setPower(initPower);
        int motorPos = motorR.getCurrentPosition();
        double newPower = 0;
        double oldPower = 0;

        while (motorPos < (start + totalDist)) {
            if(motorPos < (start + rampDist)) {
                double distThruRamp = ( (motorPos - start) ) / rampDist;
                newPower = initPower + (distThruRamp * (finalPower - initPower));
            } else if (motorPos < (start + totalDist - rampDist)) {
                newPower = finalPower;
            } else {
                double distThruRamp = (totalDist - (motorPos - start)) / rampDist;
                newPower = initPower + (distThruRamp * (finalPower - initPower));
                //newPower = 0.2;
            }
            Thread.sleep(20);

            newPower = Range.clip(newPower, initPower, finalPower);
            if (oldPower != newPower) {
                motorR.setPower(newPower);
                motorL.setPower(newPower);
                oldPower = newPower;
                System.out.println("just set power to " + newPower);
            }
            motorPos = motorR.getCurrentPosition();
            telemetry.addData("motorPos", motorPos);
            telemetry.addData("newPower", newPower);
            telemetry.update();
            this.idle();
        }

        motorR.setPower(0);
        motorL.setPower(0);
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

}