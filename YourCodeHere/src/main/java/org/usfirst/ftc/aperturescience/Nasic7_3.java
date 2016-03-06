package org.usfirst.ftc.aperturescience;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Nasic 7.2 - TeleOp
 *
 * @author FTC 5064 Aperture Science
 */
public class Nasic7_3 extends OpMode {

    // hardware
    private DcMotor arm;
    private DcMotor turntable;
    private DcMotor motorR;
    private DcMotor motorL;
    private DcMotor sweeper;
    private DcMotor tapeMotor;
    private Servo leftShield;
    private Servo rightShield;
    private Servo backShield;
    private Servo wrist;
    private Servo redArm;
    private Servo blueArm;
    private Servo leftTape;
    private Servo rightTape;
    private Servo tapeLock;
    private TouchSensor armLimit;
    private TouchSensor turnLimit;

    public enum ALLIANCE {RED, BLUE}
    private ALLIANCE alliance;

    public Nasic7_3(ALLIANCE alliance) {
        super();
        this.alliance = alliance;
    }

    @Override
    public void init() {

        // motors
        motorR = hardwareMap.dcMotor.get("motorR");
        motorL = hardwareMap.dcMotor.get("motorL");
        arm = hardwareMap.dcMotor.get("arm");
        turntable = hardwareMap.dcMotor.get("turntable");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        tapeMotor = hardwareMap.dcMotor.get("tapeMotor");

        // servos
        leftShield = hardwareMap.servo.get("leftShield");
        rightShield = hardwareMap.servo.get("rightShield");
        backShield = hardwareMap.servo.get("backShield");
        wrist = hardwareMap.servo.get("wrist");
        redArm = hardwareMap.servo.get("redArm");
        blueArm = hardwareMap.servo.get("blueArm");
        leftTape = hardwareMap.servo.get("leftTape");
        rightTape = hardwareMap.servo.get("rightTape");
        tapeLock = hardwareMap.servo.get("tapeLock");

        // touch sensors
        armLimit = hardwareMap.touchSensor.get("armSwitch");
        turnLimit = hardwareMap.touchSensor.get("ttSwitch");

        // initial servo positions
        leftShield.setPosition(LSDOWN);
        wrist.setPosition(WUP);
        rightShield.setPosition(RSDOWN);
        backShield.setPosition(BSDOWN);
        redArm.setPosition(RED_INIT);
        blueArm.setPosition(BLUE_INIT);
        tapeLock.setPosition(TAPE_LOCK_OUT);
        leftTape.setPosition(LTAPE_UP);
        rightTape.setPosition(RTAPE_UP);

        if (alliance == ALLIANCE.BLUE) {
            climberPos = BLUE_UP;
        }

        // run certain motors using encoders
        motorL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorL.setDirection(DcMotor.Direction.REVERSE);
        motorR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        arm.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        arm.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        turntable.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        turntable.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        sweeper.setDirection(DcMotor.Direction.REVERSE);
        tapeMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    private int loopCount = 0;
    private long elapsedTime = 0;

    @Override
    public void loop() {
        long start = System.currentTimeMillis();
        godControl();
        driveControl();
        armControl();
        sweeperControl();
        shieldControl();
        wristControl();
        if (alliance == ALLIANCE.RED) {
            redClimberArmControl();
        } else {
            blueClimberArmControl();
        }
        tapeControl();
        long end = System.currentTimeMillis();

        elapsedTime += (end - start);
        loopCount++;
        if (loopCount == 100) {
            System.out.println("Average loop time: " + (elapsedTime / (double)loopCount));
            elapsedTime = 0;
            loopCount = 0;
        }
    }

    /** Basic controller constants **/
    private final double TRIGGER_THRESHOLD = 0.2;
    private final double JOY_THRESHOLD = 0.2;



    /** DRIVE THE ROBOT **/

    private final double NORMAL_DRIVE_SPEED = 0.5;
    private final double LOW_DRIVE_SPEED = 0.3;
    private final double HIGH_DRIVE_SPEED = 1.0;

    private void driveControl() {
        float throttle = -gamepad1.left_stick_y;
        float direction = gamepad1.right_stick_x;

        double speedMult = NORMAL_DRIVE_SPEED;

        if (gamepad1.right_bumper) {
            speedMult = HIGH_DRIVE_SPEED;
        } else if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            speedMult = LOW_DRIVE_SPEED;
        }

        // computer motor power
        float right = throttle + direction;
        float left = throttle - direction;

        // scale and clip the motor powers
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        right = (float) scaleInput(right);
        left = (float) scaleInput(left);

        // set the power to the motors
        motorR.setPower(right * speedMult);
        motorL.setPower(left * speedMult);

        // motor telemetry
        telemetry.addData("speedMult: ", speedMult);
        telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
    }



    /** SWEEP UP BLOCKS **/

    private final double SWEEPER_POWER_IN = 0.5;
    private final double SWEEPER_POWER_OUT = 1.0;

    private double sweeperPower = 0.0;

    private void sweeperControl() {
        if (gamepad1.a) {
            sweeperPower = SWEEPER_POWER_IN;
        } else if (gamepad1.b) {
            sweeperPower = 0.0;
        } else if (gamepad1.y) {
            sweeperPower = -SWEEPER_POWER_OUT;
        }
        sweeper.setPower(sweeperPower);
    }


    /** DROP BLOCKS IN THE BUCKETS **/

    private final double NORMAL_ARM_SPEED = 0.4;
    private final double LOW_ARM_SPEED = 0.2;
    private final double HIGH_ARM_SPEED = 1.0;
    private final double ARMPOS_CLEARED_SWITCH = 2000;
    private final double ARMPOS_MOVE_BUCKET = 1500;
    private final double ARMPOS_MOVE_BUCKET_RANGE = 4000;
    private final double ARM_TTLIMIT_POS = 4800;
    private final double TT_CENTER_RANGE = 50;
    private final double TT_ARMLIMIT_POS = 4600;
    private final double AUTO_ARM_MAX = 6500;
    private final double ARM_BACK_BUCKET_UP = 1.0;
    private final double ARM_BACK_BUCKET_DOWN = 0.35;
    private final double ARM_BACK_TT_TURN = 1150;
    private final double NORMAL_TABLE_SPEED = 0.4;
    private final double LOW_TABLE_SPEED = 0.2;
    private final double HIGH_TABLE_SPEED = 1.0;
    private final double TT_FAILSAFE_LIMIT = 3000;


    private boolean prevAutoButton = false;
    private boolean prevAutoBButton = false;
    private boolean autoArmUp = false;
    private boolean autoArmDown = false;
    private boolean isArmUp = false;
    private boolean autoTTOut = false;
    private boolean autoTTCenter = false;
    private boolean resetEncoder = false;
    private boolean encoderResetTT = false;

    private void armControl() {

        // get the current position of the arm and turntable
        double ttPos = turntable.getCurrentPosition();
        double armPos = arm.getCurrentPosition();

        // requesting automatic arm?
        if (gamepad2.a && !prevAutoButton) {
            if (!isArmUp) {      // arm should go up
                autoArmUp = true;
                autoArmDown = false;
                wristTime = System.currentTimeMillis() + 500;
                wristWay = 1;
            } else {            // arm should go down
                autoArmUp = false;
                autoArmDown = true;
                wristTime = System.currentTimeMillis() + 3000;
                wristWay = -1;

                // make sure to center the turn table as well
                autoTTCenter = true;
                autoTTOut = false;
            }

            // flip the toggle
            isArmUp = !isArmUp;
        }
        prevAutoButton = gamepad2.a;

        // automatic arm raise
        if (autoArmUp) {
            //System.out.println("autoArmUp");
            wristPos = Range.clip(((armPos - ARMPOS_MOVE_BUCKET) / ARMPOS_MOVE_BUCKET_RANGE), .1, 1);
            wrist.setPosition(wristPos);

            if(armPos > ARMPOS_CLEARED_SWITCH) {
                isArmUp = true;
            }

            // have we reached the max position?
            if (armPos < AUTO_ARM_MAX) {
                //System.out.println("autoArmUp: running: " + Range.clip(armPos / 200, 0.2, 1));
                arm.setPower(Range.clip(armPos / 200, 0.2, 1));
            } else {
                //System.out.println("autoArmUp: stop");
                autoArmUp = false;  // auto arm up is done!
                arm.setPower(0.0);
            }
        }

        // automatic arm down
        if (autoArmDown) {
            wristPos = Range.clip(((armPos - ARMPOS_MOVE_BUCKET) / ARMPOS_MOVE_BUCKET_RANGE), .2, 1);
            wrist.setPosition(wristPos);

            // centered? if not, the arm cannot come down
            if(armPos < ARM_TTLIMIT_POS && Math.abs(ttPos) > TT_CENTER_RANGE) {
                arm.setPower(0.0);
            } else if (armLimit.isPressed() && arm.getCurrentPosition() > -TT_CENTER_RANGE ) {
                arm.setPower(-Range.clip(armPos / 200, 0.2, 1));
            } else {
                autoArmDown = false;
                wristPos = WDOWN;
                arm.setPower(0.0);
                bucketUp = false;
            }
        }

        // re-enable the turntable encoders after a reset
        if (encoderResetTT){
            turntable.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            encoderResetTT = false;
        }

        // turn table automation
        if (armPos > TT_FAILSAFE_LIMIT) {
            if (gamepad2.b && !prevAutoBButton) {
                if (Math.abs(ttPos) < TT_CENTER_RANGE) {
                    autoTTCenter = false;
                    autoTTOut = true;
                } else {
                    autoTTCenter = true;
                    autoTTOut = false;
                }
            }
        }
        prevAutoBButton = gamepad2.b;

        // automatically center the turntable?
        if (autoTTCenter) {
            if (ttPos < -TT_CENTER_RANGE) {
                turntable.setPower(Range.clip(-ttPos / 1000, 0.2, 1));
            } else if (ttPos > TT_CENTER_RANGE) {
                turntable.setPower(-Range.clip(ttPos / 1000, 0.2, 1));
            } else {
                autoTTCenter = false;
                autoTTOut = false;
                /* code for using the turntable switch
                if (ttSwitch.IsPressed()) {
                    turntable.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    encoderResetTT = true;
                }
                */
                turntable.setPower(0.0);
            }
        }

        // automatically turn the turntable
        if (autoTTOut){
            if (alliance == ALLIANCE.RED && ttPos < ARM_BACK_TT_TURN){
                turntable.setPower(NORMAL_TABLE_SPEED);
            } else if (alliance == ALLIANCE.BLUE && ttPos > -ARM_BACK_TT_TURN) {
                turntable.setPower(-NORMAL_TABLE_SPEED);
            } else {
                turntable.setPower(0.0);
                autoTTOut = false;
                autoTTCenter = false;
            }
        }

        /* Debugging */
        telemetry.addData("AutoArmUp", autoArmUp);
        telemetry.addData("AutoArmDown", autoArmDown);
        telemetry.addData("IsArmUp", isArmUp);
        telemetry.addData("LimitSwitch", armLimit);
        telemetry.addData("AutoWristPos", wristPos);
        telemetry.addData("ttPos",ttPos);
        telemetry.addData("AutoTTOut", autoTTOut);
        telemetry.addData("AutoTTCenter",autoTTCenter);

        // re-enable arm motor
        if (resetEncoder) {
            //System.out.println("reset arm encoder");
            arm.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            turntable.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            resetEncoder = false;
        }

        // manual controls -- overrides automatic
        double armMult = NORMAL_ARM_SPEED;

        if ((gamepad2.left_stick_y > JOY_THRESHOLD && armLimit.isPressed() == true
                && !(armPos < ARM_TTLIMIT_POS && (Math.abs(ttPos) > TT_CENTER_RANGE)))
                || (gamepad2.left_stick_y > JOY_THRESHOLD && godMode)) {
            // move the arm down
            autoArmDown = autoArmUp = false;
            double armSpeed = -gamepad2.left_stick_y;
            arm.setPower(armSpeed * armMult);
        } else if (gamepad2.left_stick_y < -TRIGGER_THRESHOLD) {
            // move the arm up
            autoArmDown = autoArmUp = false;
            double armSpeed = -gamepad2.left_stick_y;
            arm.setPower(armSpeed * armMult);
        } else if (!armLimit.isPressed() && !autoArmUp) {
            // arm is all the way down
            arm.setPower(0.0);
            arm.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            turntable.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            resetEncoder = true;
            telemetry.addData("ArmLimit: ", "Yes");
        } else if (!autoArmUp && !autoArmDown){
            // arm does nothing for now
            arm.setPower(0);
        }

        if (!armLimit.isPressed()) {
            isArmUp = false;
        }

        telemetry.addData("Arm Position: ", arm.getCurrentPosition());

        // manual control of the turntable
        double ttMult = NORMAL_TABLE_SPEED;

        if (gamepad2.left_bumper) {
            ttMult = LOW_TABLE_SPEED;
        } else if (gamepad2.left_trigger > TRIGGER_THRESHOLD) {
            ttMult = HIGH_TABLE_SPEED;
        }

        if ((armPos > TT_ARMLIMIT_POS) || godMode) {
            if (gamepad2.right_stick_x > JOY_THRESHOLD) {
                autoTTCenter = autoTTOut = false;
                double ttSpeed = -gamepad2.right_stick_x;
                turntable.setPower(ttSpeed * ttMult);
            } else if (gamepad2.right_stick_x < -JOY_THRESHOLD) {
                autoTTCenter = autoTTOut = false;
                double ttSpeed = -gamepad2.right_stick_x;
                turntable.setPower(ttSpeed * ttMult);
            } else if (!autoTTCenter && !autoTTOut) {
                turntable.setPower(0);
            }
        } else if (!autoTTCenter && !autoTTOut) {
            turntable.setPower(0);
        }

        telemetry.addData("Turn Table", turntable.getCurrentPosition());
    }



    /** RAISE AND LOWER THE SHIELDS **/

    private final double LSDOWN = 0.01;
    private final double LSUP = 0.57;
    private final double RSDOWN = 0.6;
    private final double RSUP = 0.04;
    private final double BSDOWN = 0.42;
    private final double BSUP = 0.84;

    private boolean g1PrevX = false;
    private boolean shieldDown = true;

    private void shieldControl() {

        if (gamepad1.x && g1PrevX == false && isTapeLocked == false) {
            shieldDown = !shieldDown;
            if (shieldDown) {
                tapeServoPosLeft = LTAPE_UP;
                tapeServoPosRight = RTAPE_UP;
            } else {
                tapeServoPosRight = RTAPE_UP + .4;
                tapeServoPosLeft = LTAPE_UP - .4;
            }
        }
        if (shieldDown) {
            backShield.setPosition(BSDOWN);
            rightShield.setPosition(RSDOWN);
            leftShield.setPosition(LSDOWN);
        } else {
            backShield.setPosition(BSUP);
            rightShield.setPosition(RSUP);
            leftShield.setPosition(LSUP);
        }

        g1PrevX = gamepad1.x;

    }


    /** MOVE THE BUCKET **/

    private final double WDOWN = 0.47;
    private final double WUP = 0.01;

    private boolean prevWristButton = false;
    private boolean prevWristMan = false;
    private boolean bucketUp = true;
    private double wristPos = WUP;
    private boolean wristGoingUp = false;
    private boolean wristGoingDown = false;
    private long wristTime = 0;
    private int wristWay = 0;

    private void wristControl() {

        boolean currPress = gamepad1.left_stick_button || gamepad1.right_stick_button || gamepad2.left_stick_button || gamepad2.right_stick_button;

        if (currPress && prevWristButton == false) {
            bucketUp = !bucketUp;
            if (bucketUp) {
                if(arm.getCurrentPosition() < 4000) {
                    wristPos = WUP;
                    wristTime = System.currentTimeMillis() + 500;
                    wristWay = 1;
                }
                else wristPos = ARM_BACK_BUCKET_UP;
            } else {
                if(arm.getCurrentPosition() < 4000) {
                    wristPos = WDOWN;
                    wristTime = System.currentTimeMillis() + 500;
                    wristWay = -1;
                }
                else wristPos = ARM_BACK_BUCKET_DOWN;
            }
        }

        if (wristWay == 1){
            sweeper.setPower(SWEEPER_POWER_OUT);
            if(System.currentTimeMillis() > wristTime) {
                wristWay = 0;
            }
        }
        else if (wristWay == -1) {
            sweeper.setPower(-SWEEPER_POWER_OUT);
            if (System.currentTimeMillis() > wristTime) {
                wristWay = 0;
            }
        }

        wrist.setPosition(wristPos);
        if (!autoArmUp && !autoArmDown) {
            if (gamepad2.right_bumper && !prevWristMan) {
                wristPos -= 0.1;
            }
            else if (gamepad2.right_trigger > TRIGGER_THRESHOLD && !prevWristMan) {
                wristPos += 0.1;
            }

            wristPos = Range.clip(wristPos, 0, 1);
            wrist.setPosition(wristPos);
            prevWristButton = currPress;
            prevWristMan = (gamepad2.right_trigger > TRIGGER_THRESHOLD) || gamepad2.right_bumper;
        }

        telemetry.addData("Wrist Pos", wristPos);
    }
    /* red arm constants */
    private final double RED_INIT = 1.0;
    private final double RED_UP = 0.9;
    private final double RED_FIRST = 0.2;
    private final double RED_DOWN = 0.0;

    /* blue arm constants */
    private final double BLUE_INIT = 0.0;
    private final double BLUE_UP = 0.1;
    private final double BLUE_FIRST = 0.8;
    private final double BLUE_DOWN = 1.0;

    private final double CLIMBER_INCREMENT = 0.05;

    private double climberPos = RED_UP;
    boolean prevClimberButton = false;
    boolean prevManClimberButton = false;
    boolean prevGP2ClimberButton = false;

    private void redClimberArmControl() {

        if ((gamepad1.dpad_left) && prevClimberButton == false) {
            if (Math.abs(climberPos - RED_DOWN) > 0.03) {
                climberPos = RED_DOWN;
            }
            else{
                climberPos = RED_UP;
            }
        }

        if ((gamepad1.dpad_right) && prevClimberButton == false) {
            if (Math.abs(climberPos - RED_FIRST) > 0.03) {
                climberPos = RED_FIRST;
            }
            else {
                climberPos = RED_UP;
            }
        }

        prevClimberButton = gamepad1.dpad_left || gamepad1.dpad_right;

        if (gamepad1.dpad_up && prevManClimberButton == false){
            climberPos += CLIMBER_INCREMENT;
            climberPos = Range.clip(climberPos,0,1);
        }
        else if (gamepad1.dpad_down && prevManClimberButton == false){
            climberPos -= CLIMBER_INCREMENT;
            climberPos = Range.clip(climberPos,0,1);
        }

        if(gamepad2.left_bumper) {
            climberPos = RED_FIRST;
        } else if (gamepad2.left_trigger > TRIGGER_THRESHOLD) {
            climberPos = RED_DOWN;
        } else if (prevGP2ClimberButton) {
            climberPos = RED_UP;
        }

        prevGP2ClimberButton = gamepad2.left_bumper || (gamepad2.left_trigger > TRIGGER_THRESHOLD);
        prevManClimberButton = gamepad1.dpad_down || gamepad1.dpad_up;
        redArm.setPosition(climberPos);
        telemetry.addData("redArm", redArm.getPosition());
    }

    private void blueClimberArmControl() {

        if ((gamepad1.dpad_left) && prevClimberButton == false) {
            if (Math.abs(climberPos - BLUE_DOWN) > 0.03) {
                climberPos = BLUE_DOWN;
            }
            else{
                climberPos = BLUE_UP;
            }
        }

        if ((gamepad1.dpad_right) && prevClimberButton == false) {
            if (Math.abs(climberPos - BLUE_FIRST) > 0.03) {
                climberPos = BLUE_FIRST;
            }
            else{
                climberPos = BLUE_UP;
            }
        }

        prevClimberButton = gamepad1.dpad_left || gamepad1.dpad_right;

        if (gamepad1.dpad_up && prevManClimberButton == false){
            climberPos -= CLIMBER_INCREMENT;
            climberPos = Range.clip(climberPos,0,1);
        }
        else if (gamepad1.dpad_down && prevManClimberButton == false){
            climberPos += CLIMBER_INCREMENT;
            climberPos = Range.clip(climberPos,0,1);
        }

        if(gamepad2.left_bumper) {
            climberPos = BLUE_FIRST;
        } else if (gamepad2.left_trigger > TRIGGER_THRESHOLD) {
            climberPos = BLUE_DOWN;
        } else if (prevGP2ClimberButton) {
            climberPos = BLUE_UP;
        }

        prevGP2ClimberButton = gamepad2.left_bumper || (gamepad2.left_trigger > TRIGGER_THRESHOLD);
        prevManClimberButton = gamepad1.dpad_down || gamepad1.dpad_up;
        blueArm.setPosition(climberPos);
        telemetry.addData("blueArm", blueArm.getPosition());
    }

    boolean prevTapeServoButton = false;
    //boolean prevTapeMotorButton = false;
    boolean isTapeLocked = false;
    final double TAPE_SPEED_OUT = 0.6;
    final double TAPE_SPEED_IN = 1.0;
    private final double TAPE_CONTROL_INCREMENT = 0.05;
    private final double TAPE_FINE_CONTROL_INCREMENT = 0.01;
    private final double LTAPE_UP = 0.81;
    private final double RTAPE_UP = 0.25;
    private final double TAPE_SERVO_RANGE = 0.75;
    private final double TAPE_LOCK_IN = 0.95;
    private final double TAPE_LOCK_OUT = 0.35;
    double tapeServoPosRight = RTAPE_UP;
    double tapeServoPosLeft = LTAPE_UP;

    private void tapeControl() {

        if (gamepad2.x && gamepad2.y && !gamepad2.start) {
            tapeLock.setPosition(TAPE_LOCK_IN);
            isTapeLocked = true;
        } else if (gamepad2.x && gamepad2.y && gamepad2.start) {
            tapeLock.setPosition(TAPE_LOCK_OUT);
            isTapeLocked = false;
        }

        if(gamepad2.dpad_up) {
            tapeMotor.setPower(TAPE_SPEED_OUT);
        } else if (gamepad2.dpad_down) {
            tapeMotor.setPower(-TAPE_SPEED_IN);
        } else {
            tapeMotor.setPower(0);
        }

        if(gamepad2.dpad_left && !prevTapeServoButton) {
            //Angle Tape Servos Higher
            tapeServoPosRight-=TAPE_CONTROL_INCREMENT;
            tapeServoPosLeft+=TAPE_CONTROL_INCREMENT;
        } else if(gamepad2.dpad_right && !prevTapeServoButton) {
            //Angle Tape Servos Lower
            tapeServoPosRight+=TAPE_CONTROL_INCREMENT;
            tapeServoPosLeft-=TAPE_CONTROL_INCREMENT;
        }
        if(gamepad2.left_trigger > TRIGGER_THRESHOLD && !prevTapeServoButton) {
            //Angle Tape Servos Higher
            tapeServoPosRight+=TAPE_FINE_CONTROL_INCREMENT;
        } else if(gamepad2.left_bumper && !prevTapeServoButton) {
            //Angle Tape Servos Lower
            tapeServoPosLeft-=TAPE_FINE_CONTROL_INCREMENT;
        }

        tapeServoPosLeft = Range.clip(tapeServoPosLeft, LTAPE_UP - .75, LTAPE_UP);
        tapeServoPosRight = Range.clip(tapeServoPosRight, RTAPE_UP, RTAPE_UP + .75);
        leftTape.setPosition(tapeServoPosLeft);
        rightTape.setPosition(tapeServoPosRight);

        prevTapeServoButton = gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.left_bumper || gamepad2.left_trigger > TRIGGER_THRESHOLD;
        //prevTapeMotorButton = gamepad2.dpad_up || gamepad2.dpad_down;
    }


    /** PROVIDE FAIL-SAFE COMMANDS -- IGNORE ALL LIMITS **/

    private boolean godMode = false;

    private void godControl(){
        if (gamepad2.back){
            godMode = true;
        }else {
            godMode = false;
        }
        telemetry.addData("godMode",godMode);
    }


    /** UTILITIES **/

    /**
     * Scale the speed of the drive motors to allow for
     * fine control and speed.
     *
     * @param dVal value to sacale
     * @return scaled value
     */
    private double scaleInput(double dVal) {
        double[] scaleArray = {
                0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00
        };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * (scaleArray.length - 1));
        if (index < 0) {
            index = -index;
        } else if (index > (scaleArray.length - 1)) {
            index = scaleArray.length - 1;
        }

        double dScale;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }
}

/**
 * Motor Controllers
 *  VXUV (P6) 1: motorR     2: motorL
 *  VGOE (P3) 1: arm        2: turntable
 *  YCGG (P5) 1: sweeper    2: tapeMotor
 *  QHLV (P4) off
 *
 * Servo Controllers
 *  QTS8 (P1) 1: leftShield     2: rightShield      3: backShield     5: wrist
 *  VDLE (P2) 1: leftTape       2: rightTape        4: tapeLock       5. redArm     6. blueArm
 *
 * (VD4U) Core Device
 *  D0: armSwitch
 *  D1: ttSwitch
 *  I2C0: gyro
 *  I2C1: colorSensor
 */