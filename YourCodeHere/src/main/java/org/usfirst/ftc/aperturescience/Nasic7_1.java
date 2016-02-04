package org.usfirst.ftc.aperturescience;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.interfaces.TeleOp;

/**
 * Nasic 6.0 - TeleOp
 *
 * @author FTC 5064 Aperture Science
 */
public class Nasic7_1 extends OpMode {

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
    private Servo servo;
    private TouchSensor armLimit;
    private TouchSensor turnLimit;

    private final double LSDOWN = 0.01;
    private final double LSUP = 0.57;
    private final double RSDOWN = 0.6;
    private final double RSUP = 0.04;
    private final double BSDOWN = 0.81;
    private final double BSUP = 0.35;
    private final double WDOWN = 0.53;
    private final double WUP = 0.0;


    private boolean prevWristButton = false;
    private boolean prevWristMan = false;
    private boolean bucketUp = true;
    private double wristPos = WUP;
    private boolean g1PrevX = false;
    private boolean shieldDown = true;
    private boolean autoArmUp = false;
    private boolean autoArmDown = false;
    private boolean prevAutoButton = false;
    private boolean prevAutoBButton = false;
    private boolean prevGodButtons = false;
    private boolean isArmUp = false;
    private boolean AutottOut = false;
    private boolean AutottCenter = false;
    private boolean GodMode = false;


    boolean resetEncoder = false;
    boolean encoderResetTT = false;
    double position = .5;

    @Override
    public void init() {

        // Hardware Map
        motorR = hardwareMap.dcMotor.get("motorR");
        motorL = hardwareMap.dcMotor.get("motorL");
        arm = hardwareMap.dcMotor.get("arm");
        turntable = hardwareMap.dcMotor.get("turntable");
        sweeper = hardwareMap.dcMotor.get("sweeper");

        leftShield = hardwareMap.servo.get("leftShield");
        rightShield = hardwareMap.servo.get("rightShield");
        backShield = hardwareMap.servo.get("backShield");
        wrist = hardwareMap.servo.get("wrist");

        armLimit = hardwareMap.touchSensor.get("armSwitch");
        turnLimit = hardwareMap.touchSensor.get("ttSwitch");

        leftShield.setPosition(LSDOWN);
        wrist.setPosition(WUP);
        rightShield.setPosition(RSDOWN);
        backShield.setPosition(BSDOWN);


        // Drive Base Setup
        motorL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        arm.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        arm.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        turntable.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        turntable.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        arm.setPower(0);
        turntable.setPower(0);

        motorL.setDirection(DcMotor.Direction.REVERSE);


        sweeper.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        sweeper.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {
        GodControl();
        driveControl();
        armControl();
        sweeperControl();
        shieldControl();
        autoArmControl();
        wristControl();
        //Please add a wiggle.
    }


    /**
     * *****************************
     * DRIVING           *
     * *****************************
     **/


    private final double NORMAL_DRIVE_SPEED = 0.5;
    private final double LOW_DRIVE_SPEED = 0.3;
    private final double HIGH_DRIVE_SPEED = 1.0;
    private final double TRIGGER_THRESHOLD = 0.2;
    private final double NORMAL_ARM_SPEED = 0.4;
    private final double LOW_ARM_SPEED = 0.2;
    private final double HIGH_ARM_SPEED = 1.0;
    private final double NORMAL_TABLE_SPEED = 0.5;
    private final double LOW_TABLE_SPEED = 0.3;
    private final double HIGH_TABLE_SPEED = 1.0;
    private final double JOY_THRESHOLD = 0.1;
    private final double ARM_TTLIMIT_POS = 4800;
    private final double TT_ARMLIMIT_POS = 4600;
    private final double AUTO_ARM_MAX = 6500;
    private final double AUTO_TT_POS_NEED = 4000;
    private final double TT_FINAL_POS = 1000;
    private final double ARM_BACK_BUCKET_UP = 1.0;
    private final double ARM_BACK_BUCKET_DOWN = 0.45;
    private final double ARM_BACK_TT_TURN = 1150;

    /**
     * Drive the robot!
     */
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

        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        right = (float) scaleInput(right);
        left = (float) scaleInput(left);

        // set the motors

        motorR.setPower(right * speedMult);
        motorL.setPower(left * speedMult);


        // motor telemetry
        telemetry.addData("speedMult: ", speedMult);
        telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));

    }

    private void sweeperControl() {
        if (gamepad1.a) {
            sweeper.setPower(.5);
        } else if (gamepad1.b) {
            sweeper.setPower(0);
        } else if (gamepad1.y) {
            sweeper.setPower(-.5);
        }
    }

    private void armControl() {

        double armPos = arm.getCurrentPosition();
        double ttPos = turntable.getCurrentPosition();

        if (resetEncoder) {
            arm.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            resetEncoder = false;
        }
        double armMult = NORMAL_ARM_SPEED;
        if (gamepad2.left_bumper) armMult = LOW_ARM_SPEED;
        else if (gamepad2.left_trigger > TRIGGER_THRESHOLD) armMult = HIGH_ARM_SPEED;

        if ((gamepad2.left_stick_y > JOY_THRESHOLD && armLimit.isPressed() == true && !(armPos < ARM_TTLIMIT_POS && (ttPos > 50 || ttPos < -50))) || (gamepad2.left_stick_y > JOY_THRESHOLD && GodMode)) {
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
            resetEncoder = true;
            telemetry.addData("ArmLimit: ", "Yes");
        } else {
            // arm does nothing for now
            arm.setPower(0);
        }
        if (!armLimit.isPressed()) isArmUp = false;

        telemetry.addData("Arm Position: ", arm.getCurrentPosition());


        double ttMult = NORMAL_ARM_SPEED;
        if (gamepad2.right_bumper) ttMult = LOW_ARM_SPEED;
        else if (gamepad2.right_trigger > TRIGGER_THRESHOLD) ttMult = HIGH_ARM_SPEED;

        if ((armPos > TT_ARMLIMIT_POS) || GodMode) {
            if (gamepad2.right_stick_x > JOY_THRESHOLD) {
                autoArmDown = autoArmUp = false;
                double ttSpeed = -gamepad2.right_stick_x;
                turntable.setPower(ttSpeed * ttMult);
            } else if (gamepad2.right_stick_x < -JOY_THRESHOLD) {
                autoArmDown = autoArmUp = false;
                double ttSpeed = -gamepad2.right_stick_x;
                turntable.setPower(ttSpeed * ttMult);
            } else {
                turntable.setPower(0);
            }
        } else turntable.setPower(0);
        telemetry.addData("Turn Table", turntable.getCurrentPosition());

    }

    private void shieldControl() {

        if (gamepad1.x && g1PrevX == false) shieldDown = !shieldDown;
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

    private void wristControl() {

        boolean currPress = gamepad1.left_stick_button || gamepad1.right_stick_button || gamepad2.left_stick_button || gamepad2.right_stick_button;

        if (currPress && prevWristButton == false) {
            bucketUp = !bucketUp;
            if (bucketUp) {
                if(arm.getCurrentPosition() < 4000) wristPos = WUP;
                else wristPos = ARM_BACK_BUCKET_UP;
            } else {
                if(arm.getCurrentPosition() < 4000) wristPos = WDOWN;
                else wristPos = ARM_BACK_BUCKET_DOWN;
            }
        }
        wrist.setPosition(wristPos);
        if (!autoArmUp && !autoArmDown) {
            if (gamepad2.dpad_up && !prevWristMan) wristPos -= 0.1;
            else if (gamepad2.dpad_down && !prevWristMan) wristPos += 0.1;

            wristPos = Range.clip(wristPos, 0, 1);
            wrist.setPosition(wristPos);
            prevWristButton = currPress;
            prevWristMan = gamepad2.dpad_down || gamepad2.dpad_up;
        }
        telemetry.addData("Wrist Pos", wristPos);

    }

    private void autoArmControl() {

        double ttPos = turntable.getCurrentPosition();
        double armPos = arm.getCurrentPosition();
        double ttPower = -Range.clip((ttPos / 200), 0, 1);


        //Check to see if we need to auto arm
        if (gamepad2.a && !prevAutoButton) {
            isArmUp = !isArmUp;
            if (isArmUp) {
                System.out.println("in isArmUp");
                autoArmUp = true;
                autoArmDown = false;
            } else {
                System.out.println("in else!");
                autoArmUp = false;
                autoArmDown = true;
                AutottCenter = true;
                AutottOut = false;
            }
        }
        prevAutoButton = gamepad2.a;


        if (autoArmUp) {
            wristPos = Range.clip(((armPos - 1500) / 4000), 0, 1);
            wrist.setPosition(wristPos);
            System.out.println("in AutoArm");
            if(armPos > 2000) isArmUp = true;
            if (armPos < AUTO_ARM_MAX) {
                System.out.println("arm power: " + Range.clip(armPos / 200, 0.2, 1));
                arm.setPower(Range.clip(armPos / 200, 0.2, 1));
            } else {
                System.out.println("arm power: 0");
                autoArmUp = false;
                arm.setPower(0.0);
            }
        }

        if (autoArmDown) {
            wristPos = Range.clip(((armPos - 1500) / 4000), 0, 1);
            wrist.setPosition(wristPos);
            System.out.println("in AutoArmDown");
            if(armPos < ARM_TTLIMIT_POS && Math.abs(ttPos) > 50)
            {
                arm.setPower(0.0);
            }
            else if (armLimit.isPressed() && arm.getCurrentPosition() > -50 ) {
                System.out.println("arm power: " + -Range.clip(armPos / 200, 0.2, 1));
                arm.setPower(-Range.clip(armPos / 200, 0.2, 1));
            } else {
                System.out.println("arm power: 0");
                autoArmDown = false;
                arm.setPower(0.0);
            }
        }

        if (encoderResetTT){
            turntable.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            encoderResetTT = false;
        }
        //Turn Table Automation
        if (gamepad2.b && !prevAutoBButton) {
            AutottCenter = !AutottCenter;
            if (AutottCenter) {
                System.out.println("in AutottCenter");
                AutottCenter = true;
                AutottOut = false;
            } else {
                System.out.println("in TT_else!");
                AutottCenter = false;
                AutottOut = true;
            }
        }
        prevAutoBButton = gamepad2.b;

        if (AutottCenter) {
            System.out.println("in AutoTT");
            if (ttPos < -50) {
                System.out.println("tt power: " + Range.clip(ttPos / 200, 0.2, 1));
                turntable.setPower(Range.clip(-ttPos / 1000, 0.2, 1));
            } else if (ttPos > 50) {
                System.out.println("tt power: " + Range.clip(ttPos / 200, 0.2, 1));
                turntable.setPower(-Range.clip(ttPos / 1000, 0.2, 1));
            } else {
                System.out.println("tt power: 0");
                AutottCenter = false;
                AutottOut = false;
                //if (ttSwitch.IsPressed()) {
                // turntable.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                //encoderResetTT = true;
              //}
                turntable.setPower(0.0);
            }
        }

        if (AutottOut){
            if (ttPos < ARM_BACK_TT_TURN){
                turntable.setPower(0.2);
            }
            else{
                turntable.setPower(0.0);
                AutottOut = false;
                AutottCenter = false;
            }
        }

            /*if (armPos > AUTO_TT_POS_NEED && ttPos < TT_FINAL_POS) {
                turntable.setPower(NORMAL_TABLE_SPEED);
                if (Math.abs(TT_FINAL_POS - ttPos) < 250) {
                    turntable.setPower(LOW_TABLE_SPEED);
                }
            } else turntable.setPower(0);*/

        /*if (autoArmDown){
            if (armLimit.isPressed()) arm.setPower(0); autoArmDown = false;
            if (armPos > AUTO_TT_POS_NEED) turntable.setPower(ttPower);
            else turntable.setPower(0);

            arm.setPower(Range.clip(armPos / 200, 0.2, 1));

            if (armPos < (AUTO_TT_POS_NEED + 100) && (ttPos > 50 || ttPos < -50)) arm.setPower(0);
        }*/
        telemetry.addData("AutoArmUp", autoArmUp);
        telemetry.addData("AutoArmDown", autoArmDown);
        telemetry.addData("IsArmUp", isArmUp);
        telemetry.addData("LimitSwitch", armLimit);
        telemetry.addData("AutoWristPos", wristPos);
        telemetry.addData("ttPos",ttPos);
        telemetry.addData("AutoTTOut",AutottOut);
        telemetry.addData("AutoTTCenter",AutottCenter);
    }

    private void GodControl(){
        if (gamepad2.start){
            GodMode = true;
        }else {GodMode = false;}
    }


    /**
     ******************************
     *         UTILITY            *
     ******************************
     **/


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