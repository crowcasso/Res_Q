package org.usfirst.ftc.aperturescience;

/**
 * AutoPeopleNormal_RED (Autonomous)
 *
 * @author FTC 5064 Aperture Science
 */
@org.swerverobotics.library.interfaces.Autonomous(name="RED Mountain Man Jimmy Run", group="Red")
public class AutoRedPosition1Run extends AutoCommon {

    private final double THE_DISTANCE = 28;

    @Override
    public void main() throws InterruptedException {

        // everything up to waitForStart
        super.main();

        // pull out the tapes to make room for the arm
        setTapes();

        // drive back 87 inches or until we find the white line
        boolean foundWhite = driveBackToWhite(.5, 87);
        Thread.sleep(200);  // small pause

        if (foundWhite) {
            // yeah! found the white line

            // back up 4.5 inches
            //driveBack(.3, 4.5);
            Thread.sleep(500);

            // turn left 55 degrees
            turnGyro(-50);
            Thread.sleep(200);

            // drive back to wall using the ultrasonic
            driveToDistance(.2, 6, THE_DISTANCE);

        } else {
            // did not find white line -- let's try to correct

            // drive forward 4.5 inches
            drive(.3, 4.5);
            Thread.sleep(500);

            // turn left 57 degrees
            turnGyro(-56);
            Thread.sleep(500);

            // drive back to wall using ultrasonic
            driveToDistance(.2, 6, THE_DISTANCE);
        }

        // bring the arm up
        autoArmUp();


        // need to push blocks/balls away
        sweeperOn();

        // drive forward to pull off the climbers
        drive(.1, 12);

        // bring the arm vertical for repositioning
        autoArmVert();

        // reposition for the corner
        turnGyro(80);

        // back into the red box
        driveBack(.5, 40);

        // bring the arm back into the robot to get ready for TeleOp
        autoArmDown();

        // turn the sweeper off
        sweeperOff();


    }
}