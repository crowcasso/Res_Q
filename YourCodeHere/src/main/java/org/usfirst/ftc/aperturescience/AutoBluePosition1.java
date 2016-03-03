package org.usfirst.ftc.aperturescience;

/**
 * AutoBluePosition1 (Autonomous)
 *
 * Slow and accurate.
 *
 * @author FTC 5064 Aperture Science
 */
@org.swerverobotics.library.interfaces.Autonomous(name="BLUE Mountain Man Jimmy", group="Red")
public class AutoBluePosition1 extends AutoCommon {

    // how far to stay away from the wall
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

            // back up 3.5 inches
            //driveBack(.3, 3.5);
            Thread.sleep(500);

            // turn left 50 degrees
            turnGyroSlow(62);
            Thread.sleep(200);

            // drive until we're the right distance to the wall
            driveToDistance(.2, 6, THE_DISTANCE);

        } else {
            // did not find white line -- let's try to correct

            // drive forward 3.5 inches
            drive(.3, 6);
            Thread.sleep(500);

            // turn left 56 degrees
            turnGyroSlow(64);
            Thread.sleep(500);

            // drive until we're the right distance to the wall
            driveToDistance(.2, 6, THE_DISTANCE);
        }

        // bring the arm up
        autoArmUp();

        // need to push blocks/balls away
        sweeperOn();

        // drive forward slowly to pull off the climbers
        drive(.08, 12);

        // bring the arm back into the robot to get ready for TeleOp
        autoArmDown();

        // turn the sweeper off
        sweeperOff();

        // back into the red box
        driveBack(.5, 12);
    }
}