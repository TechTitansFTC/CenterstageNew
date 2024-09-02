/*
Copyright 2024 

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.apriltag;

import com.google.mlkit.vision.pose.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Bot;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this OpMode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@Autonomous

public class AAuton extends LinearOpMode {

    public Bot bot;

    @Override
    public void runOpMode() {

        bot = new Bot();
        bot.init(hardwareMap);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // bot starts in tile F4 with a initial heading of 90 degrees
        bot.setHeading(90);
        bot.setFieldXY(12, -60);

        Bot.Pose[] path = new Bot.Pose[]{
             new Minibot.Pose(12, -36),
             new Minibot.Pose(48, 0, 180, Minibot::turnatend),
             new Minibot.Pose(20, -36),
             new Minibot.Pose(20, -56)
        };
        
        followPath(path);
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
    
    public void followPath(Minibot.Pose[] path) {
        int i = 0; //FIRST POINT IN PATH POS 0
        while (i < path.length) {
            if (!opModeIsActive()) return;
            bot.update(); //finite state machine that in parallel checks where the robot is to complete functions parallel
            bot.updateTracking(); //update field position
            double dist = bot.driveToPose(path[i], 0.4); //drives next part //number end is speed the 0.4
            telemetry.addData("path target", i);
            telemetry.addData("target pose", path[i]);
            telemetry.addData("field pose", bot.field);
            telemetry.addData("dist", dist);
            telemetry.update();
            if (dist < 1.5) i++; //when it is 1.5 away itll shift to next pose
        }
        bot.driveXYW(0,0,0);
    }
}
