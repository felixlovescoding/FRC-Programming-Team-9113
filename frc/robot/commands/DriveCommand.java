package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand {
    private final DriveSubsystem driveSub = new DriveSubsystem();

    public void moveToBall() {
        // Drives forward at half speed until the robot has moved 5 feet, then stops:
        if(driveSub.getEncoderDistance() < 5) {
            driveSub.move(0.5);
        } else {
            driveSub.stop();
        }
    }
}
