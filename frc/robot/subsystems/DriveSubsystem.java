package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Encoder;

public class DriveSubsystem {

    //Use a DifferentialDrive to control motors
    Spark m_frontLeft = new Spark(1);
    Spark m_rearLeft = new Spark(2);
    MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);

    Spark m_frontRight = new Spark(3);
    Spark m_rearRight = new Spark(4);
    MotorControllerGroup m_right = new MotorControllerGroup(m_frontRight, m_rearRight);
    DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

    //Configure drive encoder
    private Encoder wheelEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k2X);

    private final double WHEEL_DIAMETER = 6 * 2.54;   //Measure in cm so that the robot distance is recorded in cm
    private double distancePerPulse = (double) 1/2048 * WHEEL_DIAMETER * Math.PI;

    public Drive() {
        leftMotor1.setInverted(false);
        leftMotor2.setInverted(false);
        rightMotor1.setInverted(true);
        rightMotor2.setInverted(true);

        wheelEncoder.setDistancePerPulse(distancePerPulse);
        wheelEncoder.setMinRate(10);
    }

    public void move(double power) {
        leftMotor1.set(power);
        leftMotor2.set(power);
        rightMotor1.set(power);
        rightMotor2.set(power);
    }

    public void turnClockwise()  {
        double power = POWER;

        leftMotor1.set(power);
        leftMotor2.set(power);
    }

    public void turnCounterclockwise() {

    }

    public void stop() {
        leftMotor1.set(0);
        leftMotor2.set(0);
        rightMotor1.set(0);
        rightMotor2.set(0);
    }

    public double getEncoderDistance() {
        return wheelEncoder.getDistance();
    }
}
