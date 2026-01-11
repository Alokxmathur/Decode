package org.firstinspires.ftc.teamcode;

public class ShootingConfiguration {
    double heading, turretPosition, hoodPosition, shooterSpeed, movement;

    public double getInitialMovement() {
        return movement;
    }

    public double getHeading() {
        return heading;
    }

    public double getTurretPosition() {
        return turretPosition;
    }

    public void setTurretPosition(double turretPosition) {
        this.turretPosition = turretPosition;
    }

    public double getHoodPosition() {
        return hoodPosition;
    }

    public void setHoodPosition(double hoodPosition) {
        this.hoodPosition = hoodPosition;
    }

    public double getShooterSpeed() {
        return shooterSpeed;
    }

    public void setShooterSpeed(double shooterSpeed) {
        this.shooterSpeed = shooterSpeed;
    }

    public ShootingConfiguration(double movement, double heading, double turretPosition, double hoodPosition, double shooterSpeed) {
        this.movement = movement;
        this.heading = heading;
        this.turretPosition = turretPosition;
        this.hoodPosition = hoodPosition;
        this.shooterSpeed = shooterSpeed;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }
}
