package org.firstinspires.ftc.teamcode;

public class PIDController {

    double Kp;
    double Ki;
    double Kd;

    double previousError = 0;

    double integral = 0.0;

    double sampleTime = 1.0;

    double target = 0.0;

    double output = 0.0;

    double deadbandSizeCoef = 0.7;

    double deadbandDepthCoef = 0.9;

    PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        this.target = 0;
    }

    void setCoefs(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    void setTarget(double target) {
        this.target = target;
    }

    double loop(double value) {
        double error = calculateDeadband(this.target - value);
        double proportional = error;

        integral += error * sampleTime;
//        integral = Math.min(Math.max(integral, -100.0), 100.0);

        double derivative = (error - previousError) / sampleTime;

        this.output = Kp * proportional + Ki * integral + Kd * derivative;

        previousError = error;

        return output;
    }

    public void setDeadbandStuff(double deadbandSizeCoef, double deadbandDepthCoef) {
        this.deadbandSizeCoef = deadbandSizeCoef;
        this.deadbandDepthCoef = deadbandDepthCoef;
    }

    double calculateDeadband(double v) {
        double x = this.deadbandSizeCoef * v;
        return v * (1 - (4 * this.deadbandDepthCoef) * (1/(1+Math.pow(Math.E, x))) * (1-1/(1+Math.pow(Math.E, x))));
    }
}
