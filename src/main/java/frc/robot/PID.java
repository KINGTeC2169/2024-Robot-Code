package frc.robot;

public class PID {
    
    public PID(double P, double I, double D) {
		this.P = P;
		this.I = I;
		this.D = D;
	}

	double P, I, D;
	double previous_error, integral = 0;
	double setpoint = 0;
	double error;
	private double output;

	public void calculate(double actual) {
		error = setpoint - actual; // Error = Target - Actual
		this.integral += (error * .02); // Integral is increased by the error*time (which is .02 seconds using normal
		final double derivative = (error - this.previous_error) / .02;
		this.output = P * error + I * this.integral + D * derivative;
		this.previous_error = error;
		//System.out.println("Error: " + error);
	}

	public void setSetpoint(double setpoint) {
		this.setpoint = setpoint;
	}

	public double getOutput() {
		return output;
	}
    
	public void resetI() {
		this.integral = 0;
	}
}
