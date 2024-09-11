class MotorPID {
  private:
    double kp, ki, kd;
    double setpoint, input, output;
    double integral, prevError; 
    double alpha;
    double filteredOutput;
    
  public:
    MotorPID(double kp, double ki, double kd, double alpha) : kp(kp), ki(ki), kd(kd), alpha(alpha) {
    setpoint = 0;
    input = 0;
    output = 0;
    integral = 0;
    prevError = 0;
    filteredOutput = 0;
    }

    void setSetpoint(double sp) {
      setpoint = sp;
    }

    void compute(double currentSpeed) {
      // Calculate error
      double error = setpoint - currentSpeed;
      
      // Calculate integral (accumulated error)
      integral += error;
      
      // Calculate derivative (change in error)
      double derivative = error - prevError;
      
      // PID formula
      output = kp * error + ki * integral + kd * derivative;
      
      // Store the current error as previous error for the next cycle
      prevError = error;
      
      // Apply Exponential Smoothing Filter
      filteredOutput = alpha * output + (1 - alpha) * filteredOutput;
    }

    double getFilteredOutput() {
      return filteredOutput;
    }
};

MotorPID motorPID(2.0, 5.0, 1.0, 0.1); 

void setup() {
  pinMode(motorPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // Simulate motor speed feedback
  double currentSpeed = readMotorSpeed();
  
  // Set the target setpoint
  double setpoint = map(analogRead(potPin), 0, 1023, minSpeed, maxSpeed);
  motorPID.setSetpoint(setpoint);
  
  // Compute the new motor speed using PID
  motorPID.compute(currentSpeed);
  
  // Get the filtered output and apply to the motor
  analogWrite(motorPin, (int)motorPID.getFilteredOutput());

  delay(100);
}
