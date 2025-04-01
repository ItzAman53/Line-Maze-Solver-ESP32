#include <Arduino.h>

// IR Sensor Pins
#define S1 34  // Leftmost sensor
#define S2 35  // Left sensor
#define S3 13  // Center sensor
#define S4 14  // Right sensor
#define S5 25  // Rightmost sensor

// Motor Driver Pins
#define IN1 18  // Left Motor Direction
#define IN2 19  // Left Motor PWM
#define IN3 22  // Right Motor Direction
#define IN4 23  // Right Motor PWM

#define PWM_CHANNEL_LEFT  0  
#define PWM_CHANNEL_RIGHT 1  

int baseSpeed = 120;  // Base speed (adjust for best performance)
float Kp = 30;        // Proportional gain
float Kd = 50;        // Derivative gain

int lastError = 0;    // Stores previous error for derivative term

void setup() {
    // Sensor setup
    pinMode(S1, INPUT);
    pinMode(S2, INPUT);
    pinMode(S3, INPUT);
    pinMode(S4, INPUT);
    pinMode(S5, INPUT);

    // Motor driver setup
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Setup PWM channels
    ledcSetup(PWM_CHANNEL_LEFT, 1000, 8);
    ledcSetup(PWM_CHANNEL_RIGHT, 1000, 8);

    ledcAttachPin(IN2, PWM_CHANNEL_LEFT);  // Left motor PWM
    ledcAttachPin(IN4, PWM_CHANNEL_RIGHT); // Right motor PWM

    Serial.begin(115200);
}

// Motor Control Function (Updated for No Separate PWM Pins)
void motorControl(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    // Left Motor
    if (leftSpeed > 0) {
        digitalWrite(IN1, HIGH);  
        ledcWrite(PWM_CHANNEL_LEFT, leftSpeed);
    } else {
        digitalWrite(IN1, LOW);
        ledcWrite(PWM_CHANNEL_LEFT, -leftSpeed);
    }

    // Right Motor
    if (rightSpeed > 0) {
        digitalWrite(IN3, HIGH);
        ledcWrite(PWM_CHANNEL_RIGHT, rightSpeed);
    } else {
        digitalWrite(IN3, LOW);
        ledcWrite(PWM_CHANNEL_RIGHT, -rightSpeed);
    }
}

void loop() {
    int s1 = digitalRead(S1);  
    int s2 = digitalRead(S2);  
    int s3 = digitalRead(S3);  
    int s4 = digitalRead(S4);  
    int s5 = digitalRead(S5);  

    Serial.print(s1); Serial.print(" ");
    Serial.print(s2); Serial.print(" ");
    Serial.print(s3); Serial.print(" ");
    Serial.print(s4); Serial.print(" ");
    Serial.println(s5);

    // Calculate Error (Weighted sum method)
    int error = (-2 * s1) + (-1 * s2) + (0 * s3) + (1 * s4) + (2 * s5);
    
    // Proportional Term
    int P = Kp * error;

    // Derivative Term (Change in error)
    int D = Kd * (error - lastError);
    lastError = error; // Store the current error for next loop

    // Adjust motor speeds using PLD formula
    int leftSpeed = baseSpeed - (P + D);
    int rightSpeed = baseSpeed + (P + D);

    // Constrain speed values
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    motorControl(leftSpeed, rightSpeed);
    delay(10);  // Small delay for stability
}
