#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <vector>

WebServer server(80);


const char* ssid = "ESP32_HOTSPOT";
const char* password = "12345678";

#define S1 34
#define S2 35
#define S3 13
#define S4 14
#define S5 25
#define IN1 18
#define IN2 19
#define IN3 22
#define IN4 23

float sumError = 0;
int lastError = 0;
int baseSpeed = 1;
float Kp = 250.0;
float Ki = 0.0;
float Kd = 7.0;

void motorControl(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    analogWrite(IN2, leftSpeed);
    analogWrite(IN4, rightSpeed);
    digitalWrite(IN3, leftSpeed > 0);
    digitalWrite(IN1, rightSpeed > 0);
}

void turn180() {
    motorControl(100, -100);
    delay(600);
}



void handleRoot() {
    String html = "<html><head>";
    html += "<script>function updateLogs() {";
    html += "var xhr = new XMLHttpRequest();";
    html += "xhr.onreadystatechange = function() {";
    html += "if (this.readyState == 4 && this.status == 200) {";
    html += "document.getElementById('logs').innerHTML = this.responseText;";
    html += "}};";
    html += "xhr.open('GET', '/logs', true);";
    html += "xhr.send();";
    html += "}";
    html += "setInterval(updateLogs, 1000);";
    html += "</script></head><body>";
    html += "<h2>Maze Map Logs</h2>";
    html += "<pre id='logs'>Waiting for logs...</pre>";
    html += "</body></html>";

    server.send(200, "text/html", html);
}


void setup() {
    Serial.begin(115200);
    WiFi.softAP(ssid, password);
    ArduinoOTA.begin();
    server.on("/", handleRoot);
    server.begin();

    pinMode(S1, INPUT);
    pinMode(S2, INPUT);
    pinMode(S3, INPUT);
    pinMode(S4, INPUT);
    pinMode(S5, INPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

void loop() {
    ArduinoOTA.handle();
    server.handleClient();

    int s1 = digitalRead(S1);
    int s2 = digitalRead(S2);
    int s3 = digitalRead(S3);
    int s4 = digitalRead(S4);
    int s5 = digitalRead(S5);

    bool leftPossible = s1;
    bool rightPossible = s5;
    bool forwardPossible = s3;



    // Lost line handling
    if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0) {
        motorControl(-80, -80);
        delay(100);

        for (int i = 0; i < 6; i++) {
            motorControl(80, -80);
            delay(100);
            motorControl(0, 0);
            delay(100);

            int s1_check = digitalRead(S1);
            int s2_check = digitalRead(S2);
            int s3_check = digitalRead(S3);
            int s4_check = digitalRead(S4);
            int s5_check = digitalRead(S5);

            if (s1_check || s2_check || s3_check || s4_check || s5_check) break;
        }
        return;
    }

    // Turn left
    if (s1) {
        for (int i = 0; i < 6; i++) {
            motorControl(-80, 80);
            delay(80);
            motorControl(0, 0);
            delay(80);

            int s1_check = digitalRead(S1);
            int s2_check = digitalRead(S2);
            int s3_check = digitalRead(S3);
            int s4_check = digitalRead(S4);
            int s5_check = digitalRead(S5);

            if (s1_check || s2_check || s3_check || s4_check || s5_check) break;
        }
        
        return;
    }

    // Turn right
    if (s5) {
        for (int i = 0; i < 6; i++) {
            motorControl(80, -80);
            delay(80);
            motorControl(0, 0);
            delay(80);

            int s1_check = digitalRead(S1);
            int s2_check = digitalRead(S2);
            int s3_check = digitalRead(S3);
            int s4_check = digitalRead(S4);
            int s5_check = digitalRead(S5);

            if (s1_check || s2_check || s3_check || s4_check || s5_check) break;
        }
        
        return;
    }

    

    // PID Line Following
    int error = (-20 * s1) + (-10 * s2) + (0 * s3) + (10 * s4) + (20 * s5);
    sumError += error;
    int P = Kp * error;
    int I = Ki * sumError;
    int D = Kd * (error - lastError);
    lastError = error;

    int leftSpeed = baseSpeed + (P + I + D);
    int rightSpeed = baseSpeed - (P + I + D);
    motorControl(leftSpeed, rightSpeed);
    delay(5);

    // Check for maze end (all black)
    
}
