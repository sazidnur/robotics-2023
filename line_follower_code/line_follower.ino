const int dataPin = 10;
const int handshakePin = 13;

int L1 = 3, L2 = 2, L3 = 5, L4 = 4;
int leftLine = 11, rightLine = 12;
int obsPin = 8;

int baseSpeed = 150;
int turnSpeed = 200;
int turnDuration = 200;

unsigned long whiteLineStartTime = 0;

bool robotFlag = false;

void setup() {
  pinMode(dataPin, INPUT);
  pinMode(handshakePin, INPUT);

  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(L3, OUTPUT);
  pinMode(L4, OUTPUT);

  pinMode(leftLine, INPUT);
  pinMode(rightLine, INPUT);

  pinMode(obsPin, INPUT);

  Serial.begin(115200);
}

void loop() {
  Serial.println(robotFlag ? "Start" : "Stopped");
  if (digitalRead(handshakePin) == HIGH && digitalRead(dataPin) == HIGH) {
    robotFlag = !robotFlag;
    Serial.print("Robot Flag State: ");
    Serial.println(robotFlag ? "Moving" : "Stopped");
    while (digitalRead(handshakePin) == HIGH);
  }
  if (robotFlag) {
    boolean Left = digitalRead(leftLine);
    boolean Right = digitalRead(rightLine);
    boolean obs = digitalRead(obsPin);
    Serial.println(obs);

    if (Left && Right && obs) {
      Serial.println("Forward");
      forward(baseSpeed);
    } else if (!Left && !Right && obs) {
      halt();
    } else if (!Right && obs) {
      while (!digitalRead(rightLine)) {
        rightSpin(turnSpeed, 1);
      }
      rightSpin(turnSpeed, 40);
    } else if (!Left && obs) {
      while (!digitalRead(leftLine)) {
        leftSpin(turnSpeed, 1);
      }
      leftSpin(turnSpeed, 40);
    } else {
      halt();
      unsigned long startTime = millis();
      unsigned long twoSeconds = 2000;

      while (millis() - startTime < twoSeconds) {
        if (digitalRead(handshakePin) == HIGH && digitalRead(dataPin) == HIGH && robotFlag) {
          robotFlag = !robotFlag;  // Toggle the robot flag
          Serial.print("Robot Flag State: ");
          Serial.println(robotFlag ? "Moving" : "Stopped");
          while (digitalRead(handshakePin) == HIGH)
            ;
        }
      }

      if (!digitalRead(obsPin)) {
        while (digitalRead(leftLine)) {
          leftSpin(turnSpeed, 1);
        }
        leftSpin(turnSpeed, 40);
        whiteLineStartTime = 0;
      }
    }
  } else {
    analogWrite(L1, 0);
    digitalWrite(L2, LOW);
    analogWrite(L3, 0);
    digitalWrite(L4, LOW);
  }
}

void forward(int speed) {
  analogWrite(L1, speed);
  digitalWrite(L2, LOW);
  analogWrite(L3, speed);
  digitalWrite(L4, LOW);
}

void rightSpin(int speed, int duration) {
  analogWrite(L1, speed);
  digitalWrite(L2, LOW);
  analogWrite(L3, 0);
  digitalWrite(L4, HIGH);
  delay(duration);
  forward(baseSpeed);
}

void leftSpin(int speed, int duration) {
  analogWrite(L1, 0);
  digitalWrite(L2, HIGH);
  analogWrite(L3, speed);
  digitalWrite(L4, LOW);
  delay(duration);
  forward(baseSpeed);
}

void halt() {
  analogWrite(L1, 0);
  digitalWrite(L2, LOW);
  analogWrite(L3, 0);
  digitalWrite(L4, LOW);
}
