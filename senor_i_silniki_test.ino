const int PWM_MAX = 255;

const int silnikLewy_PWM = 5;       // enA
const int silnikPrawy_PWM = 10;     // enB

const int silnikLewy_PRZOD = 6;     // in1
const int silnikLewy_TYL = 7;       // in2

const int silnikPrawy_PRZOD = 8;    // in3
const int silnikPrawy_TYL = 9;      // in4

const int sensorNadajnik = 12;      // trigPin
const int sensorOdbiornik = 11;     // echoPin


void set_pin_modes() {
  pinMode(silnikLewy_PWM, OUTPUT);
  pinMode(silnikPrawy_PWM, OUTPUT);

  pinMode(silnikLewy_PRZOD, OUTPUT);
  pinMode(silnikLewy_TYL, OUTPUT);

  pinMode(silnikPrawy_PRZOD, OUTPUT);
  pinMode(silnikPrawy_TYL, OUTPUT);

  pinMode(sensorNadajnik, OUTPUT);
  pinMode(sensorOdbiornik, INPUT);
}


void stopEng() {
  analogWrite(silnikLewy_PWM, 0);
  analogWrite(silnikPrawy_PWM, 0);

  digitalWrite(silnikLewy_PRZOD, LOW);
  digitalWrite(silnikLewy_TYL, LOW);

  digitalWrite(silnikPrawy_PRZOD, LOW);
  digitalWrite(silnikPrawy_TYL, LOW);
}


void setup() {
  Serial.begin(9600);

  set_pin_modes();
  stopEng();
}


int PowerForPWM(int moc) {
  moc = constrain(moc, 0, 100);
  int pwm = map(moc, 0, 100, 0, PWM_MAX);

  return pwm;
}


void LeftEngForward(int moc) {
  int pwm = PowerForPWM(moc);

  digitalWrite(silnikLewy_PRZOD, HIGH);
  digitalWrite(silnikLewy_TYL, LOW);

  analogWrite(silnikLewy_PWM, pwm);
}


void RightEngForward(int moc) {
  int pwm = PowerForPWM(moc);

  digitalWrite(silnikPrawy_PRZOD, HIGH);
  digitalWrite(silnikPrawy_TYL, LOW);

  analogWrite(silnikPrawy_PWM, pwm);
}


void BothEngForward(int moc) {
  RightEngForward(moc);
  LeftEngForward(moc);
}


void LeftEngBackward(int moc) {
  int pwm = PowerForPWM(moc);

  digitalWrite(silnikLewy_PRZOD, LOW);
  digitalWrite(silnikLewy_TYL, HIGH);

  analogWrite(silnikLewy_PWM, pwm);
}


void RightEngBackward(int moc) {
  int pwm = PowerForPWM(moc);

  digitalWrite(silnikPrawy_PRZOD, LOW);
  digitalWrite(silnikPrawy_TYL, HIGH);

  analogWrite(silnikPrawy_PWM, pwm);
}


void TurnLeft(int moc) {
  LeftEngBackward(moc);
  RightEngForward(moc);
}


void TurnRight(int moc) {
  LeftEngForward(moc);
  RightEngBackward(moc);
}


int zmierzOdleglosc() {
  long czasTrwaniaSygnalu;
  int odleglosc;

  digitalWrite(sensorNadajnik, LOW);
  delayMicroseconds(2);

  digitalWrite(sensorNadajnik, HIGH);
  delayMicroseconds(10);

  digitalWrite(sensorNadajnik, LOW);

  czasTrwaniaSygnalu = pulseIn(sensorOdbiornik, HIGH);
  odleglosc = czasTrwaniaSygnalu / 58;

  return odleglosc;
}


void loop() {
  int odleglosc = zmierzOdleglosc();

  Serial.print(odleglosc);
  Serial.println(" cm");

  if (odleglosc >= 3 && odleglosc <= 5) {
    BothEngForward(60);
  }
  else if (odleglosc > 5) {
    TurnLeft(60);
  }
  else {
    TurnRight(60);
  }

  delay(100);
}