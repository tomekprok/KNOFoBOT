#include <pins.h>

void set_pin_modes()
{
  pinMode(pins.leftEngPWM, OUTPUT);
  pinMode(pins.rightEngPWM, OUTPUT);

  pinMode(pins.leftEngFwd, OUTPUT);
  pinMode(pins.leftEngBack, OUTPUT);

  pinMode(pins.rightEngFwd, OUTPUT);
  pinMode(pins.rightEndBack, OUTPUT);

  pinMode(pins.sensorTrig, OUTPUT);
  pinMode(pins.sensorEcho, INPUT);
}

typedef struct
{
  const int pin_przod, pin_tyl;
  const int pin_pwm;
} silnik;

void stopEng(const silnik &s)
{
  // czy moze nie warto hamowac ustawiajac oba przod/tyl na wysoki i pwm na wysoki (fast stop w tabeli prawdy) ->
  //  -> do testów na stole, co warto ustawić na te dwa piny
  analogWrite(s.pin_pwm, 0);

  digitalWrite(s.pin_przod, HIGH);
  digitalWrite(s.pin_tyl, HIGH);
}

int PowerForPWM(float moc)
{
  moc = constrain(moc, 0, 100);
  int pwm = map(moc, 0, 100, 0, PWM_MAX);

  return pwm;
}

void set_eng_forward(const silnik &s, float moc)
{
  int pwm = PowerForPWM(moc);

  digitalWrite(s.pin_przod, HIGH);
  digitalWrite(s.pin_tyl, LOW);

  analogWrite(s.pin_pwm, pwm);
}

void set_eng_backward(const silnik &s, float moc)
{
  int pwm = PowerForPWM(moc);

  digitalWrite(s.pin_przod, LOW);
  digitalWrite(s.pin_tyl, HIGH);

  analogWrite(s.pin_pwm, pwm);
}

int zmierzOdleglosc()
{
  long czasTrwaniaSygnalu;
  int odleglosc;

  digitalWrite(pins.sensorTrig, LOW);
  delayMicroseconds(2);

  digitalWrite(pins.sensorTrig, HIGH);
  delayMicroseconds(10);

  digitalWrite(pins.sensorTrig, LOW);

  czasTrwaniaSygnalu = pulseIn(pins.sensorEcho, HIGH);
  odleglosc = czasTrwaniaSygnalu / 58; // magic number?

  return odleglosc;
}

void setup()
{
  Serial.begin(9600);

  set_pin_modes();
  silnik s_lewy = {pins.leftEngFwd, pins.leftEngBack, pins.leftEngPWM};
  silnik s_prawy = {pins.rightEngFwd, pins.leftEngBack, pins.rightEngPWM};
  stopEng(s_lewy);
  stopEng(s_prawy);
}

void loop()
{
  int odleglosc = zmierzOdleglosc();

  Serial.print(odleglosc);
  Serial.println(" cm");

  if (odleglosc >= 3 && odleglosc <= 5)
  {
    set_eng_forward(s_lewy, 60);
    set_eng_forward(s_prawy, 60);
  }
  else if (odleglosc > 5)
  {
    set_eng_backward(s_lewy, 60);
    set_eng_forward(s_prawy, 60);
  }
  else
  {
    set_eng_forward(s_lewy, 60);
    set_eng_backward(s_prawy, 60);
  }

  delay(100);
}