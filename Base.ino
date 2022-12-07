#include<avr/io.h>
#include "ps3.h"
#include "mpu.h"

double vx, vy, n1, n2, n3, w, v = 0.53;
double rpm_to_pwm_m1 = 24;
double rpm_to_pwm_m2 = 29.3;
double rpm_to_pwm_m3 = 22;
double gayro_z, gayro_pr = 0, a = 0;
int16_t x, y;
double error, pr_error, an;
double time_diff, times, time_prev;
double pid_p, pid_i, pid_d, PID, PID_pr = 0;
double kp = 2;
double ki = 0;
double kd = 0.08;

void timer_configration() {
  TCCR4A = 0xAA;
  TCCR4B = 0x19;
  ICR4 = 10240;
}

void port_configration() {
  DDRG = 0x20;
  DDRH = 0x78;
  DDRB = 0x10;
}

void angle(double d) {
  vx = -(v * sin(d));
  vy = v * cos(d);
  n1 = (-(2 / 3.0) * vx) * (1000);
  n2 = (((1 / 3.0) * vx) - ((1 / 1.73) * vy)) * (1000);
  n3 = (((1 / 3.0) * vx) + ((1 / 1.73) * vy)) * (1000);
  motor1();
  motor2();
  motor3();
}

void drift(double d, double r) {
  vx = -(0.45 * sin(d));
  vy = 0.45 * cos(d);
  w = 0.45 / (r * (2));
  n1 = ((-(2 / 3.0) * vx) + (1 / 3.0) * w) * (1000);
  n2 = (((1 / 3.0) * vx) - ((1 / 1.73) * vy) + (1 / 3.0) * w) * (1000);
  n3 = (((1 / 3.0) * vx) + ((1 / 1.73) * vy) + (1 / 3.0) * w) * (1000);
  motor1();
  motor2();
  motor3();
}

void lshape(double n, double ang) {
  vx = -(0.25 * sin(ang - (((gayro_z - gayro_pr) * PI) / 180.0)));
  vy = 0.25 * cos(ang - (((gayro_z - gayro_pr) * PI) / 180.0));
  w = 0.25 / (n * (2));
  n1 = ((-(2 / 3.0) * vx) + (1 / 3.0) * w) * (1000);
  n2 = (((1 / 3.0) * vx) - ((1 / 1.73) * vy) + (1 / 3.0) * w) * (1000);
  n3 = (((1 / 3.0) * vx) + ((1 / 1.73) * vy) + (1 / 3.0) * w) * (1000);
  motor1();
  motor2();
  motor3();
  a = 1;
}

void motor1() {
  if (n1 >= 0) {
    PORTG &= ~0x20;
    OCR4A = (int(n1 * rpm_to_pwm_m1));
  }
  else if (n1 < 0) {
    PORTG = 0x20;
    OCR4A = -(int(n1 * rpm_to_pwm_m1));
  }
}

void motor2() {
  if (n2 >= 0) {
    PORTH &= ~0x40;
    OCR4B = (int(n2 * rpm_to_pwm_m2));
  }
  else if (n2 < 0) {
    PORTH = 0x40;
    OCR4B = -(int(n2 * rpm_to_pwm_m2));
  }
}

void motor3() {
  if (n3 >= 0) {
    PORTB &= ~0x10;
    OCR4C = (int(n3 * rpm_to_pwm_m3));
  }
  else if (n3 < 0) {
    PORTB = 0x10;
    OCR4C = -(int(n3 * rpm_to_pwm_m3));
  }
}

void reset() {
  rpm_to_pwm_m1 = 24;
  rpm_to_pwm_m2 = 29.3;
  rpm_to_pwm_m3 = 22;
}

void clockwise() {
  PORTG = 0x20;
  PORTH = 0x40;
  PORTB = 0x10;
  OCR4A = 150 * rpm_to_pwm_m1;
  OCR4B = 150 * rpm_to_pwm_m2;
  OCR4C = 150 * rpm_to_pwm_m3;
}

void anticlockwise() {
  PORTG &= ~0x20;
  PORTH &= ~0x40;
  PORTB &= ~0x10;
  OCR4A = 150 * rpm_to_pwm_m1;
  OCR4B = 150 * rpm_to_pwm_m2;
  OCR4C = 150 * rpm_to_pwm_m3;
}


void stops() {
  OCR4A = 0;
  OCR4B = 0;
  OCR4C = 0;
}

void udlr() {
  if (ps3.up && ps3.tri) {
    drift(0, 1.0);
    a = 1;
  }
  else if (ps3.left && ps3.tri) {
    drift(1.57, 1.0);
    a = 1;
  }
  else if (ps3.down && ps3.tri) {
    drift(3.14, 1.0);
    a = 1;
  }
  else if (ps3.right && ps3.tri) {
    drift(3 * (1.57), 1.0);
    a = 1;
  }
  else if (ps3.up && ps3.square) {
    drift(0, -1.0);
    a = 1;
  }
  else if (ps3.left && ps3.square) {
    drift(1.57, -1.0);
    a = 1;
  }
  else if (ps3.down && ps3.square) {
    drift(3.14, -1.0);
    a = 1;
  }
  else if (ps3.right && ps3.square) {
    drift(3 * (1.57), - 1.0);
    a = 1;
  }
  else if (ps3.up && ps3.circle) {
    lshape(0.4, 0);
  }
  else if (ps3.left && ps3.circle) {
    lshape(0.4, PI/2);
  }
  else if (ps3.down && ps3.circle) {
    lshape(0.4, PI);
  }
  else if (ps3.right && ps3.circle) {
    lshape(0.4, PI*1.5);
  }
  else if (ps3.up) {
    setdata();
    angle(0);
  }
  else if (ps3.left) {
    setdata();
    angle(1.57);
  }
  else if (ps3.down) {
    setdata();
    angle(3.14);
  }
  else if (ps3.right) {
    setdata();
    angle(1.5 * 3.14);
  }
  else if (ps3.R1) {
    clockwise();
    a = 1;
  }
  else if (ps3.L1) {
    anticlockwise();
    a = 1;
  }
  else if (x == 127 && y == 127) {
    if (a == 0) {
      setdata_s();
    }
    else if (a == 1) {
      stops();
      delay(150);
      gayro_pr = gayro_z;
      a = 0;
    }
  }
  if (a == 1) {
    reset();
  }
}

void setdata() {
//  if (a == 1) {
//    gayro_pr = gayro_z;
//  }
//  a = 0;
//  //  reset();
//  pid();
//  if (PID > 0.5) {
//    if (n1 > 0) {
//      rpm_to_pwm_m1 -= (PID * 5);
//    }
//    else if (n1 < 0) {
//      rpm_to_pwm_m1 += (PID * 5);
//    }
//    if (n2 > 0) {
//      rpm_to_pwm_m2 -= (PID * 5);
//    }
//    else if (n2 < 0) {
//      rpm_to_pwm_m2 += (PID * 5);
//    }
//    if (n3 > 0) {
//      rpm_to_pwm_m3 -= (PID * 5);
//    }
//    else if (n3 < 0) {
//      rpm_to_pwm_m3 += (PID * 5);
//    }
//  }
//  if (PID < -0.5) {
//    if (n1 < 0) {
//      rpm_to_pwm_m1 += (PID * 5);
//    }
//    else if (n1 > 0) {
//      rpm_to_pwm_m1 -= (PID * 5);
//    }
//    if (n2 < 0) {
//      rpm_to_pwm_m2 += (PID * 5);
//    }
//    else if (n2 > 0) {
//      rpm_to_pwm_m2 -= (PID * 5);
//    }
//    if (n3 < 0) {
//      rpm_to_pwm_m3 += (PID * 5);
//    }
//    else if (n3 > 0) {
//      rpm_to_pwm_m3 -= (PID * 5);
//    }
//  }
//  if (rpm_to_pwm_m1 > 29) {
//    rpm_to_pwm_m1 = 29;
//  }
//  if (rpm_to_pwm_m2 > 29) {
//    rpm_to_pwm_m2 = 29;
//  }
//  if (rpm_to_pwm_m3 > 29) {
//    rpm_to_pwm_m3 = 29;
//  }
//  if (rpm_to_pwm_m1 < 10) {
//    rpm_to_pwm_m1 = 10;
//  }
//  if (rpm_to_pwm_m2 < 10) {
//    rpm_to_pwm_m2 = 10;
//  }
//  if (rpm_to_pwm_m3 < 10) {
//    rpm_to_pwm_m3 = 10;
//  }

  if (a == 1) {
    gayro_pr = gayro_z;
  }
  a = 0;
  pid();
  if (PID > 1.0) {
    PORTG =  0x20;
    PORTH =  0x40;
    PORTB =  0x10;
    OCR4A = int(150 * rpm_to_pwm_m1);
    OCR4B = int(150 * rpm_to_pwm_m2);
    OCR4C = int(150 * rpm_to_pwm_m3);
  }
  if (PID < -1.0) {
    PORTG &= ~ 0x20;
    PORTH &= ~ 0x40;
    PORTB &= ~ 0x10;
    OCR4A = int(150 * rpm_to_pwm_m1);
    OCR4B = int(150 * rpm_to_pwm_m2);
    OCR4C = int(150 * rpm_to_pwm_m3);
  }
  if (PID > -1.5 && PID < 1.5) {
    stops();
  }
}

void setdata_s() {
  pid();
  if (PID > 1.5) {
    PORTG =  0x20;
    PORTH =  0x40;
    PORTB =  0x10;
    OCR4A = int(250 * rpm_to_pwm_m1);
    OCR4B = int(250 * rpm_to_pwm_m2);
    OCR4C = int(250 * rpm_to_pwm_m3);
  }
  if (PID < -1.5) {
    PORTG &= ~ 0x20;
    PORTH &= ~ 0x40;
    PORTB &= ~ 0x10;
    OCR4A = int(250 * rpm_to_pwm_m1);
    OCR4B = int(250 * rpm_to_pwm_m2);
    OCR4C = int(250 * rpm_to_pwm_m3);
  }
  if (PID > -1.5 && PID < 1.5) {
    stops();
  }
}

void joystick() {
  x = ps3.LeftHatX;
  y = ps3.LeftHatY;
  an = PI + atan2(x - 127, y - 127);
  if (an > 6.27) {
    an = 0;
  }
  if (x == 127 && y == 127) {}
  else {
    setdata();
    angle(an);
  }
}

void calculate_time() {
  time_prev = times;
  times = millis();
  time_diff = (times - time_prev) / 1000;
}

void pid() {
  error = gayro_z - gayro_pr;
  pid_p = kp * error;
  pid_i = pid_i + (ki * error);
  pid_d = kd * ((error - pr_error) / time_diff);
  PID = pid_p + pid_i + pid_d;
  pr_error = error;
}

void setup() {
  port_configration();
  timer_configration();
  Serial.begin(115200);
  ps3_setup();
  times = millis();
  mpu_setup();
}

void loop() {
  ps3_data();
  calculate_time();
  gayro_z = mpu_data(time_diff);
//  Serial.println(gayro_z);
  udlr();
  joystick();
}
