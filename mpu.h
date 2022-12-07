int16_t gayZH, gayZL, gayZ, accXH, accXL, accYH, accYL, accZH, accZL, accX, accY, accZ;
double gz_row, gz_of, gz, ax, ay, az, ax_of, ay_of, az_of;
double vt, vt_pr, ax_pr, ay_pr, dx, dx_pr;

void i2c_init()
{
  TWSR = 0x00;
  TWBR = 12;
  TWCR = (1 << TWEN) | (1 << TWINT);
}

void i2c_start()
{
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);
  while (!(TWCR & (1 << TWINT)));
}

void i2c_stop()
{
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

void i2c_trans(unsigned char a) {
  TWDR = a;
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
}

void transmit_multi(unsigned char a, unsigned char b) {
  i2c_start();
  i2c_trans(0b11010000);
  i2c_trans(a);
  i2c_trans(b);
  i2c_stop();
}

void setup_mpu() {
  transmit_multi(0x6B, 0x00);
  transmit_multi(0x1B, 0x10);
  transmit_multi(0x1C, 0x00);

}

int16_t receive(unsigned char x) {
  i2c_start();
  i2c_trans(0b11010000);
  i2c_trans(x);
  i2c_start();
  i2c_trans(0b11010001);
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
  int16_t h = TWDR;
  i2c_stop();
  return h;
}

void gayro() {
  gayZH = receive(0x47);
  gayZL = receive(0x48);
  gayZ = (gayZH << 8) | gayZL;
  gz_row = gayZ / 32.75;
}

void acc() {
  accXH = receive(0x3B);
  accXL = receive(0x3C);
  accYH = receive(0x3D);
  accYL = receive(0x3E);
  accZH = receive(0x3F);
  accZL = receive(0x40);
  accX = (accXH << 8) | accXL;
  accY = (accYH << 8) | accYL;
  accZ = (accZH << 8) | accZL;
  ax = accX / 16384.0;
  ay = accY / 16384.0;
}

void gayro_of() {
  int i = 0;
  while (i < 300) {
    gayro();
    gz_of += gz_row / 300.0;
    i++;
  }
}

void acc_of() {
  int i = 0;
  while (i < 300) {
    acc();
    ax_of += ax / 300.0;
    ay_of += ay / 300.0;
    i++;
  }
}

void set_degree(double time_diff) {
  if (int(gz_row - gz_of) == 0) {
    gz += (int(gz_row - gz_of)) * time_diff;
  }
  else {
    gz += (gz_row - gz_of) * time_diff;
  }
  ax = (ax - ax_of);
  ay = (ay - ay_of);
}

void mpu_setup() {
  i2c_init();
  setup_mpu();
}

double mpu_data(double time_diff) {
  gz_of = -4.2;
  gayro();
  set_degree(time_diff);
  return (gz);
}
