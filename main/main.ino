// 向左为正

#include <math.h>
#include <Servo.h>

const int A_PWM = 6, A_DIR = 7;  
const int B_PWM = 5, B_DIR = 4;
const int STEER = 9;
Servo steer;

const int Sensor[5] = {A0, A1, A2, A3, A4};

const double L = 0.175; // TODO: measure it
const double Width = 0.05; // TODO: measure it
const double V = 0.15; // target speed
const double WheelRadius = 0.035; // 3.5cm

double cur_theta = 0.0;

double bicycle_model(double theta) {
	return tan(theta) / L; // kappa
}

double bicycle_model_inv(double kappa) {
	return atan(kappa * L);
}


long trick_timer = -10000;
long q[205];
int pt = 0;
bool trick_flag = false;

bool trick() {
  if (millis() - trick_timer > 4000 / 200) {
    trick_timer = millis();
    
    if (trick_flag) {
      if (++pt == 200)
        pt = 0;
      
      q[pt] = millis();

      Serial.println("qwq");
    }
  }

  int cnt = 0;
  for (int i = 0; i < 200; i++)
    if (q[i] >= trick_timer - 4000)
      cnt++;
  
  // Serial.println(cnt);
  
  return cnt > 100;
}

// 简单起见 就把当前的 lateral error 乘一个系数之后作为调整的半径
// 这个系数应当 > 1，但也不可以超过太多，否则会矫枉过正
double adjust_kappa(double kappa, double lat_err) {
  // if (!lat_err)
  //   return kappa;

  // return 270 * lat_err;

  if (lat_err < -10000)
    return kappa;

  lat_err *= 100;
  if (abs(lat_err) < 0.5)
    return lat_err * 1.5;
  
  else if (abs(lat_err) < 1.0)
    return lat_err * 2.6;
  
  else if (abs(lat_err) < 1.8)
    return lat_err * 3.3;
  
  else
    return lat_err * 5.0;

  // double radius = 1.0 / lat_err;

  // const double K = 1.08;
  // radius *= K;

  // if (!kappa)
  //   return 1 / radius;
  // else {
  //   double tmp = 1 / kappa + radius;
  //   return 1 / tmp;
  // }

  // if (!lat_err)
  //   return 0;
  
  // return lat_err < 0 ? 2 : -2;
}

double calc_prob(double x) {
  const double base = exp(1);
  return (pow(base, x) - 1) / (base - 1);
}

long lat_err_dbg_timer = -1000;
double last_lat_err = 0.0;

double get_lat_err() {
  const double Gap = 0.012; // 1.2
  const double MaxVal = 1; // TODO: measure
  const int Upper[5] = {860, 1000, 1000, 995, 995};
  const int Lower[5] = {170, 350, 330, 365, 250};

  bool pr = (millis() - lat_err_dbg_timer) > 500;
  if (pr)
    lat_err_dbg_timer = millis();

  bool barbar = false;

  double s[5], p[5];
  for (int i = 0; i < 5; i++) {
    double val = analogRead(Sensor[i]);

    double bar = (Upper[i] * 1.8 + Lower[i] * 3.2) / 5.0;

    barbar |= (val <= bar);

    if (false && val > bar)
      s[i] = 0;
    else {
      s[i] = (double)(Upper[i] - val) / (Upper[i] - Lower[i]);
      if (s[i] < 0)
        s[i] = 0;
      else if (s[i] > 1)
        s[i] = 1;
    }
    p[i] = calc_prob(s[i]);

    if (pr) {
      Serial.print(s[i]);
      Serial.print(' ');
    }
  }
  if (pr) {
    Serial.println();
    for (int i = 0; i < 5; i++) {
      Serial.print(p[i]);
      Serial.print(' ');
    }
    Serial.println();
  }

  // 暂且简单取个加权平均试一下
  double pos_sum = 0.0, weight_sum = 0.0;
  for (int i = 0; i < 5; i++) {
    pos_sum += (i - 2) * Gap * p[i];
    weight_sum += p[i];
  }

  double avg = pos_sum / weight_sum;

  if (pr) {
    Serial.print("avg = ");
    Serial.print(avg * 100);
    Serial.println();
    Serial.println();
  }

  trick_flag = (avg * 100 < -1.5);

  if (/*trick() && */!barbar)
    return -20000;

  const int kMaxSpeed = 500, kMinSpeed = 400;

  double tmp = avg;
  if (tmp > 2 * Gap)
    tmp = 2 * Gap;
  else if (tmp < -2 * Gap)
    tmp = -2 * Gap;
  
  double rate = 1 - (tmp - (-2 * Gap)) / (4 * Gap);
  int leftSpeed = rate * kMaxSpeed + (1 - rate) * kMinSpeed;
  int rightSpeed = rate * kMinSpeed + (1 - rate) * kMaxSpeed;
  analogWrite(A_PWM, rightSpeed);
  analogWrite(B_PWM, leftSpeed);

  if (pr) {
    Serial.print("left ");
    Serial.print(leftSpeed);
    Serial.print("  right ");
    Serial.println(rightSpeed);
  }

  return avg; 

  // return weight_sum == 0 ? last_lat_err : (last_lat_err = avg);
}

double turning_ang = 0.0;

void turning(double kappa) {
	double theta = bicycle_model_inv(kappa);
	// theta -= system_error;

  theta *= 180 / acos(-1);
  theta += 79;

  // Serial.print("theta = ");
  // Serial.println(theta);

  int ang = (int)theta;

  // Serial.println(ang);

  double kRate = 0.983;
  turning_ang = (turning_ang * kRate + ang * (1 - kRate));

  steer.write(turning_ang);
}

void driving(double kappa) {
	double radius = 1 / kappa;

	double left_speed = V * (1 - Width / radius);
	double right_speed = V * (1 + Width / radius);

	// TODO: send left_speed and right_speed to the driving motors
}

void setup() {
  pinMode(A_DIR, OUTPUT);
  pinMode(A_PWM, OUTPUT);
  pinMode(B_DIR, OUTPUT);
  pinMode(B_PWM, OUTPUT);

  steer.attach(STEER);

	Serial.begin(9600);
	while (!Serial);

  digitalWrite(A_DIR, LOW);
  digitalWrite(B_DIR, LOW);

  analogWrite(A_PWM, 400);
  analogWrite(B_PWM, 400);

  // TODO: adjust the turning wheel

  get_lat_err();

  turning(0);
  // delay(3000);
}

double cur_kappa = 0.0;

void loop() {
  double lat_err = get_lat_err();
  // Serial.println(lat_err);
  cur_kappa = adjust_kappa(cur_kappa, lat_err);
  // Serial.print("cur kappa = ");
  // Serial.println(cur_kappa);

  // delay(1000);

  turning(cur_kappa);
}
