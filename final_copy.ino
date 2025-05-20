#include <Arduino.h>
#include <AccelStepper.h>

// Định nghĩa các công tắc hành trình
#define Endstop1 2
#define Endstop2 3
#define Endstop3 4
#define Endstop4 5
#define Endstop5 6
#define Endstop6 7

// Định nghĩa các chân cho các trục 1, 2, 3, 4, 5, 6
#define STEP1_PIN 22 //1
#define DIR1_PIN 24
#define ENABLE1_PIN 26

#define STEP2_PIN 28 //2
#define DIR2_PIN 30
#define ENABLE2_PIN 32

#define STEP3_PIN 34 //3
#define DIR3_PIN 36
#define ENABLE3_PIN 38

#define STEP4_PIN 40 //4
#define DIR4_PIN 42 
#define ENABLE4_PIN 44

#define STEP5_PIN 46 //5
#define DIR5_PIN 48
#define ENABLE5_PIN 50

#define STEP6_PIN 31 //6
#define DIR6_PIN 33
#define ENABLE6_PIN 35

// Tạo đối tượng động cơ bước
AccelStepper Step1(1, STEP1_PIN, DIR1_PIN);
AccelStepper Step2(1, STEP2_PIN, DIR2_PIN);
AccelStepper Step3(1, STEP3_PIN, DIR3_PIN);
AccelStepper Step4(1, STEP4_PIN, DIR4_PIN);
AccelStepper Step5(1, STEP5_PIN, DIR5_PIN);
AccelStepper Step6(1, STEP6_PIN, DIR6_PIN);

// Tỷ lệ xung cho mỗi độ của từng động cơ
const float PULSE_PER_DEGREE_STEP1 = 80 / 3;
const float PULSE_PER_DEGREE_STEP2 = 80;
const float PULSE_PER_DEGREE_STEP3 = 80;
const float PULSE_PER_DEGREE_STEP4 = 140 / 9;
const float PULSE_PER_DEGREE_STEP5 = 40 / 3;
const float PULSE_PER_DEGREE_STEP6 = 20.0 / 9;


void setup() {

  Serial.begin(9600);

  // Cấu hình các công tắc hành trình
  pinMode(Endstop1, INPUT_PULLUP);
  pinMode(Endstop2, INPUT_PULLUP);
  pinMode(Endstop3, INPUT_PULLUP);
  pinMode(Endstop4, INPUT_PULLUP);
  pinMode(Endstop5, INPUT_PULLUP);
  pinMode(Endstop6, INPUT_PULLUP);

  // Cấu hình động cơ bước cho từng trục
  setupStepper(Step1, ENABLE1_PIN);
  setupStepper(Step2, ENABLE2_PIN);
  setupStepper(Step3, ENABLE3_PIN);
  setupStepper(Step4, ENABLE4_PIN);
  setupStepper(Step5, ENABLE5_PIN);
  setupStepper(Step6, ENABLE6_PIN);



  // sethome
  sethome();

  // Cấu hình chân điều khiển Relay
  // pinMode(RelayDC, OUTPUT);
  // pinMode(RelayValve, OUTPUT);
}

// Hàm thiết lập động cơ
void setupStepper(AccelStepper& motor, int enablePin) {
  motor.setMaxSpeed(1000);
  motor.setAcceleration(1000);
  motor.setEnablePin(enablePin);
  motor.setPinsInverted(false, false, true);  // Để chân ENABLE ở mức LOW khi kích hoạt
  motor.enableOutputs();
}

void home_step1() {
  int home1 = 0;
  while (digitalRead(Endstop1) == HIGH) {
    Step1.moveTo(home1);
    home1 += 1;
    Step1.run();
  }
  Serial.print("Step 1 homed at position: ");
  Serial.println(Step1.currentPosition());
  Step1.setCurrentPosition(2429);
  Step1.moveTo(0);
  while (Step1.distanceToGo() != 0) {
    Step1.run();
  }
}

void home_step2() {
  int home2 = 0;
  while (digitalRead(Endstop2) == HIGH) {
    Step2.moveTo(home2);
    home2 += 1;
    Step2.run();
  }
  Serial.print("Step 2 homed at position: ");
  Serial.println(Step2.currentPosition());
  Step2.setCurrentPosition(2028);
  Step2.moveTo(0);
  while (Step2.distanceToGo() != 0) {
    Step2.run();
  }
}

void home_step3() {
  int home3 = 0;
  while (digitalRead(Endstop3) == HIGH) {
    Step3.moveTo(home3);
    home3 = home3 - 1;
    Step3.run();
  }
  Serial.print("Step 3 homed at position: ");
  Serial.println(Step3.currentPosition());


  Step3.setCurrentPosition(-2019);
  Step3.moveTo(0);
  while (Step3.distanceToGo() != 0) {
    Step3.run();
  }
}

void home_step4() {
  int home4 = 0;
  while (digitalRead(Endstop4) == HIGH) {
    Step4.moveTo(home4);
    home4 += 1;
    Step4.run();
  }
  Serial.print("Step 4 homed at position: ");
  Serial.println(Step4.currentPosition());

   Step4.setCurrentPosition(1304);
  Step4.moveTo(0);
  while (Step4.distanceToGo() != 0) {
    Step4.run();
  }
}

void home_step5() {
  int home5 = 0;
  while (digitalRead(Endstop5) == HIGH) {
    Step5.moveTo(home5);
    home5 += 1;
    Step5.run();
  }
  Serial.print("Step 5 homed at position: ");
  Serial.println(Step5.currentPosition());

     Step5.setCurrentPosition(300);
  Step5.moveTo(0);
  while (Step5.distanceToGo() != 0) {
    Step5.run();
  }
}

void home_step6() {
  int home6 = 0;
  while (digitalRead(Endstop6) == HIGH) {
    Step6.moveTo(home6);
    home6 += 1;
    Step6.run();
  }
  Serial.print("Step 6 homed at position: ");
  Serial.println(Step6.currentPosition());

  
   Step6.setCurrentPosition(385);
  Step6.moveTo(0);
  while (Step6.distanceToGo() != 0) {
   Step6.run();
  }
}

void sethome(){
  home_step1();
  home_step2();
  home_step3();  
  home_step4();
  home_step5();
  home_step6();
}


void sethome23() {
  Step2.moveTo(500);
  Step3.moveTo(-2000);
  while (Step2.distanceToGo() != 0 || Step3.distanceToGo() != 0) {
    Step2.run();
    Step3.run();
  }
}


// Hàm tính toán ma trận xoay
void calculateRotationMatrix(float thetax, float R06[3][3]) {
  float rad = thetax * PI / 180.0;  // Chuyển đổi góc từ độ sang radian
  R06[0][0] = 1;
  R06[0][1] = 0;
  R06[0][2] = 0;
  R06[1][0] = 0;
  R06[1][1] = cos(rad);
  R06[1][2] = -sin(rad);
  R06[2][0] = 0;
  R06[2][1] = sin(rad);
  R06[2][2] = cos(rad);
}


// Hàm tính toán nghịch kinematics
void inverseKinematics(float Px_inv, float Py_inv, float Pz_inv, float thetax) {
  const float d1 = 206;
  const float a1 = 0;
  const float a2 = 320;
  const float a3 = 14;
  const float d4 = 327;
  const float d6 = 108;

  float R06[3][3];
  calculateRotationMatrix(thetax, R06);

  float Wx = Px_inv - d6 * R06[0][2];
  float Wy = Py_inv - d6 * R06[1][2];
  float Wz = Pz_inv - d6 * R06[2][2];

  float theta1 = atan2(Wy, Wx) * 180 / PI;

  float sqrtXY = sqrt(Wx * Wx + Wy * Wy);
  float numerator_phi = pow(sqrtXY - a1, 2) + pow(Wz - d1, 2) + a2 * a2 - (a3 * a3 + d4 * d4);
  float denominator_phi = 2 * a2 * sqrt(pow(sqrtXY - a1, 2) + pow(Wz - d1, 2));

  float cos_phi = constrain(numerator_phi / denominator_phi, -1, 1);
  float sin_phi = sqrt(1 - cos_phi * cos_phi);

  float phi1 = atan2(sin_phi, cos_phi) * 180 / PI;
  float phi2 = atan2(-sin_phi, cos_phi) * 180 / PI;

  float sigma = atan2(Wz - d1, sqrtXY - a1) * 180 / PI;
  float t2_inv1 = sigma - phi1;
  float t2_inv2 = sigma - phi2;

  float numerator_gamma = pow(sqrtXY - a1, 2) + pow(Wz - d1, 2) - (a2 * a2 + a3 * a3 + d4 * d4);
  float denominator_gamma = 2 * a2 * sqrt(a3 * a3 + d4 * d4);

  float cos_gamma = constrain(numerator_gamma / denominator_gamma, -1, 1);
  float sin_gamma = sqrt(1 - cos_gamma * cos_gamma);

  float gamma1 = atan2(sin_gamma, cos_gamma) * 180 / PI;
  float gamma2 = atan2(-sin_gamma, cos_gamma) * 180 / PI;

  float beta = atan2(d4, a3) * 180 / PI;
  float t3_inv1 = gamma1 + beta;
  float t3_inv2 = gamma2 + beta;

  float theta2 = t2_inv2;
  float theta3 = t3_inv2;

  float T03_inv[3][3] = {
    { cos((theta2 + theta3) * PI / 180) * cos(theta1 * PI / 180),
      cos((theta2 + theta3) * PI / 180) * sin(theta1 * PI / 180),
      sin((theta2 + theta3) * PI / 180) },
    { sin(theta1 * PI / 180), -cos(theta1 * PI / 180), 0 },
    { sin((theta2 + theta3) * PI / 180) * cos(theta1 * PI / 180),
      sin((theta2 + theta3) * PI / 180) * sin(theta1 * PI / 180),
      -cos((theta2 + theta3) * PI / 180) }
  };

  float R36[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R36[i][j] = 0;
      for (int k = 0; k < 3; k++) {
        R36[i][j] += T03_inv[i][k] * R06[k][j];
      }
    }
  }

  float r13 = R36[0][2], r23 = R36[1][2], r33 = R36[2][2];
  float r31 = R36[2][0], r32 = R36[2][1];

  float costheta5 = r33;
  float sintheta5 = sqrt(1 - costheta5 * costheta5);

  float theta5_1 = atan2(-sintheta5, costheta5) * 180 / PI;
  float theta5_2 = atan2(sintheta5, costheta5) * 180 / PI;

  float theta4_1 = atan2(-r23, -r13) * 180 / PI;
  float theta4_2 = atan2(r23, r13) * 180 / PI;

  float theta6_1 = atan2(r32, -r31) * 180 / PI;
  float theta6_2 = atan2(-r32, r31) * 180 / PI;
  float theta4, theta5, theta6;
  if (abs(theta4_1) < abs(theta4_2)) {
    theta4 = round(theta4_1 * 100) / 100.0;
    theta5 = round(theta5_1 * 100) / 100.0;
    theta6 = round(theta6_1 * 100) / 100.0;
  } else {
    theta4 = round(theta4_2 * 100) / 100.0;
    theta5 = round(theta5_2 * 100) / 100.0;
    theta6 = round(theta6_2 * 100) / 100.0;
  }


  Serial.println("Các nghiệm cuối cùng của theta:");
  Serial.print("Theta1: ");
  Serial.println(theta1);
  Serial.print("Theta2: ");
  Serial.println(theta2);
  Serial.print("Theta3: ");
  Serial.println(theta3);
  Serial.print("Theta4: ");
  Serial.println(theta4);
  Serial.print("Theta5: ");
  Serial.println(theta5);
  Serial.print("Theta6: ");
  Serial.println(theta6);
  Serial.println("-------------------------------");

  // Mảng pulses[] để chứa số xung tương ứng với các giá trị theta
  float pulses[7];  // Chú ý mảng bắt đầu từ index 1 (vì index 0 không được sử dụng)
  pulses[1] = theta1 * PULSE_PER_DEGREE_STEP1;
  pulses[2] = theta2 * PULSE_PER_DEGREE_STEP2 - 90 * PULSE_PER_DEGREE_STEP2;
  pulses[3] = theta3 * PULSE_PER_DEGREE_STEP3;
  pulses[4] = -theta4 * PULSE_PER_DEGREE_STEP4;
  pulses[5] = theta5 * PULSE_PER_DEGREE_STEP5 + 90 * PULSE_PER_DEGREE_STEP5;
  pulses[6] = -theta6 * PULSE_PER_DEGREE_STEP6;

  // Điều chỉnh tốc độ dựa trên số xung lớn nhất
  // adjustSpeedsBasedOnPulses(&pulses[1], 6);  // Truyền con trỏ mảng bắt đầu từ `pulses[1]
  // Di chuyển các động cơ đến vị trí tương ứng với góc
  Step1.moveTo(pulses[1]);
  Step2.moveTo(pulses[2]);
  Step3.moveTo(pulses[3]);
  Step4.moveTo(pulses[4]);
  Step5.moveTo(pulses[5]);
  Step6.moveTo(pulses[6]);

  while (Step1.distanceToGo() != 0 || Step2.distanceToGo() != 0 || Step3.distanceToGo() != 0 || Step4.distanceToGo() != 0 || Step5.distanceToGo() != 0 || Step6.distanceToGo() != 0) {
    Step1.run();
    Step2.run();
    Step3.run();
    Step4.run();
    Step5.run();
    Step6.run();
  }
}

void moveMotorsCafe(float parsedValues[]) {
  float pulses[7];  // Mảng lưu số xung cần chạy cho mỗi động cơ

  // Tính toán số xung
  pulses[1] = parsedValues[1] * PULSE_PER_DEGREE_STEP1;
  pulses[2] = parsedValues[2] * PULSE_PER_DEGREE_STEP2 - 90 * PULSE_PER_DEGREE_STEP2;
  pulses[3] = parsedValues[3] * PULSE_PER_DEGREE_STEP3;
  pulses[4] = -parsedValues[4] * PULSE_PER_DEGREE_STEP4;
  pulses[5] = parsedValues[5] * PULSE_PER_DEGREE_STEP5 + 0 * PULSE_PER_DEGREE_STEP5;
  pulses[6] = -parsedValues[6] * PULSE_PER_DEGREE_STEP6;

  // Di chuyển các động cơ đến vị trí tương ứng với góc
  Step1.moveTo(pulses[1]);
  Step2.moveTo(pulses[2]);
  Step3.moveTo(pulses[3]);
  Step4.moveTo(pulses[4]);
  Step5.moveTo(pulses[5]);
  Step6.moveTo(pulses[6]);

  // Chạy các động cơ đến vị trí đã tính toán
  while (Step1.distanceToGo() != 0 || Step2.distanceToGo() != 0 || Step3.distanceToGo() != 0 || Step4.distanceToGo() != 0 || Step5.distanceToGo() != 0 || Step6.distanceToGo() != 0) {
    Step1.run();
    Step2.run();
    Step3.run();
    Step4.run();
    Step5.run();
    Step6.run();
  }
}

void moveMotorsMGB6(float parsedValues[]) {
  float pulses[7];  // Mảng lưu số xung cần chạy cho mỗi động cơ

  // Tính toán số xung
  pulses[1] = parsedValues[1] * PULSE_PER_DEGREE_STEP1;
  pulses[2] = parsedValues[2] * PULSE_PER_DEGREE_STEP2 - 90 * PULSE_PER_DEGREE_STEP2;
  pulses[3] = parsedValues[3] * PULSE_PER_DEGREE_STEP3;
  pulses[4] = -parsedValues[4] * PULSE_PER_DEGREE_STEP4;
  pulses[5] = parsedValues[5] * PULSE_PER_DEGREE_STEP5 + 0 * PULSE_PER_DEGREE_STEP5;
  pulses[6] = -parsedValues[6] * PULSE_PER_DEGREE_STEP6;

  // Di chuyển các động cơ đến vị trí tương ứng với góc
  Step1.moveTo(pulses[1]);
  Step2.moveTo(pulses[2]);
  Step3.moveTo(pulses[3]);
  Step4.moveTo(pulses[4]);
  Step5.moveTo(pulses[5]);
  Step6.moveTo(pulses[6]);

  // Chạy các động cơ đến vị trí đã tính toán
  while (Step1.distanceToGo() != 0 || Step2.distanceToGo() != 0 || Step3.distanceToGo() != 0 || Step4.distanceToGo() != 0 || Step5.distanceToGo() != 0 || Step6.distanceToGo() != 0) {
    Step1.run();
    Step2.run();
    Step3.run();
    Step4.run();
    Step5.run();
    Step6.run();
  }

  long currentPos1 = Step1.currentPosition();
  Step1.moveTo(currentPos1 + 1000);
  while (Step1.distanceToGo() != 0) Step1.run();

  long currentPos3 = Step3.currentPosition();
  Step3.moveTo(currentPos3 + 1000);
  while (Step3.distanceToGo() != 0) Step3.run();

  Step1.moveTo(currentPos1);
  while (Step1.distanceToGo() != 0) Step1.run();
}

void moveMotorsParacetamol(float parsedValues[]) {
  float pulses[7];  // Mảng lưu số xung cần chạy cho mỗi động cơ

  // Tính toán số xung
  pulses[1] = parsedValues[1] * PULSE_PER_DEGREE_STEP1;
  pulses[2] = parsedValues[2] * PULSE_PER_DEGREE_STEP2 - 90 * PULSE_PER_DEGREE_STEP2;
  pulses[3] = parsedValues[3] * PULSE_PER_DEGREE_STEP3;
  pulses[4] = -parsedValues[4] * PULSE_PER_DEGREE_STEP4;
  pulses[5] = parsedValues[5] * PULSE_PER_DEGREE_STEP5 + 0 * PULSE_PER_DEGREE_STEP5;
  pulses[6] = -parsedValues[6] * PULSE_PER_DEGREE_STEP6;

  // Di chuyển các động cơ đến vị trí tương ứng với góc
  Step1.moveTo(pulses[1]);
  Step2.moveTo(pulses[2]);
  Step3.moveTo(pulses[3]);
  Step4.moveTo(pulses[4]);
  Step5.moveTo(pulses[5]);
  Step6.moveTo(pulses[6]);

  // Chạy các động cơ đến vị trí đã tính toán
  while (Step1.distanceToGo() != 0 || Step2.distanceToGo() != 0 || Step3.distanceToGo() != 0 || Step4.distanceToGo() != 0 || Step5.distanceToGo() != 0 || Step6.distanceToGo() != 0) {
    Step1.run();
    Step2.run();
    Step3.run();
    Step4.run();
    Step5.run();
    Step6.run();
  }

  long currentPos1 = Step1.currentPosition();
  Step1.moveTo(currentPos1 - 2000);
  while (Step1.distanceToGo() != 0) Step1.run();

  long currentPos3 = Step3.currentPosition();
  Step3.moveTo(currentPos3 + 1000);
  while (Step3.distanceToGo() != 0) Step3.run();

  Step1.moveTo(currentPos1);
  while (Step1.distanceToGo() != 0) Step1.run();

}

void loop() {
  if (Serial.available() > 0) {
    String receivedString = Serial.readStringUntil('\n');
    receivedString.trim();

    if (receivedString.startsWith("Start")) {
      receivedString = receivedString.substring(5);  // Loại bỏ "Start"
      String values[11];
      float parsedValues[11];

      // Tách các góc từ chuỗi dữ liệu
    int i = 0;
    char *token = strtok(const_cast<char *>(receivedString.c_str()), ",");
    while (token != NULL && i < 11) {
      values[i] = String(token);
      parsedValues[i] = atof(token);  // Lưu số vào mảng float
      token = strtok(NULL, ",");
      i++;
    }

    float x0 = parsedValues[7];
    float y0 = parsedValues[8];
    float z0 = parsedValues[9];


      // Xử lý theo từng loại dữ liệu
      if (values[0] == "beroca") {
        moveMotorsParacetamol(parsedValues);
        // inverseKinematics(x0, y0, z0, 180);
        // sethome23();
      } else if (values[0] == "probio") {
        moveMotorsMGB6(parsedValues);
        // inverseKinematics(x0, y0, z0, 180);
        sethome23();

      } else if (values[0] == "topralsin") {
        moveMotorsParacetamol(parsedValues);
        // inverseKinematics(x0, y0, z0, 180);
        sethome23();
      }
      Serial.println("Done");
    }
  }
}
 //100 ,300 , 400
