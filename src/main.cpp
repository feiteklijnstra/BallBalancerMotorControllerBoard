/*
MKS ESP32 FOC Closed Loop Position Control Example; Test Library：SimpleFOC 2.1.1; Test Hardware：MKS ESP32 FOC V1.0
Enter "T+Position" in the serial monitor to make the two motors rotate in closed loop
For example, input the radian system "T3.14" to let the two motors rotate 180°
When using your own motor, do remember to modify the default number of pole pairs, the value in BLDCMotor()
The default power supply voltage set by the program is 12V
Please remember to modify the values in voltage_power_supply and voltage_limit variables if you use other voltages for power supply
The motor targeted by the default PID is the YT2804 motor. To use your own motor.
You need to modify the PID parameters to achieve better results.
*/

#include <SimpleFOC.h>

#include <esp_now.h>
#include <WiFi.h>

// angle set point variable
float targetAngle1 = 0.0;
float targetAngle2 = 0.0;

// Structure to receive data from controller
typedef struct messageFromController_t
{
  float targetAngle1;
  float targetAngle2;
  bool enable;
} messageFromController_t;

// Create a messageFromController_t called dataFromController
messageFromController_t dataFromController;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&dataFromController, incomingData, sizeof(dataFromController));
  targetAngle1 = dataFromController.targetAngle1;
  targetAngle2 = dataFromController.targetAngle2;
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.println();
}

// MAC: 08:3a:8d:bb:72:48

#define SENSOR1_CS 5  //
#define SENSOR2_CS 15 //

// motor instance
BLDCMotor motor1 = BLDCMotor(7);
BLDCMotor motor2 = BLDCMotor(7);
// driver instance
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32, 33, 25, 22);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(26, 27, 14, 12);

// Magnetic sensor instance
MagneticSensorSPI sensor1 = MagneticSensorSPI(AS5048_SPI, SENSOR1_CS);
MagneticSensorSPI sensor2 = MagneticSensorSPI(AS5048_SPI, SENSOR2_CS);

// Zero offset of motors
const float zeroAngleMotor1 = 0.0;
const float zeroAngleMotor2 = 0.0;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char *cmd) { command.scalar(&targetAngle1, cmd); }

void setup()
{
  // use monitoring with serial
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_MODE_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  // initialize magnetic sensor hardware
  sensor1.init();
  sensor2.init();
  // link the motor to the sensor
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);

  // driver config
  driver1.init();
  driver2.init();
  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);

  // set motion control loop to be used
  motor1.controller = MotionControlType::angle;
  motor2.controller = MotionControlType::angle;

  // controller configuration
  // default parameters in defaults.h

  // controller configuration based on the control type
  // velocity PID controller parameters
  // default P=0.5 I = 10 D =0
  motor1.PID_velocity.P = 0.2;
  motor1.PID_velocity.I = 20;
  motor1.PID_velocity.D = 0.001;
  motor2.PID_velocity.P = 0.2;
  motor2.PID_velocity.I = 20;
  motor2.PID_velocity.D = 0.001;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor1.PID_velocity.output_ramp = 1000;
  motor2.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best.
  // the lower the less filtered
  motor1.LPF_velocity.Tf = 0.01;
  motor2.LPF_velocity.Tf = 0.01;

  // angle P controller
  motor1.P_angle.P = 20;
  motor2.P_angle.P = 20;

  // since the phase resistance is provided we set the current limit not voltage
  // default 0.2
  motor1.current_limit = 0.2; // Amps
  motor2.current_limit = 0.2; // Amps

  motor1.sensor_direction = CW;
  motor2.sensor_direction = CW;

  motor1.zero_electric_angle = 0.76;
  motor2.zero_electric_angle = 2.53;

  // comment out if not needed
  motor2.useMonitoring(Serial);

  // initialize motor
  motor1.init();
  motor2.init();
  // align sensor and start FOC
  motor1.initFOC();
  motor2.initFOC();

  // add target command T
  command.add('T', doTarget, "target angle");

  Serial.println("Motor ready.");
  _delay(1000);
}

void loop()
{
  // main FOC algorithm function
  motor1.loopFOC();
  motor2.loopFOC();

  // Motion control function
  motor1.move(targetAngle1 + zeroAngleMotor1);
  motor2.move(targetAngle2 + zeroAngleMotor2);
  // motor1.move(sensor2.getSensorAngle());

  command.run();
}