
#include <Wire.h>
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#define HOME 90 //home poistion for servo

#define LiftFan 6  // PWM pin for lift fan
#define ThrustFan 5 // PWM pin for thrust fan

#define TRIG_PIN PB3
#define ECHO_PIN PD2
#define TRIG2_PIN PB5
#define ECHO2_PIN PD3
#define IR_PIN PC0
//global variables
volatile uint16_t pulse_length1 = 0, pulse_length2 = 0;
volatile uint8_t echo_received1 = 0, echo_received2 = 0;
uint8_t fans_on = 1;
 char buffer[10];

#define SOUND_SPEED_DIV 58
#define THRESHOLD_1 12
#define THRESHOLD_2 46
#define ERROR_MARGIN 2

#define OFF 0 //for fan speed
#define MAX 255

const int MPU6050 = 0x68;//address for MPU communication register
float yawAngle = 0.0;//initialized to 0
float position;//servo position variable 
unsigned long prevTime = 0;//previous time

//adjustable vairables and functions
float c = 30; //us bias
float gyroBiasZaxis = 0.1;//gyroscope bias
int k=1; //turning speed constant

//  Initialize Fans :TURNS ON LIFT FAN ONLY
void fan_init() {
    // Set PD5 as output for Fan 1 (PWM) (THRUST FAN)
    DDRD |= (1 << PD5); 
  // Set PD6 as output for Fan 2 (On/off) (LIFT FAN)
    DDRD |= (1 << PD6);
    // Configure Timer0 for PWM on PD6 (Fast PWM Mode)
    TCCR0A |= (1 << COM0B1) | (1 << WGM00) | (1 << WGM01); // Fast PWM, non-inverting
    TCCR0B |= (1 << CS01); // Prescaler 8 -> ~31.25kHz PWM frequency
    // Turn on Fan 1 (PD5) (THRUST)
    OCR0B=0;  // Fan 1 OFF
    // Turn on Fan 2 (PD6) at max speed (LIFT)
    PORTD |= (1<<PD6);          // Fan 2 at max speed
}
void servo_init() { //servo initialization
    DDRB |= (1 << PB1);  // P9 pin 1 (OC1A) as output

    // Timer1, Mode 14, TOP = ICR1
    TCCR1A |= (1 << COM1A1) | (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11);  // Prescaler 8

    ICR1 = 40000;  // 50Hz (20ms period)
   setServoPosition(HOME);//set servo to home 
}

void imu_init(){//initialize communication
  Wire.begin();
  Wire.beginTransmission(MPU6050);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission();
}
// Set servo position by angle using custom pulse range ===
void setServoPosition(uint8_t angle) {
    // Map 0–180° to 500–2200 µs
    uint16_t us = 400+ ((uint32_t)angle * (2200 - 400)) / 180;
    OCR1A = us * 2;  // Convert µs to timer ticks (0.5µs resolution)
}

void setThrustFan(uint8_t speed){
  OCR0B=speed;
}
void setLiftFan(uint8_t speed){
  OCR0A=speed;
}

void turnLeft(float yawAngle) {
  position = 90-k*(180-yawAngle);
  position = constrain(position, 55, 125);//test edit angle restraints
  setServoPosition(position);
}

void turnRight(float yawAngle) {
  position = 90+k*(180-yawAngle);
  position = constrain(position, 55, 125);//test edit angle restraints
  setServoPosition(position);
}

void send_trigger_pulse(uint8_t pin) {
    PORTB |= (1 << pin);
    _delay_us(10);
    PORTB &= ~(1 << pin);
}
void uart_print_string(const char *str) {
    while (*str) uart_send_byte(*str++);
}
void uart_send_byte(uint8_t c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

uint16_t us_read(uint8_t trig_pin, volatile uint8_t *echo_received, volatile uint16_t *pulse_length) {
    *pulse_length = 0;
    *echo_received = 0;

    send_trigger_pulse(trig_pin);

    uint16_t timeout = 60000;
    while (!(*echo_received) && (--timeout));
    if (timeout == 0) return 0;

    return *pulse_length;
}

// Function to read raw gyroscope Z-axis data
int16_t readGyroZValue() {
    Wire.beginTransmission(MPU6050);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050, 2, true);

    int16_t gyroZ = (Wire.read() << 8) | Wire.read();
    return gyroZ;
}

// Function to calibrate gyroscope Z-axis bias
float calibrateGyroZ() {
    int numberSamples = 500;
    long sum = 0;
    for (int i = 0; i < numberSamples; i++) {
        sum += readGyroZValue();
        delay(3);
    }
    return (sum / (float)numberSamples) / 131.0; // Convert to dps
}
float get_pulse_time_ms(uint16_t pulse) {
    return ((float)pulse * 8.0) / 16000.0;
}
void adc_init() {
    ADMUX = (1 << REFS0);  // AVcc as reference, ADC0
    ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0);  // Enable ADC, prescaler 8
}
uint16_t read_ir() {
    ADMUX = (ADMUX & 0xF0) | (IR_PIN & 0x0F); // Select ADC channel
    ADCSRA |= (1 << ADSC);  // Start conversion
    while (ADCSRA & (1 << ADSC));
    return ADC;
}
 

ISR(INT0_vect) {
    if (PIND & (1 << ECHO_PIN)) {
        TCNT1 = 0;
        echo_received1 = 0;
    } else {
        pulse_length1 = TCNT1;
        echo_received1 = 1;
    }
}

void recalibratePositon(float yawAngle){
  position = 4*(90-yawAngle);
  setServoPosition(position);
}

void recalibrate2Position(float usSensorRight, float usSensorLeft){
  float calibrate = usSensorRight-usSensorLeft;
  position = 90-1*(calibrate);
  setServoPosition(position);
}



void setup() {
 fan_init();//only turns on lift fan
  servo_init();//servo initialization
  imu_init();//initializses communcation for imu
    gyroBiasZaxis = calibrateGyroZ();// Gyro calibration: measure bias at startup
    // Setup US Sensor 1
    DDRB |= (1 << TRIG_PIN);
    DDRD &= ~(1 << ECHO_PIN);
    EIMSK |= (1 << INT0);
    EICRA |= (1 << ISC00);

    // Setup US Sensor 2
    DDRB |= (1 << TRIG2_PIN);
    DDRD &= ~(1 << ECHO2_PIN);
    EIMSK |= (1 << INT1);
    EICRA |= (1 << ISC10);

     TCCR1B |= (1 << CS11); // Timer1
    
}

void loop() {
   
    uint16_t ir_val = read_ir();
     if (ir_val > 512 && fans_on) { // IR obstacle detected
            uart_print_string("IR obstacle detected! Fans OFF.\n");
            setLiftFan(0);
            setThrustFan(0);
            fans_on = 0;
        } else if (ir_val <= 512 && !fans_on) {
            uart_print_string("IR clear. Waiting 5s to restore fans...\n");
            _delay_ms(5000);
            setLiftFan(1);
            setThrustFan(MAX);
            fans_on = 1;
        }

  
        // Ultrasonic 1
        uint16_t pulse1 = us_read(TRIG_PIN, &echo_received1, &pulse_length1);
        uint16_t dist1 = (pulse1 / 2) / SOUND_SPEED_DIV;
        float t1 = get_pulse_time_ms(pulse1);

        uart_print_string("US1 P (ms): ");
        dtostrf(t1, 6, 3, buffer);
        uart_print_string(buffer);
        uart_print_string(" | D (cm): ");
        sprintf(buffer, "%u", dist1);
        uart_print_string(buffer);
        uart_print_string("\n");

        // Ultrasonic 2
        uint16_t pulse2 = us_read(TRIG2_PIN, &echo_received2, &pulse_length2);
        uint16_t dist2 = (pulse2 / 2) / SOUND_SPEED_DIV;
        float t2 = get_pulse_time_ms(pulse2);

        uart_print_string("US2 P (ms): ");
        dtostrf(t2, 6, 3, buffer);
        uart_print_string(buffer);
        uart_print_string(" | D (cm): ");
        sprintf(buffer, "%u", dist2);
        uart_print_string(buffer);
        uart_print_string("\n");
    
   setThrustFan(MAX);// Set thrust fan speed (0-255)
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;

    int16_t gyroZ = readGyroZValue();
    float gyroZ_dps = (gyroZ / 131.0) - gyroBiasZaxis; // Subtract bias

    yawAngle += gyroZ_dps * dt; // Integrate to get yaw
  

   if (dist1 > dist2+c){
    setLiftFan(OFF);   // Set lift fan speed (0-255)
    setThrustFan(OFF);
    delay(1000); //delay to slow down entry velocity
    setLiftFan(MAX);   // Set lift fan speed (0-255)
    setThrustFan(175); //set turning velocity 
      while(abs(yawAngle) <180){ //breakout when 180 degree turn executed
        unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;
        int16_t gyroZ = readGyroZValue();
        float gyroZ_dps = (gyroZ / 131.0) - gyroBiasZaxis; // Subtract bias

        yawAngle += gyroZ_dps * dt; // Integrate to get yaw

        Serial.print("Yaw Angle: ");
        Serial.println(yawAngle);
        turnRight(yawAngle);
      }
      yawAngle=0;
    analogWrite(LiftFan, OFF);   // Set lift fan speed (0-255)
    analogWrite(ThrustFan, OFF);
    delay(1000);
    analogWrite(LiftFan, MAX);   // Set lift fan speed (0-255)
    analogWrite(ThrustFan, MAX);
     
   }

    if (dist2 > dist1+c){ //turn left
       setLiftFan(OFF);   // Set lift fan speed (0-255)
   setThrustFan(OFF);
      delay(1000);
       setLiftFan(MAX);   // Set lift fan speed (0-255)
    setThrustFan(175); //set turning velocity 
      while(abs(yawAngle) <180){
        unsigned long currentTime = millis();
       float dt = (currentTime - prevTime) / 1000.0;
        prevTime = currentTime;
        int16_t gyroZ = readGyroZValue();
        float gyroZ_dps = (gyroZ / 131.0) - gyroBiasZaxis; // Subtract bias
        yawAngle += gyroZ_dps * dt; // Integrate to get yaw
        turnLeft(yawAngle);
      }
      yawAngle=0; //breakout turn loop
    setLiftFan(OFF);   // Set lift fan speed (0-255)
   setThrustFan(OFF);
    delay(1000);
    setLiftFan(MAX);   // Set lift fan speed (0-255)
   setThrustFan(MAX);
    }
  
  position = (2*yawAngle)+85;
setServoPosition(position);

}



