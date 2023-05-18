/*******************************************************************************
 * File name: main.cpp
 *
 * Description: This is a program implementing the PID algorithm in the position 
 * and speed control application of a DC motor.
 * 
 * This is main function in application layer.
 *
 * Author: Tran Quang Dieu, Nguyen Huy Hoang, Vo Van Ling, Mai Huynh Tuan Vu
 *
 * Last Changed By:  $Author: HoangNH $
 * Revision:         $Revision: 1.0$
 * Last Changed:     $Date: $May 18, 2023
 *
 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <TimerOne.h>
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define CHAN_NGAT         2   // Motor_Ch1
#define CHAN_DOC_ENCODER  4   // Motor_Ch2
#define CHAN_PWM          3   // L298N_ENA
#define CHAN_DIR1         7   // L298N_IN1
#define CHAN_DIR2         8   // L298N_IN2
/**
 * @brief enum define STATE of the application
 */
typedef enum
{
  STATE_APP_POSITION,
  STATE_APP_SPEED,
} STATE_APP_t;

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
double T, pulse;
double pos, pos_Set;
double speed, speed_Set;
double E, E1, E2;
double alpha, beta, gamma, Kp, Kd, Ki;
double Output, LastOutput;
unsigned long time = 0;
bool PID_flag = 0;
STATE_APP_t appState = STATE_APP_POSITION; // set init state
/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
void IRQ_Position_PID();
void IRQ_Speed_PID();
void IRQ_Pulse_Counter_Encoder();
void Speed_PID_Feedback();
void Position_PID_Feedback();
/******************************************************************************/

/**
 * @brief This is the setup function called first when the program starts to
 * set up needed GPIO pin, configure Interrupt for Encoder (for count pulses)
 * and Timer1 (for PID feedback).
 */
void setup()
{
  pinMode(CHAN_NGAT, INPUT_PULLUP);
  pinMode(CHAN_DOC_ENCODER, INPUT_PULLUP);
  pinMode(CHAN_PWM, OUTPUT);
  pinMode(CHAN_DIR1, OUTPUT);
  pinMode(CHAN_DIR2, OUTPUT);

  speed_Set = 30;
  speed = 0;

  pos_Set = 180;
  pos = 0;

  E = 0;
  E1 = 0;
  E2 = 0; // E = e(t), E1 = de(t)/dt, E2 = dde(t)/dt
  Output = 0;
  LastOutput = 0;

  // Thong so PID vi tri
  T = 0.01;    // Thoi gian lay mau: 10ms
  Kp = 15;     
  Kd = 1.5;    
  Ki = 0.0001; 
  
  Serial.begin(74880);
  attachInterrupt(0, IRQ_Pulse_Counter_Encoder, FALLING);
  Timer1.initialize(10000);    // Khoi tao Timer 1 ngat moi 10 ms
  Timer1.attachInterrupt(IRQ_Position_PID); // Set PID la ham callback khi timer 1 elapsed
}

void loop()
{
  // Nếu có data được nhập từ Serial
  if (Serial.available())
  {
    String input = Serial.readString(); // đọc chuỗi nhập vào từ Serial

    if (input == "speed\n")
    {
      appState = STATE_APP_SPEED;
      Kp = 61.5;  
      Ki = 0.0001;
      Kd = 0.2;   
      Timer1.attachInterrupt(IRQ_Speed_PID);
    }
    else if (input == "position\n")
    {
      appState = STATE_APP_POSITION;
      Kp = 15;
      Kd = 1.5;    
      Ki = 0.0001; 
      Timer1.attachInterrupt(IRQ_Position_PID); 
    }
    else 
    {
      // Kiểm tra nếu input là một số thực (float/double) hợp lệ
      char *endptr = NULL;
      double input_setpoint = strtod(input.c_str(), &endptr);
      if (*endptr == '\0')
      { // Nếu không phải số hợp lệ
        Serial.println("Invalid input!");
      }
      else if (appState == STATE_APP_SPEED)
      {
        speed_Set = input_setpoint;
      }
      else if (appState == STATE_APP_POSITION)
      {
        pos_Set = input_setpoint;
      }
    }
  }

  // Xử lý feedback cho ngắt từ PID
  if(PID_flag)
  {
    PID_flag = 0;

    switch (appState)
    {
      case STATE_APP_POSITION:
        Position_PID_Feedback();
        Serial.print("Position: ");
        Serial.println(pos);
        break;
      
      case STATE_APP_SPEED:
        Speed_PID_Feedback();
        Serial.print(time);
        Serial.print(" ");
        Serial.print(speed);
        Serial.println(";");
        break;

      default:
        break;
    }
  }
}

/**
 * @brief ISR for Encoder to update the number of pulse
 */
void IRQ_Pulse_Counter_Encoder()
{
  if (digitalRead(CHAN_DOC_ENCODER) == LOW)
    pulse++;
  else
    pulse--;
}

/**
 * @brief Speed PID Feedback
 */
void Speed_PID_Feedback()
{
  E = speed_Set - speed; // Tính sai số hiện tại
  alpha = 2 * T * Kp + Ki * T * T + 2 * Kd; // Là phần tử điều chỉnh P trong công thức tính đầu ra của PID
  beta = T * T * Ki - 4 * Kd - 2 * T * Kp;  // Là phần tử điều chỉnh I trong công thức tính đầu ra của PID.
  gamma = 2 * Kd;                           // Là phần tử điều chỉnh D trong công thức tính đầu ra của PID
  Output = (alpha * E + beta * E1 + gamma * E2 + 2 * T * LastOutput) / (2 * T); // Tính đầu ra của bộ điều khiển PID bằng cách kết hợp các phần tử P, I, và D
  LastOutput = Output;  

  E2 = E1;
  E1 = E;

  if (Output > 255)
    Output = 255;
  else if (Output < 0)
    Output = 0;

  if (Output > 0)
  {
    analogWrite(CHAN_PWM, Output);
    digitalWrite(CHAN_DIR1, HIGH);
    digitalWrite(CHAN_DIR2, LOW);
  }
  else
  {
    analogWrite(CHAN_PWM, 0);
    digitalWrite(CHAN_DIR1, LOW);
    digitalWrite(CHAN_DIR2, LOW);
  }
}

/**
 * @brief Position PID Feedback
 */
void Position_PID_Feedback()
{
  E = pos_Set - pos;
  alpha = 2 * T * Kp + Ki * T * T + 2 * Kd;
  beta = T * T * Ki - 4 * Kd - 2 * T * Kp;
  gamma = 2 * Kd;
  Output = (alpha * E + beta * E1 + gamma * E2 + 2 * T * LastOutput) / (2 * T);
  LastOutput = Output;

  E2 = E1;
  E1 = E;

  if (Output > 255)
    Output = 255;
  else if (Output < -255)
    Output = -255;

  if (Output > 0)
  {
    analogWrite(CHAN_PWM, Output);
    digitalWrite(CHAN_DIR1, HIGH);
    digitalWrite(CHAN_DIR2, LOW);
  }
  else if (Output < 0)
  {
    analogWrite(CHAN_PWM, abs(Output));
    digitalWrite(CHAN_DIR1, LOW);
    digitalWrite(CHAN_DIR2, HIGH);
  }
  else
  {
    analogWrite(CHAN_PWM, 0);
    digitalWrite(CHAN_DIR1, LOW);
    digitalWrite(CHAN_DIR2, LOW);
  }
}

/**
 * @brief Interrupt Service Routine of Speed_PID, 
 * update current velocity, reset encoder pulse and get current time.
 */
void IRQ_Speed_PID()
{
  speed = (pulse / 1848) * (1.0 / T) * 60.0; // 1848 = 11.0 * 168.0 = encoder pulses * ti so truyen
  pulse = 0;
  PID_flag = 1;
  time = millis();
}

/**
 * @brief Interrupt Service Routine for Position_PID
 *        update the position and set flag to handler then.
 */
void IRQ_Position_PID()
{
  pos = ((pulse * 360) / 1848); // 11 * 168, 168 tỉ số truyền
  PID_flag = 1;
}
