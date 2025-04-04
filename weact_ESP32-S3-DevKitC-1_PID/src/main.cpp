#include <U8g2lib.h>
#include <Adafruit_MAX31865.h>
#include <Adafruit_SHT4x.h>
#include <QuickPID.h>
#include <AiEsp32RotaryEncoder.h>

// 定义常量
const int CS_PIN = 10;
const int MOS_PIN = 14;  // GPIO14用于PWM输出
const int MIN_TEMPERATURE = 0;
const int MAX_TEMPERATURE = 100;
const float MAX_SAFE_TEMPERATURE = 120.0; // 最高安全温度限制
const unsigned long UPDATE_INTERVAL = 50; // 20Hz更新频率
const unsigned long DEBOUNCE_INTERVAL = 50;
const unsigned long DOUBLE_CLICK_INTERVAL = 300;
const unsigned long HEATING_CYCLE = 5000; // 5秒加热周期
const unsigned long SAFETY_CHECK_INTERVAL = 1000; // 安全检查间隔
const int MAX_ERROR_COUNT = 3; // 最大错误计数
const float TEMP_CHANGE_THRESHOLD = 5.0; // 温度变化阈值 (°C/s)

// PWM配置
const int PWM_CHANNEL = 0;    // 使用LEDC通道0
const int PWM_FREQ = 25000;   // PWM频率25kHz
const int PWM_RESOLUTION = 8; // 8位分辨率（0-255）

// OLED 显示屏定义
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 9, 8);

// MAX31865 传感器定义
Adafruit_MAX31865 max31865 = Adafruit_MAX31865(CS_PIN);

// SHT40 传感器定义
Adafruit_SHT4x sht40;

// PID 参数
float setpoint = 80.0;
float input = 0.0;
float output = 0.0;
float Kp = 20.0;
float Ki = 0.5;
float Kd = 50.0;
QuickPID myPID(&input, &output, &setpoint, Kp, Ki, Kd, QuickPID::Action::direct);

// PID调谐参数和限制
const float DEADBAND = 0.5;        // 死区范围 ±0.5°C
const float MAX_INTEGRAL = 100.0;  // 积分限幅
const float INTEGRAL_SEPARATE_THRESHOLD = 10.0; // 积分分离阈值
const float D_FILTER_ALPHA = 0.8;  // 微分项低通滤波系数
const int MIN_SAMPLE_TIME_MS = 50; // 最小采样时间(ms)

// PID控制相关变量
float lastError = 0.0;
float integral = 0.0;
float filteredDError = 0.0;
unsigned long lastPIDTime = 0;

// 加热状态
volatile bool startHeating = false;
bool errorState = false;
bool heatingOn = false;
unsigned long lastCycleStart = 0;

// 编码器定义
#define ENCODER_A 5
#define ENCODER_B 6
#define ENCODER_SW 4

AiEsp32RotaryEncoder encoder = AiEsp32RotaryEncoder(ENCODER_A, ENCODER_B, ENCODER_SW, -1);
unsigned long lastButtonPressTime = 0;
bool lastButtonState = HIGH;
bool waitingForSecondClick = false;

// 误差历史记录
const int ERROR_HISTORY_SIZE = 10;
float errorHistory[ERROR_HISTORY_SIZE] = {0};
int errorIndex = 0;

// 系统状态变量
struct SystemState {
    float lastTemperature = 0.0;
    float maxTemperature = 0.0;
    float ambientTemperature = 0.0;  // 环境温度
    float ambientHumidity = 0.0;     // 环境湿度
    unsigned long lastSafetyCheck = 0;
    int errorCount = 0;
    bool isEmergencyStop = false;
    unsigned long lastTempUpdate = 0;
    float tempChangeRate = 0.0;
} systemState;

// 温度滤波器
const int FILTER_SIZE = 5;
float tempReadings[FILTER_SIZE] = {0};
int filterIndex = 0;

// 温度曲线显示参数
struct TemperatureGraph {
    static const int GRAPH_WIDTH = 120;
    static const int GRAPH_HEIGHT = 40;
    static const int GRAPH_X = 0;
    static const int GRAPH_Y = 20;
    static const int POINT_COUNT = 20;  // 显示20个点
    static const int GRID_COUNT = 5;    // 网格线数量
    float minTemp = 0.0;
    float maxTemp = 100.0;
    float points[POINT_COUNT] = {0};
    int pointIndex = 0;
    float lastSmoothedTemp = 0.0;  // 用于平滑曲线
    const float SMOOTHING_FACTOR = 0.3;  // 平滑因子
} tempGraph;

// 温度预测参数
struct TemperaturePrediction {
    float currentTemp = 0.0;
    float trend = 0.0;
    float predictedTemp = 0.0;
    unsigned long predictionTime = 0;
    const unsigned long PREDICTION_INTERVAL = 5000; // 5秒预测间隔
} tempPred;

// 函数声明
void updateDisplay(float temperature, float pwm, bool isHeating);
void adjustPIDParameters();
void displayError(const char* message);
void performSafetyCheck(float temperature);
float getFilteredTemperature(float rawTemperature);
void emergencyStop();
void updateTemperatureGraph(float temperature);
void predictTemperature(float currentTemp, float trend);
void drawTemperatureGraph();

void IRAM_ATTR readEncoderISR() {
  encoder.readEncoder_ISR();
}

void setup() {
  Serial.begin(115200);
  u8g2.begin();

  if (!max31865.begin(MAX31865_3WIRE)) {
    displayError("MAX31865传感器初始化失败");
    errorState = true;
  }

  if (!sht40.begin()) {
    displayError("SHT40传感器初始化失败");
    errorState = true;
  }

  // 配置LEDC PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOS_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);  // 初始化为0输出

  encoder.begin();
  encoder.setup(readEncoderISR);
  encoder.setBoundaries(MIN_TEMPERATURE, MAX_TEMPERATURE, false);
  encoder.setAcceleration(0);  // 禁用加速度以确保每次步进为1

  myPID.SetMode(QuickPID::Control::automatic);
  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTimeUs(50000); // 50ms采样时间
}

void loop() {
  static unsigned long lastUpdate = 0;
  static unsigned long lastAmbientUpdate = 0;
  unsigned long now = millis();

  // 读取编码器位置
  long encoderValue = encoder.readEncoder();
  setpoint = constrain(encoderValue, MIN_TEMPERATURE, MAX_TEMPERATURE);

  // 处理按钮输入
  bool buttonState = digitalRead(ENCODER_SW);
  if (buttonState != lastButtonState) {
    if (buttonState == LOW && (now - lastButtonPressTime) > DEBOUNCE_INTERVAL) {
      if (waitingForSecondClick && (now - lastButtonPressTime) <= DOUBLE_CLICK_INTERVAL) {
        if (!systemState.isEmergencyStop) {
          startHeating = !startHeating;
        }
        waitingForSecondClick = false;
      } else {
        waitingForSecondClick = true;
        lastButtonPressTime = now;
      }
    }
    lastButtonState = buttonState;
  }

  if (waitingForSecondClick && (now - lastButtonPressTime) > DOUBLE_CLICK_INTERVAL) {
    waitingForSecondClick = false;
    if (!systemState.isEmergencyStop) {
      startHeating = true;
    }
  }

  if (now - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = now;
    
    // 读取并过滤温度
    float rawTemperature = max31865.temperature(100.0, 430.0);
    float temperature = getFilteredTemperature(rawTemperature);
    
    // 更新温度曲线
    updateTemperatureGraph(temperature);
    
    // 预测温度
    float trend = systemState.tempChangeRate;
    predictTemperature(temperature, trend);
    
    // 更新系统状态
    systemState.tempChangeRate = (temperature - systemState.lastTemperature) / 
        ((now - systemState.lastTempUpdate) / 1000.0);
    systemState.lastTemperature = temperature;
    systemState.lastTempUpdate = now;
    
    // 每2秒更新一次环境温湿度
    if (now - lastAmbientUpdate >= 2000) {
      lastAmbientUpdate = now;
      sensors_event_t humidity, temp;
      if (sht40.getEvent(&humidity, &temp)) {
        systemState.ambientTemperature = temp.temperature;
        systemState.ambientHumidity = humidity.relative_humidity;
      }
    }
    
    // 安全检查
    performSafetyCheck(temperature);
    
    if (!systemState.isEmergencyStop) {
      if (startHeating) {
        input = temperature;
        adjustPIDParameters();
        
        // 直接使用PWM输出，不需要周期控制
        ledcWrite(PWM_CHANNEL, (int)output);
        heatingOn = output > 0;
      } else {
        ledcWrite(PWM_CHANNEL, 0);
        output = 0;
        heatingOn = false;
        // 重置PID相关变量
        integral = 0;
        lastError = 0;
        filteredDError = 0;
      }
    } else {
      ledcWrite(PWM_CHANNEL, 0);
      output = 0;
      heatingOn = false;
      startHeating = false;
      // 重置PID相关变量
      integral = 0;
      lastError = 0;
      filteredDError = 0;
    }

    updateDisplay(temperature, output, startHeating);
  }
}

float getFilteredTemperature(float rawTemperature) {
  // 更新温度读数数组
  tempReadings[filterIndex] = rawTemperature;
  filterIndex = (filterIndex + 1) % FILTER_SIZE;
  
  // 计算平均值
  float sum = 0.0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += tempReadings[i];
  }
  return sum / FILTER_SIZE;
}

void performSafetyCheck(float temperature) {
  unsigned long now = millis();
  
  // 定期安全检查
  if (now - systemState.lastSafetyCheck >= SAFETY_CHECK_INTERVAL) {
    systemState.lastSafetyCheck = now;
    
    // 检查温度是否超过安全限制
    if (temperature > MAX_SAFE_TEMPERATURE) {
      systemState.errorCount++;
      if (systemState.errorCount >= MAX_ERROR_COUNT) {
        emergencyStop();
      }
    } else {
      systemState.errorCount = 0;
    }
    
    // 检查温度变化率
    if (abs(systemState.tempChangeRate) > TEMP_CHANGE_THRESHOLD) {
      systemState.errorCount++;
      if (systemState.errorCount >= MAX_ERROR_COUNT) {
        emergencyStop();
      }
    }
    
    // 更新最高温度记录
    if (temperature > systemState.maxTemperature) {
      systemState.maxTemperature = temperature;
    }
  }
}

void emergencyStop() {
  systemState.isEmergencyStop = true;
  ledcWrite(PWM_CHANNEL, 0);  // 使用PWM写入0而不是digitalWrite
  output = 0;
  heatingOn = false;
  startHeating = false;
  displayError("EMERGENCY STOP");
}

void adjustPIDParameters() {
  float error = setpoint - input;
  unsigned long now = millis();
  float deltaTime = (now - lastPIDTime) / 1000.0; // 转换为秒
  
  // 确保最小采样时间
  if (deltaTime < MIN_SAMPLE_TIME_MS / 1000.0) {
    return;
  }
  
  // 死区控制
  if (abs(error) <= DEADBAND) {
    integral = 0;
    output = 0;
    return;
  }
  
  // 计算微分项（带低通滤波）
  float dError = (error - lastError) / deltaTime;
  filteredDError = D_FILTER_ALPHA * filteredDError + (1 - D_FILTER_ALPHA) * dError;
  
  // 积分分离
  if (abs(error) < INTEGRAL_SEPARATE_THRESHOLD) {
    integral += error * deltaTime;
  }
  
  // 积分限幅
  integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);
  
  // 自适应PID参数调整
  if (abs(error) > 5.0) {
    // 大误差时，增加比例和微分作用，减小积分作用
    Kp = constrain(Kp * 1.1, 10.0, 30.0);
    Ki = constrain(Ki * 0.9, 0.1, 1.0);
    Kd = constrain(Kd * 1.1, 20.0, 60.0);
  } else if (abs(error) > 2.0) {
    // 中等误差时，平衡三项作用
    Kp = constrain(Kp, 15.0, 25.0);
    Ki = constrain(Ki, 0.3, 0.7);
    Kd = constrain(Kd, 30.0, 50.0);
  } else {
    // 小误差时，减小比例和微分作用，增加积分作用
    Kp = constrain(Kp * 0.9, 10.0, 30.0);
    Ki = constrain(Ki * 1.1, 0.1, 1.0);
    Kd = constrain(Kd * 0.9, 20.0, 60.0);
  }
  
  // 计算PID输出
  output = Kp * error + Ki * integral + Kd * filteredDError;
  output = constrain(output, 0, 255);
  
  // 更新历史值
  lastError = error;
  lastPIDTime = now;
  
  // 更新PID控制器参数
  myPID.SetTunings(Kp, Ki, Kd);
}

void updateTemperatureGraph(float temperature) {
    // 更新温度范围
    tempGraph.minTemp = min(tempGraph.minTemp, temperature);
    tempGraph.maxTemp = max(tempGraph.maxTemp, temperature);
    
    // 确保温度范围至少有10度的跨度
    if (tempGraph.maxTemp - tempGraph.minTemp < 10.0) {
        tempGraph.maxTemp = tempGraph.minTemp + 10.0;
    }
    
    // 更新温度点
    tempGraph.points[tempGraph.pointIndex] = temperature;
    tempGraph.pointIndex = (tempGraph.pointIndex + 1) % tempGraph.POINT_COUNT;
}

void predictTemperature(float currentTemp, float trend) {
    unsigned long now = millis();
    
    if (now - tempPred.predictionTime >= tempPred.PREDICTION_INTERVAL) {
        tempPred.predictionTime = now;
        tempPred.currentTemp = currentTemp;
        tempPred.trend = trend;
        
        // 预测5秒后的温度
        tempPred.predictedTemp = currentTemp + trend * 5.0;
    }
}

void drawTemperatureGraph() {
    // 绘制坐标轴
    u8g2.drawFrame(tempGraph.GRAPH_X, tempGraph.GRAPH_Y, 
                   tempGraph.GRAPH_WIDTH, tempGraph.GRAPH_HEIGHT);
    
    // 计算温度范围
    float tempRange = tempGraph.maxTemp - tempGraph.minTemp;
    
    // 绘制网格线
    for (int i = 0; i <= tempGraph.GRID_COUNT; i++) {
        // 水平网格线
        int y = tempGraph.GRAPH_Y + (i * tempGraph.GRAPH_HEIGHT) / tempGraph.GRID_COUNT;
        u8g2.drawHLine(tempGraph.GRAPH_X, y, tempGraph.GRAPH_WIDTH);
        
        // 垂直网格线
        int x = tempGraph.GRAPH_X + (i * tempGraph.GRAPH_WIDTH) / tempGraph.GRID_COUNT;
        u8g2.drawVLine(x, tempGraph.GRAPH_Y, tempGraph.GRAPH_HEIGHT);
        
        // 显示温度刻度
        float temp = tempGraph.maxTemp - (i * tempRange) / tempGraph.GRID_COUNT;
        char tempStr[5];
        sprintf(tempStr, "%.0f", temp);
        u8g2.setFont(u8g2_font_profont10_tf);
        u8g2.setCursor(tempGraph.GRAPH_X + tempGraph.GRAPH_WIDTH + 2, y - 2);
        u8g2.print(tempStr);
    }
    
    // 绘制设定温度线
    int setpointY = tempGraph.GRAPH_Y + tempGraph.GRAPH_HEIGHT - 
                   ((setpoint - tempGraph.minTemp) / tempRange * tempGraph.GRAPH_HEIGHT);
    u8g2.drawHLine(tempGraph.GRAPH_X, setpointY, tempGraph.GRAPH_WIDTH);
    
    // 绘制温度点和平滑曲线
    float prevSmoothedTemp = tempGraph.points[tempGraph.pointIndex];
    for (int i = 0; i < tempGraph.POINT_COUNT; i++) {
        int pointIndex = (tempGraph.pointIndex + i) % tempGraph.POINT_COUNT;
        float temp = tempGraph.points[pointIndex];
        
        // 应用平滑处理
        float smoothedTemp = tempGraph.lastSmoothedTemp * (1 - tempGraph.SMOOTHING_FACTOR) + 
                           temp * tempGraph.SMOOTHING_FACTOR;
        tempGraph.lastSmoothedTemp = smoothedTemp;
        
        // 计算点的Y坐标
        int y = tempGraph.GRAPH_Y + tempGraph.GRAPH_HEIGHT - 
                ((smoothedTemp - tempGraph.minTemp) / tempRange * tempGraph.GRAPH_HEIGHT);
        
        // 计算点的X坐标
        int x = tempGraph.GRAPH_X + (i * tempGraph.GRAPH_WIDTH) / tempGraph.POINT_COUNT;
        
        // 绘制点
        u8g2.drawPixel(x, y);
        
        // 如果有点，绘制平滑连线
        if (i > 0) {
            int prevY = tempGraph.GRAPH_Y + tempGraph.GRAPH_HEIGHT - 
                       ((prevSmoothedTemp - tempGraph.minTemp) / tempRange * tempGraph.GRAPH_HEIGHT);
            int prevX = tempGraph.GRAPH_X + ((i - 1) * tempGraph.GRAPH_WIDTH) / tempGraph.POINT_COUNT;
            
            // 使用Bresenham算法绘制平滑线
            int dx = abs(x - prevX);
            int dy = abs(y - prevY);
            int sx = (prevX < x) ? 1 : -1;
            int sy = (prevY < y) ? 1 : -1;
            int err = dx - dy;
            
            while (true) {
                u8g2.drawPixel(prevX, prevY);
                if (prevX == x && prevY == y) break;
                int e2 = 2 * err;
                if (e2 > -dy) { err -= dy; prevX += sx; }
                if (e2 < dx) { err += dx; prevY += sy; }
            }
        }
        
        prevSmoothedTemp = smoothedTemp;
    }
    
    // 绘制预测线（虚线）
    if (tempPred.trend != 0.0) {
        int startY = tempGraph.GRAPH_Y + tempGraph.GRAPH_HEIGHT - 
                    ((tempPred.currentTemp - tempGraph.minTemp) / tempRange * tempGraph.GRAPH_HEIGHT);
        int endY = tempGraph.GRAPH_Y + tempGraph.GRAPH_HEIGHT - 
                  ((tempPred.predictedTemp - tempGraph.minTemp) / tempRange * tempGraph.GRAPH_HEIGHT);
        
        // 绘制虚线
        int x = tempGraph.GRAPH_X + tempGraph.GRAPH_WIDTH - 20;
        int y = startY;
        int step = (endY - startY) / 10;
        for (int i = 0; i < 10; i++) {
            if (i % 2 == 0) {
                u8g2.drawLine(x, y, x + 2, y + step);
            }
            x += 2;
            y += step;
        }
    }
}

void updateDisplay(float temperature, float pwm, bool isHeating) {
  u8g2.clearBuffer();
  
  // 标题区域 (0-12像素)
  u8g2.drawFrame(0, 0, 120, 12);
  u8g2.setFont(u8g2_font_profont12_tf);
  u8g2.setCursor(4, 10);
  u8g2.print(isHeating ? "HEATING" : "STANDBY");
  
  // 温度曲线区域 (12-52像素)
  drawTemperatureGraph();
  
  // 当前温度显示 (52-64像素)
  u8g2.setFont(u8g2_font_profont22_tf);
  char tempStr[6];
  sprintf(tempStr, "%.1f", temperature);
  int tempWidth = u8g2.getStrWidth(tempStr);
  u8g2.setCursor((120 - tempWidth) / 2, 60);
  u8g2.print(tempStr);
  u8g2.setFont(u8g2_font_profont12_tf);
  u8g2.print("C");
  
  // 预测温度显示
  if (tempPred.trend != 0.0) {
    u8g2.setFont(u8g2_font_profont10_tf);
    u8g2.setCursor(90, 60);
    u8g2.print("->");
    u8g2.print(tempPred.predictedTemp, 1);
  }
  
  // 控制按钮区域 (64-76像素)
  u8g2.drawFrame(0, 64, 120, 12);
  u8g2.setFont(u8g2_font_profont12_tf);
  u8g2.setCursor(4, 74);
  u8g2.print("Set:");
  u8g2.print(setpoint, 1);
  u8g2.print("C");
  
  // PWM显示 (76-88像素)
  u8g2.setCursor(4, 86);
  u8g2.print("PWM:");
  u8g2.print((int)((pwm / 255.0) * 100));
  u8g2.print("%");
  
  // 环境数据显示 (88-100像素)
  u8g2.setCursor(60, 86);
  u8g2.print("A:");
  u8g2.print(systemState.ambientTemperature, 1);
  u8g2.print("C");
  
  // 如果处于紧急停止状态，显示警告信息
  if (systemState.isEmergencyStop) {
    u8g2.setDrawColor(1);
    u8g2.drawBox(0, 0, 120, 100);
    u8g2.setDrawColor(0);
    u8g2.setFont(u8g2_font_profont22_tf);
    u8g2.setCursor(10, 40);
    u8g2.print("EMERGENCY");
    u8g2.setCursor(25, 60);
    u8g2.print("STOP");
  }
  
  u8g2.sendBuffer();
}

void displayError(const char* message) {
  u8g2.clearBuffer();
  
  // 错误显示框
  u8g2.drawFrame(0, 0, 120, 60);
  
  // 错误标题
  u8g2.setFont(u8g2_font_profont15_tf);
  u8g2.setCursor(4, 20);
  u8g2.print("ERROR");
  
  // 错误信息
  u8g2.setFont(u8g2_font_profont12_tf);
  u8g2.setCursor(4, 40);
  u8g2.print(message);
  
  u8g2.sendBuffer();
  errorState = true;
}