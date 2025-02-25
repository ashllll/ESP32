#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SensirionI2CSgp41.h>
#include <SensirionI2CSht4x.h>
#include <VOCGasIndexAlgorithm.h>
#include <NOxGasIndexAlgorithm.h>

// 定义OLED显示屏参数
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// 创建对象
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
SensirionI2CSgp41 sgp41;
SensirionI2CSht4x sht4x;
VOCGasIndexAlgorithm vocAlgorithm;
NOxGasIndexAlgorithm noxAlgorithm;

// 环境数据变量
float temperature = 0.0;
float humidity = 0.0;
int32_t vocIndex = 0;
int32_t noxIndex = 0;
uint16_t rawVoc = 0;
uint16_t rawNox = 0;

void setup() {
  // 初始化串口
  Serial.begin(115200);
  Serial.println("ESP32C3 SuperMini 环境监测系统启动中...");

  // 初始化I2C
  Wire.begin(7, 6); // SDA=7, SCL=6

  // 初始化OLED显示屏
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306初始化失败！");
    for(;;);
  }
  
  // 显示启动画面
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("环境监测系统");
  display.println("初始化中...");
  display.display();
  delay(2000);

  // 初始化SHT40传感器
  sht4x.begin(Wire);
  uint16_t error;
  char errorMessage[256];
  error = sht4x.softReset();
  if (error) {
    Serial.print("SHT40软复位错误: ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }

  // 初始化SGP41传感器
  sgp41.begin(Wire);
  error = sgp41.executeConditioning(0, 0); // 传入默认的湿度和温度补偿值
  if (error) {
    Serial.print("SGP41初始化错误: ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }

  // 初始化VOC和NOx算法
  vocAlgorithm.setParameters(VOCGasIndexAlgorithm::DEFAULT_SAMPLING_INTERVAL,
                            VOCGasIndexAlgorithm::DEFAULT_GATING_MAX_DURATION_MINUTES,
                            VOCGasIndexAlgorithm::DEFAULT_STD_INITIAL,
                            VOCGasIndexAlgorithm::DEFAULT_GAIN_FACTOR);
  noxAlgorithm.setParameters(NOxGasIndexAlgorithm::DEFAULT_SAMPLING_INTERVAL,
                            NOxGasIndexAlgorithm::DEFAULT_GATING_MAX_DURATION_MINUTES,
                            NOxGasIndexAlgorithm::DEFAULT_STD_INITIAL,
                            NOxGasIndexAlgorithm::DEFAULT_GAIN_FACTOR);
  
  Serial.println("初始化完成!");
}

void loop() {
  uint16_t error;
  char errorMessage[256];
  uint16_t defaultRh = 0x8000;  // 默认相对湿度50%
  uint16_t defaultT = 0x6666;   // 默认温度25°C
  uint16_t srawVoc = 0;
  uint16_t srawNox = 0;
  
  // 读取SHT40温湿度数据
  float humidity_reading;
  float temperature_reading;
  error = sht4x.measureHighPrecision(temperature_reading, humidity_reading);
  if (error) {
    Serial.print("读取SHT40数据错误: ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    temperature = temperature_reading;
    humidity = humidity_reading;
    
    // 计算SGP41补偿值
    if (humidity < 0) {
      humidity = 0;
    } else if (humidity > 100) {
      humidity = 100;
    }
    defaultRh = static_cast<uint16_t>((humidity * 65535) / 100);
    
    if (temperature < -45) {
      temperature = -45;
    } else if (temperature > 130) {
      temperature = 130;
    }
    defaultT = static_cast<uint16_t>(((temperature + 45) * 65535) / 175);
  }

  // 读取SGP41数据
  error = sgp41.measureRawSignals(defaultRh, defaultT, srawVoc, srawNox);
  if (error) {
    Serial.print("读取SGP41数据错误: ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    rawVoc = srawVoc;
    rawNox = srawNox;
    
    // 处理气体指数算法
    vocIndex = vocAlgorithm.process(srawVoc);
    noxIndex = noxAlgorithm.process(srawNox);
  }

  // 更新显示屏
  updateDisplay();
  
  // 打印调试信息
  Serial.printf("温度: %.2f°C, 湿度: %.2f%%\n", temperature, humidity);
  Serial.printf("VOC指数: %d, NOx指数: %d\n", vocIndex, noxIndex);
  Serial.printf("VOC原始值: %d, NOx原始值: %d\n", rawVoc, rawNox);
  
  // 延时1秒
  delay(1000);
}

void updateDisplay() {
  display.clearDisplay();
  
  // 显示标题
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("环境监测系统");
  display.drawLine(0, 9, display.width(), 9, SSD1306_WHITE);
  
  // 显示温湿度
  display.setCursor(0, 12);
  display.print("温度: ");
  display.print(temperature, 1);
  display.println("C");
  
  display.setCursor(0, 22);
  display.print("湿度: ");
  display.print(humidity, 1);
  display.println("%");
  
  // 显示气体指数
  display.setCursor(0, 32);
  display.print("VOC指数: ");
  display.println(vocIndex);
  
  display.setCursor(0, 42);
  display.print("NOx指数: ");
  display.println(noxIndex);
  
  // 显示空气质量评估
  display.setCursor(0, 52);
  display.print("空气质量: ");
  
  if (vocIndex < 50 && noxIndex < 50) {
    display.println("优");
  } else if (vocIndex < 100 && noxIndex < 100) {
    display.println("良");
  } else if (vocIndex < 200 && noxIndex < 200) {
    display.println("中");
  } else {
    display.println("差");
  }
  
  display.display();
}