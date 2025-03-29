# ESP32 PID Temperature Control System
# ESP32 PID 温度控制系统

This is a PID temperature control system project based on ESP32-S3-DevKitC-1 development board. The project uses PID algorithm to achieve precise temperature control and displays temperature and control parameters in real-time through an OLED display.

这是一个基于 ESP32-S3-DevKitC-1 开发板的 PID 温度控制系统项目。该项目使用 PID 算法实现精确的温度控制，并通过 OLED 显示屏实时显示温度和控制参数。

## Features | 功能特点

- Temperature acquisition using MAX31865 temperature sensor
- Precise temperature control using PID algorithm
- Parameter adjustment via rotary encoder
- Real-time temperature and control parameter display on OLED
- Temperature setpoint adjustment support
- Online PID parameter tuning support

- 使用 MAX31865 温度传感器进行温度采集
- 采用 PID 算法实现精确的温度控制
- 通过旋转编码器进行参数调节
- OLED 显示屏实时显示温度和控制参数
- 支持温度设定值调节
- 支持 PID 参数在线调节

## Hardware Requirements | 硬件要求

- ESP32-S3-DevKitC-1 development board
- MAX31865 temperature sensor
- OLED display (SSD1306)
- Rotary encoder
- Heating element
- Necessary connection wires

- ESP32-S3-DevKitC-1 开发板
- MAX31865 温度传感器
- OLED 显示屏 (SSD1306)
- 旋转编码器
- 加热元件
- 其他必要的连接线

## Software Dependencies | 软件依赖

- PlatformIO
- Arduino-ESP32 framework
- Adafruit MAX31865 library
- Adafruit SSD1306 library
- PID library
- U8g2 library

- PlatformIO
- Arduino-ESP32 框架
- Adafruit MAX31865 库
- Adafruit SSD1306 库
- PID 库
- U8g2 库

## Usage Instructions | 使用说明

1. Clone the project to local
2. Open with PlatformIO
3. Install required dependencies
4. Compile and upload to ESP32-S3 board

1. 克隆项目到本地
2. 使用 PlatformIO 打开项目
3. 安装所需依赖库
4. 编译并上传到 ESP32-S3 开发板

## Wiring Guide | 接线说明

- MAX31865 Temperature Sensor:
  - VCC -> 3.3V
  - GND -> GND
  - DO -> GPIO 19
  - CS -> GPIO 5
  - CLK -> GPIO 18

- MAX31865 温度传感器:
  - VCC -> 3.3V
  - GND -> GND
  - DO -> GPIO 19
  - CS -> GPIO 5
  - CLK -> GPIO 18

- OLED Display:
  - SDA -> GPIO 21
  - SCL -> GPIO 22

- OLED 显示屏:
  - SDA -> GPIO 21
  - SCL -> GPIO 22

- Rotary Encoder:
  - CLK -> GPIO 32
  - DT -> GPIO 33
  - SW -> GPIO 34

- 旋转编码器:
  - CLK -> GPIO 32
  - DT -> GPIO 33
  - SW -> GPIO 34

## Important Notes | 注意事项

1. This project is for personal learning and research only
2. Commercial use is strictly prohibited
3. Please read the wiring guide carefully before use
4. Ensure stable power supply
5. Ensure safe use of heating elements and circuits
6. Overcurrent protection circuit is recommended
7. Please follow relevant safety regulations

1. 本项目仅供个人学习和研究使用
2. 禁止用于商业用途
3. 使用前请仔细阅读接线说明
4. 确保电源供应稳定
5. 请确保加热元件和电路的安全使用
6. 建议使用过流保护电路
7. 使用时请遵守相关安全规范

## Disclaimer | 免责声明

This project is for learning and research purposes only. The author is not responsible for any losses that may occur from using this project, including but not limited to:
- Device damage
- Fire hazards
- Personal injury
- Property damage
- Other accidents

By using this project, you acknowledge that you fully understand and accept these risks, and agree to assume all possible consequences.

本项目仅供学习和研究使用。作者不对使用本项目可能造成的任何损失负责，包括但不限于：
- 设备损坏
- 火灾
- 人身伤害
- 财产损失
- 其他意外事故

使用本项目即表示您完全理解并接受这些风险，并同意自行承担所有可能的后果。

## License | 许可证

This project is licensed under the [MIT License](LICENSE), but limited to personal use only. Commercial use is prohibited.

本项目采用 [MIT License](LICENSE) 许可证，但仅限个人使用，禁止商业用途。

## Author | 作者

- Author: Joe Ash
- 作者：Joe Ash

## Acknowledgments | 致谢

Thanks to all contributors of the open-source libraries, especially:
- Adafruit for sensor libraries
- PlatformIO team for development environment
- ESP32 open-source community

感谢所有开源库的贡献者，特别是：
- Adafruit 公司提供的传感器库
- PlatformIO 团队提供的开发环境
- ESP32 开源社区的支持
