# Finger Tracking LED Controller

A real-time hand gesture control system that tracks finger movements using computer vision and controls LED brightness on an ESP32 microcontroller via serial communication.

![Demo](assets/demo.gif)

## Overview

This project uses MediaPipe hand tracking to detect finger positions from a webcam feed and maps each finger's extension to PWM values that control individual LEDs connected to an ESP32. The system provides smooth, responsive control with low-pass filtering to reduce jitter.

## Features

- **Real-time Hand Tracking**: Uses MediaPipe to detect and track hand landmarks
- **Individual Finger Control**: Each of the 5 fingers controls a separate LED
- **Smooth PWM Output**: Low-pass filter reduces jitter and provides stable LED brightness
- **Adaptive Scaling**: Automatically adjusts to different hand sizes using palm size as reference
- **Visual Feedback**: Live camera feed with hand landmark overlay and FPS counter
- **Serial Communication**: Reliable data transmission to ESP32 at 20 FPS

## Hardware Requirements

- ESP32 development board
- 5x LEDs
- 5x appropriate resistors (typically 220Ω - 330Ω depending on LED specs)
- Breadboard and jumper wires
- USB cable for ESP32 connection
- Webcam

## Circuit Diagram

![Circuit Diagram](assets/circuits.png)

### Pin Configuration

| Finger | ESP32 Pin | LED |
|--------|-----------|-----|
| Thumb  | GPIO 23   | LED 1 |
| Index  | GPIO 22   | LED 2 |
| Middle | GPIO 21   | LED 3 |
| Ring   | GPIO 19   | LED 4 |
| Pinky  | GPIO 18   | LED 5 |

## Software Requirements

### Python Dependencies

```bash
pip install -r requirements.txt
```

The `requirements.txt` should include:
```
opencv-python
mediapipe
pyserial
```

### Arduino/ESP32 Setup

1. Install the Arduino IDE or PlatformIO
2. Install ESP32 board support
3. Upload `arduino_code.ino` to your ESP32

## Installation

1. **Clone or download this repository**

2. **Install Python dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Set up hardware**:
   - Connect LEDs to ESP32 according to the pin configuration
   - Connect ESP32 to your computer via USB

4. **Upload Arduino code**:
   - Open `arduino_code.ino` in Arduino IDE
   - Select your ESP32 board and port
   - Upload the sketch

5. **Configure serial port** (Linux users):
   - The script uses `/dev/myESP32` (configured via udev rules)
   - To use a different port, modify line 7 in `main.py`:
     ```python
     arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=0.1)
     ```
   - Windows users: Change to `'COM3'` or your appropriate COM port
   - macOS users: Change to `'/dev/cu.usbserial-XXXX'`

## Usage

1. **Run the Python script**:
   ```bash
   python main.py
   ```

2. **Position your hand** in front of the webcam

3. **Control LEDs** by extending or closing your fingers:
   - Closed finger = LED off/dim
   - Open finger = LED bright

4. **Exit** by pressing `q` in the camera window

## How It Works

### Hand Tracking Algorithm

1. **Palm Size Detection**: Calculates distance between wrist (landmark 0) and middle finger base (landmark 9) as a scaling reference

2. **Finger Extension Measurement**: For each finger, measures the distance from wrist to fingertip and calculates a ratio relative to palm size

3. **PWM Mapping**:
   - Thumb: Ratio 0.6-1.3 → PWM 0-255
   - Other fingers: Ratio 1.0-1.8 → PWM 0-255

4. **Smoothing**: Low-pass filter (alpha = 0.5) reduces jitter:
   ```
   smoothed_value = (alpha × new_value) + ((1 - alpha) × previous_value)
   ```

5. **Serial Communication**: Sends data in format: `$pwm1,pwm2,pwm3,pwm4,pwm5\n`

### Communication Protocol

The Python script sends data to ESP32 at 20 FPS maximum in the following format:
```
$255,128,64,200,0\n
```

Where:
- `$` = Start delimiter
- Five comma-separated PWM values (0-255)
- `\n` = End delimiter

## Customization

### Adjusting Filter Sensitivity

In `main.py`, modify the `alpha` value (line 21):
```python
alpha = 0.5  # Range: 0.0 - 1.0
```
- **Higher values** (0.7-1.0): Faster response, more jitter
- **Lower values** (0.1-0.4): Smoother output, slower response

### Changing Update Rate

Modify the frame rate cap in `main.py` (line 93):
```python
if arduino and (current_time - last_time > 0.05):  # 0.05 = 20 FPS
```

### Modifying Pin Configuration

Update the `ledPins` array in `arduino_code.ino` (line 3):
```cpp
const int ledPins[5] = {23, 22, 21, 19, 18};
```

## Troubleshooting

**ESP32 not detected:**
- Check USB cable and connection
- Verify correct port in `main.py`
- Ensure ESP32 drivers are installed

**Webcam not working:**
- Change camera index in `main.py` line 18: `cap = cv2.VideoCapture(1)`
- Check camera permissions

**Jittery LED response:**
- Decrease `alpha` value for more smoothing
- Ensure good lighting conditions
- Keep hand steady and centered in frame

**LEDs not responding:**
- Verify correct wiring and pin configuration
- Check LED polarity
- Test LEDs individually with a simple blink sketch
- Verify baud rate matches (115200) in both scripts

## License

See [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Feel free to submit issues or pull requests.

## Acknowledgments

- [MediaPipe](https://mediapipe.dev/) for hand tracking solutions
- [OpenCV](https://opencv.org/) for computer vision tools
- ESP32 community for hardware support

## Future Improvements

- [ ] Add support for multiple hands
- [ ] Implement gesture recognition for commands
- [ ] Add wireless communication (WiFi/Bluetooth)
- [ ] Create GUI for parameter adjustment
- [ ] Add recording and playback features
- [ ] Support for additional output devices

---

**Note**: This project is designed for educational and hobbyist purposes. Ensure proper safety measures when working with electronics.
