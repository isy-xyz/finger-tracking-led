// Pin Configs

const int ledPins[5] = {23, 22, 21, 19, 18};

String inputString = "";
bool stringComplete = false;


void setup() {
	// In python i use 115200, so it the baud rates should be 115200
	Serial.begin(115200);

	for (int i = 0; i < 5; i++) {
		pinMode(ledPins[i], OUTPUT);

		// Test the LED while booting
		digitalWrite(ledPins[i], HIGH);
		delay(100);
		digitalWrite(ledPins[i], LOW);
	}
	Serial.println("ESP32 Ready. Waiting data...");
}

void loop() {
	while (Serial.available()) {
		char inChar = (char)Serial.read();

	// Check the string completed or not (by checking \n)
		if (inChar == '\n') {
			stringComplete = true;
		} else {
			inputString += inChar;
		}
	}

	// If data are completed
	if (stringComplete) {
		executePWM();

		// Reset string
		inputString = "";
		stringComplete = false;
	}
}

void executePWM() {
	// Check data are started with '$'
	if (inputString.startsWith("$")) {

		String data = inputString.substring(1);

	// Erase the '$' 
		int stringStart = 0;
		int commaIndex;

		for (int i = 0; i < 5; i++) {
			// Search the next comma from stringStart
			commaIndex = data.indexOf(',', stringStart);

			String pwmStr;
			if (commaIndex == -1) {
				pwmStr = data.substring(stringStart);
			} else {
				pwmStr = data.substring(stringStart, commaIndex);

				stringStart = commaIndex + 1;
			}

			// Convert the data string to int
			int pwmValue = pwmStr.toInt();

			// Limit the value (8-bit)
			pwmValue = constrain(pwmValue, 0, 255);

			// Execute the PWM
			analogWrite(ledPins[i], pwmValue);
		}
	}
}