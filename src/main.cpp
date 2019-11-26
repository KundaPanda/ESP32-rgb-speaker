#include "soc/timer_group_reg.h"
#include "soc/timer_group_struct.h"
#include <WiFi.h>
#include <Arduino.h>
#include <NeoPixelBus.h>
#include <DallasTemperature.h>
#include <FastLED.h>
#include <FastLED_GFX.h>
#include <LEDMatrix.h>
#include <arduinoFFT.h>
#include <esp32-hal-cpu.h>
#include <string>

const bool DEBUG = false;

#define tempPin 27
#define fanPin 12
#define fanChannel 0
#define micPin 32
#define switchPin 22
#define NSAMPLES 1024
#define SLEEP_TIME_MS 1000
#define STATE_SWITCH_CYCLES 9

#define DATA_PIN 25
#define COLOR_ORDER RGB
#define CHIPSET WS2812B
#define MATRIX_WIDTH 16
#define MATRIX_HEIGHT 16
#define MATRIX_TYPE VERTICAL_ZIGZAG_MATRIX
const int MATRIX_SIZE = MATRIX_WIDTH * MATRIX_HEIGHT;
const float MATRIX_FPS = 60.0;
cLEDMatrix<MATRIX_WIDTH, MATRIX_HEIGHT, MATRIX_TYPE> leds;
DEFINE_GRADIENT_PALETTE(rainbow_gp){
	0, 126, 1, 142,
	25, 171, 1, 26,
	48, 224, 9, 1,
	71, 237, 138, 1,
	94, 52, 173, 1,
	117, 1, 201, 1,
	140, 1, 211, 54,
	163, 1, 124, 168,
	186, 1, 8, 149,
	209, 12, 1, 151,
	232, 12, 1, 151,
	255, 171, 1, 190
};

DEFINE_GRADIENT_PALETTE(spellbound_gp){
	0, 232, 235, 40,
	12, 157, 248, 46,
	25, 100, 246, 51,
	45, 53, 250, 33,
	63, 18, 237, 53,
	81, 11, 211, 162,
	94, 18, 147, 214,
	101, 43, 124, 237,
	112, 49, 75, 247,
	127, 49, 75, 247,
	140, 92, 107, 247,
	150, 120, 127, 250,
	163, 130, 138, 252,
	173, 144, 131, 252,
	186, 148, 112, 252,
	196, 144, 37, 176,
	211, 113, 18, 87,
	221, 163, 33, 53,
	234, 255, 101, 78,
	247, 229, 235, 46,
	255, 229, 235, 46
};

CRGBPalette256 currentPalette = rainbow_gp;
int paletteProgress = 0;
int paletteStepSpectrum = 4;
int paletteStep = 1;
const int PALETTE_STEP_COUNTDOWN = 4;
int paletteStepCountdown = PALETTE_STEP_COUNTDOWN;
const CRGB off(0, 0, 0);
const CRGB white(255, 255, 255);
const CRGB red(255, 0, 0);

// const float CALIBRATE_FREQ = 12000.0;
// const float IDLE_MIC_VOLTAGE = 1239.0;
const int IDLE_READING = 1267;
// const int LOW_VOLTAGE_ACTUAL = 8;
// const int LOW_VOLTAGE_READING = 220;
const float HIGH_VOLTAGE = 3227.0;
const double SAMPLE_RATE = 41000.0;
const double SAMPLE_TIME = (1.0 * 1000 * 1000 / SAMPLE_RATE);

/**
 * bass - 20-250
 * mids - 250-2500
 * upper mids - 2500-5000
 * highs - 5000-20000
 **/
const int NBANDS = 16;
const int BANDS[NBANDS] = { 130, 220, 340, 520, 780, 1000, 1220, 1480, 1960, 2400, 2900, 3600, 4700, 5800, 6500, 20000 };
double bandValues[NBANDS];
const int lastBaseIndex = 2;
const int lastMidIndex = 10;
const int lastHighIndex = 15;
const float BASE_THRESHOLD = 10000.0;
const float MID_THRESHOLD = 8500.0;
const float HIGH_THRESHOLD = 7000.0;
const float upFreqModifier = 0.952;
const float downFreqModifier = 1.048;
float baseThreshold = BASE_THRESHOLD;
float midThreshold = MID_THRESHOLD;
float highThreshold = HIGH_THRESHOLD;
const double READING_THRESHOLD = 1500.0;
const double FREQ_THRESHOLD = 40.0;
const double SPECTRUM_THRESHOLD = 9500;
int cyclesWithoutBase = 0;
const int CYCLES_WITHOUT_BASE_THRESHOLD = 180;

TaskHandle_t MatrixTask;
GFXcanvas canvas(MATRIX_WIDTH, MATRIX_HEIGHT);
arduinoFFT FFT;
float peakBand;
float peakValue;
float maximum;
double real[NSAMPLES];
double imag[NSAMPLES];

typedef enum {
	STAR,
	STRIPE
} animationType;

typedef struct {
	short state;
	short stateSwitchCycles;
	animationType type;
	bool available;
} animation;

int animations = 0;
const int maxAnimations = 17;
const int minAnimations = 2;
void (*chosenAnimation)(void *);
animation animationsArray[16][16];

const int functionSwitchTimeMax = 100000;
const int functionSwitchTimeMin = 30000;
int functionSwitchTime = rand() % (functionSwitchTimeMax - functionSwitchTimeMin) + functionSwitchTimeMin;
unsigned long lastSwitchTime = 0;
bool canSwitch = false;
int btnPressed = 0;
bool animationsOff = false;
unsigned long pressedStart = 0;
unsigned long turnOffThreshold = 700;
int btnPressSleep = 0;
int baseCycles = 0;
int baseCyclesThreshold = 38;

OneWire oneWire(tempPin);
DallasTemperature tempSensor(&oneWire);
const int tempReadIntervalMs = 20000;
int lastTempReadTime = 0;
double tempReadValue = 0.0;

/**TODO: remove commented code
* add comments and documentation
* add more animations
* fan control
* clean up variables
*/

void getPeak(void) {
	peakValue = 0;
	peakBand = 0;
	for (int i = 0; i < NBANDS; i++) {
		if (peakValue < bandValues[i]) {
			peakValue = bandValues[i];
			peakBand = BANDS[i];
		}
	}
}

void doFFT(void) {
	FFT.Compute(FFT_FORWARD);
	FFT.ComplexToMagnitude();
	int i = 1;
	double freq = (i * SAMPLE_RATE) / NSAMPLES;
	int band;
	double result;
	int nbands;
	for (int bIndex = 0; bIndex < NBANDS; bIndex++) {
		band = BANDS[bIndex];
		result = 0;
		nbands = 0;
		while (freq < band) {
			freq = (i * SAMPLE_RATE) / NSAMPLES;
			result += real[i];
			nbands++;
			i++;
		}
		result /= nbands;
		// 220 - noise
		if (READING_THRESHOLD > result)
			result = 0.0;
		// Reduce vaule of OP base
		if (bIndex == 0)
			result *= 0.3;
		else if (bIndex > NBANDS - 3)
			result *= 1.56;

		bandValues[bIndex] = result;
		if (DEBUG) {
			Serial.printf("%d Hz: %lf\n", band, result);
		}
	}
	getPeak();
	maximum = (peakValue > SPECTRUM_THRESHOLD) ? peakValue : SPECTRUM_THRESHOLD;
}

double analogToVoltage(uint16_t analog) {
	return (HIGH_VOLTAGE * analog) / 4096.0;
}

int voltageToAnalog(double voltage) {
	return (4096.0 * voltage) / HIGH_VOLTAGE;
}

double calibrate_silence(void) {
	int samples = 8192;
	double result = 0;
	for (int i = 0; i < samples; i++) {
		result += 1.0 * analogRead(micPin);
		;
	}
	result /= samples;
	if (DEBUG) {
		Serial.printf("Silence result: %lf\n", result);
	}
	return result;
}

void getSamples(int amount) {
	unsigned long last = 0;
	float freeTime = 0;
	for (int i = 0; i < amount; i++) {
		last = micros();
		real[i] = 1.0 * (analogRead(micPin) - IDLE_READING);
		while (SAMPLE_TIME > micros() - last) {
			freeTime += 1.0;
		}
	}
	if (DEBUG) {
		Serial.printf("Increments: %f = %f per one read.\n", freeTime, freeTime / amount);
	}
}

// double getSampleSpeed() {
// 	int samples = 10000;
// 	double start = micros();
// 	for (int i = 0; i < samples; i++) {
// 		(analogRead(micPin) - IDLE_READING) * 1.0;
// 	}
// 	double time = micros() - start;
// 	double speed = samples / time;
// 	time = (double)samples / time;
// 	if (DEBUG) {
// 		Serial.printf("Time for one sample: %lfus, sample speed: %lf/s\n", time, speed * 1000 * 1000);
// 	}
// 	return speed;
// }

void showSpectrum(void *params) {
	if (DEBUG) {
		Serial.printf("Showing spectrum\n");
	}
	// float start = millis();
	double values[NBANDS];
	CRGB colors[NBANDS];
	// HsbColor current = rainbowCurrent;
	for (int i = 0; i < NBANDS; i++) {
		values[i] = (maximum * (17 - i) / 15);
		colors[i] = ColorFromPalette(currentPalette, paletteProgress + i * paletteStepSpectrum);
	}
	fill_solid(FastLED.leds(), MATRIX_SIZE, off);
	// strip.ClearTo(off);
	for (int x = 0; x < NBANDS; x++) {
		for (int y = 15; y >= 0; y--) {
			if (bandValues[x] >= values[y]) {
				// strip.SetPixelColor(tiles.Map(x, y), colors[15 - y]);
				leds.DrawPixel(x, y, colors[15 - y]);
			}
		}
		// strip.SetPixelColor(15 * 15, blue);
	}
	FastLED.show();
	// strip.Show();
}

void showAnimations(void) {
	CRGB inverse = ColorFromPalette(currentPalette, 255 - paletteProgress + 30);
	// HsbColor inverseColor(fmodf(rainbowCurrent.H + 0.3, 1.0), 0.0, 0.0);
	// HsbColor color(0.0, 0.0, 0.0);
	if (baseCycles > 0) {
		Serial.print("Base cycles: ");
		Serial.print(baseCycles);
		Serial.print(", baseThreshold:");
		Serial.println(baseThreshold);
		fill_solid(FastLED.leds(), MATRIX_SIZE, ColorFromPalette(currentPalette, paletteProgress));
		baseCycles--;
		cyclesWithoutBase = 0;
	} else {
		fill_solid(FastLED.leds(), MATRIX_SIZE, off);
		cyclesWithoutBase++;
	}
	// strip.ClearTo(color);
	for (int x = 0; x < MATRIX_WIDTH; x++) {
		for (int y = 0; y < MATRIX_HEIGHT; y++) {
			animation *anim = &animationsArray[y][x];
			if (anim->available)
				continue;
			switch (anim->type) {
			case STAR: {
				switch (anim->state) {
				case 0:
				case 2: {
					// strip.SetPixelColor(tiles.Map(x, y), white);
					leds.DrawPixel(x, y, white);
					anim->stateSwitchCycles--;
					if (anim->stateSwitchCycles == 0) {
						anim->state++;
						anim->stateSwitchCycles = STATE_SWITCH_CYCLES;
					}
					break;
				}
				case 1: {
					leds.DrawPixel(x + 1, y, white);
					leds.DrawPixel(x - 1, y, white);
					leds.DrawPixel(x, y + 1, white);
					leds.DrawPixel(x, y - 1, white);
					// strip.SetPixelColor(tiles.Map(x + 1, y), white);
					// strip.SetPixelColor(tiles.Map(x - 1, y), white);
					// strip.SetPixelColor(tiles.Map(x, y + 1), white);
					// strip.SetPixelColor(tiles.Map(x, y - 1), white);
					anim->stateSwitchCycles--;
					if (anim->stateSwitchCycles == 0) {
						anim->state++;
						anim->stateSwitchCycles = STATE_SWITCH_CYCLES;
					}
					break;
				}
				case 3: {
					anim->available = true;
					animations--;
					break;
				}
				}
				break;
			}
			case STRIPE: {
				switch (anim->state) {
				case 0: {
					leds.DrawPixel(x, y, inverse);
					// strip.SetPixelColor(tiles.Map(x, y), inverseColor);
					anim->stateSwitchCycles--;
					if (anim->stateSwitchCycles == 0) {
						anim->state++;
						anim->stateSwitchCycles = STATE_SWITCH_CYCLES;
					}
					break;
				}
				case 1: {
					leds.DrawPixel(x, y, inverse);
					leds.DrawPixel(x, y + 1, inverse);
					leds.DrawPixel(x, y - 1, inverse);
					// strip.SetPixelColor(tiles.Map(x, y), inverseColor);
					// strip.SetPixelColor(tiles.Map(x, y + 1), inverseColor);
					// strip.SetPixelColor(tiles.Map(x, y - 1), inverseColor);
					anim->stateSwitchCycles--;
					if (anim->stateSwitchCycles == 0) {
						anim->state++;
						anim->stateSwitchCycles = STATE_SWITCH_CYCLES;
					}
					break;
				}
				case 2: {
					leds.DrawPixel(x, y + 1, inverse);
					leds.DrawPixel(x, y - 1, inverse);
					// strip.SetPixelColor(tiles.Map(x, y + 1), inverseColor);
					// strip.SetPixelColor(tiles.Map(x, y - 1), inverseColor);
					anim->stateSwitchCycles--;
					if (anim->stateSwitchCycles == 0) {
						anim->state++;
						anim->stateSwitchCycles = STATE_SWITCH_CYCLES;
					}
					break;
				}
				case 3: {
					anim->available = true;
					animations--;
					break;
				}
				}
				break;
			}
			}
		}
	}
}

void addAnimation(animationType type) {
	if (animations < maxAnimations) {
		int pixel = rand() % (MATRIX_SIZE);
		while (animationsArray[pixel % MATRIX_WIDTH][pixel / MATRIX_WIDTH].available == false) {
			//TODO better algorithm for placement
			pixel = (pixel + 1) % (MATRIX_SIZE);
		}
		int x = pixel % MATRIX_WIDTH;
		int y = pixel / MATRIX_WIDTH;
		animationsArray[y][x].available = false;
		animationsArray[y][x].state = 0;
		animationsArray[y][x].type = type;
		animationsArray[y][x].stateSwitchCycles = STATE_SWITCH_CYCLES;
		animations++;
	}
}

void pulseToBase(void *params) {
	for (int i = 0; i < NBANDS; i++) {
		if (i < lastBaseIndex) {
			if (bandValues[i] > baseThreshold && baseCycles <= baseCyclesThreshold)
				baseCycles += 4;
		} else if (i < lastMidIndex) {
			if (bandValues[i] > midThreshold) {
				addAnimation(STAR);
			}
		} else {
			if (bandValues[i] > highThreshold) {
				addAnimation(STRIPE);
			}
		}
	}
	if (baseCycles >= baseCyclesThreshold * 0.9) {
		baseCycles -= 20;
		baseThreshold *= downFreqModifier;
	} else if (baseThreshold > BASE_THRESHOLD && cyclesWithoutBase >= CYCLES_WITHOUT_BASE_THRESHOLD)
		baseThreshold *= upFreqModifier;
	if (animations >= maxAnimations * 0.9) {
		if (midThreshold > MID_THRESHOLD)
			midThreshold *= downFreqModifier;
		if (highThreshold > HIGH_THRESHOLD)
			highThreshold *= downFreqModifier;
	} else if (midThreshold > MID_THRESHOLD && animations <= minAnimations * 0.9) {
		midThreshold *= upFreqModifier;
		highThreshold *= upFreqModifier;
	}
	showAnimations();
	FastLED.show();
	// strip.Show();
}

void (*animationTypes[])(void *) = { showSpectrum, pulseToBase };

void (*getRandomAnimation())(void *) {
	std::random_shuffle(animationTypes, std::end(animationTypes));
	return animationTypes[0];
}

void callMatrixFunction(void *params) {
	unsigned long matrixStart;
	for (;;) {
		TIMERG1.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
		TIMERG1.wdt_feed = 1;
		TIMERG1.wdt_wprotect = 0;
		if (!animationsOff) {
			matrixStart = millis();
			if (abs(millis() - lastSwitchTime) > functionSwitchTime) {
				lastSwitchTime = millis();
				chosenAnimation = getRandomAnimation();
				animations = 0;
			}
			paletteStepCountdown--;
			if (paletteStepCountdown == 0) {
				paletteProgress = (paletteProgress + paletteStep) % 256;
				paletteStepCountdown = PALETTE_STEP_COUNTDOWN;
			}
			//TODO: debug pulsetobase, remove!!!!
			// chosenAnimation = pulseToBase;
			chosenAnimation(params);
			functionSwitchTime = rand() % (functionSwitchTimeMax - functionSwitchTimeMin) + functionSwitchTimeMin;
			// rainbowCurrent.H = fmodf((rainbowCurrent.H + rainbowStep), 1.0);
			vTaskDelay(1 / portTICK_PERIOD_MS);
			while (abs(millis() - matrixStart) < 1000 / MATRIX_FPS) {
				// check for switch button in the meantime
				if (digitalRead(switchPin) == HIGH && (abs(millis() - btnPressed) > btnPressSleep)) {
					// Serial.println("Changing animation.");
					lastSwitchTime = millis();
					btnPressed = millis();
					chosenAnimation = getRandomAnimation();
					while (abs(millis() - matrixStart) < 1000 / MATRIX_FPS)
						;
					break;
				}
				if (abs(millis() - lastTempReadTime) >= tempReadIntervalMs) {
					if (tempReadValue == 0.0) {
						lastTempReadTime = millis();
						tempSensor.requestTemperatures();
						tempReadValue = tempSensor.getTempCByIndex(0);
						while (abs(millis() - matrixStart) < 1000 / MATRIX_FPS)
							;
						break;
					} else {
						int fanSpeed = tempReadValue < 40 ? 98.30973 + (-0.1301144 - 98.30973) / (1 + pow(pow(tempReadValue / 25.0224, 524.2068), 0.01626859)) : 100;
						fanSpeed = fanSpeed > 80 ? fanSpeed : 0;
						ledcWrite(fanChannel, (fanSpeed * 1023) / 100);
						if (DEBUG)
							Serial.printf("Temperature: %.1f°C -> fan at %d.\n", tempReadValue, fanSpeed);
						tempReadValue = 0.0;
						while (abs(millis() - matrixStart) < 1000 / MATRIX_FPS)
							;
						break;
					}
				}
			}
		} else {
			fill_solid(FastLED.leds(), MATRIX_SIZE, off);
			FastLED.show();
			vTaskDelay(1 / portTICK_PERIOD_MS);
		}
	}
}

void setup() {
	setCpuFrequencyMhz(240);
	Serial.begin(115200);
	WiFi.mode(WIFI_OFF);
	btStop();
	for (int i = 0; i < MATRIX_HEIGHT; i++) {
		for (int j = 0; j < MATRIX_WIDTH; j++) {
			animationsArray[i][j].available = true;
		}
	}
	pinMode(micPin, INPUT);
	pinMode(tempPin, INPUT);
	pinMode(switchPin, INPUT);
	analogReadResolution(12);
	ledcSetup(fanChannel, 25000, 10);
	ledcAttachPin(fanPin, fanChannel);
	// calibrate_silence();
	FastLED.addLeds<CHIPSET, DATA_PIN, COLOR_ORDER>(canvas.getBuffer(), canvas.width() * canvas.height()).setCorrection(TypicalSMD5050);
	leds.SetLEDArray(canvas.getBuffer());
	FastLED.setBrightness(127);
	FastLED.clear(true);
	for (int i = 1; i < sqrt(pow(MATRIX_WIDTH, 2) + pow(MATRIX_HEIGHT, 2)); i++) {
		leds.DrawFilledCircle(0, 0, i, red);
		FastLED.show();
		delay(60);
	}
	fill_solid(FastLED.leds(), MATRIX_SIZE, off);
	FastLED.show();
	tempSensor.begin();
	std::random_shuffle(animationTypes, std::end(animationTypes));
	chosenAnimation = pulseToBase;
	FFT = arduinoFFT(real, imag, NSAMPLES, SAMPLE_RATE);
	FFT.Windowing(FFT_WIN_TYP_HANN, FFT_FORWARD);
	xTaskCreatePinnedToCore(
	    callMatrixFunction,
	    "Matrix rendering",
	    2048,
	    NULL,
	    10,
	    &MatrixTask,
	    0);
}

void loop() {
	getSamples(NSAMPLES);
	std::fill(imag, std::end(imag), 0.0);
	doFFT();
	// for (int i = 0; i < NSAMPLES / 2; i++) {
	//     // Serial.printf("Peak: %lf\n", peak);
	//     Serial.printf("%f Hz: %lf\n", freq, real[i]);
	//     // Serial.printf("%f Hz: %i\n", FFT_BIN(i, SAMPLE_RATE, NSAMPLES), readings[i]);
	// }
	if (DEBUG)
		Serial.printf("---------\n");
	if (digitalRead(switchPin) == HIGH) {
		if (pressedStart == 0) {
			pressedStart = millis();
		} else if (abs(millis() - pressedStart) < turnOffThreshold && canSwitch) {
			animationsOff = !animationsOff;
			canSwitch = false;
		} else {
			pressedStart = millis();
		}
	} else {
		canSwitch = true;
	}
}
