#include "soc/timer_group_reg.h"
#include "soc/timer_group_struct.h"
#include <Arduino.h>
#include <NeoPixelBus.h>
#include <WiFi.h>
#include <analogWrite.h>
#include <arduinoFFT.h>
#include <esp32-hal-cpu.h>

#define tempPin 27
#define fanPin 12
#define micPin 32
#define NSAMPLES 512
#define SLEEP_TIME_MS 1000
#define STATE_SWITCH_CYCLES 7
const float MATRIX_FPS = 60.0;
const float CALIBRATE_FREQ = 12000.0;
const float IDLE_MIC_VOLTAGE = 1239.0;
const int IDLE_READING = 1267;
const int LOW_VOLTAGE_ACTUAL = 8;
const int LOW_VOLTAGE_READING = 220;
const float HIGH_VOLTAGE = 3227.0;
const double SAMPLE_RATE = 38000.0;
const double SAMPLE_TIME = (1.0 * 1000 * 1000 / SAMPLE_RATE);
const bool DEBUG = false;

const float BASE_THRESHOLD = 5500.0;
const float MID_THRESHOLD = 15000.0;
const float HIGH_THRESHOLD = 10000.0;
float baseThreshold = BASE_THRESHOLD;
float midThreshold = MID_THRESHOLD;
float highThreshold = HIGH_THRESHOLD;
TaskHandle_t MatrixTask;
typedef ColumnMajorAlternatingLayout MyPanelLayout;
typedef ColumnMajorLayout MyTilesLayout;
const double THRESHOLD = 500.0;
const double FREQ_THRESHOLD = 40.0;
const double SPECTRUM_THRESHOLD = 20000;

// make sure to set these panel values to the sizes of yours
const uint8_t PanelWidth = 16; // 8 pixel x 8 pixel matrix of leds
const uint8_t PanelHeight = 16;
const uint16_t PixelCount = PanelWidth * PanelHeight;
const uint8_t PixelPin = 25;
NeoTiles<MyPanelLayout, MyTilesLayout> tiles(
    PanelWidth,
    PanelHeight,
    1,
    1);
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);
RgbColor red(15, 0, 0);
RgbColor green(0, 15, 0);
RgbColor blue(0, 0, 15);
RgbColor white(15, 15, 15);
RgbColor off(0, 0, 0);
const HsbColor rainbowStart(0.0, 1.0, 1.0);
const HsbColor rainbowEnd(1.0, 1.0, 1.0);
const float rainbowStep = 0.001;
float rainbowProgress = 0.0;

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
	RgbColor color;
	bool available;
} animation;
void (*chosenAnimation)(void *);
bool addAnimations = false;
animation animationsArray[16][16];

/**
 * bass - 20-250
 * mids - 250-2500
 * upper mids - 2500-5000
 * highs - 5000-20000
 **/
const int NBANDS = 16;
const int BANDS[NBANDS] = { 200, 450, 700, 900, 1200, 1500, 1800, 2300, 2800, 3500, 4700, 5500, 6700, 8000, 12000, 20000 };
const int lastBaseIndex = 1;
const int lastMidIndex = 7;
const int lastHighIndex = 15;
const int functionSwitchTimeMax = 60000;
const int functionSwitchTimeMin = 15000;
int functionSwitchTime = rand() % (functionSwitchTimeMax - functionSwitchTimeMin) + functionSwitchTimeMin;
int lastSwitchTime = 0;
double bandValues[NBANDS];
unsigned long last = 0;
int animations = 0;
const int maxAnimations = 40;
int baseCycles = 0;
int baseCyclesThreshold = 60;

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
	FFT = arduinoFFT(real, imag, NSAMPLES, SAMPLE_RATE);
	FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
	FFT.Compute(FFT_FORWARD);
	FFT.ComplexToMagnitude();
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

void clearAll(void) {
	for (int pixel = 0; pixel < PixelCount; pixel++) {
		strip.SetPixelColor(pixel, off);
	}
}

double getSampleSpeed() {
	int samples = 10000;
	double start = micros();
	for (int i = 0; i < samples; i++) {
		(analogRead(micPin) - IDLE_READING) * 1.0;
	}
	double time = micros() - start;
	double speed = samples / time;
	time = (double)samples / time;
	if (DEBUG) {
		Serial.printf("Time for one sample: %lfus, sample speed: %lf/s\n", time, speed * 1000 * 1000);
	}
	return speed;
}

void showSpectrum(void *params) {
	if (DEBUG) {
		Serial.printf("Showing spectrum\n");
	}
	float values[NBANDS];
	for (int i = 0; i < NBANDS; i++) {
		values[i] = (maximum * (16 - i) / 15);
		// Serial.printf("%d: %lf\n", i, values[i]);
	}
	RgbColor color = red;
	clearAll();
	for (int x = 0; x < NBANDS; x++) {
		for (int y = 15; y >= 0; y--) {
			if (bandValues[x] >= values[y]) {
				if (y > 10) {
					color = red;
				} else if (y > 5) {
					color = green;
				} else {
					color = blue;
				}
				strip.SetPixelColor(tiles.Map(x, y), color);
			}
		}
	}
	// strip.SetPixelColor(15 * 15, blue);
	strip.Show();
}

void showAnimations(void) {
	HsbColor inverseColor = HsbColor::LinearBlend<NeoHueBlendClockwiseDirection>(rainbowStart, rainbowEnd, 1.0 - rainbowProgress);
	HsbColor color(0.0, 0.0, 0.0);
	if (baseCycles > 0) {
		baseCycles--;
		color = HsbColor::LinearBlend<NeoHueBlendClockwiseDirection>(rainbowStart, rainbowEnd, rainbowProgress);
	}
	for (int x = 0; x < PanelWidth; x++) {
		for (int y = 0; y < PanelHeight; y++) {
			strip.SetPixelColor(tiles.Map(x, y), color);
		}
	}
	for (int x = 0; x < PanelWidth; x++) {
		for (int y = 0; y < PanelHeight; y++) {
			animation *anim = &animationsArray[y][x];
			if (anim->available)
				continue;

			switch (anim->type) {
			case STAR: {
				switch (anim->state) {
				case 0:
				case 2: {
					strip.SetPixelColor(tiles.Map(x, y), anim->color);
					anim->stateSwitchCycles--;
					if (anim->stateSwitchCycles == 0) {
						anim->state++;
						anim->stateSwitchCycles = STATE_SWITCH_CYCLES;
					}
					break;
				}
				case 1: {
					strip.SetPixelColor(tiles.Map(x + 1, y), anim->color);
					strip.SetPixelColor(tiles.Map(x - 1, y), anim->color);
					strip.SetPixelColor(tiles.Map(x, y + 1), anim->color);
					strip.SetPixelColor(tiles.Map(x, y - 1), anim->color);
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
					strip.SetPixelColor(tiles.Map(x, y), inverseColor);
					anim->stateSwitchCycles--;
					if (anim->stateSwitchCycles == 0) {
						anim->state++;
						anim->stateSwitchCycles = STATE_SWITCH_CYCLES;
					}
					break;
				}
				case 1: {
					strip.SetPixelColor(tiles.Map(x, y), inverseColor);
					strip.SetPixelColor(tiles.Map(x, y + 1), inverseColor);
					strip.SetPixelColor(tiles.Map(x, y - 1), inverseColor);
					anim->stateSwitchCycles--;
					if (anim->stateSwitchCycles == 0) {
						anim->state++;
						anim->stateSwitchCycles = STATE_SWITCH_CYCLES;
					}
					break;
				}
				case 2: {
					strip.SetPixelColor(tiles.Map(x, y + 1), inverseColor);
					strip.SetPixelColor(tiles.Map(x, y - 1), inverseColor);
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
	if (animations >= maxAnimations) {
		return;
	}
	int pixel = rand() % (PixelCount);
	while (animationsArray[pixel % PanelWidth][pixel / PanelWidth].available == false) {
		//TODO better algorithm for placement
		pixel = (pixel + 1) % (PixelCount);
	}
	int x = pixel % PanelWidth;
	int y = pixel / PanelWidth;
	animationsArray[y][x].available = false;
	if (type == STRIPE) {
		animationsArray[y][x].color = green;
	} else {
		animationsArray[y][x].color = white;
	}
	animationsArray[y][x].state = 0;
	animationsArray[y][x].type = type;
	animationsArray[y][x].stateSwitchCycles = STATE_SWITCH_CYCLES;
	animations++;
}

void pulseToBase(void *params) {
	for (int i = 0; i < NBANDS; i++) {
		if (i < lastBaseIndex) {
			if (bandValues[i] > baseThreshold) {
				if (!(baseCycles > baseCyclesThreshold)) {
					baseCycles++;
				}
			}
		} else if (i < lastMidIndex) {
			if (bandValues[i] > midThreshold) {
				if (addAnimations)
					addAnimation(STAR);
			}
		} else {
			if (bandValues[i] > highThreshold) {
				if (addAnimations)
					addAnimation(STRIPE);
			}
		}
	}
	showAnimations();
	strip.Show();
	addAnimations = false;
}

void (*animationTypes[])(void *) = { showSpectrum, pulseToBase };

void callMatrixFunction(void *params) {
	unsigned long matrixStart;
	float voltage;
	for (;;) {
		TIMERG1.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
		TIMERG1.wdt_feed = 1;
		TIMERG1.wdt_wprotect = 0;
		matrixStart = millis();
		if (abs(millis() - lastSwitchTime) > functionSwitchTime) {
			lastSwitchTime = millis();
			std::random_shuffle(animationTypes, std::end(animationTypes));
			chosenAnimation = animationTypes[0];
			animations = 0;
			if (DEBUG) {
				voltage = 0;
				for (int i = 0; i < 64; i++) {
					voltage += (HIGH_VOLTAGE * (float)analogRead(tempPin)) / 4096.0;
				}
				voltage /= 64;
				Serial.printf("Temp sensor voltage: %f mV\n", voltage);
				//TODO: temperature measurement + fan curve + fan output (waiting for parts to arrive) -> remove DEBUG
			}
		}
		//TODO: debug pulsetobase, remove!!!!
		// chosenAnimation = pulseToBase;
		chosenAnimation(params);
		functionSwitchTime = rand() % (functionSwitchTimeMax - functionSwitchTimeMin) + functionSwitchTimeMin;
		rainbowProgress = fmodf((rainbowProgress + rainbowStep), 1.0);
		vTaskDelay(1 / portTICK_PERIOD_MS);
		while (abs(millis() - matrixStart) < 1000 / MATRIX_FPS)
			;
	}
}

void setup() {
	setCpuFrequencyMhz(240);
	Serial.begin(115200);
	WiFi.mode(WIFI_OFF);
	btStop();
	for (int i = 0; i < PanelHeight; i++) {
		for (int j = 0; j < PanelWidth; j++) {
			animationsArray[i][j].available = true;
		}
	}
	pinMode(micPin, INPUT);
	pinMode(tempPin, INPUT);
	pinMode(fanPin, OUTPUT);
	analogReadResolution(12);
	analogWriteResolution(fanPin, 10);
	analogWriteFrequency(fanPin, 40000);
	calibrate_silence();
	// put your setup code here, to run once:
	// uint16_t result = calibrate_silence();
	// getSampleSpeed();
	strip.Begin();
	for (int i = 0; i < PanelWidth; i++) {
		for (int j = 0; j < PanelHeight; j++) {
			strip.SetPixelColor(tiles.Map(i, j), red);
			strip.Show();
		}
	}
	strip.ClearTo(RgbColor(0, 0, 0));
	strip.Show();
	std::random_shuffle(animationTypes, std::end(animationTypes));
	chosenAnimation = pulseToBase;
	xTaskCreatePinnedToCore(
	    callMatrixFunction,
	    "Matrix rendering",
	    2048,
	    NULL,
	    10,
	    &MatrixTask,
	    0);
	analogWrite(fanPin, 512);
}

void loop() {
	getSamples(NSAMPLES);
	std::fill(imag, std::end(imag), 0.0);
	doFFT();
	int i = 2;
	double freq = (i * SAMPLE_RATE) / NSAMPLES;
	// while (freq < FREQ_THRESHOLD) {
	//     i++;
	//     freq = (i * SAMPLE_RATE) / NSAMPLES;
	// }
	if (DEBUG) {
		Serial.printf("Starting from: %lf\n", freq);
	}
	for (int bIndex = 0; bIndex < NBANDS; bIndex++) {
		const int band = BANDS[bIndex];
		double result = 0;
		int nbands = 0;
		while (freq < band) {
			freq = (i * SAMPLE_RATE) / NSAMPLES;
			result += real[i];
			nbands++;
			i++;
		}
		result /= nbands;
		// 220 - noise
		if (THRESHOLD > result) {
			result = 0.0;
		}
		bandValues[bIndex] = result;
		if (DEBUG) {
			Serial.printf("%d Hz: %lf\n", band, result);
		}
	}
	addAnimations = true;
	// for (int i = 0; i < NSAMPLES / 2; i++) {
	//     // Serial.printf("Peak: %lf\n", peak);
	//     Serial.printf("%f Hz: %lf\n", freq, real[i]);
	//     // Serial.printf("%f Hz: %i\n", FFT_BIN(i, SAMPLE_RATE, NSAMPLES), readings[i]);
	// }
	if (DEBUG) {
		Serial.printf("---------\n");
	}
}
