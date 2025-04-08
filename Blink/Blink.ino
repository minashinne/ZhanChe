#include <Adafruit_NeoPixel.h>

#define LED_PIN 14
#define LED_COUNT 48
#define NUM_COLS 8
#define NUM_ROWS 6

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

int meteorY[8] = { 0 };  // 每行的流星纵向位置（列号）
unsigned long lastUpdate = 0;
const int delayMs = 100;

bool meteorRainActive = false;

struct Meteor {
  int headY;
  int delayMs;
  unsigned long lastMoveTime;
  uint32_t color;
};

Meteor meteors[6];  // 每行一个流星


void setup() {
  strip.begin();
  strip.show();
  Serial.begin(115200);
  xTaskCreatePinnedToCore(
    meteorRainTask,
    "Meteor Rain",
    4096,
    NULL,
    2,
    NULL,
    1  // 运行在 Core1
  );
}

// 根据列和行返回LED index
int getLedIndex(int col, int row) {
  // int map[8][6] = {
  //   { 0, 1, 2, 3, 4, 5 },
  //   { 11, 10, 9, 8, 7, 6 },
  //   { 12, 13, 14, 15, 16, 17 },
  //   { 23, 22, 21, 20, 19, 18 },
  //   { 24, 25, 26, 27, 28, 29 },
  //   { 35, 34, 33, 32, 31, 30 },
  //   { 36, 37, 38, 39, 40, 41 },
  //   { 47, 46, 45, 44, 43, 42 }
  // };
    int map[8][6] = {
    {14, 15, 20, 21, 26, 27},
    { 7,  8,  9, 10, 13, 16},
    { 0,  1,  2,  3,  4,  5},
    {11, 12, 17, 18, 23, 24},
    { 6, 19, 22, 25, 28, 31},
    {32, 33, 34, 37, 38, 39},
    {30, 35, 36, 41, 42, 43},
    {29, 40, 44, 45, 46, 47}
  };
  return map[row][col];
}


void loop() {
  parseSerialCommands();
}

void parseSerialCommands() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "METEOR ON") {
      meteorRainActive = true;
      Serial.println("流星雨已启动");
    } else if (cmd == "METEOR OFF") {
      meteorRainActive = false;
      strip.clear();
      strip.show();
      Serial.println("流星雨已停止");
    }
  }
}


void meteorRainTask(void *pvParameters) {
  // 初始化每个流星
  for (int i = 0; i < 6; i++) {
    meteors[i].headY = random(-8, 0);       // 从上方随机开始
    meteors[i].delayMs = random(100, 300);  // 每列速度不同
    meteors[i].lastMoveTime = millis();
    meteors[i].color = strip.Color(random(100, 255), random(100, 255), random(100, 255));
  }

  while (1) {
    if (meteorRainActive) {
      unsigned long now = millis();

      for (int col = 0; col < 6; col++) {
        if (now - meteors[col].lastMoveTime >= meteors[col].delayMs) {
          meteors[col].lastMoveTime = now;

          // 擦除尾巴（所有LED颜色衰减）
          for (int row = 0; row < 8; row++) {
            int idx = getLedIndex(col, row);
            uint32_t c = strip.getPixelColor(idx);
            uint8_t r = (uint8_t)(c >> 16) * 0.6;
            uint8_t g = (uint8_t)(c >> 8) * 0.6;
            uint8_t b = (uint8_t)(c)*0.6;
            strip.setPixelColor(idx, r, g, b);
          }

          // 设置新位置
          int newY = meteors[col].headY;
          if (newY >= 0 && newY < 8) {
            int idx = getLedIndex(col, newY);
            strip.setPixelColor(idx, meteors[col].color);
          }

          meteors[col].headY++;
          if (meteors[col].headY > 8 + random(0, 5)) {
            meteors[col].headY = random(-8, 0);  // 重置回顶部
            meteors[col].color = strip.Color(random(100, 255), random(100, 255), random(100, 255));
          }
        }
      }

      strip.show();
    }

    vTaskDelay(pdMS_TO_TICKS(30));
  }
}
