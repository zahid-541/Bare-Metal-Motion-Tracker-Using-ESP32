#include <Arduino.h>
#include "esp_heap_caps.h"

// ===== 1. RESOLUTION & MEMORY =====
#define FRAME_W       160
#define FRAME_H       120
#define FRAME_SIZE    (FRAME_W * FRAME_H)   // 19,200 bytes

#define SUB_W         80
#define SUB_H         60
#define SUB_SIZE      (SUB_W * SUB_H)       // 4,800 pixels

// Permanent memory pointers
static uint8_t* cur_frame = nullptr;
static uint8_t  bg_model[SUB_SIZE];
static uint8_t  mask[SUB_SIZE];
static uint16_t flood_queue[SUB_SIZE];

// ===== 2. DETECTION TUNING =====
static const uint8_t  PIXEL_THRESH  = 25;   
static const uint16_t MIN_BLOB_AREA = 15;   
static const uint8_t  MAX_BOXES     = 8;    
static const uint16_t MATCH_DIST    = 40;
static const uint8_t  MAX_MISSED    = 5;
static const uint16_t BOX_PADDING   = 8;

// ===== 3. SERIAL PROTOCOL =====
static const uint8_t SYNC[4]   = {0xAA, 0xBB, 0xCC, 0xDD};
static const uint8_t ACK_BYTE  = 0x5A;
static const uint8_t BOOT_BYTE = 0xEE;

struct Box {
  uint16_t min_x, min_y, max_x, max_y;
};

struct Remembered {
  uint16_t min_x, min_y, max_x, max_y;
  uint8_t  missed;
  bool     active;
};
static Remembered remembered[MAX_BOXES];

// ===== 4. FAST MATH (Bit-Shift Background) =====
static IRAM_ATTR uint8_t xor_sum(const uint8_t* __restrict data, size_t n) {
  uint8_t s = 0;
  for (size_t i = 0; i < n; i++) s ^= data[i];
  return s;
}

static IRAM_ATTR void process_background_and_mask(const uint8_t* __restrict new_frame) {
  uint8_t* m = mask;
  for (int y = 0; y < SUB_H; y++) {
    for (int x = 0; x < SUB_W; x++) {
      int orig_idx = (y * 2 * FRAME_W) + (x * 2);
      int sub_idx  = (y * SUB_W) + x;

      uint8_t new_px = new_frame[orig_idx];
      uint8_t bg_px  = bg_model[sub_idx];

      // Integer-based background accumulation (Learning Rate ~ 0.125)
      bg_px = bg_px - (bg_px >> 3) + (new_px >> 3);
      bg_model[sub_idx] = bg_px;

      // Difference & Threshold
      int diff = (int)new_px - (int)bg_px;
      if (diff < 0) diff = -diff;
      *m++ = ((uint8_t)diff > PIXEL_THRESH) ? 1 : 0;
    }
  }
}

// ===== 5. FLOOD FILL & TRACKING =====
static IRAM_ATTR Box flood_fill(int start_x, int start_y, uint16_t* area_out) {
  Box b = { (uint16_t)start_x, (uint16_t)start_y, (uint16_t)start_x, (uint16_t)start_y };
  *area_out = 0;
  int qhead = 0, qtail = 0;
  
  flood_queue[qtail++] = start_y * SUB_W + start_x;
  mask[start_y * SUB_W + start_x] = 2;

  while (qhead < qtail) {
    uint16_t idx = flood_queue[qhead++];
    int x = idx % SUB_W;
    int y = idx / SUB_W;
    (*area_out)++;

    if (x < b.min_x) b.min_x = x;
    if (x > b.max_x) b.max_x = x;
    if (y < b.min_y) b.min_y = y;
    if (y > b.max_y) b.max_y = y;

    if (x > 0 && mask[idx - 1] == 1) { mask[idx - 1] = 2; flood_queue[qtail++] = idx - 1; }
    if (x < SUB_W - 1 && mask[idx + 1] == 1) { mask[idx + 1] = 2; flood_queue[qtail++] = idx + 1; }
    if (y > 0 && mask[idx - SUB_W] == 1) { mask[idx - SUB_W] = 2; flood_queue[qtail++] = idx - SUB_W; }
    if (y < SUB_H - 1 && mask[idx + SUB_W] == 1) { mask[idx + SUB_W] = 2; flood_queue[qtail++] = idx + SUB_W; }
  }
  return b;
}

static inline uint16_t cx(uint16_t mn, uint16_t mx) { return (mn + mx) / 2; }
static uint32_t dist2(uint16_t ax, uint16_t ay, uint16_t bx, uint16_t by) {
  int32_t dx = (int32_t)ax - (int32_t)bx;
  int32_t dy = (int32_t)ay - (int32_t)by;
  return (uint32_t)(dx * dx + dy * dy);
}

static void update_remembered(const Box* dets, uint8_t num_dets) {
  bool det_used[MAX_BOXES] = {false};
  const uint32_t max_d2 = (uint32_t)MATCH_DIST * MATCH_DIST;

  for (int i = 0; i < MAX_BOXES; i++) {
    if (!remembered[i].active) continue;
    uint16_t rx = cx(remembered[i].min_x, remembered[i].max_x);
    uint16_t ry = cx(remembered[i].min_y, remembered[i].max_y);

    int best = -1;
    uint32_t best_d = max_d2 + 1;
    for (int d = 0; d < num_dets; d++) {
      if (det_used[d]) continue;
      uint32_t dd = dist2(rx, ry, cx(dets[d].min_x, dets[d].max_x), cx(dets[d].min_y, dets[d].max_y));
      if (dd < best_d) { best_d = dd; best = d; }
    }

    if (best >= 0 && best_d <= max_d2) {
      remembered[i].min_x = dets[best].min_x; remembered[i].min_y = dets[best].min_y;
      remembered[i].max_x = dets[best].max_x; remembered[i].max_y = dets[best].max_y;
      remembered[i].missed = 0;
      det_used[best] = true;
    } else {
      remembered[i].missed++;
      if (remembered[i].missed > MAX_MISSED) remembered[i].active = false; 
    }
  }

  for (int d = 0; d < num_dets; d++) {
    if (det_used[d]) continue;
    for (int i = 0; i < MAX_BOXES; i++) {
      if (!remembered[i].active) {
        remembered[i].active = true;
        remembered[i].min_x = dets[d].min_x; remembered[i].min_y = dets[d].min_y;
        remembered[i].max_x = dets[d].max_x; remembered[i].max_y = dets[d].max_y;
        remembered[i].missed = 0;
        break;
      }
    }
  }
}

// ===== 6. SERIAL HELPERS =====
static bool read_exact(uint8_t* dst, size_t n, uint32_t timeout_ms = 2000) {
  size_t got = 0;
  uint32_t start = millis();
  while (got < n) {
    int avail = Serial.available();
    if (avail > 0) {
      int r = Serial.readBytes(dst + got, min((size_t)avail, n - got));
      if (r > 0) got += r;
    }
    if (millis() - start > timeout_ms) return false;
  }
  return true;
}

static bool wait_for_sync() {
  int matched = 0;
  uint32_t start = millis();
  while (true) {
    if (Serial.available() <= 0) {
      delayMicroseconds(50);
      if (millis() - start > 5000) return false;
      continue;
    }
    uint8_t b = Serial.read();
    if (b == SYNC[matched]) {
      matched++;
      if (matched == 4) return true;
    } else {
      matched = (b == SYNC[0]) ? 1 : 0;
    }
  }
}

static void send_response(uint16_t frame_id, uint8_t status, const Box* boxes, uint8_t num_boxes) {
  uint8_t pkt[5 + MAX_BOXES * 8];
  pkt[0] = ACK_BYTE;
  pkt[1] = frame_id & 0xFF;
  pkt[2] = (frame_id >> 8) & 0xFF;
  pkt[3] = status;
  pkt[4] = num_boxes;

  uint8_t* p = &pkt[5];
  for (uint8_t i = 0; i < num_boxes; i++) {
    *p++ = boxes[i].min_x & 0xFF;  *p++ = (boxes[i].min_x >> 8) & 0xFF;
    *p++ = boxes[i].min_y & 0xFF;  *p++ = (boxes[i].min_y >> 8) & 0xFF;
    *p++ = boxes[i].max_x & 0xFF;  *p++ = (boxes[i].max_x >> 8) & 0xFF;
    *p++ = boxes[i].max_y & 0xFF;  *p++ = (boxes[i].max_y >> 8) & 0xFF;
  }
  Serial.write(pkt, 5 + num_boxes * 8);
  Serial.flush();
}

// ===== SETUP & MAIN LOOP =====
void setup() {
  Serial.begin(4000000);
  Serial.setRxBufferSize(32768);
  Serial.setTimeout(50);
  delay(200);

  if (!psramFound()) {
    while (1) { Serial.write(0xFA); delay(500); } // Fatal error heartbeat
  }

  // Allocate exactly one frame in PSRAM to receive the PC data
  cur_frame = (uint8_t*)heap_caps_malloc(FRAME_SIZE, MALLOC_CAP_SPIRAM);
  if (!cur_frame) {
    while (1) { Serial.write(0xFB); delay(500); }
  }
  
  memset(cur_frame, 0, FRAME_SIZE);
  memset(bg_model, 0, SUB_SIZE);
  for (int i = 0; i < MAX_BOXES; i++) remembered[i].active = false;

  Serial.write(BOOT_BYTE);
  Serial.flush();
}

void loop() {
  // 1. Wait for PC to send header
  if (!wait_for_sync()) return;

  // 2. Read Frame ID
  uint8_t id_bytes[2];
  if (!read_exact(id_bytes, 2)) return;
  uint16_t frame_id = id_bytes[0] | (id_bytes[1] << 8);

  // 3. Download the full 160x120 raw image from PC
  if (!read_exact(cur_frame, FRAME_SIZE)) return;

  // 4. Verify checksum
  uint8_t rx_sum;
  if (!read_exact(&rx_sum, 1)) return;
  if (xor_sum(cur_frame, FRAME_SIZE) != rx_sum) {
    send_response(frame_id, 0xFF, nullptr, 0); // Checksum failed
    return;
  }

  // 5. CRUNCH THE MATH (Update background & generate mask)
  process_background_and_mask(cur_frame);

  // 6. Find Blobs
  Box dets[MAX_BOXES];
  uint8_t num_dets = 0;
  for (int y = 0; y < SUB_H; y++) {
    for (int x = 0; x < SUB_W; x++) {
      if (mask[y * SUB_W + x] == 1) {
        uint16_t area = 0;
        Box b = flood_fill(x, y, &area);
        
        if (area >= MIN_BLOB_AREA && num_dets < MAX_BOXES) {
          b.min_x *= 2; b.max_x = b.max_x * 2 + 1;
          b.min_y *= 2; b.max_y = b.max_y * 2 + 1;
          dets[num_dets++] = b;
        }
      }
    }
  }

  // 7. Track Objects
  update_remembered(dets, num_dets);

  // 8. Package final active boxes
  Box output[MAX_BOXES];
  uint8_t num_out = 0;
  for (int i = 0; i < MAX_BOXES && num_out < MAX_BOXES; i++) {
    if (!remembered[i].active) continue;
    
    // Apply Padding
    int32_t mn_x = (int32_t)remembered[i].min_x - BOX_PADDING;
    int32_t mn_y = (int32_t)remembered[i].min_y - BOX_PADDING;
    int32_t mx_x = (int32_t)remembered[i].max_x + BOX_PADDING;
    int32_t mx_y = (int32_t)remembered[i].max_y + BOX_PADDING;
    
    // Clamp to screen bounds
    output[num_out].min_x = (mn_x < 0) ? 0 : mn_x;
    output[num_out].min_y = (mn_y < 0) ? 0 : mn_y;
    output[num_out].max_x = (mx_x >= FRAME_W) ? FRAME_W - 1 : mx_x;
    output[num_out].max_y = (mx_y >= FRAME_H) ? FRAME_H - 1 : mx_y;
    num_out++;
  }

  // 9. Fire response back to PC
  send_response(frame_id, (num_out > 0) ? 1 : 0, output, num_out);
}