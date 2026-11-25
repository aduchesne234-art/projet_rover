// camera_web.h - OV2640 camera + web servers (built-in ESP-IDF esp_http_server, no extra library)
//
//   port 80 : "/"     -> control page (index_html.h)
//             "/api"  -> control + status:  /api?l=0.5&r=-0.2&mode=auto
//   port 81 : "/stream" -> MJPEG video, shown in an <img> tag on the page
//
// The stream runs on its own server because a stream handler never returns
// while someone is watching, and it must not block the control requests.
//
// For ORB-SLAM later: this file is the only place that touches the camera,
// so your colleague can add frame processing / another endpoint here.
#pragma once
#include <Arduino.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "config.h"
#include "rover.h"
#include "index_html.h"

extern Rover rover;   // defined in main.cpp

// ---------------------------------------------------------------------
// Camera
// ---------------------------------------------------------------------
inline bool cameraInit() {
  camera_config_t c = {};
  // Motors use LEDC channels 0/1 -> give the camera's XCLK a different channel + timer.
  c.ledc_channel = LEDC_CHANNEL_7;
  c.ledc_timer   = LEDC_TIMER_3;
  c.pin_d0 = CAM_Y2; c.pin_d1 = CAM_Y3; c.pin_d2 = CAM_Y4; c.pin_d3 = CAM_Y5;
  c.pin_d4 = CAM_Y6; c.pin_d5 = CAM_Y7; c.pin_d6 = CAM_Y8; c.pin_d7 = CAM_Y9;
  c.pin_xclk = CAM_XCLK;
  c.pin_pclk = CAM_PCLK;
  c.pin_vsync = CAM_VSYNC;
  c.pin_href = CAM_HREF;
  c.pin_sccb_sda = CAM_SIOD;
  c.pin_sccb_scl = CAM_SIOC;
  c.pin_pwdn = -1;
  c.pin_reset = -1;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    c.frame_size = CAM_FRAMESIZE;
    c.jpeg_quality = CAM_JPEG_QUALITY;
    c.fb_count = 2;
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.grab_mode = CAMERA_GRAB_LATEST;      // always the newest frame (low latency)
  } else {
    Serial.println("No PSRAM: using small frames");
    c.frame_size = FRAMESIZE_QVGA;
    c.jpeg_quality = 15;
    c.fb_count = 1;
    c.fb_location = CAMERA_FB_IN_DRAM;
    c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x (check wiring / PSRAM setting)\n", err);
    return false;
  }
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, CAM_VFLIP);
    s->set_hmirror(s, CAM_HMIRROR);
  }
  return true;
}

// ---------------------------------------------------------------------
// Handlers
// ---------------------------------------------------------------------
static esp_err_t rootHandler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

// /api?l=<-1..1>&r=<-1..1>&mode=auto|manual   (all optional) -> status JSON
static esp_err_t apiHandler(httpd_req_t* req) {
  char query[96];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    char vl[16], vr[16], vm[16];
    bool hasL = httpd_query_key_value(query, "l", vl, sizeof(vl)) == ESP_OK;
    bool hasR = httpd_query_key_value(query, "r", vr, sizeof(vr)) == ESP_OK;
    if (hasL && hasR) rover.command(atof(vl), atof(vr));
    if (httpd_query_key_value(query, "mode", vm, sizeof(vm)) == ESP_OK) {
      if (strcmp(vm, "auto") == 0) rover.requestMode(true);
      else if (strcmp(vm, "manual") == 0) rover.requestMode(false);
    }
  }
  char json[256];
  rover.statusJson(json, sizeof(json));
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

#define PART_BOUNDARY "rovercamframe"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t streamHandler(httpd_req_t* req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char part[64];
  while (true) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      res = ESP_FAIL;
    } else {
      res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
      if (res == ESP_OK) {
        size_t hlen = snprintf(part, sizeof(part), STREAM_PART, (unsigned)fb->len);
        res = httpd_resp_send_chunk(req, part, hlen);
      }
      if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
      esp_camera_fb_return(fb);
    }
    if (res != ESP_OK) break;   // viewer left (or camera error)
  }
  return res;
}

// ---------------------------------------------------------------------
// Start the servers. Control works even if the camera failed to start.
// ---------------------------------------------------------------------
inline void startServers(bool withCamera) {
  httpd_handle_t ctl = NULL;
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  cfg.max_open_sockets = 3;
  cfg.lru_purge_enable = true;          // recycle idle connections (phones open several)
  httpd_uri_t root = {"/", HTTP_GET, rootHandler, NULL};
  httpd_uri_t api  = {"/api", HTTP_GET, apiHandler, NULL};
  if (httpd_start(&ctl, &cfg) == ESP_OK) {
    httpd_register_uri_handler(ctl, &root);
    httpd_register_uri_handler(ctl, &api);
  } else {
    Serial.println("Failed to start control server");
  }

  if (!withCamera) return;
  httpd_handle_t strm = NULL;
  httpd_config_t scfg = HTTPD_DEFAULT_CONFIG();
  scfg.server_port = 81;
  scfg.ctrl_port += 1;                  // two servers need different control ports
  scfg.max_open_sockets = 2;
  scfg.lru_purge_enable = true;
  httpd_uri_t stream = {"/stream", HTTP_GET, streamHandler, NULL};
  if (httpd_start(&strm, &scfg) == ESP_OK) {
    httpd_register_uri_handler(strm, &stream);
  } else {
    Serial.println("Failed to start stream server");
  }
}
