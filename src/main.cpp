// 1) Motors off first, then sensors + gyro calibration (keep the rover still)
  rover.begin();

  // 2) Camera (stream only; no processing yet)
  bool cam = cameraInit();

  // 3) Wi-Fi access point: works anywhere, no router needed
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD, 1, 0, 2);   // channel 1, visible, max 2 clients
  WiFi.setSleep(false);                         // lower latency
  WiFi.setTxPower(WIFI_POWER_19_5dBm);          // max range
  Serial.printf("Wi-Fi '%s' ready -> http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  // 4) Web servers (control page + API on :80, video on :81)
  startServers(cam);
  Serial.println("Ready. Rover is in MANUAL mode and stopped.");
}

void loop() {
  rover.update();
  delay(LOOP_MS);
}
