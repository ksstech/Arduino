// EspReporting.h - Copyright (c) André M. Maree / KSS Technologies (Pty) Ltd. 2025

void reportSDK(void) {
  fmtMessageLn("SDK: %s  CORE: %s", ESP.getSdkVersion(), ESP.getCoreVersion());
}

void reportMCU(void) {
  uint64_t macAddr = ESP.getEfuseMac();
  fmtMessageLn("%s  Rev:%hu  Cores:%hhu  Clock: %dMhz  MAC:0x%llX", ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(), ESP.getCpuFreqMHz(), macAddr);
}

void reportMEM(void) {
  uint32_t flashSize;
  esp_flash_get_physical_size(NULL, &flashSize);
  fmtMessageLn("Total Flash:%dMB  Heap: %d used of %dB total  PSRAM: %d/%dB", flashSize>>20, ESP.getFreeHeap(), ESP.getHeapSize(), ESP.getPsramSize(), ESP.getFreePsram());
}
