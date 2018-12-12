#pragma once

extern int SlaveCnt;
void ScanForSlave();
void manageSlave();
void setupEspNow();
void sendData(const void *data, size_t n_bytes);