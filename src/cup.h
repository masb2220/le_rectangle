#pragma once
void cupBegin();
void cupUpdate();
bool cupStart(bool pickupAfterScan = true);
bool pinStart();
int cupResult();
void cupAbort();
void cupPrintStatus();
bool cupAdjustOffset(int direction);
void cupPrintOffset();
