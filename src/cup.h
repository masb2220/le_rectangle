#pragma once
void cupBegin();
void cupUpdate();
bool cupStart(bool pickupAfterScan = true);
int cupResult();
void cupAbort();
void cupPrintStatus();
bool cupAdjustOffset(int direction);
void cupPrintOffset();
