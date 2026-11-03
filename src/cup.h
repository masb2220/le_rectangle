#pragma once
void cupBegin();
void cupUpdate();
bool cupStart(bool pickupAfterScan = true);
void cupAbort();
void cupPrintStatus();
bool cupAdjustOffset(int direction);
void cupPrintOffset();
