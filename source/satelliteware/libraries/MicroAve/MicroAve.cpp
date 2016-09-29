// MicroAve.cpp : Timing and averaging debug class

/*
 *  Copyright (c) 2016 Wind River Systems, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 */

/* 
 * <credits>
 *   { David Reyna,  david.reyna@windriver.com,  },
 * </credits>
 */

#include <Arduino.h>
#include "MicroAve.h"

void MicroAve::reset()
{
  this->isFirstTime = 0L;
  this->lastTime = 0L;

  this->sum = 0L;
  this->count = 0L;

  this->min0 = 2L<<20;
  this->min1 = 2L<<20;
  this->min2 = 2L<<20;

  this->max0 = 0L;
  this->max1 = 0L;
  this->max2 = 0L;

  for (int i=0;i < HISTOGRAM_MAX; i++) {
    this->hist[i] = 0L;
  }

};

MicroAve::MicroAve() {
  this->reset();
}

void MicroAve::setStart() {
  this->isFirstTime = 1;
  this->lastTime = (unsigned long) micros();
}

void MicroAve::setStop() {
  unsigned long u_diff;
  unsigned long nextTime;

  if (0 == this->isFirstTime) return;

  nextTime = (unsigned long) micros();
  if (nextTime < this->lastTime) {
    // roll over
    nextTime += 2L << 16;
  }
  u_diff = nextTime - this->lastTime;
  addValue(u_diff);
}

void MicroAve::addValue(unsigned long value) {

  this->sum += value;
  this->count++;

  if (value < this->min2) {
    this->min0 = this->min1;
    this->min1 = this->min2;
    this->min2 = value;
  } else if (value < this->min1) {
    this->min0 = this->min1;
    this->min1 = value;
  } else if (value < this->min0) {
    this->min0 = value;
  }

  if (value > this->max2) {
    this->max0 = this->max1;
    this->max1 = this->max2;
    this->max2 = value;
  } else if (value > this->max1) {
    this->max0 = this->max1;
    this->max1 = value;
  } else if (value > this->max0) {
    this->max0 = value;
  }

  if (value < HISTOGRAM_MAX) {
    this->hist[value]++;
  }
}

void MicroAve::displayResults(const char *msg, int showHist) {
  Serial.print("Ave [");
  Serial.print(msg);
  Serial.print("]:");

  Serial.print(" Max=");
  Serial.print(this->max2);
  Serial.print(",");
  Serial.print(this->max1);
  Serial.print(",");
  Serial.print(this->max0);

  Serial.print(" [");
  Serial.print(this->count);
  Serial.print("]=");
  if (0 == this->count)
	  Serial.print("none");
  else
	  Serial.print(this->sum/this->count);
  Serial.print(" uSec, Min=");

  Serial.print(this->min0);
  Serial.print(",");
  Serial.print(this->min1);
  Serial.print(",");
  Serial.println(this->min2);

  if (showHist) {
    Serial.println("Histogram:");
    for (int i=0;i < HISTOGRAM_MAX; i++) {
      if (this->hist[i]) {
        Serial.print(" [");
        Serial.print(i);
        Serial.print("]=");
        Serial.print(this->hist[i]);
        Serial.print(" (");
        Serial.print((this->hist[i]*100L)/this->count);
        Serial.println(")");
      }
    }
  }

  Serial.println("");
}
