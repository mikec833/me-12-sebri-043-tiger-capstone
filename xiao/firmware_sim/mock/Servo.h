#pragma once
struct Servo { int us = 1500; void attach(int, int, int) {} void writeMicroseconds(int u) { us = u; } };
