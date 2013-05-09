#include "stm32f4xx.h"

void iddle();

void iddle() {
	int32_t index;
	while (1) {
		if (index == 0xFFFFFFFF) {
			index = 0;
		} else {
			index++;
		}
	}
}
