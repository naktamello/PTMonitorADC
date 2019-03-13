//
// Created by camel on 7/4/18.
//

#ifndef PROJECT_US_DELAY_H
#define PROJECT_US_DELAY_H

#include <stdint-gcc.h>

static inline void us_Delay(uint32_t us) {
    if (us > 1) {
        volatile uint32_t count = us * 13;
        while (count--);
    } else {
        volatile uint32_t count = 8;
        while (count--);
    }
}

#endif //PROJECT_US_DELAY_H
