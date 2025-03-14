/*************************************************************************
 *
 * Copyright 2023 ETH Zurich and University of Bologna
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Author: Giovanni Bambini (gv.bambini@gmail.com)
 *
 **************************************************************************/

#include "print_float.h"
// #include "pmsis.h"
#include <stdio.h>

float fAbs(float x) {
    return ((x < 0.0F) ? (-x) : (x));
}

void printFloatnl(float f) {
    printFloat(f);
    printf("\r\n");
}
void printFloat(float f) {
    int j = 100000000;
    int i = j;
    float fi;
    char sign;
    unsigned int *ph = (unsigned int *)&f;
    unsigned int h = *ph;
    sign = (f < 0.0) ? '-' : '+';

    f = fAbs(f);
    fi = (float)i;
    i = (int)f;
    i = (int)((fi * f) - ((float)i) * fi);

    if (i < 0) {
        i += j;
        j = ((int)f) - 1;
    } else {
        j = (int)f;
    }

    printf("%c%d.%08d", sign, j, i);
    //  printf("%c%d.%08d (mantissa) %d exponent %d",sign,j,i, h & 0x7FFFF,  h & 0x7F800000 >> 23);
    return;
}
