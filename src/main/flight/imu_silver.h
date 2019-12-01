#pragma once

#include "common/axis.h"
#include "common/time.h"
#include "common/maths.h"
#include "pg/pg.h"

void imuSilverCalc(float dt, float gx, float gy, float gz, float ax, float ay, float az);
