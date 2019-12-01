#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/imu_silver.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/gps.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

// TODO: remove unused includes

float GEstG[3] = {0.0, 0.0, 1.0f};
float heading = 0;

void imuSilverCalc(float dt, float gx, float gy, float gz, float ax, float ay, float az)
{
    float deltaGyroAngle[3];

    deltaGyroAngle[0] = gx * dt;
    deltaGyroAngle[1] = gy * dt;
    deltaGyroAngle[2] = gz * dt;

    GEstG[2] = GEstG[2] - (deltaGyroAngle[0]) * GEstG[0];
    GEstG[0] = (deltaGyroAngle[0]) * GEstG[2] +  GEstG[0];
    GEstG[1] =  GEstG[1] + (deltaGyroAngle[1]) * GEstG[2];
    GEstG[2] = -(deltaGyroAngle[1]) * GEstG[1] +  GEstG[2];
    GEstG[0] = GEstG[0] - (deltaGyroAngle[2]) * GEstG[1];
    GEstG[1] = (deltaGyroAngle[2]) * GEstG[0] +  GEstG[1];

    heading += deltaGyroAngle[2];   

    attitude.values.roll = lrintf( atan2_approx(GEstG[X],GEstG[Z]) * 1800.f / M_PIf );
    attitude.values.pitch = lrintf( atan2_approx( GEstG[Y], sqrtf(GEstG[X]*GEstG[X] + GEstG[Z]*GEstG[Z])) * 1800.f / M_PIf);
    attitude.values.yaw = lrintf( -heading * 1800.f / M_PIf);

    if (attitude.values.yaw < 0)
        attitude.values.yaw += 3600;
}

