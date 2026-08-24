#line 1 "D:\\Projects\\ov2640\\esp32s3_camera_serial\\lib\\esp32-camera\\sensors\\private_include\\bf3005.h"
/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * BF3005 driver.
 *
 */
#ifndef __BF3005_H__
#define __BF3005_H__
#include "sensor.h"

/**
 * @brief Detect sensor pid
 *
 * @param slv_addr SCCB address
 * @param id Detection result
 * @return
 *     0:       Can't detect this sensor
 *     Nonzero: This sensor has been detected
 */
int esp32_camera_bf3005_detect(int slv_addr, sensor_id_t *id);

/**
 * @brief initialize sensor function pointers
 *
 * @param sensor pointer of sensor
 * @return
 *      Always 0
 */
int esp32_camera_bf3005_init(sensor_t *sensor);

#endif // __BF3005_H__