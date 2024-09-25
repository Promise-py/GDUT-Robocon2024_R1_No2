/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 Keten. All Rights Reserved.
 *
 * \file      path.h
 * \author    Keten
 * \version   1.0
 * \date      2024年5月12日
 * \brief     path.c 的头文件
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2024年5月12日
      author: Keten
      change: create file

*****************************************************************************/
#ifndef PATH_H
#define PATH_H

#include "stm32f4xx_hal.h"

extern float X0[4+15];
extern float Y0[4+15];
extern float Yaw0[4+15];

extern float X1[6];
extern float Y1[6];
extern float Yaw1[6];

#endif /* PATH_H */

