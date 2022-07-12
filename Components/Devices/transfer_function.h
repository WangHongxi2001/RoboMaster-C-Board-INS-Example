/**
  ******************************************************************************
  * @file    transfer_function.h
  * @author  Wang Hongxi
  * @version V1.0.0
  * @date    2020/5/12
  * @brief   
  ******************************************************************************
  * @attention 
  *
  ******************************************************************************
  */
#ifndef __TRANSFER_FUNCTION_H
#define __TRANSFER_FUNCTION_H

#include "stdint.h"
#include "stdlib.h"
#include "math.h"
#include "bsp_dwt.h"

#ifdef USE_HAL_DRIVER
#include "main.h"
#endif

typedef struct
{
    uint8_t Order; //ÏµÍ³½×Êý

    double *NUM;
    double *DEN;

} Transfer_Func_t;

typedef struct
{
    double dt; // Control period

    double c[3]; // G(s) = 1/(c2s^2 + c1s + c0)

    double u; //System input

    double y;      // System output
    double y_dot;  // First-Order derivative of system output
    double y_ddot; // Second-Order derivative of system output
    double Last_y_dot;
    double Last_y_ddot;

    uint32_t DWT_CNT;
} Second_Order_TF_t;

void Second_Order_TF_Init(Second_Order_TF_t *tf, float *c);
double Second_Order_TF_Calculate(Second_Order_TF_t *tf, double input);
double Gauss_Rand(void);

#endif
