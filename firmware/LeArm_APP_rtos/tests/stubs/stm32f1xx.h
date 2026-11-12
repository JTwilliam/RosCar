#ifndef SAFETY_HOST_STM32F1XX_H
#define SAFETY_HOST_STM32F1XX_H
#include <stdint.h>
uint32_t safety_host_ipsr(void);
#define __get_IPSR() safety_host_ipsr()
#endif
