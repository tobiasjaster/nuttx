/****************************************************************************
 * include/nuttx/ioexpander/bl602_expander.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_IOEXPANDER_STM32L4_EXPANDER_H
#define __INCLUDE_NUTTX_IOEXPANDER_STM32L4_EXPANDER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/ioexpander/ioexpander.h>
#include "./stm32l4_gpio.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_STM32L4_EXPANDER
/* Public instance of STM32L4 GPIO Expander */

extern FAR struct ioexpander_dev_s *stm32l4_expander;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_STM32L4_EXPANDER
/* Init BL602 GPIO Expander */

FAR struct ioexpander_dev_s *stm32l4_expander_initialize(
  const uint32_t *gpios,
  uint8_t gpio_count);
#endif

#endif /* __INCLUDE_NUTTX_IOEXPANDER_STM32L4_EXPANDER_H */