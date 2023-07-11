/****************************************************************************
 * boards/arm/stm32l4/nucleo-l496zg/src/stm32_ioexpander.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include "stm32l4_i2c.h"
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/ioexpander/mcp23x17.h>
#include <nuttx/ioexpander/ioexpander.h>

#include "nucleo-144.h"

#if defined(CONFIG_DEV_GPIO) && defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_gpio_initialize
 *
 * Description:
 *   Initialize simulated GPIO expander for use with /apps/examples/gpio
 *
 ****************************************************************************/

int stm32l4_gpio_initialize(void)
{
  struct i2c_master_s *i2c;
  struct mcp23x17_config_s *config;
  syslog(LOG_INFO, "INFO: Get I2C2 instance.\n");
  i2c = stm32l4_i2cbus_initialize(2);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  gpioinfo("INFO: Initializing MCP23x17 GPIO Expander over I2C2\n");
  if (!(config = (struct mcp23x17_config_s *)kmm_malloc(sizeof(struct mcp23x17_config_s))))
    {
      return -ENOMEM;
    }
  config->address = 0x20;
  config->frequency = 400000;
  syslog(LOG_INFO, "INFO: Initializing MCP23x17 GPIO Expander over I2C2\n");
  struct ioexpander_dev_s *mcp = mcp23x17_initialize(i2c, config);
  if (mcp == NULL)
    {
      gpioerr("ERROR: mcp23x17_initialize failed\n");
      syslog(LOG_INFO, "ERROR: mcp23x17_initialize failed\n");
      return -ENOMEM;
    }
  
  syslog(LOG_INFO, "INFO: Configurate GPIOs\n");

  /* Register four pin drivers */

  /* Pin 0: an non-inverted, input pin */

  IOEXP_SETDIRECTION(mcp, 0, IOEXPANDER_DIRECTION_IN);
  IOEXP_SETOPTION(mcp, 0, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_SETOPTION(mcp, 0, IOEXPANDER_OPTION_INTCFG,
                  (void *)IOEXPANDER_VAL_DISABLE);
  gpio_lower_half(mcp, 0, GPIO_INPUT_PIN, 0);

  /* Pin 1: an non-inverted, output pin */

  IOEXP_SETDIRECTION(mcp, 1, IOEXPANDER_DIRECTION_OUT);
  IOEXP_SETOPTION(mcp, 1, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_SETOPTION(mcp, 1, IOEXPANDER_OPTION_INTCFG,
                  (void *)IOEXPANDER_VAL_DISABLE);
  gpio_lower_half(mcp, 1, GPIO_OUTPUT_PIN, 1);

  /* Pin 2: an non-inverted, edge interrupting pin */

  IOEXP_SETDIRECTION(mcp, 2, IOEXPANDER_DIRECTION_IN);
  IOEXP_SETOPTION(mcp, 2, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_SETOPTION(mcp, 2, IOEXPANDER_OPTION_INTCFG,
                  (void *)IOEXPANDER_VAL_BOTH);
  gpio_lower_half(mcp, 2, GPIO_INTERRUPT_PIN, 2);

  /* Pin 3: a non-inverted, level interrupting pin */

  IOEXP_SETDIRECTION(mcp, 3, IOEXPANDER_DIRECTION_IN);
  IOEXP_SETOPTION(mcp, 3, IOEXPANDER_OPTION_INVERT,
                  (void *)IOEXPANDER_VAL_NORMAL);
  IOEXP_SETOPTION(mcp, 3, IOEXPANDER_OPTION_INTCFG,
                  (void *)IOEXPANDER_VAL_HIGH);
  gpio_lower_half(mcp, 3, GPIO_INTERRUPT_PIN, 3);

  return 0;
}
#endif /* CONFIG_EXAMPLES_GPIO && CONFIG_GPIO_LOWER_HALF */
