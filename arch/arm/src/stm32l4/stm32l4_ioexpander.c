/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_ioexpander.c
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

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/gpio.h>
#include <arch/board/board.h>
#include "../../common/arm_internal.h"
#include "./stm32l4_ioexpander.h"
#include "./stm32l4_gpio.h"

#if defined(CONFIG_IOEXPANDER_STM32L4_EXPANDER)

#ifndef CONFIG_IOEXPANDER_INT_ENABLE
#error GPIO Interrupts must be handled by STM32L4 GPIO Expander
#endif /* !CONFIG_IOEXPANDER_INT_ENABLE */

#ifndef CONFIG_GPIO_LOWER_HALF
#error GPIO Lower Half is required by STM32L4 GPIO Expander
#endif /* !CONFIG_GPIO_LOWER_HALF */

#ifdef CONFIG_STM32L4_EXPANDER_MULTIPLE
#error Multiple STM32L4 GPIO Expanders are not supported
#endif /* CONFIG_STM32L4_EXPANDER_MULTIPLE */

#ifdef CONFIG_IOEXPANDER_MULTIPIN
#error Multipin STM32L4 GPIO Expander is not supported
#endif /* CONFIG_IOEXPANDER_MULTIPIN */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
/* Callback for a registered pin interrupt */

struct stm32l4_expander_callback_s
{
  ioe_pinset_t pinset;   /* Set of pins that will trigger the interrupt callback */
  ioe_callback_t cbfunc; /* Callback function */
  FAR void* cbarg;       /* Callback argument */
};
#endif

/* I/O Expander Driver State */

struct stm32l4_expander_dev_s
{
  struct ioexpander_dev_s dev;  /* Nested structure to allow casting as public gpio expander */
  sem_t exclsem;                /* Mutual exclusion */

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  struct work_s work;           /* Supports the interrupt handling "bottom half" */

  /* Saved callback information for each I/O expander client */

  struct stm32l4_expander_callback_s cb[CONFIG_STM32L4_IOEXPANDER_NPINS];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stm32l4_expander_lock(FAR struct stm32l4_expander_dev_s *priv);

static int stm32l4_expander_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          int dir);
static int stm32l4_expander_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                       int opt, void *val);
static int stm32l4_expander_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                         bool value);
static int stm32l4_expander_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                        FAR bool *value);
static int stm32l4_expander_readbuf(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                        FAR bool *value);
#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int stm32l4_expander_multiwritepin(FAR struct ioexpander_dev_s *dev,
                              FAR uint8_t *pins, FAR bool *values,
                              int count);
static int stm32l4_expander_multireadpin(FAR struct ioexpander_dev_s *dev,
                             FAR uint8_t *pins, FAR bool *values, int count);
static int stm32l4_expander_multireadbuf(FAR struct ioexpander_dev_s *dev,
                             FAR uint8_t *pins, FAR bool *values, int count);
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static FAR void *stm32l4_expander_attach(FAR struct ioexpander_dev_s *dev,
                       ioe_pinset_t pinset,
                       ioe_callback_t callback, FAR void *arg);
static int stm32l4_expander_detach(FAR struct ioexpander_dev_s *dev,
                       FAR void *handle);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32l4_expander_dev_s g_stm32l4_expander_dev;

/* I/O Expander Operations */

static const struct ioexpander_ops_s g_stm32l4_expander_ops =
{
  stm32l4_expander_direction,
  stm32l4_expander_option,
  stm32l4_expander_writepin,
  stm32l4_expander_readpin,
  stm32l4_expander_readbuf
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  , stm32l4_expander_multiwritepin
  , stm32l4_expander_multireadpin
  , stm32l4_expander_multireadbuf
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  , stm32l4_expander_attach
  , stm32l4_expander_detach
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_expander_interrupt
 *
 * Description:
 *   Handle GPIO Interrupt.
 *
 ****************************************************************************/

static int stm32l4_expander_interrupt(int irq, void *context, void *arg)
{
  FAR struct stm32l4_expander_dev_s *priv = (FAR struct stm32l4_expander_dev_s *)arg;
  uint32_t time_out = 0;
  unsigned int gpio_pin;

  gpioinfo("Interrupt! context=%p, priv=%p\n", context, priv);
  DEBUGASSERT(priv != NULL);

  gpio_pin = (unsigned int)*arg;

  FAR struct stm32l4_expander_callback_s *cb = &priv->cb[gpio_pin];
  ioe_callback_t cbfunc = cb->cbfunc;
  FAR void* cbarg = cb->cbarg;

  /* NOTE: Callback will run in the context of Interrupt Handler */

  if (cbfunc == NULL)
    {
      gpioinfo("Missing callback for GPIO %d\n", gpio_pin);
    }
  else
    {
      gpioinfo("Call gpio=%d, callback=%p, arg=%p\n", gpio_pin, cbfunc, cbarg);
      cbfunc(&priv->dev, gpio_pin, cbarg);
    }
  }

  return OK;
}

/****************************************************************************
 * Name: stm32l4_expander_lock
 *
 * Description:
 *   Get exclusive access to the I/O Expander
 *
 ****************************************************************************/

static int stm32l4_expander_lock(FAR struct stm32l4_expander_dev_s *priv)
{
  return nxsem_wait_uninterruptible(&priv->exclsem);
}

#define stm32l4_expander_unlock(p) nxsem_post(&(p)->exclsem)

/****************************************************************************
 * Name: stm32l4_expander_direction
 *
 * Description:
 *   Set the direction of an ioexpander pin. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   dir - One of the IOEXPANDER_DIRECTION_ macros
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int stm32l4_expander_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                          int direction)
{
  FAR struct stm32l4_expander_dev_s *priv = (FAR struct stm32l4_expander_dev_s *)dev;
  int ret;
  uintptr_t base;
  unsigned int stm32_port;
  unsigned int stm32_pin;

  if (direction != IOEXPANDER_DIRECTION_IN &&
      direction != IOEXPANDER_DIRECTION_IN_PULLUP &&
      direction != IOEXPANDER_DIRECTION_IN_PULLDOWN &&
      direction != IOEXPANDER_DIRECTION_OUT && 
      direction != IOEXPANDER_DIRECTION_OUT_OPENDRAIN)
    {
      return -EINVAL;
    }

  DEBUGASSERT(priv != NULL && pin < CONFIG_STM32L4_IOEXPANDER_NPINS);

  /* Get exclusive access to the I/O Expander */

  ret = stm32l4_expander_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  unsigned int pinmode;
  uint32_t pupd = GPIO_PUPDR_NONE;
  bool od = false;
  switch (direction)
    {
      case IOEXPANDER_DIRECTION_IN:
        pinmode = GPIO_MODER_INPUT;
        break;
      case IOEXPANDER_DIRECTION_IN_PULLUP:
        pinmode = GPIO_MODER_INPUT;
        pupd = GPIO_PUPDR_PULLUP;
        break;
      case IOEXPANDER_DIRECTION_IN_PULLDOWN:
        pinmode = GPIO_MODER_INPUT;
        pupd = GPIO_PUPDR_PULLDOWN;
        break;
      case IOEXPANDER_DIRECTION_OUT:
        pinmode = GPIO_MODER_OUTPUT;
        break;
      case IOEXPANDER_DIRECTION_OUT_OPENDRAIN:
        pinmode = GPIO_MODER_OUTPUT;
        od = true;
        break;
      default:
        break;
    }
  stm32l4_gpio_set_pinmode(stm32_port, stm32_pin, pinmode);

  if (pupd != GPIO_PUPDR_NONE)
    {
        stm32l4_gpio_set_pupd(stm32_port, stm32_pin, pupd);
    }
  if (od)
    {
        stm32l4_gpio_set_od(stm32_port, stm32_pin, true);
    }

  stm32l4_expander_unlock(priv);
  return ret;
}

/****************************************************************************
 * Name: stm32l4_expander_option
 *
 * Description:
 *   Set pin options. Required.
 *   Since all IO expanders have various pin options, this API allows setting
 *     pin options in a flexible way.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   opt - One of the IOEXPANDER_OPTION_ macros
 *   val - The option's value
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int stm32l4_expander_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                       int opt, FAR void *value)
{
  FAR struct stm32l4_expander_dev_s *priv = (FAR struct stm32l4_expander_dev_s *)dev;
  int ret = -ENOSYS;
  unsigned int stm32_port;
  unsigned int stm32_pin;
  uint32_t exti;

  gpioinfo("pin=%u, option=%u, value=%p\n", pin, opt, value);

  DEBUGASSERT(priv != NULL);

  /* Get exclusive access to the I/O Expander */
  stm32_pin = pin%16;
  stm32_port = (pin-stm32_pin)/16;
  exti = STM32L4_EXTI1_BIT(stm32_pin);

  ret = stm32l4_expander_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle each option */

  switch(opt)
    {
      case IOEXPANDER_OPTION_INTCFG: /* Interrupt Trigger */
        {
          switch((uint32_t)value)
            {
              case IOEXPANDER_VAL_RISING: /* Rising Edge */
                {
                  gpioinfo("Rising edge: pin=%u\n", pin);
                  modifyreg32(STM32L4_EXTI1_RTSR, 0, exti);
                  modifyreg32(STM32L4_EXTI1_FTSR, exti, 0);
                  stm32l4_gpio_exti(stm32_port, stm32_pin, true);
                  break;
                }

              case IOEXPANDER_VAL_FALLING: /* Falling Edge */
                {
                  gpioinfo("Falling edge: pin=%u\n", pin);
                  modifyreg32(STM32L4_EXTI1_RTSR, exti, 0);
                  modifyreg32(STM32L4_EXTI1_FTSR, 0, exti);
                  stm32l4_gpio_exti(stm32_port, stm32_pin, true);
                  break;
                }

              case IOEXPANDER_VAL_BOTH: /* Both Edge (Unimplemented) */
                {
                  modifyreg32(STM32L4_EXTI1_RTSR, 0, exti);
                  modifyreg32(STM32L4_EXTI1_FTSR, 0, exti);
                  stm32l4_gpio_exti(stm32_port, stm32_pin, true);
                  break;
                }

              case IOEXPANDER_VAL_DISABLE: /* Disable (Unimplemented) */
                {
                  modifyreg32(STM32L4_EXTI1_RTSR, exti, 0);
                  modifyreg32(STM32L4_EXTI1_FTSR, exti, 0);
                  stm32l4_gpio_exti(stm32_port, stm32_pin, false);
                  break;
                }

              default: /* Unsupported Interrupt */
                {
                  gpioerr("ERROR: Unsupported interrupt: %d, pin=%u\n", value, pin);
                  ret = -EINVAL;
                  break;
                }
            }
          break;
        }

      default: /* Unsupported Option */
        {
          gpioerr("ERROR: Unsupported option: %d, pin=%u\n", opt, pin);
          ret = -ENOSYS;
        }
    }

  /* Unlock the I/O Expander */

  stm32l4_expander_unlock(priv);
  return ret;
}

/****************************************************************************
 * Name: stm32l4_expander_writepin
 *
 * Description:
 *   Set the pin level. Required.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pin - The index of the pin to alter in this call
 *   val - The pin level. Usually TRUE will set the pin high,
 *         except if OPTION_INVERT has been set on this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int stm32l4_expander_writepin(FAR struct ioexpander_dev_s *dev,
                                   uint8_t pin,
                                   bool value)
{
  FAR struct stm32l4_expander_dev_s *priv = (FAR struct stm32l4_expander_dev_s *)dev;
  int ret;
  unsigned int stm32_port;
  unsigned int stm32_pin;
  uint32_t pinset;

  gpioinfo("pin=%u, value=%u\n", pin, value);

  DEBUGASSERT(priv != NULL && pin < CONFIG_STM32L4_IOEXPANDER_NPINS);

  /* Get exclusive access to the I/O Expander */
  stm32_pin = pin%16;
  stm32_port = (pin-stm32_pin)/16;
  pinset |= (stm32_port << GPIO_PORT_SHIFT);
  pinset |= (stm32_pin << GPIO_PIN_SHIFT);
  ret = stm32l4_expander_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Write the pin value. Warning: Pin Number passed as STM32L4 Pinset */

  stm32l4_gpiowrite(pinset, value);

  /* Unlock the I/O Expander */

  stm32l4_expander_unlock(priv);
  return ret;
}

/****************************************************************************
 * Name: stm32l4_expander_readpin
 *
 * Description:
 *   Read the actual PIN level. This can be different from the last value
 *   written to this pin. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   valptr - Pointer to a buffer where the pin level is stored. Usually TRUE
 *            if the pin is high, except if OPTION_INVERT has been set on
 *            this pin.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int stm32l4_expander_readpin(FAR struct ioexpander_dev_s *dev, 
                                  uint8_t pin,
                                  FAR bool *value)
{
  FAR struct stm32l4_expander_dev_s *priv = (FAR struct stm32l4_expander_dev_s *)dev;
  int ret;
  unsigned int stm32_port;
  unsigned int stm32_pin;
  uint32_t pinset;

  DEBUGASSERT(priv != NULL && pin < CONFIG_STM32L4_IOEXPANDER_NPINS &&
              value != NULL);

  /* Get exclusive access to the I/O Expander */
  stm32_pin = pin%16;
  stm32_port = (pin-stm32_pin)/16;
  pinset |= (stm32_port << GPIO_PORT_SHIFT);
  pinset |= (stm32_pin << GPIO_PIN_SHIFT);
  ret = stm32l4_expander_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Read the pin value. Warning: Pin Number passed as STM32L4 Pinset */

  *value = stm32l4_gpioread(pinset);

  /* Unlock the I/O Expander */

  stm32l4_expander_unlock(priv);
  gpioinfo("pin=%u, value=%u\n", pin, *value);
  return ret;
}

/****************************************************************************
 * Name: stm32l4_expander_readbuf
 *
 * Description:
 *   Read the buffered pin level.
 *   This can be different from the actual pin state. Required.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   valptr - Pointer to a buffer where the level is stored.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

static int stm32l4_expander_readbuf(FAR struct ioexpander_dev_s *dev, 
                                  uint8_t pin,
                                  FAR bool *value)
{
  FAR struct stm32l4_expander_dev_s *priv = (FAR struct stm32l4_expander_dev_s *)dev;
  int ret;
  unsigned int stm32_port;
  unsigned int stm32_pin;
  uint32_t pinset;

  gpioerr("ERROR: Not supported\n");
  DEBUGPANIC();

  stm32_pin = pin%16;
  stm32_port = (pin-stm32_pin)/16;
  pinset |= (stm32_port << GPIO_PORT_SHIFT);
  pinset |= (stm32_pin << GPIO_PIN_SHIFT);
  /* Get exclusive access to the I/O Expander */

  ret = stm32l4_expander_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* TODO: Read the buffered pin level */

  /* Unlock the I/O Expander */

  stm32l4_expander_unlock(priv);
  return ret;
}

/****************************************************************************
 * Name: stm32l4_expander_multiwritepin
 *
 * Description:
 *   Set the pin level for multiple pins. This routine may be faster than
 *   individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   pins - The list of pin indexes to alter in this call
 *   val - The list of pin levels.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int stm32l4_expander_multiwritepin(FAR struct ioexpander_dev_s *dev,
                              FAR uint8_t *pins, FAR bool *values, int count)
{
  FAR struct stm32l4_expander_dev_s *priv = (FAR struct stm32l4_expander_dev_s *)dev;
  int ret;

  gpioinfo("count=%u\n", count);

  DEBUGASSERT(priv != NULL && pins != NULL && values != NULL && count > 0);

  /* Get exclusive access to the I/O Expander */
  if (ret < 0)
    {
      return ret;
    }

  for (int i=0; i<count; i++)
    {
        ret = stm32l4_expander_writepin(priv, pins[i], values[i]);
        if (ret < 0)
          {
            return ret;
          }
    }
  return OK;
}
#endif

/****************************************************************************
 * Name: stm32l4_expander_multireadpin
 *
 * Description:
 *   Read the actual level for multiple pins. This routine may be faster than
 *   individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The list of pin indexes to read
 *   valptr - Pointer to a buffer where the pin levels are stored.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int stm32l4_expander_multireadpin(FAR struct ioexpander_dev_s *dev,
                             FAR uint8_t *pins, FAR bool *values, int count)
{
  FAR struct stm32l4_expander_dev_s *priv = (FAR struct stm32l4_expander_dev_s *)dev;
  int ret;

  gpioinfo("count=%u\n", count);

  DEBUGASSERT(priv != NULL && pins != NULL && values != NULL && count > 0);

  if (ret < 0)
    {
      return ret;
    }

  
  for (int i=0; i<count; i++)
    {
        ret = stm32l4_expander_readpin(priv, pins[i], &values[i]);
        if (ret < 0)
          {
            return ret;
          }
    }
  return ret;
}
#endif

/****************************************************************************
 * Name: stm32l4_expander_multireadbuf
 *
 * Description:
 *   Read the buffered level of multiple pins. This routine may be faster
 *   than individual pin accesses. Optional.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   pin    - The index of the pin
 *   valptr - Pointer to a buffer where the buffered levels are stored.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_MULTIPIN
static int stm32l4_expander_multireadbuf(FAR struct ioexpander_dev_s *dev,
                             FAR uint8_t *pins, FAR bool *values, int count)
{
  FAR struct stm32l4_expander_dev_s *priv = (FAR struct stm32l4_expander_dev_s *)dev;
  int ret;

  gpioinfo("count=%u\n", count);

  DEBUGASSERT(priv != NULL && pins != NULL && values != NULL && count > 0);

  if (ret < 0)
    {
      return ret;
    }

  for (int i=0;i<count;i++)
    {
      ret = stm32l4_expander_readbuf(priv, pins[i], &values[i]);
      if (ret < 0)
        {
          return ret;
        }
    }
  return ret;
}
#endif

/****************************************************************************
 * Name: stm32l4_expander_attach
 *
 * Description:
 *   Attach a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   pinset   - The set of pin events that will generate the callback
 *   callback - The pointer to callback function.  NULL will detach the
 *              callback. NOTE: Callback will run in the context of
 *              Interrupt Handler.
 *   arg      - Argument that will be provided to the callback function
 *
 * Returned Value:
 *   Callback Handle on success, else NULL
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static FAR void *stm32l4_expander_attach(FAR struct ioexpander_dev_s *dev,
                       ioe_pinset_t pinset,
                       ioe_callback_t callback, FAR void *arg)
{
  FAR struct stm32l4_expander_dev_s *priv = (FAR struct stm32l4_expander_dev_s *)dev;
  FAR struct stm32l4_expander_callback_s *cb = NULL;
  int ret = 0;
  unsigned int stm32_pin;

  gpioinfo("pinset=0x%lx, callback=%p, arg=%p\n", pinset, callback, arg);
  DEBUGASSERT(priv != NULL);

  /* Get exclusive access to the I/O Expander */

  ret = stm32l4_expander_lock(priv);
  if (ret < 0)
    {
      gpioerr("ERROR: Lock failed\n");
      return NULL;
    }

  /* Handle each GPIO Pin in the pinset */
#if CONFIG_STM32L4_IOEXPANDER_NPINS <= 64

  for (uint8_t gpio_pin = 0; gpio_pin < CONFIG_STM32L4_IOEXPANDER_NPINS; gpio_pin++)
    {
      /* If GPIO Pin is set in the pinset... */

      if (pinset & ((ioe_pinset_t)1 << gpio_pin))
        {
          cb = &priv->cb[gpio_pin];

          if (callback == NULL) /* Detach Callback */
            {
              /* Disable GPIO Interrupt and clear Interrupt Callback */

              gpioinfo("Detach callback for gpio=%lu, callback=%p, arg=%p\n",
                      cb->pinset, cb->cbfunc, cb->cbarg);
              
              stm32_pin = gpio_pin%16;
              stm32l4_exti_gpio_set_irq(stm32_pin, false);
              modifyreg32(STM32L4_EXTI1_EMR,
                        STM32L4_EXTI1_BIT(stm32_pin), 0);
              modifyreg32(STM32L4_EXTI1_IMR,
                        STM32L4_EXTI1_BIT(stm32_pin), 0);
              g_gpio_handlers[stm32_pin].callback = NULL;
              kmm_free(g_gpio_handlers[stm32_pin].arg);
              g_gpio_handlers[stm32_pin].arg = NULL;
              cb->pinset = 0;
              cb->cbfunc = NULL;
              cb->cbarg  = NULL;
              ret = 0;
            }
          else if (cb->cbfunc == NULL) /* Attach Callback */
            {
              /* Set Interrupt Callback and enable GPIO Interrupt */

              gpioinfo("Attach callback for gpio=%d, callback=%p, arg=%p\n", 
                      gpio_pin, callback, arg);

              stm32_pin = gpio_pin%16;
              modifyreg32(STM32L4_EXTI1_EMR,
                        0, STM32L4_EXTI1_BIT(stm32_pin));
              modifyreg32(STM32L4_EXTI1_IMR,
                        0, STM32L4_EXTI1_BIT(stm32_pin));
              g_gpio_handlers[stm32_pin].callback = stm32l4_expander_interrupt;
              unsigned int *arg_int = (unsigned int *)kmm_zalloc(sizeof(unsigned int));
              memcpy(arg_int, &stm32_pin, sizeof(unsigned int));
              g_gpio_handlers[stm32_pin].arg      = arg_int;
              cb->pinset = gpio_pin;
              cb->cbfunc = callback;
              cb->cbarg  = arg;
              ret = 0;
            }
          else /* Callback already attached */
            {
              gpioerr("ERROR: GPIO %d already attached\n", gpio_pin);
              ret = -EBUSY;
            }

          /* Only 1 GPIO Pin allowed */

          DEBUGASSERT(pinset == ((ioe_pinset_t)1 << gpio_pin));
          break;
        }
    }
#else
    gpio_pin = (uint8_t)pinset;
    cb = &priv->cb[gpio_pin];

    if (callback == NULL) /* Detach Callback */
    {
        /* Disable GPIO Interrupt and clear Interrupt Callback */

        gpioinfo("Detach callback for gpio=%lu, callback=%p, arg=%p\n",
                cb->pinset, cb->cbfunc, cb->cbarg);
        
        stm32_pin = gpio_pin%16;
        stm32l4_exti_gpio_set_irq(stm32_pin, false);
        modifyreg32(STM32L4_EXTI1_EMR,
                STM32L4_EXTI1_BIT(stm32_pin), 0);
        modifyreg32(STM32L4_EXTI1_IMR,
                STM32L4_EXTI1_BIT(stm32_pin), 0);
        g_gpio_handlers[stm32_pin].callback = NULL;
        kmm_free(g_gpio_handlers[stm32_pin].arg);
        g_gpio_handlers[stm32_pin].arg = NULL;
        cb->pinset = 0;
        cb->cbfunc = NULL;
        cb->cbarg  = NULL;
        ret = 0;
    }
    else if (cb->cbfunc == NULL) /* Attach Callback */
    {
        /* Set Interrupt Callback and enable GPIO Interrupt */

        gpioinfo("Attach callback for gpio=%d, callback=%p, arg=%p\n", 
                gpio_pin, callback, arg);

        stm32_pin = gpio_pin%16;
        modifyreg32(STM32L4_EXTI1_EMR,
                0, STM32L4_EXTI1_BIT(stm32_pin));
        modifyreg32(STM32L4_EXTI1_IMR,
                0, STM32L4_EXTI1_BIT(stm32_pin));
        g_gpio_handlers[stm32_pin].callback = stm32l4_expander_interrupt;
        unsigned int *arg_int = (unsigned int *)kmm_zalloc(sizeof(unsigned int));
        memcpy(arg_int, &stm32_pin, sizeof(unsigned int));
        g_gpio_handlers[stm32_pin].arg      = arg_int;
        cb->pinset = gpio_pin;
        cb->cbfunc = callback;
        cb->cbarg  = arg;
        ret = 0;
    }
    else /* Callback already attached */
    {
        gpioerr("ERROR: GPIO %d already attached\n", gpio_pin);
        ret = -EBUSY;
    }
#endif // CONFIG_STM32L4_IOEXPANDER_NPINS
  /* Unlock the I/O Expander and return the handle */

  stm32l4_expander_unlock(priv);
  return (ret == 0) ? cb : NULL;
}
#endif

/****************************************************************************
 * Name: stm32l4_expander_detach
 *
 * Description:
 *   Detach and disable a pin interrupt callback function.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   handle   - The non-NULL opaque value return by stm32l4_expander_attach()
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static int stm32l4_expander_detach(FAR struct ioexpander_dev_s *dev, FAR void *handle)
{
  FAR struct stm32l4_expander_dev_s *priv = (FAR struct stm32l4_expander_dev_s *)dev;
  FAR struct stm32l4_expander_callback_s *cb =
    (FAR struct stm32l4_expander_callback_s *)handle;
  unsigned int stm32_pin;

  DEBUGASSERT(priv != NULL && cb != NULL);
  DEBUGASSERT((uintptr_t)cb >= (uintptr_t)&priv->cb[0] &&
              (uintptr_t)cb <=
              (uintptr_t)&priv->cb[CONFIG_STM32L4_IOEXPANDER_NPINS - 1]);
  UNUSED(priv);
  gpioinfo("Detach callback for gpio=%lu, callback=%p, arg=%p\n",
           cb->pinset, cb->cbfunc, cb->cbarg);

  /* Disable the GPIO Interrupt */

#if CONFIG_STM32L4_IOEXPANDER_NPINS <= 64
  for (uint8_t gpio_pin = 0; gpio_pin < CONFIG_STM32L4_IOEXPANDER_NPINS; gpio_pin++)
    {
      if (cb->pinset & ((ioe_pinset_t)1 << gpio_pin))
        {
            stm32_pin = gpio_pin%16;
            stm32l4_exti_gpio_set_irq(stm32_pin, false);
            modifyreg32(STM32L4_EXTI1_EMR,
                    STM32L4_EXTI1_BIT(stm32_pin), 0);
            modifyreg32(STM32L4_EXTI1_IMR,
                    STM32L4_EXTI1_BIT(stm32_pin), 0);
            g_gpio_handlers[stm32_pin].callback = NULL;
            kmm_free(g_gpio_handlers[stm32_pin].arg);
            g_gpio_handlers[stm32_pin].arg = NULL;
        }
    }
#else
    stm32_pin = cb->pinset%16;
    stm32l4_exti_gpio_set_irq(stm32_pin, false);
    modifyreg32(STM32L4_EXTI1_EMR,
            STM32L4_EXTI1_BIT(stm32_pin), 0);
    modifyreg32(STM32L4_EXTI1_IMR,
            STM32L4_EXTI1_BIT(stm32_pin), 0);
    g_gpio_handlers[stm32_pin].callback = NULL;
    kmm_free(g_gpio_handlers[stm32_pin].arg);
    g_gpio_handlers[stm32_pin].arg = NULL;
#endif

  DEBUGASSERT(cb->pinset < CONFIG_STM32L4_IOEXPANDER_NPINS);

  /* Clear the Interrupt Callback */
  cb->pinset = 0;
  cb->cbfunc = NULL;
  cb->cbarg  = NULL;
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_expander_initialize
 *
 * Description:
 *   Initialize a I/O Expander device.
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *stm32l4_expander_initialize(
  const uint32_t *gpios,
  uint8_t gpio_count)
{
  int i;
  int ret;
  uint8_t pin;
  bool gpio_is_used[CONFIG_STM32L4_IOEXPANDER_NPINS];
  FAR struct stm32l4_expander_dev_s *priv;

  DEBUGASSERT(gpio_count <= CONFIG_STM32L4_IOEXPANDER_NPINS);

  /* Use the one-and-only I/O Expander driver instance */
  priv = &g_stm32l4_expander_dev;

  /* Initialize the device state structure */

  priv->dev.ops = &g_stm32l4_expander_ops;
  nxsem_init(&priv->exclsem, 0, 1);

  /* Mark the GPIOs in use */

  memset(gpio_is_used, 0, sizeof(gpio_is_used));

  /* Configure and register the GPIO Inputs */

  for (i = 0; i < gpio_count; i++)
    {
      uint32_t cfgset = gpios[i];
      uint8_t gpio_pin = ((cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT) + ((cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT)*16;
      DEBUGASSERT(gpio_pin < CONFIG_STM32L4_IOEXPANDER_NPINS);
      if (gpio_is_used[gpio_pin])
        {
          gpioerr("ERROR: GPIO pin %d is already in use\n", gpio_pin);
          return NULL;
        }
      gpio_is_used[gpio_pin] = true;

      ret = stm32l4_configgpio(cfgset & (GPIO_PIN_MASK | GPIO_PORT_MASK));
      DEBUGASSERT(ret == OK);
      // gpio_lower_half(&priv->dev, gpio_pin, GPIO_INPUT_PIN, gpio_pin);
    }
  return &priv->dev;
}

#endif /* CONFIG_IOEXPANDER_STM32L4_EXPANDER */