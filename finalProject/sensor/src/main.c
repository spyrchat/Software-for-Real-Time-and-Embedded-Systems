/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define SENSOR_NODE DT_NODELABEL(temp)
#define SENSOR_DATA_TYPE SENSOR_CHAN_DIE_TEMP
#define STACKSIZE 1024
#define THREAD0_PRIORITY 7
#define THREAD1_PRIORITY 4
#define THREAD2_PRIORITY 5
#define THREAD3_PRIORITY 6
#define QUEUE_SIZE 10
#define DATA_ITEM_TYPE struct sensor_value
#define HIGHTEMP 30
#define TIME_SECOND 1000
#define MESSAGE_SIZE 7
#define TEAM_NUMBER 11
#define KILL_LENGTH 20
#define ADVERTISING_PERIOD 400

K_MSGQ_DEFINE(temperatureMessage, sizeof(DATA_ITEM_TYPE), QUEUE_SIZE, 4);
K_SEM_DEFINE(hTempSemaphore, 0, 1);
K_SEM_DEFINE(lTempSemaphore, 0, 1);
K_SEM_DEFINE(hTempSemaphoreSend, 0, 1);
K_SEM_DEFINE(lTempSemaphoreSend, 0, 1);
K_SEM_DEFINE(avgTempNotify, 0, 1);
K_SEM_DEFINE(btReady, 1, 1);

static const struct device *tempdev = DEVICE_DT_GET(SENSOR_NODE);
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
struct k_msgq temperatureMessage;

// For BT connectivity
struct bt_conn *my_conn = NULL;

// For message transfer
// Send BT message
void btAdvertiseData(int value, int msgNum, int type)
{
  char *btMessage[MESSAGE_SIZE + 1];
  int err;

  if (!k_sem_take(&btReady, K_FOREVER))
  {
    btMessageUpdate(value, type, msgNum, btMessage);
    struct bt_data ad[] = {BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
                           BT_DATA(BT_DATA_MANUFACTURER_DATA, btMessage, MESSAGE_SIZE)};
    err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err)
    {
      return;
    }

    k_msleep(ADVERTISING_PERIOD);
    bt_le_adv_stop();
    k_sem_give(&btReady);
  }
}

void btMessageUpdate(int value, int type, int msgNum, char *btMsg)
{

  char *msg[MESSAGE_SIZE + 1];
  char *msgNumS[3];
  char *valueS[3];

  if (msgNum < 10)
  {
    snprintk(msgNumS, 3, "0%d", msgNum);
  }
  else
  {
    snprintk(msgNumS, 3, "%d", msgNum);
  }

  if (value < 10)
  {
    snprintk(valueS, 3, "0%d", value);
  }
  else
  {
    snprintk(valueS, 3, "%d", value);
  }

  if (type == 0)
  {
    snprintk(msg, 8, "T%d%s%s", TEAM_NUMBER, msgNumS, valueS);
  }
  else if (type == 1)
  {
    snprintk(msg, 8, "H%d%s%s", TEAM_NUMBER, msgNumS, valueS);
  }
  strcpy(btMsg, msg);
}

void tReadTemperature(void)
{ // THREAD 0
  struct sensor_value currTemp;
  int count = 0;
  while (1)
  {
    sensor_sample_fetch(tempdev);
    int err = sensor_channel_get(tempdev, SENSOR_DATA_TYPE, &currTemp);
    if (err == 0)
    {

      if (currTemp.val1 >= HIGHTEMP)
      {
        k_sem_give(&hTempSemaphore);
        k_sem_give(&hTempSemaphoreSend);
      }
      else
      {
        k_sem_give(&lTempSemaphore);
        k_sem_give(&lTempSemaphoreSend);
      }

      k_msgq_put(&temperatureMessage, &currTemp, K_NO_WAIT);

      count += 1;
      if (count == 10)
      {
        k_sem_give(&avgTempNotify);
        count = 0;
      }

      k_msleep(1 * TIME_SECOND);
    }
    else
    {
      printk("Temperature read error: %d \n", err);
    }
  }
}

void tSendTemperature(void)
{ // THREAD 1
  int sCount = 0;
  int err = 0;

  while (sCount < KILL_LENGTH)
  {
    if (!k_sem_take(&avgTempNotify, K_FOREVER))
    {
      int v1Sum = 0;
      int v2Sum = 0;
      struct sensor_value tempMessage;
      int counter = 0;

      while (counter < QUEUE_SIZE)
      {
        if (k_msgq_get(&temperatureMessage, &tempMessage, K_FOREVER) == 0)
        {
          v1Sum = v1Sum + tempMessage.val1;
          v2Sum = v2Sum + tempMessage.val2;
          counter++;
        }
      }

      int average = NRFX_ROUNDED_DIV(v1Sum, QUEUE_SIZE);
      int v2Average = NRFX_ROUNDED_DIV(v2Sum, QUEUE_SIZE);
      v2Average = NRFX_ROUNDED_DIV(v2Average, 1000000);
      average = average + v2Average;
      sCount++;

      btAdvertiseData(average, sCount, 0);
    }
  }
  // Abort all thread
  enterLowSleep();
}

void tHighTempWarning(void)
{ // THREAD 2
  int err;
  while (1)
  {

    if (!k_sem_take(&hTempSemaphore, K_FOREVER))
    {
      toggleLED(&led0, 1);
      k_sem_reset(&lTempSemaphore);
    }

    if (!k_sem_take(&lTempSemaphore, K_FOREVER))
    {
      toggleLED(&led0, 0);
      k_sem_reset(&hTempSemaphore);
    }
  }
}

void tSendHighTempWarning(void)
{ // THREAD 3
  int err;
  int sCount = 0;
   while (1)
  {

    if (!k_sem_take(&hTempSemaphoreSend, K_FOREVER))
    {
      k_sem_reset(&lTempSemaphoreSend);
      btAdvertiseData(1, sCount, 1);
      sCount += 1;
      // Send alert temp high
    }

    if (!k_sem_take(&lTempSemaphoreSend, K_FOREVER))
    {
      k_sem_reset(&hTempSemaphoreSend);
      btAdvertiseData(0, sCount, 1);
      sCount += 1;
    }

    if (sCount > 99) {
      sCount = 0;
    }
  }
}

K_THREAD_DEFINE(ReadTemperature, STACKSIZE, tReadTemperature, NULL, NULL, NULL, THREAD0_PRIORITY, 0, 0);
K_THREAD_DEFINE(SendTemperature, STACKSIZE, tSendTemperature, NULL, NULL, NULL, THREAD1_PRIORITY, 0, 0);
K_THREAD_DEFINE(HighTempWarning, STACKSIZE, tHighTempWarning, NULL, NULL, NULL, THREAD2_PRIORITY, 0, 0);
K_THREAD_DEFINE(SendHighTempWarning, STACKSIZE, tSendHighTempWarning, NULL, NULL, NULL, THREAD3_PRIORITY, 0, 0);

void enterLowSleep()
{
  int err;
  // turn off BT
  if (bt_is_ready())
  {
    err = bt_disable();
    if (err)
    {
      printk("Blueooth disable failed! code: %d\n", err);
    }
  }

  // turn off LEDs
  toggleLED(&led1, 0);
  toggleLED(&led0, 0);

  // suspend threads
  k_thread_suspend(ReadTemperature);
  k_thread_suspend(HighTempWarning);
  k_thread_suspend(SendHighTempWarning);

  pm_system_suspend();
}

void toggleLED(struct gpio_dt_spec *LED, int state)
{
  int ret;
  if (!gpio_is_ready_dt(LED))
  {
    return;
  }

  ret = gpio_pin_configure_dt(LED, GPIO_OUTPUT_ACTIVE);
  if (ret < 0)
  {
    return;
  }

  ret = gpio_pin_set_dt(LED, state); // Toggle LED.
  if (ret < 0)
  {
    return;
  }
}

int main()
{
  int err = 0;

  // enable USb
  if (usb_enable(NULL))
  {
    return 0;
  }

  // enable BT
  err = bt_enable(NULL);
  if (err)
  {
    return;
  }
}