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
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci.h>

#define STACKSIZE 1024
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)



static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
struct k_msgq avgTempMessage;

#define NAME_LEN 30
#define MESSAGE_SIZE 7
#define GROUP_NUM_STR "11"

static char *lastTMsgNum[3];
static char *lastHMsgNum[3];


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

  ret = gpio_pin_set_dt(LED, state); // Toggle LED off.
  if (ret < 0)
  {
    return;
  }
}

static uint8_t processMsg(const char *data, int length)
{


  char msgNumS[3];
  msgNumS[2] = '\0';
  strncpy(msgNumS, ((char *)data) + 3, 2);

  char cmdType[2];
  cmdType[1] = '\0';
  strncpy(cmdType, ((char *)data), 1);

  char valueS[3];
  valueS[2] = '\0';
  strncpy(valueS, ((char *)data) + 5, 2);

  // printk("type %s, num %s, val %s\n", cmdType, msgNumS, valueS);

  if (!strcmp(cmdType, "T"))
  {
    if (!strcmp(lastTMsgNum, msgNumS))
    {
      // printk("Duplicate message received for T msg %s\n", msgNumS);
      return;
    }
    strncpy(lastTMsgNum, msgNumS, 3);
    printk("----- Avg temp received: %s\n----- Msg num: %s\n", valueS, msgNumS);
    if (!strcmp(msgNumS, "20\0"))
    {
      toggleLED(&led0, 0);
    }
  }
  else if (!strcmp(cmdType, "H"))
  {
    if (!strcmp(lastHMsgNum, msgNumS))
    {
      // printk("Duplicate message received for H msg %s\n", msgNumS);
      return;
    }
    strncpy(lastHMsgNum, msgNumS, 3);

    if (!strcmp(valueS, "01"))
    {
      printk("----- High temp warning active\n");
      toggleLED(&led0, 1);

      return;
    }
    else if (!strcmp(valueS, "00"))
    {
      printk("----- High temp warning inactive\n");
      toggleLED(&led0, 0);
      return;
    }
    else
    {
      printk("Message values incorrect!\n");
    }

    printk("----- Msg num: %s\n", msgNumS);
  }
  else
  {
    printk("Message not recognised\n");
  }
}

static bool msgCB(struct bt_data *data, void *user_data)
{
  char *msg = user_data;
  uint8_t len;

  switch (data->type)
  {
  case BT_DATA_MANUFACTURER_DATA:
    len = MIN(data->data_len, MESSAGE_SIZE + 1);
    (void)memcpy(msg, data->data, len);
    msg[len] = '\0';
    // printk("Data got: %s \n", msg);
    return false;
  default:
    return true;
  }
}

static void scanCB(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                   struct net_buf_simple *data)
{
  int err;
  char addr_str[BT_ADDR_LE_STR_LEN];
  char msg[MESSAGE_SIZE + 1];
  char gpNum[3];
  gpNum[2] = '\0';
  uint8_t data_status;
  uint16_t data_len;

  (void)memset(msg, 0, sizeof(msg));
  bt_data_parse(data, msgCB, msg);

  strncpy(gpNum, ((char *)msg) + 1, 2);
  bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

  /* connect only to specific device name*/
  if (strcmp(gpNum, GROUP_NUM_STR))
  {
    return;
  }

  if (bt_le_scan_stop())
  {
    return;
  }

  // printk("Device found: %s %s (RSSI %d dBw)\n", gpNum, addr_str, rssi);
  processMsg(msg, sizeof(msg));
  startscan();
}

void startscan()
{
  int err;
  struct bt_le_scan_param scan_param = {
      .type = BT_HCI_LE_SCAN_PASSIVE,
      .options = BT_LE_SCAN_OPT_NONE,
      .interval = BT_GAP_SCAN_FAST_INTERVAL,
      .window = BT_GAP_SCAN_FAST_WINDOW, // on steps of 0.625ms
  };
  err = bt_le_scan_start(&scan_param, scanCB);

  if (err)
  {
    // printk("Scanning failed to start (err %d)\n", err);
    return;
  }

  // printk("Scanning successfully started\n");
}

int main(void)
{
  int err;

  if (usb_enable(NULL))
  {
    return 0;
  }

  err = bt_enable(NULL);
  if (err)
  {
    printk("Bluetooth init failed (err %d)\n", err);
    return 0;
  }

  printk("Bluetooth initialized\n");

  startscan();
}