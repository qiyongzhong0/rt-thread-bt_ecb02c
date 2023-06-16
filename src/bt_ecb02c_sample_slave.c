/*
 * bt_ecb02c_sample_slave.c
 *
 * Change Logs:
 * Date           Author            Notes
 * 2023-05-06     qiyongzhong       first version
 */

#include <bt_ecb02c.h>

#ifdef BT_ECB02C_USING_SAMPLE_SLAVE

#include <drv_common.h>

#define DBG_TAG "bt.ecb02c.slave"
#define DBG_LVL DBG_LOG //DBG_INFO  //
#include <rtdbg.h>

#define BT_PIN_SLEEP            GET_PIN(C, 12) //pin - 44
#define BT_UART_NAME            "uart4"
#define BT_FIFO_SIZE            512

static const bt_cfg_t cfg =
{
    .serial     = BT_UART_NAME,
    .sleep      = BT_PIN_SLEEP,
    .fifo_size  = BT_FIFO_SIZE,
    .mode       = BT_MODE_SLAVE,
    .power      = BT_PWR_3dB,
    .pwd        = RT_NULL,
    .slave      = RT_NULL,
};

static char bt_ecb02c_buf[256];

static rt_err_t bt_ecb02c_recv_ind_hook(rt_device_t dev, rt_size_t size)
{
    int len = bt_read((bt_dev_t)dev, -1, bt_ecb02c_buf, sizeof(bt_ecb02c_buf));
    LOG_D("recv data length = %d", len);
    bt_write((bt_dev_t)dev, -1, bt_ecb02c_buf, len);

    return(RT_EOK);
}

static int bt_ecb02c_slave_init(void)
{
    bt_dev_t dev = bt_create("ecb02c_slv", &cfg);
    RT_ASSERT(dev != RT_NULL);

    bt_control(dev, BT_CTRL_SET_NOTIFY, (void *)bt_ecb02c_recv_ind_hook);
    bt_open(dev);

    return(RT_EOK);
}
INIT_APP_EXPORT(bt_ecb02c_slave_init);

#endif
