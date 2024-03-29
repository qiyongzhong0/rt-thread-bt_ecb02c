﻿# bt_ecb02c软件包

## 1.简介

**bt_ecb02c** 软件包是低功耗蓝牙芯片ECB02C的驱动包。ECB02C是一款支持低功耗蓝牙协议的串口透传芯片；具有小体积、高性能、高性价比、低功耗、平台兼容性强等优点；可以帮助用户快速掌握蓝牙技术，加速产品开发；模块已兼容的软件平台包括：IOS应用程序、 Android 应用程序、PC 电脑应用程序、微信支付宝小程序等。MCU通过串口连接模块，既可与手机、平板、PC 电脑进行数据通讯，轻松实现智能无线控制和数据采集；模块广泛应用在智能家居、共享售货机等领域。

### 1.1目录结构

`bt_ecb02c` 软件包目录结构如下所示：

``` 
bt_ecb02c
├───inc                             // 头文件目录
│   └───bt_ecb02c.h                 // API接口头文件
├───src                             // 源码目录
│   |   bt_ecb02c.c                 // 源文件
│   |   bt_ecb02c_sample_master.c   // 主机示例源文件
│   └───bt_ecb02c_sample_slave.c    // 从机示例源文件
│   license                         // 软件包许可证
│   readme.md                       // 软件包使用说明
└───SConscript                      // RT-Thread 默认的构建脚本
```

### 1.2许可证

bt_ecb02c package 遵循 LGPLv2.1 许可，详见 `LICENSE` 文件。

### 1.3依赖

- RT_Thread 4.0
- uAT

## 2.使用

### 2.1接口函数说明

#### 配置参数项定义如下：
```
typedef struct{
    const char *serial;     //串口设备名
    int sleep;              //休眠控制引脚
    int fifo_size;          //FIFO尺寸
    bt_mode_t mode;         //工作模式
    bt_power_t power;       //蓝牙功率
    const char *pwd;        //连接密码
    const char *slave;      //绑定的从机名字，主模式时有效且必须配置
}bt_cfg_t;
```

#### bt_dev_t bt_create(const char *name, const bt_cfg_t *cfg);
- 功能 ：创建蓝牙设备
- 参数 ：name--蓝牙设备的名称
- 参数 ：cfg--蓝牙工作参数配置指针
- 返回 ：成功返回蓝牙设备指针，失败返回NULL

#### void bt_destory(bt_dev_t dev);
- 功能 ：销毁蓝牙设备
- 参数 ：dev--蓝牙设备指针
- 返回 ：无

#### int bt_open(bt_dev_t dev);
- 功能 ：打开蓝牙设备
- 参数 ：dev--蓝牙设备指针
- 返回 ：0-成功，<0-错误

#### int bt_close(bt_dev_t dev);
- 功能 ：关闭蓝牙设备
- 参数 ：dev--蓝牙设备指针
- 返回 ：0-成功，<0-错误

#### int bt_read(bt_dev_t dev, int pos, void *buf, int bufsize);
- 功能 ：从蓝牙设备读取接收数据
- 参数 ：dev--蓝牙设备指针
- 参数 ：pos--位置偏移，本参数未使用
- 参数 ：buf--数据缓冲区指针
- 参数 ：bufsize--缓冲区尺寸
- 返回 ：>=0-读取数据长度，<0-错误

#### int bt_write(bt_dev_t dev, int pos, void *buf, int size);
- 功能 ：向蓝牙设备写入发送数据
- 参数 ：dev--蓝牙设备指针
- 参数 ：pos--位置偏移，本参数未使用
- 参数 ：buf--数据缓冲区指针
- 参数 ：size--数据尺寸
- 返回 ：>=0-写入数据长度，<0-错误

#### int bt_control(bt_dev_t dev, int cmd, void *args);
- 功能 ：控制蓝牙设备
- 参数 ：dev--蓝牙设备指针
- 参数 ：cmd--命令码
- 参数 ：args--命令参数表
- 返回 ：0-成功，<0-错误


### 2.3获取组件

- **方式1：**
通过 *Env配置工具* 或 *RT-Thread studio* 开启软件包，根据需要配置各项参数；配置路径为 *RT-Thread online packages -> peripherals packages -> bt_ecb02c* 


### 2.4配置参数说明

| 参数宏 | 说明 |
| ---- | ---- |
| BT_ECB02C_USING_SAMPLE_MASTER | 使用主机示例
| BT_ECB02C_USING_SAMPLE_SLAVE  | 使用从机示例


## 3. 联系方式

* 维护：qiyongzhong
* 主页：https://github.com/qiyongzhong0/rt-thread-bt_ecb02c
* 主页：https://gitee.com/qiyongzhong0/rt-thread-bt_ecb02c
* 邮箱：917768104@qq.com
