/*
 * lmp92001-debug.c -- Debug file system for TI 92001
 *
 * Copyright 2016 Celestica Ltd.
 *
 * Author: Abhisit Sangjan <asang@celestica.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/mfd/lmp92001/core.h>

static ssize_t lmp92001_id_ver_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
        struct lmp92001 *lmp92001 = dev_get_drvdata(dev);
        int ret;
        unsigned int comid, ver;

        ret = regmap_read(lmp92001->regmap, LMP92001_ID, &comid);
        if (ret < 0) {
                dev_err(lmp92001->dev, "Failed to read Company ID: %d\n", ret);
                return 0;
        }

        ret = regmap_read(lmp92001->regmap, LMP92001_VER, &ver);
        if (ret < 0) {
                dev_err(lmp92001->dev, "Failed to read Version: %d\n", ret);
                return 0;
        }

        ret = sprintf(buf, "Company ID 0x%02x (%d), Version 0x%02x (%d)\n",
                        comid, comid, ver, ver);

        return ret;
}
static DEVICE_ATTR(lmp92001_id_ver, 0444, lmp92001_id_ver_show, NULL);

int lmp92001_debug_init(struct lmp92001 *lmp92001)
{
        int ret;

        ret = device_create_file(lmp92001->dev, &dev_attr_lmp92001_id_ver);
        if (ret != 0)
                dev_err(lmp92001->dev, "Unique ID attribute not created: %d\n",
                                ret);

        return ret;
}

void lmp92001_debug_exit(struct lmp92001 *lmp92001)
{
        device_remove_file(lmp92001->dev, &dev_attr_lmp92001_id_ver);
}
