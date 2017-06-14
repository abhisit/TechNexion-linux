/*
 * lmp92001-dac.c - Support for TI LMP92001 DACs
 *
 * Copyright 2016-2017 Celestica Ltd.
 *
 * Author: Abhisit Sangjan <asang@celestica.com>
 *
 * Inspired by wm831x driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/mfd/core.h>
#include <linux/platform_device.h>

#include <linux/mfd/lmp92001/core.h>

static int lmp92001_read_raw(struct iio_dev *indio_dev,
                                struct iio_chan_spec const *channel, int *val,
                                int *val2, long mask)
{
        struct lmp92001 *lmp92001 = iio_device_get_drvdata(indio_dev);
        int ret;

        switch (mask)
        {
        case IIO_CHAN_INFO_RAW:
                switch (channel->type) {
                case IIO_VOLTAGE:
                        ret = regmap_read(lmp92001->regmap,
                                        0x7F + channel->channel, val);
                        if (ret < 0)
                                return ret;

                        return IIO_VAL_INT;
                default:
                        break;
                }
                break;
        default:
                break;
        }

        return -EINVAL;
}

int lmp92001_write_raw(struct iio_dev *indio_dev,
                                struct iio_chan_spec const *channel,
                                int val, int val2, long mask)
{
        struct lmp92001 *lmp92001 = iio_device_get_drvdata(indio_dev);
        int ret;

        if (val > 4095)
                return -EINVAL;

        switch (mask)
        {
        case IIO_CHAN_INFO_RAW:
                switch (channel->type) {
                case IIO_VOLTAGE:
                        ret = regmap_write(lmp92001->regmap,
                                        0x7F + channel->channel, val);
                        if (ret < 0)
                                return ret;

                        return 0;
                default:
                        break;
                }
                break;
        default:
                break;
        }

        return -EINVAL;
}

static const struct iio_info lmp92001_info = {
        .read_raw = lmp92001_read_raw,
        .write_raw = lmp92001_write_raw,
        .driver_module = THIS_MODULE,
};

static const char * const lmp92001_dvref_opts[] = {
        [0] = "internal",
        [1] = "external",
};

static int lmp92001_dvref_read(struct iio_dev *indio_dev,
                struct iio_chan_spec const *channel)
{
        struct lmp92001 *lmp92001 = iio_device_get_drvdata(indio_dev);
        unsigned int cref;
        int ret;

        ret = regmap_read(lmp92001->regmap, LMP92001_CREF, &cref);
        if (ret < 0)
                return ret;

        return cref & 1;
}

static int lmp92001_dvref_write(struct iio_dev *indio_dev,
                const struct iio_chan_spec *channel, unsigned int mode)
{
        struct lmp92001 *lmp92001 = iio_device_get_drvdata(indio_dev);
        unsigned int cref;

        if (mode == 1)
                cref = 1;
        else if (mode == 0)
                cref = 0;
        else
                return -EINVAL;

        return regmap_update_bits(lmp92001->regmap, LMP92001_CREF, 1, cref);
}

static const struct iio_enum lmp92001_dvref_enum = {
        .items = lmp92001_dvref_opts,
        .num_items = ARRAY_SIZE(lmp92001_dvref_opts),
        .get = lmp92001_dvref_read,
        .set = lmp92001_dvref_write,
};

static const char * const lmp92001_outx_opts[] = {
        [0] = "0",
        [1] = "1",
        [2] = "dac",
        [3] = "hiz",
        [4] = "0 or dac",
        [5] = "1 or dac",
};

static int lmp92001_outx_read(struct iio_dev *indio_dev,
                struct iio_chan_spec const *channel)
{
        struct lmp92001 *lmp92001 = iio_device_get_drvdata(indio_dev);
        unsigned int cdac;
        int ret;
        unsigned int mode;

        ret = regmap_read(lmp92001->regmap, LMP92001_CDAC, &cdac);
        if (ret < 0)
                return ret;

        if (cdac & 1)
                mode = 3;
        else
        {
                if (cdac & 2)
                        mode = 5;
                else
                        mode = 4;
        }

        return mode;
}

static int lmp92001_outx_write(struct iio_dev *indio_dev,
                const struct iio_chan_spec *channel, unsigned int mode)
{
        struct lmp92001 *lmp92001 = iio_device_get_drvdata(indio_dev);
        unsigned int cdac, mask;

        switch (mode)
        {
        case 3:
                cdac = 1;
                mask = 1;
                break;
        case 2:
                cdac = 0;
                mask = 1;
                break;
        case 1:
                cdac = 2;
                mask = 3;
                break;
        case 0:
                cdac = 0;
                mask = 3;
                break;
        default:
                return -EINVAL;
                break;
        }

        return regmap_update_bits(lmp92001->regmap, LMP92001_CDAC, mask, cdac);
}

static const struct iio_enum lmp92001_outx_enum = {
        .items = lmp92001_outx_opts,
        .num_items = ARRAY_SIZE(lmp92001_outx_opts),
        .get = lmp92001_outx_read,
        .set = lmp92001_outx_write,
};

ssize_t lmp92001_gang_read(struct iio_dev *indio_dev, uintptr_t private,
                        struct iio_chan_spec const *channel, char *buf)
{
        struct lmp92001 *lmp92001 = iio_device_get_drvdata(indio_dev);
        unsigned int cdac;
        int ret;

        ret = regmap_read(lmp92001->regmap, LMP92001_CDAC, &cdac);
        if (ret < 0)
                return ret;

        return sprintf(buf, "%s\n", cdac & 4 ? "1" : "0");
}

ssize_t lmp92001_gang_write(struct iio_dev *indio_dev, uintptr_t private,
                         struct iio_chan_spec const *channel, const char *buf,
                         size_t len)
{
        struct lmp92001 *lmp92001 = iio_device_get_drvdata(indio_dev);
        unsigned int cdac = 0;
        int ret;

        if (strcmp("0\n", buf) == 0)
                cdac = 0;
        else if (strcmp("1\n", buf) == 0)
                cdac = 4;
        else
                return -EINVAL;

        ret = regmap_update_bits(lmp92001->regmap, LMP92001_CDAC, 4, cdac);
        if (ret < 0)
                return ret;

        return len;
}

static const struct iio_chan_spec_ext_info lmp92001_ext_info[] = {
        IIO_ENUM("vref", IIO_SHARED_BY_ALL, &lmp92001_dvref_enum),
        IIO_ENUM_AVAILABLE("vref", &lmp92001_dvref_enum),
        IIO_ENUM("outx", IIO_SHARED_BY_ALL, &lmp92001_outx_enum),
        IIO_ENUM_AVAILABLE("outx", &lmp92001_outx_enum),
        {
                .name = "gang",
                .read = lmp92001_gang_read,
                .write = lmp92001_gang_write,
                .shared = IIO_SHARED_BY_ALL,
        },
        { },
};

#define LMP92001_CHAN_SPEC(_ch) \
{ \
        .channel = _ch, \
        .scan_index = _ch, \
        .type = IIO_VOLTAGE, \
        .indexed = 1, \
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
        .ext_info = lmp92001_ext_info, \
        .output = 1, \
}

static const struct iio_chan_spec lmp92001_dac_channels[] = {
        LMP92001_CHAN_SPEC(1),
        LMP92001_CHAN_SPEC(2),
        LMP92001_CHAN_SPEC(3),
        LMP92001_CHAN_SPEC(4),
        LMP92001_CHAN_SPEC(5),
        LMP92001_CHAN_SPEC(6),
        LMP92001_CHAN_SPEC(7),
        LMP92001_CHAN_SPEC(8),
        LMP92001_CHAN_SPEC(9),
        LMP92001_CHAN_SPEC(10),
        LMP92001_CHAN_SPEC(11),
        LMP92001_CHAN_SPEC(12),
};

static int lmp92001_dac_probe(struct platform_device *pdev)
{
        struct lmp92001 *lmp92001 = dev_get_drvdata(pdev->dev.parent);
        struct iio_dev *indio_dev;
        struct device_node *np = pdev->dev.of_node;
        u8 gang, outx, hiz;
        unsigned int cdac = 0;
        int ret;

        indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*lmp92001));
        if (!indio_dev)
                return -ENOMEM;

        iio_device_set_drvdata(indio_dev, lmp92001);

        indio_dev->name = pdev->name;
        indio_dev->modes = INDIO_DIRECT_MODE;
        indio_dev->info = &lmp92001_info;
        indio_dev->channels = lmp92001_dac_channels;
        indio_dev->num_channels = ARRAY_SIZE(lmp92001_dac_channels);

        of_property_read_u8(np, "ti,lmp92001-dac-hiz", &hiz);
        cdac |= hiz;

        of_property_read_u8(np, "ti,lmp92001-dac-outx", &outx);
        cdac |= outx << 1;

        of_property_read_u8(np, "ti,lmp92001-dac-gang", &gang);
        cdac |= gang << 2;

        ret = regmap_update_bits(lmp92001->regmap, LMP92001_CDAC,
                                        7, cdac);
        if (ret < 0)
                return ret;

        platform_set_drvdata(pdev, indio_dev);

        return iio_device_register(indio_dev);
}

static int lmp92001_dac_remove(struct platform_device *pdev)
{
        struct iio_dev *indio_dev = platform_get_drvdata(pdev);

        iio_device_unregister(indio_dev);

        return 0;
}

static struct platform_driver lmp92001_dac_driver = {
        .driver.name    = "lmp92001-dac",
        .driver.owner   = THIS_MODULE,
        .probe          = lmp92001_dac_probe,
        .remove         = lmp92001_dac_remove,
};

static int __init lmp92001_dac_init(void)
{
        return platform_driver_register(&lmp92001_dac_driver);
}
subsys_initcall(lmp92001_dac_init);

static void __exit lmp92001_dac_exit(void)
{
        platform_driver_unregister(&lmp92001_dac_driver);
}
module_exit(lmp92001_dac_exit);

MODULE_AUTHOR("Abhisit Sangjan <asang@celestica.com");
MODULE_DESCRIPTION("IIO DAC interface for TI LMP92001");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lmp92001-dac");
