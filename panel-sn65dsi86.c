/*
 * Texas Instruments DSI-to-eDP (SN65DSI86) driver
 *
 * Author: KunYi Chen <kunyi_chen@fic.com.tw>
 *
 * Based on original version from panel_tc358765.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//#define DEBUG_COLOR_BAR   // uncomment this defined for colorbar testing
#define COLOR_BAR_TYPE  (0)  // should 0 ~ 7

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/pm_runtime.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#include <video/omapdss.h>

#define A_RO 0x1
#define A_WO 0x2
#define A_RW (A_RO|A_WO)

#define FLD_MASK(start, end)	(((1 << ((start) - (end) + 1)) - 1) << (end))
#define FLD_VAL(val, start, end) (((val) << (end)) & FLD_MASK(start, end))
#define FLD_GET(val, start, end) (((val) & FLD_MASK(start, end)) >> (end))
#define FLD_MOD(orig, val, start, end) \
	(((orig) & ~FLD_MASK(start, end)) | FLD_VAL(val, start, end))


static struct omap_video_timings sn65dsi86_timings;

/* device private data structure */
struct sn65dsi86_data {
	struct mutex lock;

	struct omap_dss_device *dssdev;

	int channel0;
	int channel1;
};

static struct {
	struct i2c_client *client;
	struct mutex xfer_lock;
} *sn65dsi86_i2c;


static int dsi86_i2c_read(u8 reg, u8* data)
{
	unsigned char addr;
	struct i2c_msg msg;
	int r;

	addr = reg;
	msg.addr = sn65dsi86_i2c->client->addr;
	msg.len = 1;
	msg.flags = 0;
	msg.buf = &addr;

	mutex_lock(&sn65dsi86_i2c->xfer_lock);
	r = i2c_transfer(sn65dsi86_i2c->client->adapter, &msg, 1);
	mutex_unlock(&sn65dsi86_i2c->xfer_lock);

	if (r < 0)
		pr_err("sn65dsi86 i2c bus access failed, dummy write address %02x\n", reg);

	msg.flags = I2C_M_RD;
	msg.buf = data;

	mutex_lock(&sn65dsi86_i2c->xfer_lock);
	r = i2c_transfer(sn65dsi86_i2c->client->adapter, &msg, 1);
	mutex_unlock(&sn65dsi86_i2c->xfer_lock);

	if (r < 0)
		pr_err("sn65dsi86 i2c bus access failed, read data, reg address %02x\n", reg);

	return r;
}

static int dsi86_i2c_write(u8 reg, u8 data)
{
	u8 val[2] = { reg, data };
	struct i2c_msg msg;
	int r;

	msg.addr = sn65dsi86_i2c->client->addr;
	msg.len = 2;
	msg.flags = 0;
	msg.buf = val;

	mutex_lock(&sn65dsi86_i2c->xfer_lock);
	r = i2c_transfer(sn65dsi86_i2c->client->adapter, &msg, 1);
	mutex_unlock(&sn65dsi86_i2c->xfer_lock);
	if (r < 0)
		pr_err("sn65dsi86 i2c bus access failed,  write register address %02x\n", reg);

	return r;
}

static int dsi86_read_register(struct omap_dss_device *dssdev,
			u8 reg, u8* val)
{
	int ret = 0;
	pm_runtime_get_sync(&dssdev->dev);
	/* I2C is preferred way of reading, but fall back to DSI
	 * if I2C didn't got initialized
	*/
	if (sn65dsi86_i2c)
		ret = dsi86_i2c_read(reg, val);

	pm_runtime_put_sync(&dssdev->dev);
	return ret;
}

static int dsi86_write_register(struct omap_dss_device *dssdev,
			u8 reg, u8 val)
{
	int ret = 0;
	pm_runtime_get_sync(&dssdev->dev);
	/* I2C is preferred way of reading, but fall back to DSI
	 * if I2C didn't got initialized
	*/
	if (sn65dsi86_i2c)
		ret = dsi86_i2c_write(reg, val);

	pm_runtime_put_sync(&dssdev->dev);
	udelay(100);
	return ret;
}

static void dsi86_write_dpcd(struct omap_dss_device *dssdev,
		u32 reg, u8* val, u8 len)
{
	int i;

	if (len > 16) {
		pr_err("dsi86_read_dpcd max len must less 16");
		return;
	}

	pm_runtime_get_sync(&dssdev->dev);
	if (sn65dsi86_i2c) {
		// address 20bits
		dsi86_i2c_write(0x74, reg>>16 & 0x0f);
		dsi86_i2c_write(0x75, reg>>8 & 0xff);
		dsi86_i2c_write(0x76, reg & 0xff);
		// write len
		dsi86_i2c_write(0x77, len);
		// write value
		for (i = 0; i < len; i++) {
			dsi86_i2c_write(0x64+i, *(val+i));
		}
		// send aux command
		dsi86_i2c_write(0x78, (8<<4) | (1<<0));
	}
	pm_runtime_put_sync(&dssdev->dev);
}

#ifdef DEBUG
static void dsi86_read_dpcd(struct omap_dss_device *dssdev,
		u32 reg, u8* val, u8 len)
{
	int i;

	if (len > 16) {
		pr_err("dsi86_read_dpcd max len must less 16");
		return;
	}

	pm_runtime_get_sync(&dssdev->dev);
	if (sn65dsi86_i2c) {
		// address 20bits
		dsi86_i2c_write(0x74, reg>>16 & 0x0f);
		dsi86_i2c_write(0x75, reg>>8 & 0xff);
		dsi86_i2c_write(0x76, reg & 0xff);
		// write read len
		dsi86_i2c_write(0x77, len);
		// send AUX command
		dsi86_i2c_write(0x78, (9<<4) | (1<<0));
	}
	pm_runtime_put_sync(&dssdev->dev);

	msleep(10); // wait for aux read

	pm_runtime_get_sync(&dssdev->dev);
	if (sn65dsi86_i2c) {
		for (i = 0; i < len; i++) {
			dsi86_i2c_read(0x79+i, val+i);
		}
	}
	pm_runtime_put_sync(&dssdev->dev);
}

static int dsi86_register_show(struct omap_dss_device *dssdev)
{
	int ret = 0;
	int i;
	u8 buf[256];

	pm_runtime_get_sync(&dssdev->dev);
	if (sn65dsi86_i2c) {
		for (i = 0; i < 256; i++)
			ret = dsi86_i2c_read(i, buf+i);
	}

	pm_runtime_put_sync(&dssdev->dev);

	pr_info("Dump sn65dsi86 register\n");
	pr_info("     00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
	pr_info("-------------------------------------------------------\n");
	for (i = 0; i < 256; i+=16) {
		pr_info("%02x | %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
				i,
				buf[i+0], buf[i+1], buf[i+2], buf[i+3], buf[i+4], buf[i+5], buf[i+6], buf[i+7],
				buf[i+8], buf[i+9], buf[i+10], buf[i+11], buf[i+12], buf[i+13], buf[i+14], buf[i+15]);
	}

	return ret;
}
#endif

static void sn65dsi86_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void sn65dsi86_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	dev_info(&dssdev->dev, "set_timings() not implemented\n");
}

static int sn65dsi86_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	if (unlikely(!timings)) {
		WARN(true, "%s: timings NULL pointer was passed\n", __func__);
		return -EINVAL;
	}

if (sn65dsi86_timings.x_res != timings->x_res ||
			sn65dsi86_timings.y_res != timings->y_res ||
			sn65dsi86_timings.pixel_clock != timings->pixel_clock ||
			sn65dsi86_timings.hsw != timings->hsw ||
			sn65dsi86_timings.hfp != timings->hfp ||
			sn65dsi86_timings.hbp != timings->hbp ||
			sn65dsi86_timings.vsw != timings->vsw ||
			sn65dsi86_timings.vfp != timings->vfp ||
			sn65dsi86_timings.vbp != timings->vbp)
		return -EINVAL;

	return 0;
}

static void sn65dsi86_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	*xres = sn65dsi86_timings.x_res;
	*yres = sn65dsi86_timings.y_res;
}

static int sn65dsi86_hw_reset(struct omap_dss_device *dssdev)
{

	if (dssdev == NULL || dssdev->reset_gpio == -1)
		return 0;

	gpio_set_value(dssdev->reset_gpio, 1);
	udelay(200);
	/* reset the panel */
	gpio_set_value(dssdev->reset_gpio, 0);
	/* assert reset */
	udelay(200);
	gpio_set_value(dssdev->reset_gpio, 1);
	/* wait after releasing reset */
	msleep(200);

	return 0;
}

static int sn65dsi86_probe(struct omap_dss_device *dssdev)
{
	struct sn65dsi86_data *d2d;
	int r = 0;

	dev_dbg(&dssdev->dev, "sn65dsi86_probe\n");

	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.dsi_pix_fmt = OMAP_DSS_DSI_FMT_RGB666_PACKED;
	sn65dsi86_timings = dssdev->panel.timings;

	d2d = kzalloc(sizeof(*d2d), GFP_KERNEL);
	if (!d2d) {
		r = -ENOMEM;
		goto err;
	}

	d2d->dssdev = dssdev;

	mutex_init(&d2d->lock);

	dev_set_drvdata(&dssdev->dev, d2d);

	/* channel0 used for video packets */
	r = omap_dsi_request_vc(dssdev, &d2d->channel0);
	if (r) {
		dev_err(&dssdev->dev, "failed to get virtual channel0\n");
		goto err;
	}

	r = omap_dsi_set_vc_id(dssdev, d2d->channel0, 0);
	if (r) {
		dev_err(&dssdev->dev, "failed to set VC_ID0\n");
		goto err_ch0;
	}

	/* channel1 used for registers access in LP mode */
	r = omap_dsi_request_vc(dssdev, &d2d->channel1);
	if (r) {
		dev_err(&dssdev->dev, "failed to get virtual channel1\n");
		goto err_ch0;
	}

	r = omap_dsi_set_vc_id(dssdev, d2d->channel1, 0);
	if (r) {
		dev_err(&dssdev->dev, "failed to set VC_ID1\n");
		goto err_ch1;
	}
	
	if (r)
		dev_warn(&dssdev->dev, "failed to create sysfs files\n");

	dev_dbg(&dssdev->dev, "sn65dsi86_probe done\n");
	return 0;

err_ch1:
	omap_dsi_release_vc(dssdev, d2d->channel1);
err_ch0:
	omap_dsi_release_vc(dssdev, d2d->channel0);
err:
	mutex_destroy(&d2d->lock);
	kfree(d2d);
	return r;
}

static void sn65dsi86_remove(struct omap_dss_device *dssdev)
{
	struct sn65dsi86_data *d2d = dev_get_drvdata(&dssdev->dev);

	omap_dsi_release_vc(dssdev, d2d->channel0);
	omap_dsi_release_vc(dssdev, d2d->channel1);
	mutex_destroy(&d2d->lock);

	kfree(d2d);
}

static int sn65dsi86_init_ppi(struct omap_dss_device *dssdev)
{
	int ret = 0;
	const u8 dsi86_id[] = { 0x36, 0x38, 0x49, 0x53, 0x44, 0x20, 0x20 };
	u8 val;
	int i;

	val = 0;
	for (i = 0; i < sizeof (dsi86_id); i++) {
		dsi86_read_register(dssdev, i, &val);
		if (val != dsi86_id[i]) {
			pr_err("FAILED!! SN65SDSI86 can't access!!\n");
		}
	}

	// software reset again
	dsi86_write_register(dssdev, 0x09, 1); // software reset
	msleep(10);

	dsi86_read_register(dssdev, 0x5a, &val);
	val &= ~(1<<3);
	dsi86_write_register(dssdev, 0x5a, 0); // disable VSTREAM

	dsi86_write_register(dssdev, 0x0d, 0); // disable PLL en
	dsi86_read_register(dssdev, 0x3c, &val);
	val &= ~(1<<4);
	dsi86_write_register(dssdev, 0x3c, 0); // disable color bar
	msleep(10);
	dsi86_write_register(dssdev, 0x09, 1); // software reset
	msleep(10);
	dsi86_write_register(dssdev, 0x0a, 2); // REF 19.2MHz
	dsi86_write_register(dssdev, 0x12, 327/5);

	dsi86_read_register(dssdev, 0xf1, &val); // for clean error
	msleep(10);
	dsi86_write_register(dssdev, 0xf1, val);

#if 0
	// for PWM setting
	dsi86_write_register(dssdev, 0x5f, (2<<6)); // config GPIO4 to PWM mode
	dsi86_write_register(dssdev, 0xa0, 1);  // backlight PWM pre divsor
	dsi86_write_register(dssdev, 0xa1, 96); // backlight scale, 200KHz=19.2MHz/(1x96)
	dsi86_write_register(dssdev, 0xa2, 0);
	dsi86_write_register(dssdev, 0xa3, 70);
	dsi86_write_register(dssdev, 0xa4,  0);
	dsi86_write_register(dssdev, 0xa5, 1<<1); // enable PWM
#else
	dsi86_write_register(dssdev, 0x5f, 0); // default input
#endif
	dsi86_write_register(dssdev, 0x10, 1<<5 | (0<<3) | 0); // ONLY CHA, 4 Lanes, SOT_ERR_TOL_DIS
	dsi86_write_register(dssdev, 0x5a, 0); // Disable ASSR
	dsi86_write_register(dssdev, 0x5b, 0); // 24bpp
	dsi86_write_register(dssdev, 0x5c, 0); // HPD Enable

	dsi86_write_register(dssdev, 0x93, (0 << 6) | // DP pre emphasis lv0
									   (2 << 4) | // DP 2 lanes
									   (2 << 1) | // Downspread 3750ppm
									   (0 << 0)); // disable ssc

	dsi86_write_register(dssdev, 0x94, (4 << 5) | // 2.7 Gbps
									   (0 << 2) | // 61ps
									   (0 << 1)); // Voltage

	return ret;
}

static int sn65dsi86_init_video_timings(struct omap_dss_device *dssdev)
{
	int i;
	int ret = 0;
	u8 val;
// try mini
#define HA	 (1920)
#define HSPW ((16))
#define HFP	 ((24))
#define HBP  ((40))
#define HSP  (0)

#define VA   (1200)
#define VSPW (1)
#define VFP  (5)
#define VBP  (4)
#define VSP  (1)


	dsi86_write_register(dssdev, 0x20, HA & 0xff);
	dsi86_write_register(dssdev, 0x21, (HA>>8) & 0xff);

	dsi86_write_register(dssdev, 0x24, VA & 0xff);
	dsi86_write_register(dssdev, 0x25, (VA>>8) & 0xff);

	dsi86_write_register(dssdev, 0x2C, HSPW & 0xff);
	dsi86_write_register(dssdev, 0x2D, ((HSPW>>8) & 0x7f) | (HSP<<7));

	dsi86_write_register(dssdev, 0x30, VSPW & 0xff);
	dsi86_write_register(dssdev, 0x31, ((VSPW>>8) & 0x7f) | (VSP<<7));

	dsi86_write_register(dssdev, 0x36, VBP);

	dsi86_write_register(dssdev, 0x38, HFP);
	dsi86_write_register(dssdev, 0x3A, VFP);

	dsi86_write_register(dssdev, 0x0D, 1); // enable PLL

	for (i = 0; i < 10; i++) {
		msleep(10);
		dsi86_read_register(dssdev, 0x0A, &val);
		if ((val & 0x80)) {
			break;
		}
	}
	if (i >= 10) {
		pr_err("SN65DSI86 PLL not ready!\n");
		ret = -1;
	}
#if 1
	// for ASSR panel
	dsi86_write_register(dssdev, 0x5a, (1<<2) | (1<<0));
	val = 0x1;
	dsi86_write_dpcd(dssdev, 0x10a, &val, 1); // setting panel ASSR
#else
	// for NonASSR panel, need a HW stripping please contact TI to get more information
	dsi86_write_register(dssdev, 0xff, 7); // move to page 7
	dsi86_write_register(dssdev, 0x16, 1); // support non-ASSR panel
	dsi86_write_register(dssdev, 0xff, 0); // move back page 0
	dsi86_write_register(dssdev, 0x5a, 4);
#endif
	dsi86_write_register(dssdev, 0x96, (1<<3|1<<1)); // semi auto link training
	msleep(50);
	dsi86_read_register(dssdev, 0x96, &val);
	dsi86_read_register(dssdev, 0xF8, &val);
#ifdef DEBUG_COLOR_BAR
	dsi86_write_register(dssdev, 0x3c, 0x18 | COLOR_BAR_TYPE); // color bar
#endif
	dsi86_write_register(dssdev, 0x5a, ((1<<3)|4)); // vstream
	return ret;
}

static int sn65dsi86_write_init_config(struct omap_dss_device *dssdev)
{
	int r = 0;
	/* HACK: dummy read: if we read via DSI, first reads always fail */
	r = sn65dsi86_init_ppi(dssdev);
	mdelay(2);
	r = sn65dsi86_init_video_timings(dssdev);
	return r;
}

static int sn65dsi86_power_on(struct omap_dss_device *dssdev)
{
	struct sn65dsi86_data *d2d = dev_get_drvdata(&dssdev->dev);
	uint8_t *buf = NULL;
	int r;

	dev_dbg(&dssdev->dev, "power_on\n");
	buf = kzalloc(256, GFP_KERNEL);

	if (dssdev->platform_enable)
		dssdev->platform_enable(dssdev);

	r = omapdss_dsi_display_enable(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to enable DSI\n");
		goto err_disp_enable;
	}

	/*turn on HS clock to bring up bridge i2c slave */
	omapdss_dsi_vc_enable_hs(dssdev, d2d->channel0, true);
	// todo
	msleep(10);
	sn65dsi86_hw_reset(dssdev);

	r = dsi_enable_video_output(dssdev, d2d->channel0);
	r = sn65dsi86_write_init_config(dssdev);
	if (r)
		goto err_write_init;

	dev_dbg(&dssdev->dev, "power_on done\n");
	return r;

err_write_init:
	omapdss_dsi_display_disable(dssdev, false, false);
err_disp_enable:
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
	if (buf != NULL) kfree(buf);
	return r;
}

static void sn65dsi86_power_off(struct omap_dss_device *dssdev)
{
	struct sn65dsi86_data *d2d = dev_get_drvdata(&dssdev->dev);

	dsi_disable_video_output(dssdev, d2d->channel0);
	dsi_disable_video_output(dssdev, d2d->channel1);

	omapdss_dsi_display_disable(dssdev, false, false);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	gpio_set_value(dssdev->reset_gpio, 0);
}

static inline void sn65dsi86_disable_int(struct omap_dss_device *dssdev,
		enum omap_dss_display_state target)
{
	struct sn65dsi86_data *d2d = dev_get_drvdata(&dssdev->dev);

	mutex_lock(&d2d->lock);
	dsi_bus_lock(dssdev);

	sn65dsi86_power_off(dssdev);
	dssdev->state = target;

	dsi_bus_unlock(dssdev);
	mutex_unlock(&d2d->lock);
}

static inline int sn65dsi86_enable_int(struct omap_dss_device *dssdev)
{
	struct sn65dsi86_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	mutex_lock(&d2d->lock);
	dsi_bus_lock(dssdev);

	r = sn65dsi86_power_on(dssdev);

	if (r) {
		dev_dbg(&dssdev->dev, "enable failed\n");
		pr_err("enabled panel failed\n");
	}
	else
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	dsi_bus_unlock(dssdev);
	mutex_unlock(&d2d->lock);

	return r;
}

static void sn65dsi86_disable(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "disable\n");

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		sn65dsi86_disable_int(dssdev, OMAP_DSS_DISPLAY_DISABLED);
}

static int sn65dsi86_enable(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "enable\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;

	return sn65dsi86_enable_int(dssdev);
}

static int sn65dsi86_suspend(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "suspend\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return -EINVAL;

	sn65dsi86_disable_int(dssdev, OMAP_DSS_DISPLAY_SUSPENDED);

	return 0;
}

static int sn65dsi86_resume(struct omap_dss_device *dssdev)
{
	dev_dbg(&dssdev->dev, "resume\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED)
		return -EINVAL;

	return sn65dsi86_enable_int(dssdev);
}

static struct omap_dss_driver sn65dsi86_driver = {
	.probe		= sn65dsi86_probe,
	.remove		= sn65dsi86_remove,

	.enable		= sn65dsi86_enable,
	.disable	= sn65dsi86_disable,
	.suspend	= sn65dsi86_suspend,
	.resume		= sn65dsi86_resume,

	.get_resolution	= sn65dsi86_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.get_timings	= sn65dsi86_get_timings,
	.set_timings	= sn65dsi86_set_timings,
	.check_timings	= sn65dsi86_check_timings,

	.driver         = {
		.name   = "sn65dsi86",
		.owner  = THIS_MODULE,
	},
};

static int __devinit sn65dsi86_i2c_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	sn65dsi86_i2c = kzalloc(sizeof(*sn65dsi86_i2c), GFP_KERNEL);
	if (sn65dsi86_i2c == NULL)
		return -ENOMEM;

	/* store i2c_client pointer on private data structure */
	sn65dsi86_i2c->client = client;

	/* store private data structure pointer on i2c_client structure */
	i2c_set_clientdata(client, sn65dsi86_i2c);

	/* init mutex */
	mutex_init(&sn65dsi86_i2c->xfer_lock);
	dev_err(&client->dev, "D2EDP i2c initialized\n");

	return 0;
}

/* driver remove function */
static int __devexit sn65dsi86_i2c_remove(struct i2c_client *client)
{
	/* remove client data */
	i2c_set_clientdata(client, NULL);

	/* destroy mutex */
	mutex_destroy(&sn65dsi86_i2c->xfer_lock);

	/* free private data memory */
	kfree(sn65dsi86_i2c);

	return 0;
}

static const struct i2c_device_id sn65dsi86_i2c_idtable[] = {
	{"sn65dsi86_i2c_drv", 0},
	{},
};

static struct i2c_driver sn65dsi86_i2c_driver = {
	.probe = sn65dsi86_i2c_probe,
	.remove = __exit_p(sn65dsi86_i2c_remove),
	.id_table = sn65dsi86_i2c_idtable,
	.driver = {
		   .name  = "d2edp",
		   .owner = THIS_MODULE,
	},
};


static int __init sn65dsi86_init(void)
{
	int r;
	sn65dsi86_i2c = NULL;
	r = i2c_add_driver(&sn65dsi86_i2c_driver);
	if (r < 0) {
		printk(KERN_WARNING "d2edp i2c driver registration failed\n");
		return r;
	}

	omap_dss_register_driver(&sn65dsi86_driver);
	return 0;
}

static void __exit sn65dsi86_exit(void)
{
	omap_dss_unregister_driver(&sn65dsi86_driver);
	i2c_del_driver(&sn65dsi86_i2c_driver);
}

module_init(sn65dsi86_init);
module_exit(sn65dsi86_exit);

MODULE_AUTHOR("KunYi Chen <kunyi_chen@fic.com.tw>");
MODULE_DESCRIPTION("SN65DSI86 DSI-2-eDP Driver");
MODULE_LICENSE("GPL");
