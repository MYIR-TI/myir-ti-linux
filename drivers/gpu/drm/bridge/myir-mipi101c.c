// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025 MYIR
 *
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <video/mipi_display.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

struct mipi101c {
	struct device *dev;
	struct drm_bridge bridge;
	struct regulator *regulator;
	struct drm_bridge *panel_bridge;
	struct gpio_desc *reset_gpio;
	struct drm_display_mode mode;
	bool pre_enabled;
	int error;
};

static inline struct mipi101c *bridge_to_mipi101c(struct drm_bridge *bridge)
{
	return container_of(bridge, struct mipi101c, bridge);
}

static int mipi101c_init(struct mipi101c *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	mipi_dsi_generic_write(dsi, (u8[]){ 0x11, 0x00 }, 2);
	msleep(120);
	mipi_dsi_generic_write(dsi, (u8[]){ 0x29, 0x00 }, 2);

	msleep(20);

	return 0;
}

static void mipi101c_post_disable(struct drm_bridge *bridge, struct drm_bridge_state *state)
{
	struct mipi101c *ctx = bridge_to_mipi101c(bridge);
	int ret;

	if (!ctx->pre_enabled)
		return;

	ctx->pre_enabled = false;

	if (ctx->reset_gpio)
		gpiod_set_value_cansleep(ctx->reset_gpio, 0);

	ret = regulator_disable(ctx->regulator);
	if (ret < 0)
		dev_err(ctx->dev, "error disabling regulators (%d)\n", ret);
}

static void mipi101c_pre_enable(struct drm_bridge *bridge, struct drm_bridge_state *state)
{
	struct mipi101c *ctx = bridge_to_mipi101c(bridge);
	int ret;

	ret = regulator_enable(ctx->regulator);
	if (ret < 0)
		dev_err(ctx->dev, "error enabling regulators (%d)\n", ret);

	if (ctx->reset_gpio) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		usleep_range(5000, 10000);
	}

	ctx->pre_enabled = true;
}

static void mipi101c_enable(struct drm_bridge *bridge, struct drm_bridge_state *state)
{
	struct mipi101c *ctx = bridge_to_mipi101c(bridge);
	int ret;

	ret = mipi101c_init(ctx);
	if (ret < 0)
		dev_err(ctx->dev, "error initializing bridge (%d)\n", ret);
}

static int mipi101c_attach(struct drm_bridge *bridge,
			   enum drm_bridge_attach_flags flags)
{
	struct mipi101c *ctx = bridge_to_mipi101c(bridge);

	return drm_bridge_attach(bridge->encoder, ctx->panel_bridge,
				 bridge, flags);
}

static void mipi101c_bridge_mode_set(struct drm_bridge *bridge,
				     const struct drm_display_mode *mode,
				     const struct drm_display_mode *adj)
{
	struct mipi101c *ctx = bridge_to_mipi101c(bridge);

	drm_mode_copy(&ctx->mode, mode);
}

static const struct drm_bridge_funcs mipi101c_bridge_funcs = {
	.atomic_post_disable = mipi101c_post_disable,
	.atomic_pre_enable = mipi101c_pre_enable,
	.atomic_enable = mipi101c_enable,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
	.attach = mipi101c_attach,
	.mode_set = mipi101c_bridge_mode_set,
};

static int mipi101c_parse_dt(struct mipi101c *ctx)
{
	struct drm_bridge *panel_bridge;
	struct device *dev = ctx->dev;

	panel_bridge = devm_drm_of_get_bridge(dev, dev->of_node, 1, 0);
	if (IS_ERR(panel_bridge))
		return PTR_ERR(panel_bridge);

	ctx->panel_bridge = panel_bridge;

	/* Reset GPIO is optional */
	ctx->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio))
		return PTR_ERR(ctx->reset_gpio);

	return 0;
}

static int mipi101c_configure_regulators(struct mipi101c *ctx)
{
	ctx->regulator = devm_regulator_get(ctx->dev, "vddc");
	if (IS_ERR(ctx->regulator))
		return PTR_ERR(ctx->regulator);

	return 0;
}

static int mipi101c_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct mipi101c *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(struct mipi101c), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	ctx->pre_enabled = false;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_VIDEO_HSE;

	ret = mipi101c_parse_dt(ctx);
	if (ret < 0)
		return ret;

	ret = mipi101c_configure_regulators(ctx);
	if (ret < 0)
		return ret;

	ctx->bridge.funcs = &mipi101c_bridge_funcs;
	ctx->bridge.type = DRM_MODE_CONNECTOR_DSI;
	ctx->bridge.of_node = dev->of_node;
	ctx->bridge.pre_enable_prev_first = true;

	drm_bridge_add(&ctx->bridge);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_bridge_remove(&ctx->bridge);
		dev_err(dev, "failed to attach dsi\n");
	}

	return ret;
}

static void mipi101c_remove(struct mipi_dsi_device *dsi)
{
	struct mipi101c *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_bridge_remove(&ctx->bridge);
}

static const struct of_device_id mipi101c_of_match[] = {
	{ .compatible = "myir,mipi101c" },
	{ }
};
MODULE_DEVICE_TABLE(of, mipi101c_of_match);

static struct mipi_dsi_driver mipi101c_driver = {
	.probe = mipi101c_probe,
	.remove = mipi101c_remove,
	.driver = {
		.name = "mipi101c",
		.of_match_table = mipi101c_of_match,
	},
};
module_mipi_dsi_driver(mipi101c_driver);

MODULE_DESCRIPTION("MIPI-DSI based Driver for mipi101c DSI Bridge");
MODULE_LICENSE("GPL v2");
