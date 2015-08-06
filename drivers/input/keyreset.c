/* drivers/input/keyreset.c
 *
 * Copyright (C) 2014 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/input.h>
#include <linux/of_gpio.h>

#include <linux/keyreset.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/keycombo.h>

struct keyreset_state {
	int restart_requested;
	int (*reset_fn)(void);
	struct platform_device *pdev_child;
};

static void do_restart(void)
{
#if defined(CONFIG_POWER_KEY_CLR_RESET)
	clear_hw_reset();
#endif
	sys_sync();
	KEY_LOGI("[PWR] Show Blocked State -- long press power key\n");
	show_state_filter(TASK_UNINTERRUPTIBLE);
	machine_restart("power-key-force-hard");
}

static void do_reset_fn(void *priv)
{
	struct keyreset_state *state = priv;
	if (state->restart_requested)
		panic("[KEY] keyboard reset failed, %d", state->restart_requested);
	if (state->reset_fn) {
		KEY_LOGI("keyboard reset\n");
		state->restart_requested = state->reset_fn();
	} else {
		KEY_LOGI("keyboard reset (default)\n");
		do_restart();
		state->restart_requested = 1;
	}
}

static int keyreset_parse_dt(struct device_node *dt,
		struct keyreset_platform_data *pdata)
{
	int ret = 0, cnt = 0, num_keys;
	struct property *prop;
	const char *parser_st[] = {"key_down_delay", "keys_down", "keys_up"};

	
	if (of_property_read_u32(dt, parser_st[0], &pdata->key_down_delay))
		KEY_LOGI("DT:%s parser gets nothing\n", parser_st[0]);

	KEY_LOGI("DT:%s=%d\n", parser_st[0], pdata->key_down_delay);

	
	
	prop = of_find_property(dt, parser_st[1], NULL);
	if (!prop) {
		KEY_LOGE("DT:%s property not found\n", parser_st[1]);
		ret = -EINVAL;
		goto err_parse_keys_down_failed;
	} else {
		num_keys = prop->length / sizeof(uint32_t);
		KEY_LOGI("DT:%s num_keys=%d\n", parser_st[1], num_keys);
	}

	pdata->keys_down = kzalloc(num_keys * sizeof(uint32_t), GFP_KERNEL);
	if (!pdata->keys_down) {
		KEY_LOGE("DT:%s fail to allocate memory\n", parser_st[1]);
		ret = -ENOMEM;
		goto err_parse_keys_down_failed;
	}

	if (of_property_read_u32_array(dt, parser_st[1], pdata->keys_down, num_keys)) {
		KEY_LOGE("DT:%s parse err\n", parser_st[1]);
		ret = -EINVAL;
		goto err_keys_down_failed;
	}

	for(cnt = 0; cnt < num_keys; cnt++)
		KEY_LOGI("DT:%s=%d\n", parser_st[1], pdata->keys_down[cnt]);

	
	
	prop = of_find_property(dt, parser_st[2], NULL);
	if (!prop) {
		KEY_LOGI("DT:%s property not found\n", parser_st[2]);
		return 0;
	} else {
		num_keys = prop->length / sizeof(uint32_t);
		KEY_LOGI("DT:%s num_keys=%d\n", parser_st[2], num_keys);
	}

	pdata->keys_up = kzalloc(num_keys * sizeof(uint32_t), GFP_KERNEL);
	if (!pdata->keys_up) {
		KEY_LOGE("DT:%s fail to allocate memory\n", parser_st[2]);
		ret = -ENOMEM;
		goto err_parse_keys_up_failed;
	}

	if (of_property_read_u32_array(dt, parser_st[2], pdata->keys_up, num_keys)) {
		KEY_LOGE("DT:%s parse err\n", parser_st[2]);
		ret = -EINVAL;
		goto err_keys_up_failed;
	}

	for(cnt = 0; cnt < num_keys; cnt++)
		KEY_LOGI("DT:%s=%d\n", parser_st[2], pdata->keys_up[cnt]);

	return 0;

err_keys_up_failed:
	kfree(pdata->keys_up);
err_parse_keys_up_failed:
err_keys_down_failed:
	kfree(pdata->keys_down);
err_parse_keys_down_failed:
	return ret;
}

static int keyreset_probe(struct platform_device *pdev)
{
	int ret = -ENOMEM;
	struct keycombo_platform_data *pdata_child;
	struct keyreset_platform_data *pdata;
	int up_size = 0, down_size = 0, size;
	int key, *keyp;
	struct keyreset_state *state;

	KEY_LOGI("%s: +++\n", __func__);

	if (pdev->dev.of_node) {
		pdata = kzalloc(sizeof(struct keyreset_platform_data), GFP_KERNEL);
		if (!pdata) {
			KEY_LOGE("[KEY] fail to allocate keyreset_platform_data\n");
			ret = -ENOMEM;
			goto err_get_pdata_fail;
		}
		ret = keyreset_parse_dt(pdev->dev.of_node, pdata);
		if (ret < 0) {
			KEY_LOGE("[KEY] keyreset_parse_dt fail\n");
			ret = -ENOMEM;
			goto err_parse_fail;
		}
	} else {
		pdata = pdev->dev.platform_data;
		if(!pdata) {
			KEY_LOGE("[KEY] keyreset_platform_data does not exist\n");
			ret = -ENOMEM;
			goto err_get_pdata_fail;
		}
	}

	state = devm_kzalloc(&pdev->dev, sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	state->pdev_child = platform_device_alloc(KEYCOMBO_NAME,
							PLATFORM_DEVID_AUTO);
	if (!state->pdev_child)
		return -ENOMEM;
	state->pdev_child->dev.parent = &pdev->dev;

	keyp = pdata->keys_down;
	while ((key = *keyp++)) {
		if (key >= KEY_MAX)
			continue;
		down_size++;
	}
	if (pdata->keys_up) {
		keyp = pdata->keys_up;
		while ((key = *keyp++)) {
			if (key >= KEY_MAX)
				continue;
			up_size++;
		}
	}
	size = sizeof(struct keycombo_platform_data);
	pdata_child = devm_kzalloc(&pdev->dev, sizeof(struct keycombo_platform_data),
				GFP_KERNEL);
	if (!pdata_child)
		goto error;

	pdata_child->keys_down = devm_kzalloc(&pdev->dev,
				sizeof(uint32_t) * (down_size + 1), GFP_KERNEL);
	if (!pdata_child->keys_down)
		goto error;

	memcpy(pdata_child->keys_down, pdata->keys_down,
						sizeof(uint32_t) * down_size);
	if (!pdata_child->keys_down)
		goto error;

	if (up_size > 0) {
		pdata_child->keys_up = devm_kzalloc(&pdev->dev,
					sizeof(uint32_t) * (up_size + 1), GFP_KERNEL);
		if (!pdata_child->keys_up)
			goto error;
		memcpy(pdata_child->keys_up, pdata->keys_up,
							sizeof(uint32_t) * up_size);
		if (!pdata_child->keys_up)
			goto error;
	}
	state->reset_fn = pdata->reset_fn;
	pdata_child->key_down_fn = do_reset_fn;
	pdata_child->priv = state;
	pdata_child->key_down_delay = pdata->key_down_delay;
	ret = platform_device_add_data(state->pdev_child, pdata_child, size);
	if (ret)
		goto error;
	platform_set_drvdata(pdev, state);

	KEY_LOGI("%s: ---\n", __func__);

	return platform_device_add(state->pdev_child);
error:
	platform_device_put(state->pdev_child);
err_parse_fail:
	if (pdev->dev.of_node)
		kfree(pdata);
err_get_pdata_fail:
	return ret;
}

int keyreset_remove(struct platform_device *pdev)
{
	struct keyreset_state *state = platform_get_drvdata(pdev);
	platform_device_put(state->pdev_child);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id keyreset_mttable[] = {
	{ .compatible = KEYRESET_NAME},
	{},
};
#else
#define keyreset_mttable NULL
#endif

struct platform_driver keyreset_driver = {
	.probe = keyreset_probe,
	.remove = keyreset_remove,
	.driver = {
		.name = KEYRESET_NAME,
		.owner = THIS_MODULE,
		.of_match_table = keyreset_mttable,
	},
};

static int __init keyreset_init(void)
{
	return platform_driver_register(&keyreset_driver);
}

static void __exit keyreset_exit(void)
{
	return platform_driver_unregister(&keyreset_driver);
}

module_init(keyreset_init);
module_exit(keyreset_exit);
