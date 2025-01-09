#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x6e98403d, "module_layout" },
	{ 0xb077e70a, "clk_unprepare" },
	{ 0xf9a482f9, "msleep" },
	{ 0x617acba3, "regulator_set_voltage" },
	{ 0x815588a6, "clk_enable" },
	{ 0x98cf60b3, "strlen" },
	{ 0xe02098d4, "i2c_del_driver" },
	{ 0x61bd2bba, "of_parse_phandle" },
	{ 0xdefc6cab, "regulator_disable" },
	{ 0xb6e6d99d, "clk_disable" },
	{ 0x12a4e128, "__arch_copy_from_user" },
	{ 0x55b9c770, "gpio_to_desc" },
	{ 0xeae3dfd6, "__const_udelay" },
	{ 0x3213f038, "mutex_unlock" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0x556e4390, "clk_get_rate" },
	{ 0xdc1a9e63, "_dev_warn" },
	{ 0xdcb764ad, "memset" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0xc29a9c1, "of_graph_get_next_endpoint" },
	{ 0x60f253f8, "media_entity_pads_init" },
	{ 0xcd2a9013, "fwnode_property_read_u64_array" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0xb9733443, "_dev_err" },
	{ 0x50573316, "devm_gpio_request_one" },
	{ 0x5b71a735, "i2c_register_driver" },
	{ 0x483328d5, "devm_regulator_get" },
	{ 0x6cbbfc54, "__arch_copy_to_user" },
	{ 0xc292a4fd, "v4l2_async_register_subdev_sensor" },
	{ 0x3ea1b6e4, "__stack_chk_fail" },
	{ 0x92997ed8, "_printk" },
	{ 0x7c9a7371, "clk_prepare" },
	{ 0x3684bf45, "of_get_named_gpio_flags" },
	{ 0xf3946c2, "devm_clk_get" },
	{ 0x76d9b876, "clk_set_rate" },
	{ 0xc3055d20, "usleep_range_state" },
	{ 0x1a6bce4, "i2c_transfer_buffer_flags" },
	{ 0x4829a47e, "memcpy" },
	{ 0x18a8db20, "v4l2_async_unregister_subdev" },
	{ 0x3bf28f64, "gpiod_set_raw_value_cansleep" },
	{ 0x98a620b1, "of_property_read_variable_u32_array" },
	{ 0xb426df06, "devm_kmalloc" },
	{ 0x896683ba, "v4l2_i2c_subdev_init" },
	{ 0xb3ee2328, "regulator_enable" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("i2c:ar0830");
MODULE_ALIAS("of:N*T*Consemi,ar0830");
MODULE_ALIAS("of:N*T*Consemi,ar0830C*");
