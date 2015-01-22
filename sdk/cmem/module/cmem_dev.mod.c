#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x9c3f02a1, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0xbb05a5d4, __VMLINUX_SYMBOL_STR(cdev_del) },
	{ 0x43a69ab4, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0x12da5bb2, __VMLINUX_SYMBOL_STR(__kmalloc) },
	{ 0x85dafab0, __VMLINUX_SYMBOL_STR(cdev_init) },
	{ 0xc2b1a7a1, __VMLINUX_SYMBOL_STR(device_destroy) },
	{ 0x8a623588, __VMLINUX_SYMBOL_STR(x86_dma_fallback_dev) },
	{ 0x7485e15e, __VMLINUX_SYMBOL_STR(unregister_chrdev_region) },
	{ 0x68dfc59f, __VMLINUX_SYMBOL_STR(__init_waitqueue_head) },
	{ 0x2bc95bd4, __VMLINUX_SYMBOL_STR(memset) },
	{ 0x6483b924, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0x50eedeb8, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x20c55ae0, __VMLINUX_SYMBOL_STR(sscanf) },
	{ 0xb4390f9a, __VMLINUX_SYMBOL_STR(mcount) },
	{ 0x40c81757, __VMLINUX_SYMBOL_STR(device_create) },
	{ 0xb9642e7b, __VMLINUX_SYMBOL_STR(dma_release_from_coherent) },
	{ 0xed0ed2ac, __VMLINUX_SYMBOL_STR(dma_alloc_from_coherent) },
	{ 0x92cd0937, __VMLINUX_SYMBOL_STR(cdev_add) },
	{ 0xfa774f26, __VMLINUX_SYMBOL_STR(_dev_info) },
	{ 0xba966047, __VMLINUX_SYMBOL_STR(kmem_cache_alloc_trace) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0x74f2483b, __VMLINUX_SYMBOL_STR(remap_pfn_range) },
	{ 0x2e60bace, __VMLINUX_SYMBOL_STR(memcpy) },
	{ 0xcc84ea7c, __VMLINUX_SYMBOL_STR(class_destroy) },
	{ 0x21041166, __VMLINUX_SYMBOL_STR(pci_get_device) },
	{ 0x843aff35, __VMLINUX_SYMBOL_STR(pci_dev_put) },
	{ 0x8d7724c0, __VMLINUX_SYMBOL_STR(__class_create) },
	{ 0xc0e8ca8b, __VMLINUX_SYMBOL_STR(dma_ops) },
	{ 0x29537c9e, __VMLINUX_SYMBOL_STR(alloc_chrdev_region) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "24A0C309F04AEC1471817D9");
