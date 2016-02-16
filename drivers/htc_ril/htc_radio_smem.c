#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/htc_flags.h>
#include <linux/ctype.h>
#include <linux/dma-mapping.h>
#include <linux/debugfs.h>
#include <linux/dma-contiguous.h>

#include "htc_radio_smem.h"

#define CONFIG_RADIO_FEEDBACK 1
#define CONFIG_RAMDUMP_SMLOG

#ifdef CONFIG_RADIO_FEEDBACK
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#endif 

#ifdef CONFIG_RAMDUMP_SMLOG
#include <soc/qcom/smem.h>
#include <soc/qcom/ramdump.h>
#include <linux/remote_spinlock.h>
#include <soc/qcom/subsystem_notif.h>
#endif 

static int boot_mode = APP_IN_HLOS;
static unsigned long radioflag;
static unsigned long radioflagex1;
static unsigned long radioflagex2;
static char *rom_version;
static bool smlog_enabled;
phys_addr_t smem_start_addr;
static bool cma_reserved;
void *smlog_base_vaddr;

#define HTC_ROM_VERSION_PATH "/chosen/misc"
#define HTC_ROM_VERSION_PROPERTY "firmware_main_version"
#define RADIO_SMLOG_FLAG BIT(27)
#define UT_LONG_SKU_LEN 5
#define UT_LONG_SKU_FIRST_NUM '9'
#define UT_SHORT_SKU_NUM "999"

#ifdef CONFIG_RADIO_FEEDBACK
#define RADIO_FEEDBACK_IOCTL_MAGIC	'p'
#define RADIO_FEEDBACK_GET_CDLOG_INFO	_IOW(RADIO_FEEDBACK_IOCTL_MAGIC, 89, unsigned)

struct msm_radio_feedback_config {
	uint32_t start_addr;
	uint32_t max_size;
};
struct mutex radio_feedback_lock;
struct msm_radio_feedback_config radio_feedback_config;
#endif 

#ifdef CONFIG_RAMDUMP_SMLOG
struct restart_notifier_block {
	unsigned processor;
	char *name;
	struct notifier_block nb;
};

static uint32_t num_smlog_areas;
static struct ramdump_segment *smlog_ramdump_segments;
static void *smlog_ramdump_dev;
#endif 

int __init cmdline_boot_mode_read(char *s)
{
	if (!strcmp(s, "ftm"))
		boot_mode = APP_IN_FTM;
	else if (!strcmp(s, "mfgkernel"))
		boot_mode = APP_IN_MFGKERNEL;
	else if (!strcmp(s, "mfgkernel:diag58"))
		boot_mode = APP_IN_DIAG;
	else if (!strcmp(s, "recovery") || !strcmp(s, "recovery_manual") ||
			 !strcmp(s, "offmode_charging") || !strcmp(s, "repartition"))
		boot_mode = APP_IN_RECOVERY;
	else if (!strcmp(s, "power_test"))
		boot_mode = APP_IN_TESTBOOTLOADER;
	else
		boot_mode = APP_IN_HLOS;
        return 1;
}
__setup("androidboot.mode=", cmdline_boot_mode_read);

int __init cmdline_radioflag_read(char *s)
{
	int res;

	res = kstrtoul(s, 16, &radioflag);
	return 1;
}
__setup("radioflag=", cmdline_radioflag_read);

int __init cmdline_radioflagex1_read(char *s)
{
	int res;

	res = kstrtoul(s, 16, &radioflagex1);
	return 1;
}
__setup("radioflagex1=", cmdline_radioflagex1_read);

int __init cmdline_radioflagex2_read(char *s)
{
	int res;

	res = kstrtoul(s, 16, &radioflagex2);
	return 1;
}
__setup("radioflagex2=", cmdline_radioflagex2_read);

#ifdef CONFIG_DEBUG_FS
static int dump_smem(char *buf, int max){
	int i = 0;
	struct htc_smem_type *smem;

	smem = ioremap(smem_start_addr, sizeof(struct htc_smem_type));

	if(!smem){
        i += scnprintf(buf + i, max - i, "ioremap fail.\n");
		pr_err("[smem]%s: ioremap fail.\n", __func__);
		return i;
	}

	i += scnprintf(buf + i, max - i,
				   "version:        %x\n"
				   "size:           %d\n"
				   "boot mode:      %d\n"
				   "radioflag:      0x%x\n"
				   "radioflagex1:   0x%x\n"
				   "radioflagex2:   0x%x\n"
				   "CMA reserved:   %s\n"
				   "smlog_magic:    0x%x\n"
				   "smlog_base:     0x%x\n"
				   "smlog_size:     0x%x\n"
				   "ROM version:    %s\n"
				   "RCMS name:      %s\n"
				   "SKU name:       %s\n",
				   smem->version,
				   smem->struct_size,
				   smem->htc_smem_app_run_mode,
				   smem->htc_smem_ce_radio_dbg_flag,
				   smem->htc_smem_ce_radio_dbg_flag_ext1,
				   smem->htc_smem_ce_radio_dbg_flag_ext2,
				   cma_reserved ? "Success" : "Fail",
				   smem->htc_smlog_magic,
				   smem->htc_smlog_base,
				   smem->htc_smlog_size,
				   smem->htc_rom_ver,
				   smem->RCMS_name,
				   smem->SKU_Name
				   );

	iounmap(smem);
	return i;
}

#define DEBUG_BUFMAX 4096
static char debug_buffer[DEBUG_BUFMAX];

static ssize_t debug_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	int (*fill)(char *buf, int max) = file->private_data;
	int bsize = fill(debug_buffer, DEBUG_BUFMAX);
	return simple_read_from_buffer(buf, count, ppos, debug_buffer, bsize);
}

static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations debug_ops = {
	.read = debug_read,
	.open = debug_open,
};

static void debug_create(const char *name, mode_t mode,
				struct dentry *dent,
				int (*fill)(char *buf, int max))
{
	struct dentry *file;

	file = debugfs_create_file(name, mode, dent, fill, &debug_ops);
	if (IS_ERR(file))
		pr_err("%s: debugfs create failed %d\n", __func__,
				(int)PTR_ERR(file));
}
#endif

phys_addr_t get_smem_base(void){
	return smem_start_addr;
}
EXPORT_SYMBOL(get_smem_base);

static bool is_ut_rom(void)
{
	int len = 0;
	int c = 0;
	int i = 0;
	char *main_version, *sku_version;

	if(!rom_version){
		pr_err("[smem]%s: no ROM version.\n", __func__);
		return false;
	}

	main_version = strstr(rom_version, ".");
	if(!main_version || strlen(main_version+1) == 0) {
		pr_err("[smem]%s: no main version.\n", __func__);
		return false;
	}

	sku_version = strstr(main_version+1, ".") + 1;
	if(!sku_version || strlen(sku_version+1) == 0) {
		pr_err("[smem]%s: no sku version.\n", __func__);
		return false;
	}

	len = strlen(sku_version);
	for(i = 0; i < len; i++){
		if(isdigit(sku_version[i]))
			c++;
		else
			break;
	}

	if(sku_version[0] == UT_LONG_SKU_FIRST_NUM && c == UT_LONG_SKU_LEN)
		return true;

	if( strstr(sku_version, UT_SHORT_SKU_NUM) && c == strlen(UT_SHORT_SKU_NUM))
		return true;

	return false;
}

static bool is_smlog_enabled(void)
{
	if(boot_mode == APP_IN_HLOS){
		if(is_ut_rom()){
			if(get_radio_flag() & RADIO_SMLOG_FLAG)
				return false;
			else
				return true;
		}else if(get_radio_flag() & RADIO_SMLOG_FLAG)
			return true;
		else
			return false;
	}else
		return false;
}

static void set_smlog_magic(bool is_enabled, struct htc_smem_type *smem, dma_addr_t base, int size)
{
	if(is_enabled){
		smem->htc_smlog_magic = HTC_SMLOG_MAGIC_NUMBER;
		smem->htc_smlog_base = (uint32_t) base;
		smem->htc_smlog_size = size;
	}else{
		smem->htc_smlog_magic = 0;
		smem->htc_smlog_base = 0;
		smem->htc_smlog_size = 0;
	}
	pr_info("[smem]%s: smlog_magic:0x%x, smlog_base:0x%x, smlog_size:0x%x.\n",
			__func__, smem->htc_smlog_magic, smem->htc_smlog_base, smem->htc_smlog_size);
}

static int check_smlog_alloc(struct device *dev, struct htc_smem_type *smem)
{
	dma_addr_t addr = 0;
	int ret = 0;
	int smlog_buf_size = 0;

	
	if(!dev->cma_area){
		pr_err("[smem]%s: CMA reserved fail.\n", __func__);
		cma_reserved = false;
		ret = -ENOMEM;
		goto alloc_fail;
	}

	cma_reserved = true;
	smlog_enabled = is_smlog_enabled();

	if(smlog_enabled){
		smlog_buf_size = cma_get_size(dev);
		smlog_base_vaddr = dma_alloc_writecombine(dev, smlog_buf_size, &addr,
						   GFP_KERNEL);
		if (!smlog_base_vaddr) {
			pr_err("[smem]%s: cannot alloc memory for smlog.\n", __func__);
			ret = -ENOMEM;
			goto alloc_fail;
		}

		pr_info("[smem]%s: smlog is enabled.\n", __func__);
	}else
		pr_info("[smem]%s: smlog is disabled.\n", __func__);

	set_smlog_magic(smlog_enabled, smem, addr, smlog_buf_size);
	return ret;

alloc_fail:
	smlog_enabled = false;
	set_smlog_magic(smlog_enabled, smem, addr, smlog_buf_size);

	return ret;
}

static void htc_radio_smem_write(struct htc_smem_type *smem)
{
	smem->version = HTC_RADIO_SMEM_VERSION;
	smem->struct_size = sizeof(struct htc_smem_type);
	smem->htc_smem_app_run_mode = boot_mode;
	smem->htc_smem_ce_radio_dbg_flag = (uint32_t) radioflag;
	smem->htc_smem_ce_radio_dbg_flag_ext1 = (uint32_t) radioflagex1;
	smem->htc_smem_ce_radio_dbg_flag_ext2 = (uint32_t) radioflagex2;

	smem->htc_smem_is_nv_backup = 0;
	strncpy(smem->RCMS_name, RCMS_NAME, sizeof(smem->RCMS_name));
	strncpy(smem->SKU_Name, SKU_NAME, sizeof(smem->SKU_Name));

	pr_info("[smem]%s: RCMS NAME=%s, version=0x%x, size=%d, boot_mode=%d, "
			"radioflag=0x%x, radioflagex1=0x%x, radioflagex2=0x%x\n",
			__func__, smem->RCMS_name, smem->version,
			smem->struct_size,
			smem->htc_smem_app_run_mode,
			smem->htc_smem_ce_radio_dbg_flag,
			smem->htc_smem_ce_radio_dbg_flag_ext1,
			smem->htc_smem_ce_radio_dbg_flag_ext2);

	if(rom_version) {
		strncpy(smem->htc_rom_ver, rom_version, sizeof(smem->htc_rom_ver));
	}else{
		pr_err("[smem]%s: ROM version does not be found.\n", __func__);
	}
}

static void smem_init(struct htc_smem_type *smem){
       int i = 0;

       smem->version = 0;
       smem->struct_size = 0;
       smem->htc_smem_app_run_mode = 0;
       smem->htc_smem_ce_radio_dbg_flag = 0;
       smem->htc_smem_ce_radio_dbg_flag_ext1 = 0;
       smem->htc_smem_ce_radio_dbg_flag_ext2 = 0;
       smem->htc_smlog_magic = 0;
       smem->htc_smlog_base = 0;
       smem->htc_smlog_size = 0;

       for(i=0; i<sizeof(smem->htc_rom_ver); i++)
		   smem->htc_rom_ver[i] = 0;

       smem->htc_smem_is_nv_backup = 0;

	   for(i=0; i<sizeof(smem->RCMS_name); i++)
		   smem->RCMS_name[i] = 0;

	   smem->htc_smem_ce_radio_dbg_ril_fatal = 0;

	   for(i=0; i<sizeof(smem->SKU_Name); i++)
		   smem->SKU_Name[i] = 0;

	   for(i=0; i<sizeof(smem->reserved); i++)
		   smem->reserved[i] = 0;
}

#ifdef CONFIG_RAMDUMP_SMLOG
static int restart_notifier_cb(struct notifier_block *this,
				unsigned long code,
				void *data)
{
#if defined(CONFIG_HTC_DEBUG_SSR)
	if (code == SUBSYS_AFTER_SHUTDOWN) {
		struct restart_notifier_block *notifier;

		notifier = container_of(this,
					struct restart_notifier_block, nb);
		pr_info("[smlog]%s: ssrestart for processor %d ('%s')\n",
				__func__, notifier->processor,
				notifier->name);

		remote_spin_release_all(notifier->processor);

		if (smlog_ramdump_dev) {
			int ret;

			pr_info("[smlog]%s: saving smlog ramdump.\n", __func__);
			ret = do_ramdump(smlog_ramdump_dev,
					smlog_ramdump_segments, num_smlog_areas);
			if (ret < 0)
				pr_err("[smlog]%s: unable to dump smlog %d\n",
								__func__, ret);
		}
	}
#endif
	return NOTIFY_DONE;
}

static struct restart_notifier_block restart_notifiers[] = {
	{SMEM_MODEM, "modem", .nb.notifier_call = restart_notifier_cb},
};
#endif 

static int htc_radio_smem_probe(struct platform_device *pdev)
{
	int ret = -1;
	char *key;
	struct resource *res;
	struct htc_smem_type *htc_radio_smem;
	struct device_node *dnp;
	int property_size = 0;
#ifdef CONFIG_RAMDUMP_SMLOG
       int i;
       void *handle;
       struct restart_notifier_block *nb;
       int smlog_idx = 0;
       struct ramdump_segment *ramdump_segments_tmp = NULL;
#endif 

	pr_info("[smem]%s: start.\n", __func__);

	
	key = "smem-start-addr";
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, key);
	if(!res){
		ret = -ENODEV;
		goto missing_key;
	}

	smem_start_addr = res->start;

	htc_radio_smem = ioremap(smem_start_addr, sizeof(struct htc_smem_type));

	if(htc_radio_smem) {
		pr_info("[smem]%s: htc_radio_smem=0x%p.\n", __func__, htc_radio_smem);
	}else{
		ret = -ENOMEM;
		goto ioremap_fail;
	}

	
	dnp = of_find_node_by_path(HTC_ROM_VERSION_PATH);
	if(dnp) {
		rom_version = (char *) of_get_property(dnp, HTC_ROM_VERSION_PROPERTY, &property_size);
	}else
		pr_err("[smem]%s: cannot find path %s.\n", __func__, HTC_ROM_VERSION_PATH);

	
	smem_init(htc_radio_smem);

	ret = check_smlog_alloc(&pdev->dev, htc_radio_smem);
	if(ret < 0)
		pr_err("[smem]%s smlog region alloc fail.\n", __func__);

	
	htc_radio_smem_write(htc_radio_smem);

#ifdef CONFIG_RADIO_FEEDBACK
	radio_feedback_config.start_addr = htc_radio_smem->htc_smlog_base;
	radio_feedback_config.max_size = htc_radio_smem->htc_smlog_size;
#endif 

#ifdef CONFIG_RAMDUMP_SMLOG
	
	if(smlog_enabled) {

		num_smlog_areas = 1;
		ramdump_segments_tmp = kmalloc_array(num_smlog_areas,
                                             sizeof(struct ramdump_segment), GFP_KERNEL);
		if (!ramdump_segments_tmp) {
			pr_err("[smlog] ramdump segment kmalloc failed.\n");
			ret = -ENOMEM;
			goto free_smlog_areas;
		}

		smlog_ramdump_dev = create_ramdump_device("smlog", NULL);
		if (IS_ERR_OR_NULL(smlog_ramdump_dev)) {
			pr_err("[smlog]%s: Unable to create smlog ramdump device.\n", __func__);
			smlog_ramdump_dev = NULL;
		}

		for (i = 0; i < ARRAY_SIZE(restart_notifiers); i++) {
			nb = &restart_notifiers[i];
			handle = subsys_notif_register_notifier(nb->name, &nb->nb);
			pr_info("[smlog] registering notif for '%s', handle=0x%p\n", nb->name, handle);
		}

		ramdump_segments_tmp[smlog_idx].address = (unsigned long)cma_get_base(&pdev->dev);
		ramdump_segments_tmp[smlog_idx].size = cma_get_size(&pdev->dev);
		ramdump_segments_tmp[smlog_idx].v_address = smlog_base_vaddr;

		smlog_ramdump_segments = ramdump_segments_tmp;
		pr_info("[smlog] smlog address= 0x%lx, size= 0x%lx, v_address= 0x%p\n",
				smlog_ramdump_segments[smlog_idx].address,
				smlog_ramdump_segments[smlog_idx].size,
				smlog_ramdump_segments[smlog_idx].v_address);

		goto finish_probe;

free_smlog_areas:
		num_smlog_areas = 0;
		kfree(ramdump_segments_tmp);
	}

finish_probe:
#endif 
	iounmap(htc_radio_smem);

	pr_info("[smem]%s: end.\n", __func__);

	return 0;

missing_key:
	pr_err("[smem]%s: missing key: %s", __func__, key);
	return ret;
ioremap_fail:
	pr_err("[smem]%s: ioremap fail, htc_radio_smem:%p.\n", __func__, htc_radio_smem);
	return ret;
}

static struct of_device_id htc_radio_smem_of_match[] = {
	{.compatible = "htc,htc_radio_smem",},
	{},
};
MODULE_DEVICE_TABLE(of, htc_radio_smem_of_match);

#ifdef CONFIG_RADIO_FEEDBACK
static long radio_feedback_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	switch (cmd) {
	case RADIO_FEEDBACK_GET_CDLOG_INFO:
		printk("start addr: 0x%x, max_size: 0x%x\n", radio_feedback_config.start_addr, radio_feedback_config.max_size);
		if(copy_to_user((void *)arg, &radio_feedback_config, sizeof(radio_feedback_config)))
			rc = -EFAULT;
		break;
	default:
		rc = -EINVAL;
	}
	return rc;
}

static int radio_feedback_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long pgoff;
	size_t size = vma->vm_end - vma->vm_start;
	if (vma->vm_pgoff != 0)
		return -EINVAL;

	if (size <= radio_feedback_config.max_size)
		pgoff = radio_feedback_config.start_addr >> PAGE_SHIFT;
	else
		return -EINVAL;

	vma->vm_flags |= VM_IO | (VM_DONTEXPAND | VM_DONTDUMP);
	if (io_remap_pfn_range(vma, vma->vm_start, pgoff,
			       size, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static struct file_operations radio_feedback_fops = {
	.owner = THIS_MODULE,
	.mmap = radio_feedback_mmap,
	.unlocked_ioctl = radio_feedback_ioctl,
};

static struct miscdevice radio_feedback_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "radio_feedback",
	.fops = &radio_feedback_fops,
};
#endif 

static struct platform_driver htc_radio_smem_driver = {
	.probe = htc_radio_smem_probe,
	.driver = {
		.name = "htc_radio_smem",
		.owner = THIS_MODULE,
		.of_match_table = htc_radio_smem_of_match,
	},
};

static int __init htc_radio_smem_init(void)
{
	int ret = -1;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dent;
#endif

	pr_info("[smem]%s.\n", __func__);

	ret = platform_driver_register(&htc_radio_smem_driver);
	if(ret < 0 ) {
		pr_err("[smem]%s platform_driver register fail. ret:%d\n", __func__, ret);
		goto register_fail;
	}

#ifdef CONFIG_DEBUG_FS
	dent = debugfs_create_dir("htc_radio_smem", 0);
	if (!IS_ERR(dent)) {
		debug_create("dump_smem", 0444, dent, dump_smem);
	}
#endif

#ifdef CONFIG_RADIO_FEEDBACK
	ret = misc_register(&radio_feedback_misc);
	if (ret < 0) {
		platform_driver_unregister(&htc_radio_smem_driver);
		pr_err("failed to register misc device!\n");
		goto register_fail;
	}
	mutex_init(&radio_feedback_lock);
#endif 

register_fail:
	return ret;
}

static void __exit htc_radio_smem_exit(void)
{
#ifdef CONFIG_RADIO_FEEDBACK
	int ret;
	ret = misc_deregister(&radio_feedback_misc);
	if (ret < 0)
		pr_err("failed to unregister misc device!\n");
#endif 
	platform_driver_unregister(&htc_radio_smem_driver);
}

module_init(htc_radio_smem_init);
module_exit(htc_radio_smem_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("htc radio smem driver");
