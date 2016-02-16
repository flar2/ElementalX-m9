#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <asm/io.h>
#include <linux/skbuff.h>
#include <linux/wifi_tiwlan.h>
#include "bcm_dev_reg.h"
#ifdef CONFIG_PCI_MSM
#include <linux/msm_pcie.h>
#else
#include <mach/msm_pcie.h>
#endif
#include <linux/random.h>

extern unsigned char *get_wifi_nvs_ram(void);
extern struct wlan_gpio_info gpio_wl_en;
extern struct wlan_gpio_info gpio_irq;
extern struct wlan_gpio_info gpio_seci_in;

int hima_wifi_power(int on);
int hima_wifi_reset(int on);
int hima_wifi_set_carddetect(int on);
int hima_wifi_get_mac_addr(unsigned char *buf);

#define WLAN_STATIC_SCAN_BUF0		5
#define WLAN_STATIC_SCAN_BUF1		6
#define WLAN_STATIC_DHD_INFO_BUF	7
#define WLAN_STATIC_DHD_WLFC_BUF        8
#define WLAN_STATIC_DHD_IF_FLOW_LKUP    9
#define WLAN_STATIC_DHD_MEMDUMP_BUF	10

#define WLAN_SCAN_BUF_SIZE		(64 * 1024)
#define WLAN_DHD_INFO_BUF_SIZE		(24 * 1024)
#define WLAN_DHD_WLFC_BUF_SIZE          (16 * 1024)
#define WLAN_DHD_IF_FLOW_LKUP_SIZE      (34 * 1024)
#define WLAN_DHD_MEMDUMP_SIZE		(768 * 1024)

#define PREALLOC_WLAN_SEC_NUM		4
#define PREALLOC_WLAN_BUF_NUM		160
#define PREALLOC_WLAN_SECTION_HEADER	24

#define DHD_SKB_1PAGE_BUFSIZE	(PAGE_SIZE*1)
#define DHD_SKB_2PAGE_BUFSIZE	(PAGE_SIZE*2)
#define DHD_SKB_4PAGE_BUFSIZE	(PAGE_SIZE*4)

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1	0
#define WLAN_SECTION_SIZE_2	0
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_BUF_NUM * 1024)

#define DHD_SKB_1PAGE_BUF_NUM	0
#define DHD_SKB_2PAGE_BUF_NUM	64
#define DHD_SKB_4PAGE_BUF_NUM	0

#define WLAN_SKB_1_2PAGE_BUF_NUM	((DHD_SKB_1PAGE_BUF_NUM) + \
	(DHD_SKB_2PAGE_BUF_NUM))
#define WLAN_SKB_BUF_NUM	((WLAN_SKB_1_2PAGE_BUF_NUM) + \
	(DHD_SKB_4PAGE_BUF_NUM))

#define HW_OOB 1
#define PREALLOC_FREE_MAGIC		0xFEDC

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

struct wlan_mem_prealloc {
	void *mem_ptr;
	unsigned long size;
};

static struct wlan_mem_prealloc wlan_mem_array[PREALLOC_WLAN_SEC_NUM] = {
	{NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER)}
};

void *wlan_static_scan_buf0 = NULL;
void *wlan_static_scan_buf1 = NULL;
void *wlan_static_dhd_info_buf = NULL;
void *wlan_static_dhd_wlfc_buf = NULL;
void *wlan_static_if_flow_lkup = NULL;
void *wlan_static_dhd_memdump_buf = NULL;

static int hima_wifi_get_irq_gpio(unsigned long *flags);

static void *hima_wifi_mem_prealloc(int section, unsigned long size)
{
	pr_err("%s: request section %d size %lu\n", __FUNCTION__, section, size);
	if (section == PREALLOC_WLAN_SEC_NUM)
		return wlan_static_skb;

	if (section == WLAN_STATIC_SCAN_BUF0)
		return wlan_static_scan_buf0;

	if (section == WLAN_STATIC_SCAN_BUF1)
		return wlan_static_scan_buf1;

	if (section == WLAN_STATIC_DHD_INFO_BUF) {
		if (size > WLAN_DHD_INFO_BUF_SIZE) {
			pr_err("%s: request DHD_INFO size(%lu) is bigger than"
				" static size(%d).\n", __FUNCTION__, size,
				WLAN_DHD_INFO_BUF_SIZE);
			return NULL;
		}
		return wlan_static_dhd_info_buf;
	}

	if (section == WLAN_STATIC_DHD_WLFC_BUF)  {
		if (size > WLAN_DHD_WLFC_BUF_SIZE) {
			pr_err("%s: request DHD_WLFC size(%lu) is bigger than"
				" static size(%d).\n", __FUNCTION__,
				size, WLAN_DHD_WLFC_BUF_SIZE);
			return NULL;
		}
		return wlan_static_dhd_wlfc_buf;
	}

	if (section == WLAN_STATIC_DHD_IF_FLOW_LKUP)  {
		if (size > WLAN_DHD_IF_FLOW_LKUP_SIZE) {
			pr_err("%s: request IF_FLOW_LKUP size(%lu) is bigger than"
				" static size(%d).\n", __FUNCTION__,
				size, WLAN_DHD_IF_FLOW_LKUP_SIZE);
			return NULL;
		}
		return wlan_static_if_flow_lkup;
	}

	if (section == WLAN_STATIC_DHD_MEMDUMP_BUF) {
		if (size > WLAN_DHD_MEMDUMP_SIZE) {
			pr_err("request DHD_MEMDUMP_BUF size(%lu) is bigger"
				" than static size(%d).\n",
				size, WLAN_DHD_MEMDUMP_SIZE);
			return NULL;
		}
		return wlan_static_dhd_memdump_buf;
	}
	if ((section < 0) || (section > PREALLOC_WLAN_SEC_NUM)) {
		pr_err("%s: invalid section(%d).\n", __FUNCTION__,
			section);
		return NULL;
	}

	if (wlan_mem_array[section].size < size) {
		pr_err("%s: request section(%d) size(%lu) is bigger than"
			" static size(%lu).\n", __FUNCTION__,
			section, size, wlan_mem_array[section].size);
		return NULL;
	}

	return wlan_mem_array[section].mem_ptr;
}

int __init hima_init_wifi_mem(void)
{
	int i;
	int j;

	printk(KERN_INFO "%s: Enter\n", __FUNCTION__);
	pr_err("%s: Enter\n", __FUNCTION__);

	for (i = 0; i < DHD_SKB_1PAGE_BUF_NUM; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
		wlan_static_skb[i]->mac_len = PREALLOC_FREE_MAGIC;
	}

	pr_err("%s: Enter 111 \n", __FUNCTION__);
	for (i = DHD_SKB_1PAGE_BUF_NUM; i < WLAN_SKB_1_2PAGE_BUF_NUM; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
		wlan_static_skb[i]->mac_len = PREALLOC_FREE_MAGIC;
	}
	pr_err("%s: Enter 222 \n", __FUNCTION__);

#if !defined(CONFIG_BCMDHD_PCIE)
	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[i])
		goto err_skb_alloc;
#endif

	for (i = 0; i < PREALLOC_WLAN_SEC_NUM; i++) {
		if (wlan_mem_array[i].size > 0) {
			wlan_mem_array[i].mem_ptr =
				kmalloc(wlan_mem_array[i].size, GFP_KERNEL);

			if (!wlan_mem_array[i].mem_ptr)
				goto err_mem_alloc;
		}
	}

	wlan_static_scan_buf0 = kmalloc(WLAN_SCAN_BUF_SIZE, GFP_KERNEL);
	if (!wlan_static_scan_buf0) {
		pr_err("Failed to alloc wlan_static_scan_buf0\n");
		goto err_mem_alloc;
	}

	wlan_static_scan_buf1 = kmalloc(WLAN_SCAN_BUF_SIZE, GFP_KERNEL);
	if (!wlan_static_scan_buf1) {
		pr_err("Failed to alloc wlan_static_scan_buf1\n");
		goto err_mem_alloc;
	}

	wlan_static_dhd_info_buf = kmalloc(WLAN_DHD_INFO_BUF_SIZE, GFP_KERNEL);
	if (!wlan_static_dhd_info_buf) {
		pr_err("Failed to alloc wlan_static_dhd_info_buf\n");
		goto err_mem_alloc;
	}

	wlan_static_if_flow_lkup = kmalloc(WLAN_DHD_IF_FLOW_LKUP_SIZE,
		GFP_KERNEL);
	if (!wlan_static_if_flow_lkup) {
		pr_err("Failed to alloc wlan_static_if_flow_lkup\n");
		goto err_mem_alloc;
	}

	wlan_static_dhd_memdump_buf = kmalloc(WLAN_DHD_MEMDUMP_SIZE, GFP_KERNEL);
	pr_err("%s: wlan_static_dhd_memdump_buf alloc\n", __FUNCTION__);
	if (!wlan_static_dhd_memdump_buf) {
		pr_err("Failed to alloc wlan_static_dhd_memdump_buf\n");
		goto err_mem_alloc;
	}

	pr_err("%s: WIFI MEM Allocated\n", __FUNCTION__);
	return 0;

err_mem_alloc:
	printk(KERN_INFO "%s: failed to mem_alloc for WLAN\n", __FUNCTION__);

	if (wlan_static_dhd_memdump_buf)
		kfree(wlan_static_dhd_memdump_buf);

	if (wlan_static_if_flow_lkup)
		kfree(wlan_static_if_flow_lkup);

	if (wlan_static_dhd_info_buf)
		kfree(wlan_static_dhd_info_buf);

	if (wlan_static_scan_buf1)
		kfree(wlan_static_scan_buf1);

	if (wlan_static_scan_buf0)
		kfree(wlan_static_scan_buf0);

	pr_err("Failed to mem_alloc for WLAN\n");

	for (j = 0; j < i; j++)
		kfree(wlan_mem_array[j].mem_ptr);

	i = WLAN_SKB_BUF_NUM;

err_skb_alloc:
	printk(KERN_INFO "%s: failed to skb_alloc for WLAN\n", __FUNCTION__);
	pr_err("Failed to skb_alloc for WLAN\n");
	for (j = 0; j < i; j++)
		dev_kfree_skb(wlan_static_skb[j]);

	return -ENOMEM;
}

#if 1 
static struct resource hima_wifi_resources[] = {
	[0] = {
		.name		= "bcmdhd_wlan_irq",
		.start		= 679,
		.end		= 679,
#ifdef HW_OOB
		.flags          = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
#else
		.flags          = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
#endif
	},
};
#endif

static struct wifi_platform_data hima_wifi_control = {
	.set_power      = hima_wifi_power,
	.set_reset      = hima_wifi_reset,
	.set_carddetect = hima_wifi_set_carddetect,
	.mem_prealloc   = hima_wifi_mem_prealloc,
	.get_mac_addr	= hima_wifi_get_mac_addr,
	.get_irq_number	= hima_wifi_get_irq_gpio,
};

static struct platform_device hima_wifi_device = {
	.name           = "bcmdhd_wlan",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(hima_wifi_resources),  
	.resource       = hima_wifi_resources,  
	.dev            = {
		.platform_data = &hima_wifi_control,
	},
};

#define NVS_LEN_OFFSET		0x0C
#define NVS_DATA_OFFSET		0x40

#if 0 
static unsigned hima_wifi_update_nvs(char *str)
{
	unsigned char *ptr;
	unsigned len;

	if (!str)
		return -EINVAL;
	ptr = get_wifi_nvs_ram();
	
	memcpy(&len, ptr + NVS_LEN_OFFSET, sizeof(len));

	
	if (ptr[NVS_DATA_OFFSET + len - 1] == 0)
		len -= 1;

	if (ptr[NVS_DATA_OFFSET + len - 1] != '\n') {
		len += 1;
		ptr[NVS_DATA_OFFSET + len - 1] = '\n';
	}

	strcpy(ptr + NVS_DATA_OFFSET + len, str);
	len += strlen(str);
	memcpy(ptr + NVS_LEN_OFFSET, &len, sizeof(len));
	return 0;
}
#endif

#ifdef HW_OOB
#if 0
static unsigned strip_nvs_param(char *param)
{
	unsigned char *nvs_data;

	unsigned param_len;
	int start_idx, end_idx;

	unsigned char *ptr;
	unsigned len;

	if (!param)
		return -EINVAL;
	ptr = get_wifi_nvs_ram();
	
	memcpy(&len, ptr + NVS_LEN_OFFSET, sizeof(len));

	
	if (ptr[NVS_DATA_OFFSET + len - 1] == 0)
		len -= 1;

	nvs_data = ptr + NVS_DATA_OFFSET;

	param_len = strlen(param);

	
	for (start_idx = 0; start_idx < len - param_len; start_idx++) {
		if (memcmp(&nvs_data[start_idx], param, param_len) == 0)
			break;
	}

	end_idx = 0;
	if (start_idx < len - param_len) {
		
		for (end_idx = start_idx + param_len; end_idx < len; end_idx++) {
			if (nvs_data[end_idx] == '\n' || nvs_data[end_idx] == 0)
				break;
		}
	}

	if (start_idx < end_idx) {
		
		for (; end_idx + 1 < len; start_idx++, end_idx++)
			nvs_data[start_idx] = nvs_data[end_idx+1];

		len = len - (end_idx - start_idx + 1);
		memcpy(ptr + NVS_LEN_OFFSET, &len, sizeof(len));
	}
	return 0;
}
#endif
#endif

#define WIFI_MAC_PARAM_STR     "macaddr="
#define WIFI_MAX_MAC_LEN       17 

static uint
get_mac_from_wifi_nvs_ram(char *buf, unsigned int buf_len)
{
	unsigned char *nvs_ptr;
	unsigned char *mac_ptr;
	uint len = 0;

	if (!buf || !buf_len)
		return 0;

	nvs_ptr = get_wifi_nvs_ram();
	if (nvs_ptr)
		nvs_ptr += NVS_DATA_OFFSET;

	mac_ptr = strstr(nvs_ptr, WIFI_MAC_PARAM_STR);
	if (mac_ptr) {
		mac_ptr += strlen(WIFI_MAC_PARAM_STR);

		
		while (mac_ptr[0] == ' ')
			mac_ptr++;

		
		len = 0;
		while (mac_ptr[len] != '\r' && mac_ptr[len] != '\n' &&
			mac_ptr[len] != '\0') {
			len++;
		}

		if (len > buf_len)
			len = buf_len;

		memcpy(buf, mac_ptr, len);
	}

	return len;
}

#define ETHER_ADDR_LEN 6
int hima_wifi_get_mac_addr(unsigned char *buf)
{
	static u8 ether_mac_addr[] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
	char mac[WIFI_MAX_MAC_LEN];
	unsigned mac_len;
	unsigned int macpattern[ETHER_ADDR_LEN];
	int i;
	u32 rand_mac;

	mac_len = get_mac_from_wifi_nvs_ram(mac, WIFI_MAX_MAC_LEN);
	if (mac_len > 0) {
		
		sscanf(mac, "%02x:%02x:%02x:%02x:%02x:%02x",
		&macpattern[0], &macpattern[1], &macpattern[2],
		&macpattern[3], &macpattern[4], &macpattern[5]
		);

		for (i = 0; i < ETHER_ADDR_LEN; i++)
			ether_mac_addr[i] = (u8)macpattern[i];
	} else {
		printk(KERN_INFO "fail to get mac from nvs!\n");
	}

	if (ether_mac_addr[0] == 0x00 &&
		ether_mac_addr[1] == 0x11 &&
		ether_mac_addr[2] == 0x22 &&
		ether_mac_addr[3] == 0x33 &&
		ether_mac_addr[4] == 0x44 &&
		ether_mac_addr[5] == 0x55) {
		printk(KERN_INFO "Warning! Get a default mac -> use random\n");
		prandom_seed((uint)jiffies);
		rand_mac = prandom_u32();
		ether_mac_addr[3] = (unsigned char)rand_mac;
		ether_mac_addr[4] = (unsigned char)(rand_mac >> 8);
		ether_mac_addr[5] = (unsigned char)(rand_mac >> 16);
	}

	memcpy(buf, ether_mac_addr, sizeof(ether_mac_addr));

	printk(KERN_INFO "hima_wifi_get_mac_addr = %02x %02x %02x %02x %02x %02x \n",
		ether_mac_addr[0], ether_mac_addr[1], ether_mac_addr[2], ether_mac_addr[3], ether_mac_addr[4], ether_mac_addr[5]);

	return 0;
}

int hima_wifi_power(int on)
{
    int err = -1;

    if (on) {
        if (gpio_seci_in.prop == false) {
            err = wlan_seci_gpio_init(&gpio_seci_in, 1);
            if (err != 0)
                goto err_power;
        }
        err = bcm_gpio_set(&gpio_wl_en, WLAN_EN_HIGH);
    } else {
        if (gpio_seci_in.prop == true)
            err = wlan_seci_gpio_init(&gpio_seci_in, 0);
        err = bcm_gpio_set(&gpio_wl_en, WLAN_EN_LOW);
    }
err_power:
    return err;

}

int hima_wifi_reset(int on)
{
	printk(KERN_INFO "%s: do nothing\n", __func__);
	return 0;
}

int hima_wifi_set_carddetect(int val)
{
	int ret, rc_num = 1;

	ret = msm_pcie_enumerate(rc_num);
    
    return ret;
}

static int hima_wifi_get_irq_gpio(unsigned long *flags)
{
	int irq_num = 0;

	pr_err("%s: gpio_irq.num %d\n", __FUNCTION__, gpio_irq.num);
	if (gpio_irq.num) {
		irq_num = gpio_to_irq(gpio_irq.num);
		pr_err("%s: irq_num %d\n", __FUNCTION__, irq_num);
	}
	if (hima_wifi_device.resource != NULL) {
		pr_err("%s: flags %lx\n", __FUNCTION__, hima_wifi_device.resource[0].flags);
		*flags = hima_wifi_device.resource[0].flags;
	}
	return irq_num;
}

int release_wlan_seci_gpio(void)
{
   return wlan_seci_gpio_init(&gpio_seci_in, 0);
}
EXPORT_SYMBOL(release_wlan_seci_gpio);

int lock_wlan_seci_gpio(void)
{
   return wlan_seci_gpio_init(&gpio_seci_in, 1);
}
EXPORT_SYMBOL(lock_wlan_seci_gpio);

bool get_gpio4_state(void)
{
	return gpio_seci_in.prop;
}
EXPORT_SYMBOL(get_gpio4_state);

int __init hima_wifi_init(void)
{
	int ret;
	unsigned long flags;

	printk(KERN_INFO "%s: start\n", __func__);

#ifdef HW_OOB
	
#else
	
#endif
	
	
	hima_init_wifi_mem();

	if (hima_wifi_device.resource != NULL) {
		hima_wifi_device.resource[0].start = hima_wifi_get_irq_gpio(&flags);
		hima_wifi_device.resource[0].end = hima_wifi_get_irq_gpio(&flags);
	}
	ret = platform_device_register(&hima_wifi_device);
	return ret;
}

