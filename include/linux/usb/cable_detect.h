#ifndef _CABLE_DETECT_H_
#define _CABLE_DETECT_H_

#define DOCK_STATE_UNDEFINED				-1
#define DOCK_STATE_UNDOCKED				0
#define DOCK_STATE_DESK					(1 << 0)
#define DOCK_STATE_CAR						(1 << 1)
#define DOCK_STATE_USB_HEADSET			(1 << 2)
#define DOCK_STATE_MHL						(1 << 3)
#define DOCK_STATE_USB_HOST				(1 << 4)
#define DOCK_STATE_DMB						(1 << 5)
#define DOCK_STATE_AUDIO_DOCK				(1 << 6)
#define DOCK_STATE_THREE_POGO_DOCK		(1 << 7)

#define DOCK_DET_DELAY		HZ/4
#ifdef CONFIG_MACH_TC2
__maybe_unused static int htc_cable_detect_retry_times = 5;
#define ADC_RETRY htc_cable_detect_retry_times
#else
__maybe_unused static int htc_cable_detect_retry_times = 3;
#define ADC_RETRY htc_cable_detect_retry_times
#endif
#define ADC_DELAY HZ/8

#define PM8058ADC_15BIT(adc) ((adc * 2200) / 32767) 

#define CABLE_ERR(fmt, args...) \
	printk(KERN_ERR "[CABLE:ERR] " fmt, ## args)
#define CABLE_WARNING(fmt, args...) \
	printk(KERN_WARNING "[CABLE] " fmt, ## args)
#define CABLE_INFO(fmt, args...) \
	printk(KERN_INFO "[CABLE] " fmt, ## args)
#define CABLE_DEBUG(fmt, args...) \
	printk(KERN_DEBUG "[CABLE] " fmt, ## args)

enum accessory_type {
	CABLE_TYPE_UNKOWN = 0,
	CABLE_TYPE_ID_PIN,
	CABLE_TYPE_PMIC_ADC,
};

enum usb_id_pin_type {
	CABLE_TYPE_APP_GPIO = 0,
	CABLE_TYPE_PMIC_GPIO,
};

enum dpdn_path_type {
	PATH_USB = 0,
	PATH_MHL,
	PATH_USB_AUD,
	PATH_UART,
};

enum usb_host_mhl_type {
	CABLE_TYPE_NONE = 0,
	CABLE_TYPE_HOST,
	CABLE_TYPE_MHL,
};

#if 0
static struct switch_dev dock_switch = {
	.name = "dock",
};
#endif

struct usb_id_mpp_config_data {
	u32 usbid_mpp;
	u32 usbid_amux;
};

#ifdef CONFIG_HTC_MHL_DETECTION_8620
struct t_mhl_status_notifier{
        struct list_head mhl_notifier_link;
        const char *name;
        void (*func)(int charging_type);
};
int mhl_detect_register_notifier(struct t_mhl_status_notifier *);
static LIST_HEAD(g_lh_mhl_detect_notifier_list);
#endif

struct cable_detect_platform_data {
	int vbus_mpp_gpio;
	int vbus_mpp_irq;
	void (*vbus_mpp_config)(void);
	
	void (*usb_uart_switch)(int);
	void (*usb_dpdn_switch)(int);

	int ad_en_active_state;
	int ad_en_gpio;
	int ad_en_irq;
	
	u8 accessory_type;
	u8 mfg_usb_carkit_enable;
	int usb_id_pin_type;
	int usb_id_pin_gpio;
	__u8 detect_type;

#ifdef CONFIG_HTC_MHL_DETECTION_8620
	void (*switch_to_d3)(void);
	void (*mhl_disable_irq)(void);
	void (*mhl_wakeup)(void);
	int (*mhl_detect_register_notifier)(struct t_mhl_status_notifier *notifier);
#endif
	u8 mhl_reset_gpio;
	bool mhl_version_ctrl_flag;
	struct usb_id_mpp_config_data mpp_data;
	void (*config_usb_id_gpios)(bool enable);
	void (*mhl_1v2_power)(bool enable);
	int (*is_wireless_charger)(void);
	int64_t (*get_adc_cb)(void);

	int ac_9v_gpio;
	void (*configure_ac_9v_gpio) (int);
	u8 mhl_internal_3v3;

#ifdef CONFIG_CABLE_DETECT_GPIO_DOCK
	bool dock_detect;
	int dock_pin_gpio;
#endif
	int idpin_irq;
	int carkit_only;
	int (*detect_three_pogo_dock)(void);
	int enable_vbus_usb_switch;
	int (*is_pwr_src_plugged_in)(void);
	int vbus_debounce_retry;
};

#ifdef CONFIG_FB_MSM_MDSS_HDMI_MHL_SII9234
extern void sii9234_mhl_device_wakeup(void);
#endif
extern int cable_get_connect_type(void);
extern void set_mfg_usb_carkit_enable(int enable);
extern int cable_get_accessory_type(void);
extern int cable_get_usb_id_level(void);
extern void cable_set_uart_switch(int);
extern irqreturn_t cable_detection_vbus_irq_handler(void);
extern int check_three_pogo_dock(void);
#endif

void msm_hsusb_set_vbus_state(int online);
void msm_otg_set_vbus_state(int online);
void htc_dwc3_msm_otg_set_vbus_state(int online);
#if 0
enum usb_connect_type {
	CONNECT_TYPE_NOTIFY = -3,
	CONNECT_TYPE_CLEAR = -2,
	CONNECT_TYPE_UNKNOWN = -1,
	CONNECT_TYPE_NONE = 0,
	CONNECT_TYPE_USB,
	CONNECT_TYPE_AC,
	CONNECT_TYPE_9V_AC,
	CONNECT_TYPE_WIRELESS,
	CONNECT_TYPE_INTERNAL,
	CONNECT_TYPE_UNSUPPORTED,
#ifdef CONFIG_MACH_VERDI_LTE
	
	CONNECT_TYPE_USB_9V_AC,
#endif
	CONNECT_TYPE_MHL_AC,
};
struct t_usb_status_notifier{
	struct list_head notifier_link;
	const char *name;
	void (*func)(int cable_type);
};
int htc_usb_register_notifier(struct t_usb_status_notifier *notifer);
int64_t htc_qpnp_adc_get_usbid_adc(void);
int usb_get_connect_type(void);
static LIST_HEAD(g_lh_usb_notifier_list);

struct t_cable_status_notifier{
	struct list_head cable_notifier_link;
	const char *name;
	void (*func)(int cable_type);
};
int cable_detect_register_notifier(struct t_cable_status_notifier *);
static LIST_HEAD(g_lh_calbe_detect_notifier_list);
#endif

#ifdef CONFIG_HTC_MHL_DETECTION
struct t_mhl_status_notifier{
        struct list_head mhl_notifier_link;
        const char *name;
        void (*func)(bool isMHL, int charging_type);
};
int mhl_detect_register_notifier(struct t_mhl_status_notifier *);
static LIST_HEAD(g_lh_mhl_detect_notifier_list);
#endif

struct t_usb_host_mhl_status_notifier{
	struct list_head usb_host_mhl_notifier_link;
	const char *name;
	void (*func)(enum usb_host_mhl_type);
};
int usb_host_mhl_detect_register_notifier(struct t_usb_host_mhl_status_notifier *);
static LIST_HEAD(g_lh_usb_host_mhl_detect_notifier_list);
