#ifndef _SI_MHL_CALLBACK_API_H_
#define _SI_MHL_CALLBACK_API_H_

union __attribute__((__packed__))
avif_or_cea_861_dtd_u {
	struct detailed_timing_descriptor_t cea_861_dtd;
	struct avi_info_frame_t avif;
};

union __attribute__((__packed__))
vsif_mhl3_or_hdmi_u {
	struct vendor_specific_info_frame_t hdmi_vsif;
	struct mhl3_vsif_t mhl3_vsif;
};

enum hpd_high_callback_status {
	

	/* a DTD has been written to the p_avif_or_dtd buffer instead of an AVI
	 * infoframe
	 */
	HH_FMT_DVI = 0x00000000,
	
	HH_FMT_HDMI_VSIF_NONE = 0x00000001,
	
	HH_FMT_HDMI_VSIF_HDMI = 0x00000002,
	
	HH_FMT_HDMI_VSIF_MHL3 = 0x00000003,
	/* a DTD has been written to the DTD buffer instead of an AVI infoframe
	 * and 8620 shall expect HDCP enabled on its HDMI input
	 */
	HH_FMT_DVI_HDCP_ON = 0x00000004,
	HH_FMT_HDMI_VSIF_NONE_HDCP_ON = 0x00000005,
	HH_FMT_HDMI_VSIF_HDMI_HDCP_ON = 0x00000006,
	HH_FMT_HDMI_VSIF_MHL3_HDCP_ON = 0x00000007,
	/* a DTD has been written to the DTD buffer instead of an AVI
	 * infoframe
	 */
	HH_FMT_DVI_NOT_RPT = 0x00000008,
	
	HH_FMT_HDMI_VSIF_NONE_NOT_RPT = 0x00000009,
	
	HH_FMT_HDMI_VSIF_HDMI_NOT_RPT = 0x0000000A,
	
	HH_FMT_HDMI_VSIF_MHL3_NOT_RPT = 0x0000000B,
	/* a DTD has been written to the DTD buffer instead of an AVI infoframe
	 * and 8620 shall expect HDCP enabled on its HDMI input
	 */
	HH_FMT_DVI_HDCP_ON_NOT_RPT = 0x0000000C,
	HH_FMT_HDMI_VSIF_NONE_HDCP_ON_NOT_RPT = 0x0000000D,
	HH_FMT_HDMI_VSIF_HDMI_HDCP_ON_NOT_RPT = 0x0000000E,
	HH_FMT_HDMI_VSIF_MHL3_HDCP_ON_NOT_RPT = 0x0000000F,

	

	
	HH_AVI_BUFFER_TOO_SMALL = 0x80000001,
	
	HH_VSIF_BUFFER_TOO_SMALL = 0x80000002,
	
	HH_VIDEO_NOT_RDY = 0x80000004
};

struct __attribute__((__packed__)) si_mhl_callback_api_t {
	void *context;
#if 1
	int (*display_timing_enum_begin) (void *context);
	int (*display_timing_enum_item) (void *context, uint16_t columns,
		uint16_t rows, uint8_t bits_per_pixel,
		uint32_t vertical_refresh_rate_in_milliHz, uint16_t burst_id,
		union video_burst_descriptor_u *p_descriptor);
	int (*display_timing_enum_end) (void *context);
#endif
	void (*hpd_driven_low) (void *context);

	enum hpd_high_callback_status(*hpd_driven_high) (void *context,
		uint8_t *p_edid, size_t edid_length,
		struct MHL3_hev_dtd_item_t *p_hev_dtd, size_t num_hev_dtds,
		struct MHL3_hev_vic_item_t *p_hev_vic, size_t num_hev_vic_items,
		struct MHL3_3d_dtd_item_t *p_3d_dtd_items,
		size_t num_3d_dtd_items,
		struct MHL3_3d_vic_item_t *p_3d_vic, size_t num_3d_vic_items,
		union avif_or_cea_861_dtd_u *p_avif_or_dtd,
		size_t avif_or_dtd_max_length,
		union vsif_mhl3_or_hdmi_u *p_vsif,
		size_t vsif_max_length);
};

int si_8620_register_callbacks(struct si_mhl_callback_api_t *p_callbacks);

int si_8620_info_frame_change(enum hpd_high_callback_status status,
	union avif_or_cea_861_dtd_u *p_avif_or_dtd,
	size_t avif_or_dtd_max_length, union vsif_mhl3_or_hdmi_u *p_vsif,
	size_t vsif_max_length);

int si_8620_get_hpd_status(int *hpd_status);
int si_8620_get_hdcp2_status(uint32_t *hdcp2_status);
#endif
