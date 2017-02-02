/* OIS calibration interface for LC898123 F40
 *
 */
#include "Ois.h"

void RamWrite32A(UINT16 RamAddr, UINT32 RamData);
void RamRead32A(UINT16 RamAddr, UINT32 * ReadData);
int CntWrt(UINT8 * PcSetDat, UINT16 CntWrt);
int CntRd3(UINT32 addr, void *	PcSetDat, UINT16	UsDatNum);
void WitTim(UINT16) ;
void WPBCtrl(UINT8 UcCtrl);
unsigned char htc_ext_GyroReCalib(struct msm_sensor_ctrl_t *s_ctrl, int cam_id);
unsigned char htc_ext_WrGyroOffsetData( void );
void htc_ext_FlashSectorRead(struct msm_sensor_ctrl_t *s_ctrl, unsigned char *data_ptr, UINT32 address, UINT32 blk_num);
