/* OIS calibration interface for LC898123 F40
 *
 */

#include "../msm_sensor.h"
#include "lc898123F40_htc.h"

static struct msm_sensor_ctrl_t *g_s_ctrl = NULL;

void RamWrite32A( UINT16 RamAddr, UINT32 RamData )
{
//Add 32 bit I2C writing function
	int rc = 0;
	uint8_t data[4] = {0,0,0,0};
	struct msm_sensor_ctrl_t *s_ctrl = g_s_ctrl;

	data[0] = (RamData >> 24) & 0xFF;
	data[1] = (RamData >> 16) & 0xFF;
	data[2] = (RamData >> 8)  & 0xFF;
	data[3] = (RamData) & 0xFF;
	
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_seq(
		s_ctrl->sensor_i2c_client, RamAddr, &data[0], 4);
	if (rc < 0)
		pr_err("[OIS_Cali] %s : write failed\n", __func__);
}

void RamRead32A( UINT16 RamAddr, UINT32 * ReadData )
{
//Add 32 bit I2C writing function   
	int rc = 0;
	uint8_t buf[4] = {0,0,0,0};
	struct msm_sensor_ctrl_t *s_ctrl = g_s_ctrl;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read_seq(
		s_ctrl->sensor_i2c_client, RamAddr, &buf[0], 4);
	if (rc < 0)
		pr_err("[OIS_Cali] %s : read failed\n", __func__);
	else
		*ReadData = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
}

void WitTim( UINT16	UsWitTim )
{
	mdelay(UsWitTim);
}
 
int CntWrt( UINT8 * PcSetDat, UINT16 UsDatNum)
{
	int rc = 0;
	int temp = 0;
	struct msm_sensor_ctrl_t *s_ctrl = g_s_ctrl;
	temp = s_ctrl->sensor_i2c_client->addr_type;
	s_ctrl->sensor_i2c_client->addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_seq(s_ctrl->sensor_i2c_client, PcSetDat[0], &PcSetDat[1], UsDatNum-1);
	s_ctrl->sensor_i2c_client->addr_type = temp;
	if (rc < 0) {
		pr_err("[OIS_Cali] %s:i2c write sequence error:%d\n", __func__, rc);
		return rc;
	}
	return rc;
}

int CntRd3( UINT32 addr, void * PcSetDat, UINT16 UsDatNum )
{
	int rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl = g_s_ctrl;
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read_seq(s_ctrl->sensor_i2c_client, addr, PcSetDat, UsDatNum);

	if (rc < 0) {
		pr_err("[OIS_Cali] %s:i2c write sequence error:%d\n", __func__, rc);
		return rc;
	}
	return rc;
}

void WPBCtrl( UINT8 UcCtrl )
{
	//do nothing because lc898123F40 uses UnlockCodeSet() to handle WPB by itself
}

unsigned char htc_ext_GyroReCalib(struct msm_sensor_ctrl_t *s_ctrl, int cam_id)
{
	UINT8	UcSndDat ;
	struct file*fp = NULL;
	uint8_t *m_path= "/data/misc/camera/GYRO_main_result.txt";
	uint8_t *f_path= "/data/misc/camera/GYRO_front_result.txt";
	char gyro_mem[1024];
	int count = 0;
	stReCalib pReCalib = {0};
	g_s_ctrl = s_ctrl;
	if (g_s_ctrl == NULL)
		return -1;

	//Do gyro offset calibration
	UcSndDat = GyroReCalib(&pReCalib);
	pr_info("[OIS_Cali]%s: %d, pReCalib->SsDiffX = %d (%#x), pReCalib->SsDiffY = %d (%#x)\n", __func__, UcSndDat, pReCalib.SsDiffX, pReCalib.SsDiffX, pReCalib.SsDiffY, pReCalib.SsDiffY);

	//Write calibration result
	if (cam_id == 0)
	{
		fp=msm_fopen (m_path, O_CREAT|O_RDWR|O_TRUNC, 0666);
	} else if (cam_id == 1)
	{
		fp=msm_fopen (f_path, O_CREAT|O_RDWR|O_TRUNC, 0666);
	}else
		pr_info("Can't write result.\n");

	if (fp != NULL)
	{
		count += sprintf(gyro_mem + count,"pReCalib->SsFctryOffX = %d (%#x), pReCalib->SsFctryOffY = %d (%#x) \n", pReCalib.SsFctryOffX, pReCalib.SsFctryOffX, pReCalib.SsFctryOffY, pReCalib.SsFctryOffY);
		count += sprintf(gyro_mem + count,"pReCalib->SsRecalOffX = %d (%#x), pReCalib->SsRecalOffY = %d (%#x) \n", pReCalib.SsRecalOffX, pReCalib.SsRecalOffX, pReCalib.SsRecalOffY, pReCalib.SsRecalOffY);
		count += sprintf(gyro_mem + count,"pReCalib->SsDiffX = %d (%#x), pReCalib->SsDiffY = %d (%#x) \n", pReCalib.SsDiffX, pReCalib.SsDiffX, pReCalib.SsDiffY, pReCalib.SsDiffY);
		msm_fwrite (fp, 0, gyro_mem, strlen(gyro_mem)+1);
		msm_fclose (fp);
	}else
		pr_info("Can't write result.\n");

   /* if(pReCalib.SsDiffX >= 0x226 || pReCalib.SsDiffY >= 0x226)
	{
		pr_info("[OIS_Cali]%s:Threadhold check failed.\n", __func__);
		GYRO_Cali_release();
		return -1;
	}
	else
		return UcSndDat;*/
    return UcSndDat;
}

unsigned char htc_ext_WrGyroOffsetData( void )
{
	UINT8	ans;
    pr_info("[OIS_Cali]%s: E\n", __func__);
    ans = WrGyroOffsetData();
	return ans;
}

void htc_ext_FlashSectorRead(struct msm_sensor_ctrl_t *s_ctrl, unsigned char *data_ptr, UINT32 address, UINT32 blk_num)
{
	int i = 0;
	g_s_ctrl = s_ctrl;
	if (g_s_ctrl == NULL)
		return;

	for (i = 0; i < blk_num; i++)
	{
		FlashSectorRead_htc(address, data_ptr+64*4*i);
		address += 64;
	}
	//dump data
	//for(i = 0; i < 64*4*blk_num; i+=4)
	//	pr_info("[CAM_PDAF]%s: {%d, %d, %d, %d}\n", __func__, data_ptr[i], data_ptr[i+1], data_ptr[i+2], data_ptr[i+3]);
}

#if 0
#define		BASEVWNUM_M		0x0009
#define		BASEVWNUM_F		0x000B

int htc_checkFWUpdate(struct msm_sensor_ctrl_t *s_ctrl)
{
    int rc = 0;
    unsigned int UlFWDat;
    g_s_ctrl = s_ctrl;

    pr_info("[OIS_Cali]%s VERNUM_M = %x\n", __func__, VERNUM_M);
    pr_info("[OIS_Cali]%s VERNUM_F = %x\n", __func__, VERNUM_F);
    WitTim(100);

    RamRead32A(0x8000,&UlFWDat );
    pr_info("[OIS_Cali]%s CAM:%d FW Ver = %x\n", __func__,g_s_ctrl->sensordata->sensor_info->position, UlFWDat);

    GYRO_Cali_init(s_ctrl);

    if((g_s_ctrl->sensordata->sensor_info->position == 0)&&((UlFWDat&0xF) >= (BASEVWNUM_M&0xF))&&((VERNUM_M&0xF)>(UlFWDat&0xF))&&((UlFWDat&0xF)!=(VERNUM_M&0xF)))
    {
        pr_info("[OIS_Cali]%s:main camera FW update. %x -> %x", __func__, UlFWDat, VERNUM_M);
        rc = FlashUpdateM();
        if(rc!=0)
            pr_info("[OIS_Cali]%s:FlashUpdateM = %d  fail.", __func__, rc);
    }else if ((g_s_ctrl->sensordata->sensor_info->position == 1)&&((UlFWDat&0xF) >= (BASEVWNUM_F&0xF))&&((VERNUM_F&0xF)>(UlFWDat&0xF))&&((UlFWDat&0xF)!=(VERNUM_F&0xF))){
        pr_info("[OIS_Cali]%s:front camera FW update. %x -> %x", __func__, UlFWDat, VERNUM_F);
        rc = FlashUpdateF();
        if(rc!=0)
            pr_info("[OIS_Cali]%s:FlashUpdateF = %d  fail.", __func__, rc);
    }else
        pr_info("[OIS_Cali]%s:FlashUpdate camera ID %d no need to update.", __func__, g_s_ctrl->sensordata->sensor_info->position);

    GYRO_Cali_release();

    RamRead32A(0x8000,&UlFWDat );
    pr_info("[OIS_Cali]%s rc = %d\n", __func__, rc);

    return rc;
}
#endif


