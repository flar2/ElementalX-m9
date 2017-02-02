/*===========================================================================

  Copyright (c) 2013 HTC.
  All Rights Reserved.
  HTC Proprietary and Confidential.

  ===========================================================================*/

#ifndef RPM_HTC_CMD_H
#define RPM_HTC_CMD_H

#define RAILWAY_NO_REQUEST		(0)
#define RAILWAY_SVS_SOC			(3)
#define RAILWAY_SVS_NOMINAL		(5)
#define RAILWAY_SVS_SUPER_TURBO		(8)

/* Public HTC RPM CMD */
int htc_rpm_cmd_vote_vdd_dig(uint32_t corner);

#endif
