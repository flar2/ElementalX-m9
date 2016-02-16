/* Code to extract Camera AWB calibration information from ATAG
set up by the bootloader.

Copyright (C) 2008 Google, Inc.
Author: Dmitry Shmidt <dimitrysh@google.com>

This software is licensed under the terms of the GNU General Public
License version 2, as published by the Free Software Foundation, and
may be copied, distributed, and modified under those terms.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <asm/setup.h>

#include <linux/fs.h>
#include <linux/syscalls.h>
#include <linux/vmalloc.h>

#include <linux/of.h>
#include <linux/slab.h>
#include "msm_sensor.h"



#define AWB_CAL_MAX_SIZE	0x4000U     
#define DUAL_CAL_SIZE 4095

#define CALIBRATION_DATA_PATH "/calibration_data"
#define CAM_AWB_CAL_DATA "cam_awb"

struct qct_lsc_struct{
	unsigned long int	lsc_verify;
	unsigned long int	lsc_fuseid[4];
	float 			pCalcParam[17*13*4];
	unsigned long int	lsc_checksum;
};

struct qct_awb_lsc_struct{
	unsigned long int caBuff[8];
	struct qct_lsc_struct qct_lsc_data;
	
	unsigned long int flashcaBuff[8];  
	
	unsigned long int aec_caBuff[9]; 
	unsigned long int alight_caBuff[8]; 
	unsigned long int dualflashcaBuff[12];  
	unsigned long int awb_verify;
};

static unsigned char cam_awb_ram[AWB_CAL_MAX_SIZE];

#define InHouse3D_CAL
#ifdef InHouse3D_CAL
#define GEOMETRY_SIZE 1843200
#define STEREO_SIZE 160
#define InHouse3D_Size (GEOMETRY_SIZE+STEREO_SIZE)

#define InHouse3D_Size_C 131072   

typedef struct _U16_S { uint16_t v; } __packed U16_S;
typedef struct _U32_S { uint32_t v; } __packed U32_S;
#define A32(x) (((U32_S *)(x))->v)
#define A16(x) (((U16_S *)(x))->v)

#define MINMATCH 4

#define DICTIONARY_LOGSIZE 16
#define MAXD (1<<DICTIONARY_LOGSIZE)
#define MAXD_MASK ((uint32_t)(MAXD - 1))
#define MAX_DISTANCE (MAXD - 1)

#define HASH_LOG (DICTIONARY_LOGSIZE-1)
#define HASHTABLESIZE (1 << HASH_LOG)
#define HASH_MASK (HASHTABLESIZE - 1)

#define MAX_NB_ATTEMPTS 256

#define ML_BITS  4
#define ML_MASK  (size_t)((1U<<ML_BITS)-1)
#define RUN_BITS (8-ML_BITS)
#define RUN_MASK ((1U<<RUN_BITS)-1)

#define COPYLENGTH 8
#define LASTLITERALS 5
#define MFLIMIT (COPYLENGTH+MINMATCH)
#define MINLENGTH (MFLIMIT+1)
#define OPTIMAL_ML (int)((ML_MASK-1)+MINMATCH)

#define LZ4_ARCH64 0
#define STEPSIZE 4
#define INITBASE(b,s)         const unsigned char* const b = s
#define LZ4_COPYSTEP(s,d)     A32(d) = A32(s); d+=4; s+=4;
#define LZ4_COPYPACKET(s,d)   LZ4_COPYSTEP(s,d); LZ4_COPYSTEP(s,d);
#define UARCH uint32_t
#define AARCH A32

#define LZ4_READ_LITTLEENDIAN_16(d,s,p) { d = (s) - A16(p); }
#define LZ4_WRITE_LITTLEENDIAN_16(p,v)  { A16(p) = v; p+=2; }

struct LZ4HC_Data_Structure
{
    const unsigned char* inputBuffer;
    const unsigned char* base;
    const unsigned char* end;
    uint32_t hashTable[HASHTABLESIZE];
    uint16_t chainTable[MAXD];
    const unsigned char* nextToUpdate;
};

#define LZ4_WILDCOPY(s,d,e)    do { LZ4_COPYPACKET(s,d) } while (d<e);
#define LZ4_BLINDCOPY(s,d,l)   { unsigned char* e=d+l; LZ4_WILDCOPY(s,d,e); d=e; }
#define HASH_FUNCTION(i)       (((i) * 2654435761U) >> ((MINMATCH*8)-HASH_LOG))
#define HASH_VALUE(p)          HASH_FUNCTION(A32(p))
#define HASH_POINTER(p)        (HashTable[HASH_VALUE(p)] + base)
#define DELTANEXT(p)           chainTable[(size_t)(p) & MAXD_MASK]
#define GETNEXT(p)             ((p) - (size_t)DELTANEXT(p))

#endif

int gCAM_AWB_CAL_LEN;

unsigned char *dummy(unsigned char *p)
{
    return p;
}

unsigned char *get_cam_awb_cal( void )
{
     struct device_node *offset = of_find_node_by_path(CALIBRATION_DATA_PATH);
     int p_size;
     unsigned char *p_data;
#ifdef CAM_AWB_CAL_DEBUG
     unsigned int i;
#endif

     p_size = 0;
     p_data = NULL;
     if (offset) {
          
          p_data = (unsigned char*) of_get_property(offset, CAM_AWB_CAL_DATA, &p_size);
#ifdef CAM_AWB_CAL_DEBUG
          if (p_data) {
		  printk("[CAM]size = %d ", p_size);
               for (i = 0; i < p_size; ++i)
                   printk("%02x ", p_data[i]);
          }
#endif
     }
        if (p_data != NULL) {
		gCAM_AWB_CAL_LEN = p_size;
            memcpy(cam_awb_ram, p_data, p_size);
        }

	return( cam_awb_ram );
}
EXPORT_SYMBOL(get_cam_awb_cal);


#ifdef InHouse3D_CAL

int LZ4_NbCommonBytes (register uint32_t val)
{
    static const int DeBruijnBytePos[32] = { 0, 0, 3, 0, 3, 1, 3, 0, 3, 2, 2, 1, 3, 2, 0, 1, 3, 3, 1, 2, 2, 2, 2, 0, 3, 1, 2, 0, 1, 0, 1, 1 };
    return DeBruijnBytePos[((uint32_t)((val & -(int32_t)val) * 0x077CB531U)) >> 27];
}

void LZ4_initHC (struct LZ4HC_Data_Structure* hc4, const unsigned char* base)
{
    memset((void*)hc4->hashTable, 0, sizeof(hc4->hashTable));
    memset(hc4->chainTable, 0xFF, sizeof(hc4->chainTable));
    hc4->nextToUpdate = base + 1;
    hc4->base = base;
    hc4->inputBuffer = base;
    hc4->end = base;
}

void LZ4HC_Insert (struct LZ4HC_Data_Structure* hc4, const unsigned char* ip)
{
    uint16_t* chainTable = hc4->chainTable;
    uint32_t* HashTable  = hc4->hashTable;
    const unsigned char* base = hc4->base;
    while(hc4->nextToUpdate < ip)
    {
        const unsigned char* const p = hc4->nextToUpdate;
        size_t delta = (p) - HASH_POINTER(p);
        if (delta>MAX_DISTANCE) delta = MAX_DISTANCE;
        DELTANEXT(p) = (uint16_t)delta;
        HashTable[HASH_VALUE(p)] = (uint32_t)((p) - base);
        hc4->nextToUpdate++;
    }
}

size_t LZ4HC_CommonLength (const unsigned char* p1, const unsigned char* p2, const unsigned char* const matchlimit)
{
    const unsigned char* p1t = p1;

    while (p1t<matchlimit-(STEPSIZE-1))
    {
        UARCH diff = AARCH(p2) ^ AARCH(p1t);
        if (!diff) { p1t+=STEPSIZE; p2+=STEPSIZE; continue; }
        p1t += LZ4_NbCommonBytes(diff);
        return (p1t - p1);
    }
    if (LZ4_ARCH64) if ((p1t<(matchlimit-3)) && (A32(p2) == A32(p1t))) { p1t+=4; p2+=4; }
    if ((p1t<(matchlimit-1)) && (A16(p2) == A16(p1t))) { p1t+=2; p2+=2; }
    if ((p1t<matchlimit) && (*p2 == *p1t)) p1t++;
    return (p1t - p1);
}

int LZ4HC_InsertAndFindBestMatch (struct LZ4HC_Data_Structure* hc4, const unsigned char* ip, const unsigned char* const matchlimit, const unsigned char** matchpos)
{
    uint16_t* const chainTable = hc4->chainTable;
    uint32_t* const HashTable = hc4->hashTable;
    const unsigned char* ref;
    const unsigned char* base = hc4->base;
    int nbAttempts=MAX_NB_ATTEMPTS;
    size_t repl=0, ml=0;
    uint16_t delta=0;  
    
    LZ4HC_Insert(hc4, ip);
    ref = HASH_POINTER(ip);

#define REPEAT_OPTIMIZATION
#ifdef REPEAT_OPTIMIZATION
    
    if ((uint32_t)(ip-ref) <= 4)        
    {
        if (A32(ref) == A32(ip))   
        {
            delta = (uint16_t)(ip-ref);
            repl = ml  = LZ4HC_CommonLength(ip+MINMATCH, ref+MINMATCH, matchlimit) + MINMATCH;
            *matchpos = ref;
        }
        ref = GETNEXT(ref);
    }
#endif

    while (((uint32_t)(ip-ref) <= MAX_DISTANCE) && (nbAttempts))
    {
        nbAttempts--;
        if (*(ref+ml) == *(ip+ml))
        if (A32(ref) == A32(ip))
        {
            size_t mlt = LZ4HC_CommonLength(ip+MINMATCH, ref+MINMATCH, matchlimit) + MINMATCH;
            if (mlt > ml) { ml = mlt; *matchpos = ref; }
        }
        ref = GETNEXT(ref);
    }

#ifdef REPEAT_OPTIMIZATION
    
    if (repl)
    {
        const unsigned char* ptr = ip;
        const unsigned char* end;

        end = ip + repl - (MINMATCH-1);
        while(ptr < end-delta)
        {
            DELTANEXT(ptr) = delta;    
            ptr++;
        }
        do
        {
            DELTANEXT(ptr) = delta;
            HashTable[HASH_VALUE(ptr)] = (uint32_t)((ptr) - base);     
            ptr++;
        } while(ptr < end);
        hc4->nextToUpdate = end;
    }
#endif
    return (int)ml;
}

int LZ4HC_InsertAndGetWiderMatch (struct LZ4HC_Data_Structure* hc4, const unsigned char* ip, const unsigned char* startLimit, const unsigned char* matchlimit, int longest, const unsigned char** matchpos, const unsigned char** startpos)
{
    uint16_t* const  chainTable = hc4->chainTable;
    uint32_t* const HashTable = hc4->hashTable;
    INITBASE(base,hc4->base);
    const unsigned char*  ref;
    int nbAttempts = MAX_NB_ATTEMPTS;
    int delta = (int)(ip-startLimit);

    
    LZ4HC_Insert(hc4, ip);
    ref = HASH_POINTER(ip);

    while (((uint32_t)(ip-ref) <= MAX_DISTANCE) && (nbAttempts))
    {
        nbAttempts--;
        if (*(startLimit + longest) == *(ref - delta + longest))
        if (A32(ref) == A32(ip))
        {
#if 1
            const unsigned char* reft = ref+MINMATCH;
            const unsigned char* ipt = ip+MINMATCH;
            const unsigned char* startt = ip;

            while (ipt<matchlimit-(STEPSIZE-1))
            {
                UARCH diff = AARCH(reft) ^ AARCH(ipt);
                if (!diff) { ipt+=STEPSIZE; reft+=STEPSIZE; continue; }
                ipt += LZ4_NbCommonBytes(diff);
                goto _endCount;
            }
            if (LZ4_ARCH64) if ((ipt<(matchlimit-3)) && (A32(reft) == A32(ipt))) { ipt+=4; reft+=4; }
            if ((ipt<(matchlimit-1)) && (A16(reft) == A16(ipt))) { ipt+=2; reft+=2; }
            if ((ipt<matchlimit) && (*reft == *ipt)) ipt++;
_endCount:
            reft = ref;
#else
            
            const unsigned char* startt = ip;
            const unsigned char* reft = ref;
            const unsigned char* ipt = ip + MINMATCH + LZ4HC_CommonLength(ip+MINMATCH, ref+MINMATCH, matchlimit);
#endif

            while ((startt>startLimit) && (reft > hc4->inputBuffer) && (startt[-1] == reft[-1])) {startt--; reft--;}

            if ((ipt-startt) > longest)
            {
                longest = (int)(ipt-startt);
                *matchpos = reft;
                *startpos = startt;
            }
        }
        ref = GETNEXT(ref);
    }

    return longest;
}


typedef enum { noLimit = 0, limitedOutput = 1 } limitedOutput_directive;

int LZ4HC_encodeSequence (
                       const unsigned char** ip,
                       unsigned char** op,
                       const unsigned char** anchor,
                       int matchLength,
                       const unsigned char* ref,
                       limitedOutput_directive limitedOutputBuffer,
                       unsigned char* oend)
{
    int length;
    unsigned char* token;

    
    length = (int)(*ip - *anchor);
    token = (*op)++;
    if ((limitedOutputBuffer) && ((*op + length + (2 + 1 + LASTLITERALS) + (length>>8)) > oend)) return 1;   
    if (length>=(int)RUN_MASK) { int len; *token=(RUN_MASK<<ML_BITS); len = length-RUN_MASK; for(; len > 254 ; len-=255) *(*op)++ = 255;  *(*op)++ = (unsigned char)len; }
    else *token = (unsigned char)(length<<ML_BITS);

    
    LZ4_BLINDCOPY(*anchor, *op, length);

    
    LZ4_WRITE_LITTLEENDIAN_16(*op,(uint16_t)(*ip-ref));

    
    length = (int)(matchLength-MINMATCH);
    if ((limitedOutputBuffer) && (*op + (1 + LASTLITERALS) + (length>>8) > oend)) return 1;   
    if (length>=(int)ML_MASK) { *token+=ML_MASK; length-=ML_MASK; for(; length > 509 ; length-=510) { *(*op)++ = 255; *(*op)++ = 255; } if (length > 254) { length-=255; *(*op)++ = 255; } *(*op)++ = (unsigned char)length; }
    else *token += (unsigned char)(length);

    
    *ip += matchLength;
    *anchor = *ip;

    return 0;
}

static int LZ4HC_compress_generic (
                 void* ctxvoid,
                 const char* source,
                 char* dest,
                 int inputSize,
                 int maxOutputSize,
                 limitedOutput_directive limit
                )
{
    struct LZ4HC_Data_Structure* ctx = (struct LZ4HC_Data_Structure*) ctxvoid;
    const unsigned char* ip = (const unsigned char*) source;
    const unsigned char* anchor = ip;
    const unsigned char* const iend = ip + inputSize;
    const unsigned char* const mflimit = iend - MFLIMIT;
    const unsigned char* const matchlimit = (iend - LASTLITERALS);

    unsigned char* op = (unsigned char*) dest;
    unsigned char* const oend = op + maxOutputSize;

    int   ml, ml2, ml3, ml0;
    const unsigned char* ref=NULL;
    const unsigned char* start2=NULL;
    const unsigned char* ref2=NULL;
    const unsigned char* start3=NULL;
    const unsigned char* ref3=NULL;
    const unsigned char* start0;
    const unsigned char* ref0;


    
    if (ip != ctx->end) return 0;
    ctx->end += inputSize;

    ip++;

    
    while (ip < mflimit)
    {
        ml = LZ4HC_InsertAndFindBestMatch (ctx, ip, matchlimit, (&ref));
        if (!ml) { ip++; continue; }

        
        start0 = ip;
        ref0 = ref;
        ml0 = ml;

_Search2:
        if (ip+ml < mflimit)
            ml2 = LZ4HC_InsertAndGetWiderMatch(ctx, ip + ml - 2, ip + 1, matchlimit, ml, &ref2, &start2);
        else ml2 = ml;

        if (ml2 == ml)  
        {
            if (LZ4HC_encodeSequence(&ip, &op, &anchor, ml, ref, limit, oend)) return 0;
            continue;
        }

        if (start0 < ip)
        {
            if (start2 < ip + ml0)   
            {
                ip = start0;
                ref = ref0;
                ml = ml0;
            }
        }

        
        if ((start2 - ip) < 3)   
        {
            ml = ml2;
            ip = start2;
            ref =ref2;
            goto _Search2;
        }

_Search3:
        
        
        
        if ((start2 - ip) < OPTIMAL_ML)
        {
            int correction;
            int new_ml = ml;
            if (new_ml > OPTIMAL_ML) new_ml = OPTIMAL_ML;
            if (ip+new_ml > start2 + ml2 - MINMATCH) new_ml = (int)(start2 - ip) + ml2 - MINMATCH;
            correction = new_ml - (int)(start2 - ip);
            if (correction > 0)
            {
                start2 += correction;
                ref2 += correction;
                ml2 -= correction;
            }
        }
        
        if (start2 + ml2 < mflimit)
            ml3 = LZ4HC_InsertAndGetWiderMatch(ctx, start2 + ml2 - 3, start2, matchlimit, ml2, &ref3, &start3);
        else ml3 = ml2;

        if (ml3 == ml2) 
        {
            
            if (start2 < ip+ml)  ml = (int)(start2 - ip);
            
            if (LZ4HC_encodeSequence(&ip, &op, &anchor, ml, ref, limit, oend)) return 0;
            ip = start2;
            if (LZ4HC_encodeSequence(&ip, &op, &anchor, ml2, ref2, limit, oend)) return 0;
            continue;
        }

        if (start3 < ip+ml+3) 
        {
            if (start3 >= (ip+ml)) 
            {
                if (start2 < ip+ml)
                {
                    int correction = (int)(ip+ml - start2);
                    start2 += correction;
                    ref2 += correction;
                    ml2 -= correction;
                    if (ml2 < MINMATCH)
                    {
                        start2 = start3;
                        ref2 = ref3;
                        ml2 = ml3;
                    }
                }

                if (LZ4HC_encodeSequence(&ip, &op, &anchor, ml, ref, limit, oend)) return 0;
                ip  = start3;
                ref = ref3;
                ml  = ml3;

                start0 = start2;
                ref0 = ref2;
                ml0 = ml2;
                goto _Search2;
            }

            start2 = start3;
            ref2 = ref3;
            ml2 = ml3;
            goto _Search3;
        }

        
        
        if (start2 < ip+ml)
        {
            if ((start2 - ip) < (int)ML_MASK)
            {
                int correction;
                if (ml > OPTIMAL_ML) ml = OPTIMAL_ML;
                if (ip + ml > start2 + ml2 - MINMATCH) ml = (int)(start2 - ip) + ml2 - MINMATCH;
                correction = ml - (int)(start2 - ip);
                if (correction > 0)
                {
                    start2 += correction;
                    ref2 += correction;
                    ml2 -= correction;
                }
            }
            else
            {
                ml = (int)(start2 - ip);
            }
        }
        if (LZ4HC_encodeSequence(&ip, &op, &anchor, ml, ref, limit, oend)) return 0;

        ip = start2;
        ref = ref2;
        ml = ml2;

        start2 = start3;
        ref2 = ref3;
        ml2 = ml3;

        goto _Search3;

    }

    
    {
        int lastRun = (int)(iend - anchor);
        if ((limit) && (((char*)op - dest) + lastRun + 1 + ((lastRun+255-RUN_MASK)/255) > (uint32_t)maxOutputSize)) return 0;  
        if (lastRun>=(int)RUN_MASK) { *op++=(RUN_MASK<<ML_BITS); lastRun-=RUN_MASK; for(; lastRun > 254 ; lastRun-=255) *op++ = 255; *op++ = (unsigned char) lastRun; }
        else *op++ = (unsigned char)(lastRun<<ML_BITS);
        memcpy(op, anchor, iend - anchor);
        op += iend-anchor;
    }

    
    return (int) (((char*)op)-dest);
}

int LZ4_compressHC(const char* source, char* dest, int inputSize)
{
	void* ctx;
	int result = 0;

	ctx = vmalloc(sizeof(struct LZ4HC_Data_Structure));
	LZ4_initHC((struct LZ4HC_Data_Structure *)ctx, source);
	if (ctx == NULL) return 0;

	result = LZ4HC_compress_generic (ctx, source, dest, inputSize, 0, noLimit);

	vfree(ctx);
	return result;
}
#endif

static ssize_t awb_calibration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned char *ptr;

	ptr = get_cam_awb_cal();
	

	ret = sizeof(struct qct_awb_lsc_struct);
	printk(KERN_INFO "[CAM]awb_calibration_show(%d)\n", ret);
	memcpy(buf, ptr, ret);

#ifdef CAM_AWB_CAL_DEBUG
   {
	 int i, *pint;
	 printk(KERN_INFO "[CAM]awb_calibration_show():\n");
	 pint = (int *)buf;
	 for (i = 0; i < 914; i++)
	   printk(KERN_INFO "%d-%x\n", i, pint[i]);

   }
#endif

	return ret;
}

static ssize_t awb_calibration_front_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned char *ptr;

	ptr = get_cam_awb_cal();
	

	ret = sizeof(struct qct_awb_lsc_struct);
	printk(KERN_INFO "[CAM]awb_calibration_front_show(%d)\n", ret);
	memcpy(buf, ptr + 0x1000U, ret);


#ifdef CAM_AWB_CAL_DEBUG
   {
	 int i, *pint;
	 printk(KERN_INFO "[CAM]awb_calibration_front_show():\n");
	 pint = (int *)buf;
	 for (i = 0; i < 898; i++)
	   printk(KERN_INFO "%x\n", pint[i]);

   }
#endif

	return ret;
}

static ssize_t awb_calibration_sub_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	unsigned char *ptr;

	ptr = get_cam_awb_cal();
	

	ret = sizeof(struct qct_awb_lsc_struct);
	printk(KERN_INFO "[CAM]awb_calibration_sub_show(%d)\n", ret);
	memcpy(buf, ptr + 0x2000U, ret);


#ifdef CAM_AWB_CAL_DEBUG
   {
	 int i, *pint;
	 printk(KERN_INFO "[CAM]awb_calibration_sub_show():\n");
	 pint = (int *)buf;
	 for (i = 0; i < 898; i++)
	   printk(KERN_INFO "%x\n", pint[i]);

   }
#endif

	return ret;
}


static ssize_t awb_calibration_3D_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = DUAL_CAL_SIZE;
	unsigned char *ptr;

	ptr = get_cam_awb_cal();
	

	printk(KERN_INFO "[CAM]awb_calibration_3D_show(%d)\n", ret);
	memcpy(buf, ptr + 0x3000U, ret);


#ifdef CAM_AWB_CAL_DEBUG
   {
	 int i, *pint;
	 printk(KERN_INFO "[CAM]awb_calibration_3D_show():\n");
	 pint = (int *)buf;
	 for (i = 0; i < DUAL_CAL_SIZE; i++)
	   printk(KERN_INFO "%x\n", pint[i]);

   }
#endif

	return ret;
}

#ifdef InHouse3D_CAL
static ssize_t awb_calibration_InHouse3D_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	printk(KERN_INFO "[CAM]awb_calibration_InHouse3D_show\n");
	get_cam_emmc_cal(&ret);
	if(ret)
		memcpy(buf, &ret, sizeof(uint32_t));

	return DUAL_CAL_SIZE;
}


unsigned char *get_cam_emmc_cal(int *length)
{
	int ret = 0;
	uint8_t m_bCompreess = 1;
	static int first = 1;
	static int valid = 0;
	struct file *fp, *f;
	unsigned char *cam_buf_emmc = NULL;
	unsigned char *cam_emmc_ram = NULL;
	static int LZW_output = 0;

	if(first || !valid)
	{
		fp = filp_open("/dev/block/bootdevice/by-name/camera", O_RDONLY, 0);
		if(!IS_ERR(fp))
		{
			printk(KERN_INFO "[CAM]open camera partition ok\n");
			cam_buf_emmc = vmalloc(InHouse3D_Size);
			cam_emmc_ram = vmalloc(InHouse3D_Size_C);
			if(cam_buf_emmc == NULL || cam_emmc_ram == NULL)
			{
				if(cam_buf_emmc)
					vfree(cam_buf_emmc);
				if(cam_emmc_ram)
					vfree(cam_emmc_ram);
				printk(KERN_INFO "[CAM]get_cam_emmc_cal, allocate buffer failed!!!\n");
			}
			else
			{
				ret = kernel_read(fp, 0, cam_buf_emmc, InHouse3D_Size);
				if(ret == InHouse3D_Size)
				{
						LZW_output = LZ4_compressHC((const char *)cam_buf_emmc,  (char *)cam_emmc_ram+5, GEOMETRY_SIZE);
						memcpy(cam_emmc_ram, &m_bCompreess, sizeof(uint8_t));   
						memcpy(cam_emmc_ram+1, &LZW_output, sizeof(uint32_t));   
						memcpy(cam_emmc_ram+5+LZW_output, cam_buf_emmc+GEOMETRY_SIZE, STEREO_SIZE);   
						*length = 5+LZW_output+STEREO_SIZE;
						valid = 1;

						f = msm_fopen ("/data/InHouseData.dat", O_CREAT|O_RDWR|O_TRUNC, 0666);
						if (f) {
							msm_fwrite (f, 0, cam_emmc_ram, *length);
							msm_fclose (f);
						} else {
							printk(KERN_INFO "[CAM]fail to open file to write\n");
						}
						printk(KERN_INFO "[CAM]load calibration data successfully\n");
				}
				else
					printk(KERN_INFO "[CAM]mismatched read size: %d\n", ret);
				if(cam_buf_emmc)
					vfree(cam_buf_emmc);
				if(cam_emmc_ram)
					vfree(cam_emmc_ram);
			}
			filp_close(fp, NULL);
		}
		else
		{
			printk(KERN_INFO "[CAM]open camera partition failed\n");
		}
		first = 0;
	}

	if(valid)
	{
		*length = 5+LZW_output+STEREO_SIZE;
		return cam_emmc_ram;
	}
	else
	{
		*length = 0;
		return NULL;
	}
}
EXPORT_SYMBOL(get_cam_emmc_cal);
#endif


static DEVICE_ATTR(awb_cal, 0444, awb_calibration_show, NULL);
static DEVICE_ATTR(awb_cal_front, 0444, awb_calibration_front_show, NULL);
static DEVICE_ATTR(awb_cal_sub, 0444, awb_calibration_sub_show, NULL);
static DEVICE_ATTR(awb_cal_3D, 0444, awb_calibration_3D_show, NULL);
#ifdef InHouse3D_CAL
static DEVICE_ATTR(awb_cal_InHouse3D, 0444, awb_calibration_InHouse3D_show, NULL);
#endif


static struct kobject *cam_awb_cal;

static int cam_get_awb_cal(void)
{
	int ret ;

	
	cam_awb_cal = kobject_create_and_add("android_camera_awb_cal", NULL);
	if (cam_awb_cal == NULL) {
		pr_info("[CAM]cam_get_awb_cal: subsystem_register failed\n");
		ret = -ENOMEM;
		return ret ;
	}

	ret = sysfs_create_file(cam_awb_cal, &dev_attr_awb_cal.attr);
	if (ret) {
		pr_info("[CAM]cam_get_awb_cal:: sysfs_create_file failed\n");
		kobject_del(cam_awb_cal);
		goto end;
	}


	ret = sysfs_create_file(cam_awb_cal, &dev_attr_awb_cal_front.attr);
	if (ret) {
		pr_info("[CAM]cam_get_awb_cal_front:: sysfs_create_file failed\n");
		kobject_del(cam_awb_cal);
		goto end;
	}

	ret = sysfs_create_file(cam_awb_cal, &dev_attr_awb_cal_sub.attr);
	if (ret) {
		pr_info("[CAM]cam_get_awb_cal_sub:: sysfs_create_file failed\n");
		kobject_del(cam_awb_cal);
		goto end;
	}

	ret = sysfs_create_file(cam_awb_cal, &dev_attr_awb_cal_3D.attr);
	if (ret) {
		pr_info("[CAM]cam_get_awb_cal_3D:: sysfs_create_file failed\n");
		kobject_del(cam_awb_cal);
		goto end;
	}

	
	#ifdef InHouse3D_CAL
	ret = sysfs_create_file(cam_awb_cal, &dev_attr_awb_cal_InHouse3D.attr);
	if (ret) {
		pr_info("[CAM]cam_get_awb_cal_InHouse3D:: sysfs_create_file failed\n");
		kobject_del(cam_awb_cal);
		goto end;
	}
	#endif
	

end:
	return 0 ;
}

late_initcall(cam_get_awb_cal);
