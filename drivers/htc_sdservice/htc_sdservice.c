/* drivers/htc_sdservice/htc_sdservice.c
 *
 * Copyright (C) 2014 HTC Corporation.
 * Author: HTC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/types.h>

#include <crypto/hash.h>
#include <linux/err.h>
#include <linux/scatterlist.h>
#include <linux/string.h>

#include <soc/qcom/scm.h>
#include <soc/qcom/smd.h>
#include "qseecom_kernel.h"

#define DEVICE_NAME "htc_sdservice"
/*
static char *appname = "htc_sdservice";
*/

#define HTC_SDKEY_LEN 32
#define HTC_IOCTL_SDSERVICE 0x9527
#define HTC_IOCTL_SEC_ATS_GET	0x9528
#define HTC_IOCTL_SEC_ATS_SET	0x9529

#define ITEM_SD_KEY_ENCRYPT     0x33
#define ITEM_SD_KEY_DECRYPT     0x34

#define TAG "[SEC] "
#define HTC_SDSERVICE_DEBUG	0
#undef PDEBUG
#if HTC_SDSERVICE_DEBUG
#define PDEBUG(fmt, args...) printk(KERN_INFO TAG "[K] %s(%i, %s): " fmt "\n", \
		__func__, current->pid, current->comm, ## args)
#else
#define PDEBUG(fmt, args...) do {} while (0)
#endif /* HTC_SDSERVICE_DEBUG */

#undef PERR
#define PERR(fmt, args...) printk(KERN_ERR TAG "[E] %s(%i, %s): " fmt "\n", \
		__func__, current->pid, current->comm, ## args)

#undef PINFO
#define PINFO(fmt, args...) printk(KERN_INFO TAG "[I] %s(%i, %s): " fmt "\n", \
		__func__, current->pid, current->comm, ## args)

static int htc_sdservice_major;
static struct class *htc_sdservice_class;
static const struct file_operations htc_sdservice_fops;

static unsigned char *htc_sdkey;

typedef struct _htc_sdservice_msg_s{
	int func;
	int offset;
	unsigned char *req_buf;
	int req_len;
	unsigned char *resp_buf;
	int resp_len;
} htc_sdservice_msg_s;

/* ATS structure, total size 6 * uint32 = 24 bytes */
typedef struct {
	struct {
		uint8_t func_id;
		uint8_t func_cur_state;
		int8_t  func_return;
		uint8_t func_exec;
	} func_info;
	uint32_t    input[2];
	uint32_t    output[2];
	uint8_t reserve[4];
} htc_sec_ats_t;

struct qsc_send_cmd {
    uint32_t cmd_id;
    uint32_t data;
    uint32_t data2;
    uint32_t len;
    uint32_t start_pkt;
    uint32_t end_pkt;
    uint32_t test_buf_size;
};

static int do_cipher(int32_t is_enc, uint8_t *in_buf, uint8_t *key)
{
	struct scatterlist sg_in[1], sg_out[1];
	struct crypto_blkcipher *tfm = crypto_alloc_blkcipher("cbc(aes)", 0, CRYPTO_ALG_ASYNC);
	struct blkcipher_desc desc = { .tfm = tfm, .flags = 0 };
    int err = 0;
    unsigned char tmp[32] = {0};
	void *iv;
	int ivsize;

    if (IS_ERR(tfm)) {
        PERR("alg: cipher: Failed to load transform for "
                "%s: %ld\n", "cbc(aes)", PTR_ERR(tfm));
        return PTR_ERR(tfm);
    }

    err = crypto_blkcipher_setkey(tfm, key, 32);
    if (err) {
        PERR("alg: cipher: Failed to setkey for "
                "%s: %ld\n", "cbc(aes)", PTR_ERR(tfm));
        goto out;
    }
	sg_init_table(sg_in, 1);
	sg_set_buf(&sg_in[0], in_buf, 32);
	sg_init_table(sg_out, 1);
	sg_set_buf(sg_out, tmp, sizeof(tmp));

	iv = crypto_blkcipher_crt(tfm)->iv;
	ivsize = crypto_blkcipher_ivsize(tfm);
	memcpy(iv, key, ivsize);

    if (is_enc) {
        err = crypto_blkcipher_encrypt(&desc, sg_out, sg_in, 32);
    }
    else {
        err = crypto_blkcipher_decrypt(&desc, sg_out, sg_in, 32);
    }
    /*
	print_hex_dump(KERN_DEBUG, "enc: ", DUMP_PREFIX_NONE, 16, 1, tmp, 32, 0);
    */
    memcpy(in_buf, tmp, 32);
out:
    crypto_free_blkcipher(tfm);

    return err;
}

static int do_hash(uint8_t *input_buf, uint32_t type, uint32_t mask)
{
	struct crypto_hash *tfm;
	int err;
    struct hash_desc desc;
    struct scatterlist sg[1];
    unsigned char tmp[32];

    /*
	print_hex_dump(KERN_DEBUG, "input_buf: ", DUMP_PREFIX_NONE, 16, 1, input_buf, 32, 0);
     */
	tfm = crypto_alloc_hash("sha256", type, mask);
	if (IS_ERR(tfm)) {
		PERR("alg: hash: Failed to load transform for %s: "
		       "%ld\n", "sha256", PTR_ERR(tfm));
		return PTR_ERR(tfm);
	}
    desc.tfm = tfm;
    desc.flags = 0;

    sg_init_table(sg, 1);
    sg_set_buf(&sg[0], input_buf, 32);

    err = crypto_hash_digest(&desc, sg, 32, tmp);
    /*
	print_hex_dump(KERN_DEBUG, "hash: ", DUMP_PREFIX_NONE, 16, 1, tmp, 32, 0);
     */
    memcpy(input_buf, tmp, 32);
	crypto_free_hash(tfm);

	return err;
}

static long htc_sdservice_ioctl(struct file *file, unsigned int command, unsigned long arg)
{
	htc_sdservice_msg_s hmsg;
	htc_sec_ats_t amsg;
	int32_t ret = 0;
    uint8_t tmp_key[32] = {0};
    int32_t ii;
    /*
    struct qseecom_handle *l_QSEEComHandle = NULL;
    struct qsc_send_cmd *send_cmd;
    void *resp;
    */

	PDEBUG("command = %x", command);
	switch (command) {
	case HTC_IOCTL_SDSERVICE:
		if (copy_from_user(&hmsg, (void __user *)arg, sizeof(hmsg))) {
			PERR("copy_from_user error (msg)");
			return -EFAULT;
		}
		PDEBUG("func = %x", hmsg.func);
		switch (hmsg.func) {
		case ITEM_SD_KEY_ENCRYPT:
			if ((hmsg.req_buf == NULL) || (hmsg.req_len != HTC_SDKEY_LEN)) {
				PERR("invalid arguments");
				return -EFAULT;
			}
			if (copy_from_user(htc_sdkey, (void __user *)hmsg.req_buf, hmsg.req_len)) {
				PERR("copy_from_user error (sdkey)");
				return -EFAULT;
			}

            /*
            ret = qseecom_start_app(&l_QSEEComHandle, appname, 1024);
            if (ret) {
                PERR("Start app: fail");
                return -1;
            } else {
                PDEBUG("Start app: pass");
            }
            if(l_QSEEComHandle == NULL) {
                PERR("Failed to get QSEECOM handle\n");
                return -1;
            }

            send_cmd = (struct qsc_send_cmd *)l_QSEEComHandle->sbuf;
            resp = (struct qsc_send_cmd *)((uintptr_t)(l_QSEEComHandle->sbuf) + l_QSEEComHandle->sbuf_len/2);
            memset(resp, 0, HTC_SDKEY_LEN);

            send_cmd->cmd_id = ITEM_SD_KEY_ENCRYPT;
            send_cmd->test_buf_size = HTC_SDKEY_LEN;
            memcpy((uint8_t *)send_cmd + sizeof(struct qsc_send_cmd), htc_sdkey, HTC_SDKEY_LEN);

            ret = qseecom_set_bandwidth(l_QSEEComHandle, true);
            if (ret) {
                PERR("qseecom_set_bandwidth fail(%d)", ret);
                return -1;
            }
            ret = qseecom_send_command(l_QSEEComHandle, send_cmd, HTC_SDKEY_LEN, resp, HTC_SDKEY_LEN);
            if (ret) {
                PERR("qseecom_send_cmd fail(%d)", ret);
                return -1;
            }
            ret = qseecom_set_bandwidth(l_QSEEComHandle, false);
            if (ret) {
                PERR("qseecom_set_bandwidth fail(%d)", ret);
                return -1;
            }
            memcpy(htc_sdkey, resp, HTC_SDKEY_LEN);
            ret = qseecom_shutdown_app(&l_QSEEComHandle);

			scm_flush_range((uint32_t)htc_sdkey, (uint32_t)htc_sdkey + HTC_SDKEY_LEN);
			ret = secure_access_item(0, ITEM_SD_KEY_ENCRYPT, hmsg.req_len, htc_sdkey);
			if (ret)
				PERR("Encrypt SD key fail (%d)", ret);
             */
            secure_get_msm_serial((uint32_t *)tmp_key);
            for (ii = 0; ii < sizeof(tmp_key) - 4; ii++)
                tmp_key[ii + 4] = tmp_key[ii];
            do_hash(tmp_key, 0, 0);
            do_cipher(1, htc_sdkey, tmp_key);

			if (copy_to_user((void __user *)hmsg.resp_buf, htc_sdkey, hmsg.req_len)) {
				PERR("copy_to_user error (sdkey)");
				return -EFAULT;
			}
			break;

		case ITEM_SD_KEY_DECRYPT:
			if ((hmsg.req_buf == NULL) || (hmsg.req_len != HTC_SDKEY_LEN)) {
				PERR("invalid arguments");
				return -EFAULT;
			}
			if (copy_from_user(htc_sdkey, (void __user *)hmsg.req_buf, hmsg.req_len)) {
				PERR("copy_from_user error (sdkey)");
				return -EFAULT;
			}
            /*
            ret = qseecom_start_app(&l_QSEEComHandle, appname, 1024);
            if (ret) {
                PERR("Start app: fail");
                return -1;
            } else {
                PDEBUG("Start app: pass");
            }
            if(l_QSEEComHandle == NULL) {
                PERR("Failed to get QSEECOM handle\n");
                return -1;
            }

            send_cmd = (struct qsc_send_cmd *)l_QSEEComHandle->sbuf;
            resp = (struct qsc_send_cmd *)((uintptr_t)(l_QSEEComHandle->sbuf) + l_QSEEComHandle->sbuf_len/2);
            memset(resp, 0, HTC_SDKEY_LEN);

            send_cmd->cmd_id = ITEM_SD_KEY_DECRYPT;
            send_cmd->test_buf_size = HTC_SDKEY_LEN;
            memcpy((uint8_t *)send_cmd + sizeof(struct qsc_send_cmd), htc_sdkey, HTC_SDKEY_LEN);

            ret = qseecom_set_bandwidth(l_QSEEComHandle, true);
            if (ret) {
                PERR("qseecom_set_bandwidth fail(%d)", ret);
                return -1;
            }
            ret = qseecom_send_command(l_QSEEComHandle, send_cmd, HTC_SDKEY_LEN, resp, HTC_SDKEY_LEN);
            if (ret) {
                PERR("qseecom_send_cmd fail(%d)", ret);
                return -1;
            }
            ret = qseecom_set_bandwidth(l_QSEEComHandle, false);
            if (ret) {
                PERR("qseecom_set_bandwidth fail(%d)", ret);
                return -1;
            }
            memcpy(htc_sdkey, resp, HTC_SDKEY_LEN);
            ret = qseecom_shutdown_app(&l_QSEEComHandle);

			scm_flush_range((uint32_t)htc_sdkey, (uint32_t)htc_sdkey + HTC_SDKEY_LEN);
			ret = secure_access_item(0, ITEM_SD_KEY_DECRYPT, hmsg.req_len, htc_sdkey);
			if (ret)
				PERR("Encrypt SD key fail (%d)", ret);
             */
            secure_get_msm_serial((uint32_t *)tmp_key);
            for (ii = 0; ii < sizeof(tmp_key) - 4; ii++)
                tmp_key[ii + 4] = tmp_key[ii];
            do_hash(tmp_key, 0, 0);
            do_cipher(0, htc_sdkey, tmp_key);

			if (copy_to_user((void __user *)hmsg.resp_buf, htc_sdkey, hmsg.req_len)) {
				PERR("copy_to_user error (sdkey)");
				return -EFAULT;
			}
			break;

		default:
			PERR("func error");
			return -EFAULT;
		}
		break;

	case HTC_IOCTL_SEC_ATS_GET:
		if (!arg) {
			PERR("invalid arguments");
			return -ENOMEM;
		}
        /*
		scm_flush_range((uint32_t)&amsg, (uint32_t)&amsg + sizeof(htc_sec_ats_t));
		ret = secure_access_item(0, ITEM_SEC_ATS, sizeof(htc_sec_ats_t), (unsigned char *)&amsg);
		if (ret) {
			PERR("ATS service fail (%d)", ret);
			return ret;
		}
        */

		if (copy_to_user((void __user *)arg, &amsg, sizeof(htc_sec_ats_t))) {
			PERR("copy_to_user error (msg)");
			return -EFAULT;
		}
		break;

	case HTC_IOCTL_SEC_ATS_SET:
		if (!arg) {
			PERR("invalid arguments");
			return -ENOMEM;
		}
		if (copy_from_user(&amsg, (void __user *)arg, sizeof(htc_sec_ats_t))) {
			PERR("copy_from_user error (msg)");
			return -EFAULT;
		}
		PDEBUG("func = %x, sizeof htc_sec_ats_t = %zd", amsg.func_info.func_id, sizeof(htc_sec_ats_t));
        /*
		ret = secure_access_item(1, ITEM_SEC_ATS, sizeof(htc_sec_ats_t), (unsigned char *)&amsg);
		if (ret)
			PERR("ATS service fail (%d)", ret);
         */
		break;

	default:
		PERR("command error");
		return -EFAULT;
	}
	return ret;
}


static int htc_sdservice_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int htc_sdservice_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static const struct file_operations htc_sdservice_fops = {
	.unlocked_ioctl = htc_sdservice_ioctl,
	.open = htc_sdservice_open,
	.release = htc_sdservice_release,
	.owner = THIS_MODULE,
};

static int __init htc_sdservice_init(void)
{
	int ret;

	htc_sdkey = kzalloc(HTC_SDKEY_LEN, GFP_KERNEL);
	if (htc_sdkey == NULL) {
		PERR("allocate the space for SD key failed");
		return -1;
	}

	ret = register_chrdev(0, DEVICE_NAME, &htc_sdservice_fops);
	if (ret < 0) {
		PERR("register module fail");
		return ret;
	}
	htc_sdservice_major = ret;

	htc_sdservice_class = class_create(THIS_MODULE, "htc_sdservice");
	device_create(htc_sdservice_class, NULL, MKDEV(htc_sdservice_major, 0), NULL, DEVICE_NAME);

	PDEBUG("register module ok");
	return 0;
}

static void  __exit htc_sdservice_exit(void)
{
	device_destroy(htc_sdservice_class, MKDEV(htc_sdservice_major, 0));
	class_unregister(htc_sdservice_class);
	class_destroy(htc_sdservice_class);
	unregister_chrdev(htc_sdservice_major, DEVICE_NAME);
	kfree(htc_sdkey);
	PDEBUG("un-registered module ok");
}

module_init(htc_sdservice_init);
module_exit(htc_sdservice_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("HTC");

