/*
 * HND generic packet pool operation primitives
 *
 * Copyright (C) 1999-2015, Broadcom Corporation
 * 
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 * 
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 * 
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 * $Id: $
 */

#include <typedefs.h>
#include <osl.h>
#include <bcmutils.h>
#include <hnd_pktpool.h>

#define PKTPOOLREG_RSVD_ID				(0U)
#define PKTPOOLREG_RSVD_PTR				(POOLPTR(0xdeaddead))
#define PKTPOOLREG_FREE_PTR				(POOLPTR(NULL))

#define PKTPOOL_REGISTRY_SET(id, pp)	(pktpool_registry_set((id), (pp)))
#define PKTPOOL_REGISTRY_CMP(id, pp)	(pktpool_registry_cmp((id), (pp)))

#define PKTPOOL_REGISTRY_CLR(id)		\
		PKTPOOL_REGISTRY_SET((id), PKTPOOLREG_FREE_PTR)
#define PKTPOOL_REGISTRY_ISCLR(id)		\
		(PKTPOOL_REGISTRY_CMP((id), PKTPOOLREG_FREE_PTR))

#define PKTPOOL_REGISTRY_RSV()			\
		PKTPOOL_REGISTRY_SET(PKTPOOLREG_RSVD_ID, PKTPOOLREG_RSVD_PTR)
#define PKTPOOL_REGISTRY_ISRSVD()		\
		(PKTPOOL_REGISTRY_CMP(PKTPOOLREG_RSVD_ID, PKTPOOLREG_RSVD_PTR))

#define PKTPOOL_REGISTRY_FOREACH(id)	\
		for ((id) = 1U; (id) <= pktpools_max; (id)++)

uint32 pktpools_max = 0U; 
pktpool_t *pktpools_registry[PKTPOOL_MAXIMUM_ID + 1]; 

static int pktpool_register(pktpool_t * poolptr);
static int pktpool_deregister(pktpool_t * poolptr);

static void
BCMRAMFN(pktpool_registry_set)(int id, pktpool_t *pp)
{
	pktpools_registry[id] = pp;
}

static bool
BCMRAMFN(pktpool_registry_cmp)(int id, pktpool_t *pp)
{
	return pktpools_registry[id] == pp;
}

int 
pktpool_attach(osl_t *osh, uint32 total_pools)
{
	uint32 poolid;

	if (pktpools_max != 0U) {
		return BCME_ERROR;
	}

	ASSERT(total_pools <= PKTPOOL_MAXIMUM_ID);

	
	PKTPOOL_REGISTRY_RSV();		

	PKTPOOL_REGISTRY_FOREACH(poolid) {	
		PKTPOOL_REGISTRY_CLR(poolid);
	}

	pktpools_max = total_pools;

	return (int)pktpools_max;
}

int 
pktpool_dettach(osl_t *osh)
{
	uint32 poolid;

	if (pktpools_max == 0U) {
		return BCME_OK;
	}

	
	ASSERT(PKTPOOL_REGISTRY_ISRSVD()); 

	PKTPOOL_REGISTRY_FOREACH(poolid) {	
		ASSERT(PKTPOOL_REGISTRY_ISCLR(poolid));
	}

	pktpools_max = 0U; 

	return BCME_OK;
}

static int	
pktpool_register(pktpool_t * poolptr)
{
	uint32 poolid;

	if (pktpools_max == 0U) {
		return PKTPOOL_INVALID_ID; 
	}

	ASSERT(pktpools_max != 0U);

	
	PKTPOOL_REGISTRY_FOREACH(poolid) {
		if (PKTPOOL_REGISTRY_ISCLR(poolid)) {
			PKTPOOL_REGISTRY_SET(poolid, POOLPTR(poolptr)); 
			return (int)poolid; 
		}
	} 

	return PKTPOOL_INVALID_ID;	
}

static int	
pktpool_deregister(pktpool_t * poolptr)
{
	uint32 poolid;

	ASSERT(POOLPTR(poolptr) != POOLPTR(NULL));

	poolid = POOLID(poolptr);
	ASSERT(poolid <= pktpools_max);

	
	if (PKTPOOL_REGISTRY_CMP(poolid, POOLPTR(poolptr))) {
		PKTPOOL_REGISTRY_CLR(poolid); 
	} else {
		ASSERT(0);
		return BCME_ERROR; 
	}

	return BCME_OK;
}


int
pktpool_init(osl_t *osh, pktpool_t *pktp, int *pplen, int plen, bool istx, uint8 type)
{
	int i, err = BCME_OK;
	int pktplen;
	uint8 pktp_id;

	ASSERT(pktp != NULL);
	ASSERT(osh != NULL);
	ASSERT(pplen != NULL);

	pktplen = *pplen;

	bzero(pktp, sizeof(pktpool_t));

	
	if ((pktp_id = (uint8) pktpool_register(pktp)) == PKTPOOL_INVALID_ID) {
		return BCME_ERROR;
	}
	POOLSETID(pktp, pktp_id);

	pktp->inited = TRUE;
	pktp->istx = istx ? TRUE : FALSE;
	pktp->plen = (uint16)plen;
	pktp->type = type;

	pktp->maxlen = PKTPOOL_LEN_MAX;
	pktplen = LIMIT_TO_MAX(pktplen, pktp->maxlen);

	for (i = 0; i < pktplen; i++) {
		void *p;
		p = PKTGET(osh, plen, TRUE);

		if (p == NULL) {
			if (pktp->freelist == NULL) 
				err = BCME_NOMEM;

			goto exit;
		}

		PKTSETPOOL(osh, p, TRUE, pktp); 

		PKTSETFREELIST(p, pktp->freelist); 
		pktp->freelist = p;

		pktp->avail++;

#ifdef BCMDBG_POOL
		pktp->dbg_q[pktp->dbg_qlen++].p = p;
#endif
	}

exit:
	pktp->len = pktp->avail;

	*pplen = pktp->len;
	return err;
}

int
pktpool_deinit(osl_t *osh, pktpool_t *pktp)
{
	uint16 freed = 0;

	ASSERT(osh != NULL);
	ASSERT(pktp != NULL);

#ifdef BCMDBG_POOL
	{
		int i;
		for (i = 0; i <= pktp->len; i++) {
			pktp->dbg_q[i].p = NULL;
		}
	}
#endif

	while (pktp->freelist != NULL) {
		void * p = pktp->freelist;

		pktp->freelist = PKTFREELIST(p); 
		PKTSETFREELIST(p, NULL);

		PKTSETPOOL(osh, p, FALSE, NULL); 

		PKTFREE(osh, p, pktp->istx); 

		freed++;
		ASSERT(freed <= pktp->len);
	}

	pktp->avail -= freed;
	ASSERT(pktp->avail == 0);

	pktp->len -= freed;

	pktpool_deregister(pktp); 
	POOLSETID(pktp, PKTPOOL_INVALID_ID);

	pktp->inited = FALSE;

	
	ASSERT(pktp->len == 0);

	return 0;
}

int
pktpool_fill(osl_t *osh, pktpool_t *pktp, bool minimal)
{
	void *p;
	int err = 0;
	int len, psize, maxlen;

	ASSERT(pktp->plen != 0);

	maxlen = pktp->maxlen;
	psize = minimal ? (maxlen >> 2) : maxlen;
	for (len = (int)pktp->len; len < psize; len++) {

		p = PKTGET(osh, pktp->len, TRUE);

		if (p == NULL) {
			err = BCME_NOMEM;
			break;
		}

		if (pktpool_add(pktp, p) != BCME_OK) {
			PKTFREE(osh, p, FALSE);
			err = BCME_ERROR;
			break;
		}
	}

	return err;
}

static void *
pktpool_deq(pktpool_t *pktp)
{
	void *p;

	if (pktp->avail == 0)
		return NULL;

	ASSERT(pktp->freelist != NULL);

	p = pktp->freelist;  
	pktp->freelist = PKTFREELIST(p); 
	PKTSETFREELIST(p, NULL);

	pktp->avail--;

	return p;
}

static void
pktpool_enq(pktpool_t *pktp, void *p)
{
	ASSERT(p != NULL);

	PKTSETFREELIST(p, pktp->freelist); 
	pktp->freelist = p; 

	pktp->avail++;
	ASSERT(pktp->avail <= pktp->len);
}

int
(pktpool_hostaddr_fill_register)(pktpool_t *pktp, pktpool_cb_extn_t cb, void *arg)
{

	ASSERT(cb != NULL);

	ASSERT(pktp->cbext.cb == NULL);
	pktp->cbext.cb = cb;
	pktp->cbext.arg = arg;
	return 0;
}

int
pktpool_rxcplid_fill_register(pktpool_t *pktp, pktpool_cb_extn_t cb, void *arg)
{

	ASSERT(cb != NULL);

	ASSERT(pktp->rxcplidfn.cb == NULL);
	pktp->rxcplidfn.cb = cb;
	pktp->rxcplidfn.arg = arg;
	return 0;
}
void
pktpool_invoke_dmarxfill(pktpool_t *pktp)
{
	ASSERT(pktp->dmarxfill.cb);
	ASSERT(pktp->dmarxfill.arg);

	if (pktp->dmarxfill.cb)
		pktp->dmarxfill.cb(pktp, pktp->dmarxfill.arg);
}
int
pkpool_haddr_avail_register_cb(pktpool_t *pktp, pktpool_cb_t cb, void *arg)
{

	ASSERT(cb != NULL);

	pktp->dmarxfill.cb = cb;
	pktp->dmarxfill.arg = arg;

	return 0;
}
int
pktpool_avail_register(pktpool_t *pktp, pktpool_cb_t cb, void *arg)
{
	int i;

	ASSERT(cb != NULL);

	i = pktp->cbcnt;
	if (i == PKTPOOL_CB_MAX)
		return BCME_ERROR;

	ASSERT(pktp->cbs[i].cb == NULL);
	pktp->cbs[i].cb = cb;
	pktp->cbs[i].arg = arg;
	pktp->cbcnt++;

	return 0;
}

int
pktpool_empty_register(pktpool_t *pktp, pktpool_cb_t cb, void *arg)
{
	int i;

	ASSERT(cb != NULL);

	i = pktp->ecbcnt;
	if (i == PKTPOOL_CB_MAX)
		return BCME_ERROR;

	ASSERT(pktp->ecbs[i].cb == NULL);
	pktp->ecbs[i].cb = cb;
	pktp->ecbs[i].arg = arg;
	pktp->ecbcnt++;

	return 0;
}

static int
pktpool_empty_notify(pktpool_t *pktp)
{
	int i;

	pktp->empty = TRUE;
	for (i = 0; i < pktp->ecbcnt; i++) {
		ASSERT(pktp->ecbs[i].cb != NULL);
		pktp->ecbs[i].cb(pktp, pktp->ecbs[i].arg);
	}
	pktp->empty = FALSE;

	return 0;
}

#ifdef BCMDBG_POOL
int
pktpool_dbg_register(pktpool_t *pktp, pktpool_cb_t cb, void *arg)
{
	int i;

	ASSERT(cb);

	i = pktp->dbg_cbcnt;
	if (i == PKTPOOL_CB_MAX)
		return BCME_ERROR;

	ASSERT(pktp->dbg_cbs[i].cb == NULL);
	pktp->dbg_cbs[i].cb = cb;
	pktp->dbg_cbs[i].arg = arg;
	pktp->dbg_cbcnt++;

	return 0;
}

int pktpool_dbg_notify(pktpool_t *pktp);

int
pktpool_dbg_notify(pktpool_t *pktp)
{
	int i;

	for (i = 0; i < pktp->dbg_cbcnt; i++) {
		ASSERT(pktp->dbg_cbs[i].cb);
		pktp->dbg_cbs[i].cb(pktp, pktp->dbg_cbs[i].arg);
	}

	return 0;
}

int
pktpool_dbg_dump(pktpool_t *pktp)
{
	int i;

	printf("pool len=%d maxlen=%d\n",  pktp->dbg_qlen, pktp->maxlen);
	for (i = 0; i < pktp->dbg_qlen; i++) {
		ASSERT(pktp->dbg_q[i].p);
		printf("%d, p: 0x%x dur:%lu us state:%d\n", i,
			pktp->dbg_q[i].p, pktp->dbg_q[i].dur/100, PKTPOOLSTATE(pktp->dbg_q[i].p));
	}

	return 0;
}

int
pktpool_stats_dump(pktpool_t *pktp, pktpool_stats_t *stats)
{
	int i;
	int state;

	bzero(stats, sizeof(pktpool_stats_t));
	for (i = 0; i < pktp->dbg_qlen; i++) {
		ASSERT(pktp->dbg_q[i].p != NULL);

		state = PKTPOOLSTATE(pktp->dbg_q[i].p);
		switch (state) {
			case POOL_TXENQ:
				stats->enq++; break;
			case POOL_TXDH:
				stats->txdh++; break;
			case POOL_TXD11:
				stats->txd11++; break;
			case POOL_RXDH:
				stats->rxdh++; break;
			case POOL_RXD11:
				stats->rxd11++; break;
			case POOL_RXFILL:
				stats->rxfill++; break;
			case POOL_IDLE:
				stats->idle++; break;
		}
	}

	return 0;
}

int
pktpool_start_trigger(pktpool_t *pktp, void *p)
{
	uint32 cycles, i;

	if (!PKTPOOL(OSH_NULL, p))
		return 0;

	OSL_GETCYCLES(cycles);

	for (i = 0; i < pktp->dbg_qlen; i++) {
		ASSERT(pktp->dbg_q[i].p != NULL);

		if (pktp->dbg_q[i].p == p) {
			pktp->dbg_q[i].cycles = cycles;
			break;
		}
	}

	return 0;
}

int pktpool_stop_trigger(pktpool_t *pktp, void *p);
int
pktpool_stop_trigger(pktpool_t *pktp, void *p)
{
	uint32 cycles, i;

	if (!PKTPOOL(OSH_NULL, p))
		return 0;

	OSL_GETCYCLES(cycles);

	for (i = 0; i < pktp->dbg_qlen; i++) {
		ASSERT(pktp->dbg_q[i].p != NULL);

		if (pktp->dbg_q[i].p == p) {
			if (pktp->dbg_q[i].cycles == 0)
				break;

			if (cycles >= pktp->dbg_q[i].cycles)
				pktp->dbg_q[i].dur = cycles - pktp->dbg_q[i].cycles;
			else
				pktp->dbg_q[i].dur =
					(((uint32)-1) - pktp->dbg_q[i].cycles) + cycles + 1;

			pktp->dbg_q[i].cycles = 0;
			break;
		}
	}

	return 0;
}
#endif 

int
pktpool_avail_notify_normal(osl_t *osh, pktpool_t *pktp)
{
	ASSERT(pktp);
	pktp->availcb_excl = NULL;
	return 0;
}

int
pktpool_avail_notify_exclusive(osl_t *osh, pktpool_t *pktp, pktpool_cb_t cb)
{
	int i;

	ASSERT(pktp);
	ASSERT(pktp->availcb_excl == NULL);
	for (i = 0; i < pktp->cbcnt; i++) {
		if (cb == pktp->cbs[i].cb) {
			pktp->availcb_excl = &pktp->cbs[i];
			break;
		}
	}

	if (pktp->availcb_excl == NULL)
		return BCME_ERROR;
	else
		return 0;
}

static int
pktpool_avail_notify(pktpool_t *pktp)
{
	int i, k, idx;
	int avail;

	ASSERT(pktp);
	if (pktp->availcb_excl != NULL) {
		pktp->availcb_excl->cb(pktp, pktp->availcb_excl->arg);
		return 0;
	}

	k = pktp->cbcnt - 1;
	for (i = 0; i < pktp->cbcnt; i++) {
		avail = pktp->avail;

		if (avail) {
			if (pktp->cbtoggle)
				idx = i;
			else
				idx = k--;

			ASSERT(pktp->cbs[idx].cb != NULL);
			pktp->cbs[idx].cb(pktp, pktp->cbs[idx].arg);
		}
	}

	pktp->cbtoggle ^= 1;

	return 0;
}

void *
pktpool_get(pktpool_t *pktp)
{
	void *p;

	p = pktpool_deq(pktp);

	if (p == NULL) {
		
		if (pktp->ecbcnt)
			pktpool_empty_notify(pktp);

		p = pktpool_deq(pktp);
		if (p == NULL)
			return NULL;
	}

	return p;
}

void
pktpool_free(pktpool_t *pktp, void *p)
{
	ASSERT(p != NULL);
#ifdef BCMDBG_POOL
	
#endif

	pktpool_enq(pktp, p);

	if (pktp->emptycb_disable)
		return;

	if (pktp->cbcnt) {
		if (pktp->empty == FALSE)
			pktpool_avail_notify(pktp);
	}
}

int
pktpool_add(pktpool_t *pktp, void *p)
{
	ASSERT(p != NULL);

	if (pktp->len == pktp->maxlen)
		return BCME_RANGE;

	
	ASSERT(pktp->plen == PKTLEN(OSH_NULL, p));
	PKTSETPOOL(OSH_NULL, p, TRUE, pktp);

	pktp->len++;
	pktpool_enq(pktp, p);

#ifdef BCMDBG_POOL
	pktp->dbg_q[pktp->dbg_qlen++].p = p;
#endif

	return 0;
}

int
BCMRAMFN(pktpool_setmaxlen)(pktpool_t *pktp, uint16 maxlen)
{
	if (maxlen > PKTPOOL_LEN_MAX)
		maxlen = PKTPOOL_LEN_MAX;

	pktp->maxlen = (pktp->len > maxlen) ? pktp->len : maxlen;

	return pktp->maxlen;
}

void
pktpool_emptycb_disable(pktpool_t *pktp, bool disable)
{
	ASSERT(pktp);

	pktp->emptycb_disable = disable;
}

bool
pktpool_emptycb_disabled(pktpool_t *pktp)
{
	ASSERT(pktp);
	return pktp->emptycb_disable;
}
