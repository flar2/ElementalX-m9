#include "fusb30x_global.h"

struct fusb30x_chip* fusb_g_chip = NULL;  // Our driver's relevant data

struct fusb30x_chip* fusb30x_GetChip(void)
{
    return fusb_g_chip;      // return a pointer to our structs
}

void fusb30x_SetChip(struct fusb30x_chip* newChip)
{
    fusb_g_chip = newChip;   // assign the pointer to our struct
    printk(KERN_INFO "FUSB [%s] chip->client addr = 0x%x\n", __func__, fusb_g_chip->client->addr);
}
