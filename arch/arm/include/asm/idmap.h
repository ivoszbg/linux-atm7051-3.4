#ifndef __ASM_IDMAP_H
#define __ASM_IDMAP_H

#include <linux/compiler.h>
#include <asm/pgtable.h>

/* Tag a function as requiring to be executed via an identity mapping. */
#define __idmap __section(.idmap.text) noinline notrace

extern pgd_t *idmap_pgd;

void identity_mapping_add(pgd_t *pgd, unsigned long addr, unsigned long end);
void identity_mapping_del(pgd_t *pgd, unsigned long addr, unsigned long end);
void setup_mm_for_reboot(void);

#endif	/* __ASM_IDMAP_H */
