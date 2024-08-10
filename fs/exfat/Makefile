
obj-$(CONFIG_EXFAT_FS) += exfat_core.o exfat_fs.o

exfat_fs-y	:= exfat_super.o exfat_nfs.o

exfat_core-y	:= exfat.o exfat_api.o exfat_blkdev.o exfat_cache.o \
			   exfat_data.o exfat_global.o exfat_nls.o exfat_oal.o exfat_upcase.o


EXTRA_CFLAGS += -Wextra -Wno-unused-parameter -Wno-sign-compare 
#-fprofile-arcs -ftest-coverage
	
