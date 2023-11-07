#
#  Copyright (c) 2012-2018 Texas Instruments Incorporated - http://www.ti.com
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  *  Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#
#  *  Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#
#  *  Neither the name of Texas Instruments Incorporated nor the names of
#     its contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
#  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
#  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
#  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
#  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

EXBASE = ..
include $(EXBASE)/products.mak

srcs = src/main.c src/dsp_server.c src/cv_algorithms.cpp
objs = $(addprefix bin/$(PROFILE)/obj/,$(patsubst %.c,%.oe674,$(patsubst %.cpp,%.oe674,$(srcs))))
CONFIG = bin/$(PROFILE)/configuro

PKGPATH := $(BIOS_INSTALL_DIR)/packages
PKGPATH := $(PKGPATH)+$(IPC_INSTALL_DIR)/packages
PKGPATH := $(PKGPATH)+$(XDC_INSTALL_DIR)/packages
ifneq ($(PDK_INSTALL_DIR),)
PKGPATH := $(PKGPATH)+$(PDK_INSTALL_DIR)/packages
endif

-include $(addprefix bin/$(PROFILE)/obj/,$(patsubst %.c,%.oe674.dep,$(patsubst %.cpp,%.oe674.dep,$(srcs))))

#  ======== install validation ========
ifeq (install,$(MAKECMDGOALS))
ifeq (,$(EXEC_DIR))
$(error must specify EXEC_DIR)
endif
endif

#  ======== toolchain macros ========
CGTOOLS = $(ti.targets.elf.C674)

CC = $(CGTOOLS)/bin/cl6x -c
LD = $(CGTOOLS)/bin/cl6x -z

CPPFLAGS = -I./include/ -I../shared/include -pdsw225 -ppd=$@.dep -ppa $(CCPROFILE_$(PROFILE)) -@$(CONFIG)/compiler.opt -I.
CFLAGS = -I./include/ -I../shared/include -pdsw225 -ppd=$@.dep -ppa $(CCPROFILE_$(PROFILE)) -@$(CONFIG)/compiler.opt -I.

# entry point is set to an aligned address so that IPC can load the slave
LDFLAGS = -w -q -u _c_int00 -c -m $(@D)/obj/$(@F).map
LDLIBS = -l $(CGTOOLS)/lib/libc.a -l ./lib/imglib.ae64P #-l./lib/common.ae674 -l./lib/vlib.ae674 -l./lib/vlib_cn.ae674 #--export=__c6xabi_divi --export=__c6xabi_remi

CCPROFILE_debug = -D_DEBUG_=1 --symdebug:dwarf
CCPROFILE_release = -O3 --optimizer_interlist --debug_software_pipeline --c_src_interlist --gen_opt_info=2 --advice:performance=all -DNDEBUG=1 --auto_inline --speculate_loads=16 --opt_for_speed=5 --call_assumptions=3 --no_bad_aliases --no_reload_errors --gen_func_subsections

#  ======== standard macros ========
ifneq (,$(wildcard $(XDC_INSTALL_DIR)/xdc.exe))
    # use these on Windows
    CP      = $(XDC_INSTALL_DIR)/bin/cp
    ECHO    = $(XDC_INSTALL_DIR)/bin/echo
    MKDIR   = $(XDC_INSTALL_DIR)/bin/mkdir -p
    RM      = $(XDC_INSTALL_DIR)/bin/rm -f
    RMDIR   = $(XDC_INSTALL_DIR)/bin/rm -rf
else
    # use these on Linux
    CP      = cp
    ECHO    = echo
    MKDIR   = mkdir -p
    RM      = rm -f
    RMDIR   = rm -rf
endif

#  ======== create output directories ========
ifneq (clean,$(MAKECMDGOALS))
ifneq (,$(PROFILE))
ifeq (,$(wildcard bin/$(PROFILE)/obj))
    $(shell $(MKDIR) -p bin/$(PROFILE)/obj)
endif
endif
endif

.PRECIOUS: %/compiler.opt %/linker.cmd

all: debug release


debug:
	$(MAKE) PROFILE=debug PROCLIST="$(PROCLIST)" server_dsp.x

release:
	$(MAKE) PROFILE=release PROCLIST="$(PROCLIST)" server_dsp.x

server_dsp.x: bin/$(PROFILE)/server_dsp.xe674
bin/$(PROFILE)/server_dsp.xe674: $(objs) $(libs) $(CONFIG)/linker.cmd
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	@-$(MKDIR) $(dir $@)
	$(LD) $(LDFLAGS) -o $@ $(objs) \
            $(addprefix -l ,$(libs)) $(CONFIG)/linker.cmd $(LDLIBS)

bin/$(PROFILE)/obj/%.oe674: %.c $(CONFIG)/compiler.opt
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	@-$(MKDIR) $(dir $@)
	$(CC) $(CFLAGS) --output_file=$@ -fc $<
bin/$(PROFILE)/obj/%.oe674: %.cpp $(CONFIG)/compiler.opt
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	@-$(MKDIR) $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) --output_file=$@ -fp $<

%/linker.cmd %/compiler.opt: $(CONFIG)/.config ;
$(CONFIG)/.config: Dsp.cfg ../dsp/config.bld
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(XDC_INSTALL_DIR)/xs --xdcpath="$(subst +,;,$(PKGPATH));." \
            xdc.tools.configuro -o $(CONFIG) \
            -t ti.targets.elf.C674 \
            -c $(ti.targets.elf.C674) \
            -p ti.platforms.evmOMAPL138:dsp \
            -b ../dsp/config.bld \
            -r $(PROFILE) \
            Dsp.cfg
	@$(ECHO) "" > $@

install:
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	@$(MKDIR) $(EXEC_DIR)/debug
	$(CP) bin/debug/server_dsp.xe674 $(EXEC_DIR)/debug
	@$(MKDIR) $(EXEC_DIR)/release
	$(CP) bin/release/server_dsp.xe674 $(EXEC_DIR)/release

install_rov:
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	@$(MKDIR) $(EXEC_DIR)/debug
	$(CP) bin/debug/configuro/package/cfg/Dsp_pe674.rov.xs $(EXEC_DIR)/debug
	@$(MKDIR) $(EXEC_DIR)/release
	$(CP) bin/release/configuro/package/cfg/Dsp_pe674.rov.xs $(EXEC_DIR)/release

help:
	@$(ECHO) "make                   # build executable"
	@$(ECHO) "make clean             # clean everything"

clean::
	$(RMDIR) bin
