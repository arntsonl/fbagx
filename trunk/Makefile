#---------------------------------------------------------------------------------
# Clear the implicit built in rules
#---------------------------------------------------------------------------------
.SUFFIXES:
#---------------------------------------------------------------------------------
ifeq ($(strip $(DEVKITPPC)),)
$(error "Please set DEVKITPPC in your environment. export DEVKITPPC=<path to>devkitPPC")
endif

include $(DEVKITPPC)/wii_rules

#---------------------------------------------------------------------------------
# TARGET is the name of the output
# BUILD is the directory where object files & intermediate files will be placed
# SOURCES is a list of directories containing source code
# INCLUDES is a list of directories containing extra header files
#---------------------------------------------------------------------------------
TARGET		:=	fbagx_beta
BUILD		:=	build
HBC			:=  hbc
SOURCES		:=	source/ngc source/ngc/libwiigui source/ngc/images source/ngc/fonts source/ngc/sounds \
				source/burn source/burner source/interfacegx source/burn/atari source/burn/capcom source/burn/cave source/burn/cps3 \
				source/burn/galaxian source/burn/konami source/burn/megadrive \
				source/burn/misc source/burn/misc/dec0 source/burn/misc/post90s source/burn/misc/pre90s \
				source/burn/neogeo source/burn/pgm source/burn/psikyo source/burn/sega source/burn/taito \
				source/burn/toaplan source/cpu source/cpu/arm7 source/cpu/hd6309 source/cpu/i8039 \
				source/cpu/konami source/cpu/m68k source/cpu/m6502 source/cpu/m6800 source/cpu/m6809 \
				source/cpu/nec source/cpu/s2650 source/cpu/sh2 source/cpu/z80
				
INCLUDES	:=	source/ngc/ \
				source/burn source/burner source/interfacegx source/burn/atari source/burn/capcom source/burn/cave source/burn/cps3 \
				source/burn/galaxian source/burn/konami source/burn/megadrive \
				source/burn/misc source/burn/misc/dec0 source/burn/misc/post90s source/burn/misc/pre90s \
				source/burn/neogeo source/burn/pgm source/burn/psikyo source/burn/sega source/burn/taito \
				source/burn/toaplan source/cpu source/cpu/arm7 source/cpu/hd6309 source/cpu/i8039 \
				source/cpu/konami source/cpu/m68k source/cpu/m6502 source/cpu/m6800 source/cpu/m6809 \
				source/cpu/nec source/cpu/s2650 source/cpu/sh2 source/cpu/z80

#---------------------------------------------------------------------------------
# options for code generation
#---------------------------------------------------------------------------------

CFLAGS		=	-g -O2 -Wall $(MACHDEP) $(INCLUDE)
CXXFLAGS	=	-save-temps -Xassembler -aln=$@.lst $(CFLAGS)
LDFLAGS		=	-g $(MACHDEP) -Wl,-Map,$(notdir $@).map

#---------------------------------------------------------------------------------
# any extra libraries we wish to link with the project
#---------------------------------------------------------------------------------
LIBS :=	-lpng -lz -lfat -lwiiuse -lbte -lasnd -logc -lvorbisidec -lfreetype -ldb
#---------------------------------------------------------------------------------
# list of directories containing libraries, this must be the top level containing
# include and lib
#---------------------------------------------------------------------------------
LIBDIRS	:= $(PORTLIBS)

#---------------------------------------------------------------------------------
# no real need to edit anything past this point unless you need to add additional
# rules for different file extensions
#---------------------------------------------------------------------------------
ifneq ($(BUILD),$(notdir $(CURDIR)))
#---------------------------------------------------------------------------------

export OUTPUT	:=	$(CURDIR)/$(TARGETDIR)/$(TARGET)
export VPATH	:=	$(foreach dir,$(SOURCES),$(CURDIR)/$(dir))
export DEPSDIR	:=	$(CURDIR)/$(BUILD)

#---------------------------------------------------------------------------------
# automatically build a list of object files for our project
#---------------------------------------------------------------------------------
CFILES		:=	$(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.c)))
CPPFILES	:=	$(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.cpp)))
sFILES		:=	$(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.s)))
SFILES		:=	$(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.S)))
TTFFILES	:=	$(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.ttf)))
PNGFILES	:=	$(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.png)))
OGGFILES	:=	$(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.ogg)))
PCMFILES	:=	$(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.pcm)))
	
#---------------------------------------------------------------------------------
# use CXX for linking C++ projects, CC for standard C
#---------------------------------------------------------------------------------
ifeq ($(strip $(CPPFILES)),)
	export LD	:=	$(CC)
else
	export LD	:=	$(CXX)
endif

export OFILES	:=	$(CFILES:.c=.o) $(CPPFILES:.cpp=.o)  \
					$(sFILES:.s=.o) $(SFILES:.S=.o) \
					$(TTFFILES:.ttf=.ttf.o) $(PNGFILES:.png=.png.o) \
					$(OGGFILES:.ogg=.ogg.o) $(PCMFILES:.pcm=.pcm.o)

#---------------------------------------------------------------------------------
# build a list of include paths
#---------------------------------------------------------------------------------
export INCLUDE	:=	$(foreach dir,$(INCLUDES),-I$(CURDIR)/$(dir)) \
					$(foreach dir,$(LIBDIRS),-I$(dir)/include) \
					-I$(CURDIR)/$(BUILD) \
					-I$(LIBOGC_INC) -I$(PORTLIBS)/include/freetype2

#---------------------------------------------------------------------------------
# build a list of library paths
#---------------------------------------------------------------------------------
export LIBPATHS	:=	$(foreach dir,$(LIBDIRS),-L$(dir)/lib) \
					-L$(LIBOGC_LIB)

export OUTPUT	:=	$(CURDIR)/$(TARGET)
.PHONY: $(BUILD) clean

#---------------------------------------------------------------------------------
$(BUILD):
	@[ -d $@ ] || mkdir -p $@
	@make --no-print-directory -C $(BUILD) -f $(CURDIR)/Makefile
	@cp $(OUTPUT).dol $(HBC)/boot.dol

#---------------------------------------------------------------------------------
clean:
	@echo clean ...
	@rm -fr $(BUILD) $(OUTPUT).elf $(OUTPUT).dol $(HBC)/boot.dol

#---------------------------------------------------------------------------------
run:
	wiiload $(OUTPUT).dol

#---------------------------------------------------------------------------------
debug:
	wiiload $(TARGET).dol
	powerpc-eabi-gdb -n $(TARGET).elf -x gdb.txt
	
#---------------------------------------------------------------------------------
reload:
	wiiload -r $(OUTPUT).dol

#---------------------------------------------------------------------------------
else

DEPENDS	:=	$(OFILES:.o=.d)

#---------------------------------------------------------------------------------
# main targets
#---------------------------------------------------------------------------------
$(OUTPUT).dol: $(OUTPUT).elf
$(OUTPUT).elf: $(OFILES)

#---------------------------------------------------------------------------------
# This rule links in binary data with .ttf, .png, and .mp3 extensions
#---------------------------------------------------------------------------------
%.ttf.o : %.ttf
	@echo $(notdir $<)
	$(bin2o)

%.png.o : %.png
	@echo $(notdir $<)
	$(bin2o)
	
%.ogg.o : %.ogg
	@echo $(notdir $<)
	$(bin2o)
	
%.pcm.o : %.pcm
	@echo $(notdir $<)
	$(bin2o)

-include $(DEPENDS)

#---------------------------------------------------------------------------------
endif
#---------------------------------------------------------------------------------
