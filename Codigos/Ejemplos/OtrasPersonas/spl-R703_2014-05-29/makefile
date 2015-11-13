# commands to produce release packages and etc.
#
# Copyright (C) 2012 Marsette A. Vona

PROJECT := spl
PUBLISH_DEST := login.ccs.neu.edu:/proj/vona/gpc/www/$(PROJECT)

COMMA := ,
EMPTY :=
SPACE := $(EMPTY) $(EMPTY)

SHELL = /bin/bash

CD := cd
SVNVERSION := svnversion
RSYNC := rsync --progress

# excludes the usual suspects
RSYNC_FLAGS := -rv --progress --exclude "**~" --exclude ".\#**" --exclude CVS --exclude .svn --exclude "*.tmp" --cvs-exclude

SVNVERSION_FILE := SVNVERSION.txt
SVNVERSION_STRING := $(if $(wildcard $(SVNVERSION_FILE)),R$(subst $(SPACE),_,$(filter-out %die,$(subst :,die ,$(strip $(shell cat $(SVNVERSION_FILE)))))),unknown)

ZIPBN = $(PROJECT)-SVNVERSION
ZIPBN_NEWEST := $(subst SVNVERSION,newest,$(ZIPBN))
ZIPBN_PAT := $(subst SVNVERSION,R*,$(ZIPBN))
ZIPBN_SUBST := $(subst SVNVERSION,$(SVNVERSION_STRING),$(ZIPBN))

ZIP_INCLUDE := data/rock.ply /data/rockpatch*.ply data/newrock.pcd
ZIP_EXCLUDE := 'data/**' priv kinfu DEAD kinect_mex tmp TBD.txt

# this should have dynamic binding so that makefile.package can override
# ZIPBN
ZIP_DIR = .$(ZIPBN).tmp

.PHONY: show-settings
show-settings:
	@echo make: no target specified. settings:
	@echo PROJECT: $(PROJECT)
	@echo ZIPBN_PAT: $(ZIPBN_PAT)
	@echo ZIPBN_NEWEST: $(ZIPBN_NEWEST)
	@echo ZIPBN: $(ZIPBN)
	@echo ZIPBN_SUBST: $(ZIPBN_SUBST)
	@echo ZIP_DIR: $(ZIP_DIR)
	@echo PUBLISH_DEST: $(PUBLISH_DEST)
	@echo SVNVERSION_STRING: $(SVNVERSION_STRING)
	@echo SVNVERSION_FILE: $(SVNVERSION_FILE)

# use this target as a prereq to force another target
.PHONY: FORCE
FORCE:

.PHONY: svnversion-clean
svnversion-clean:
	@echo removing generated $(SVNVERSION_FILE)
	$(RM) $(SVNVERSION_FILE)

.PHONY: zip-clean
zip-clean:
	$(RM) -rf $(ZIP_DIR)
	$(RM) -f $(ZIPBN_PAT).zip
	$(RM) -f $(ZIPBN_NEWEST).zip

$(SVNVERSION_FILE): FORCE
	$(SVNVERSION) > $@
	date +%Y-%m-%d >> $@

.PHONY: zip
zip: $(SVNVERSION_FILE)
	$(MAKE) zip-impl
.PHONY: zip-impl
zip-impl: zip-clean
	mkdir $(ZIP_DIR)
	mkdir $(ZIP_DIR)/$(ZIPBN_SUBST)
	$(RSYNC) $(RSYNC_FLAGS) --exclude "*.zip" \
		$(foreach f,$(ZIP_INCLUDE),--include $(f)) \
		$(foreach f,$(ZIP_EXCLUDE),--exclude $(f)) \
		* $(ZIP_DIR)/$(ZIPBN_SUBST)
	$(CD) $(ZIP_DIR) && zip -rp $(ZIPBN_SUBST).zip $(ZIPBN_SUBST)
	mv $(ZIP_DIR)/$(ZIPBN_SUBST).zip .
	ln -s $(ZIPBN_SUBST).zip $(ZIPBN_NEWEST).zip
	$(RM) -rf $(ZIP_DIR)

.PHONY: publish
publish: $(ZIPBN_SUBST).zip
	$(MAKE) publish-impl
.PHONY: publish-impl
publish-impl:
	@echo publishing to $(PUBLISH_DEST)
	$(RSYNC) $(ZIPBN_SUBST).zip $(PUBLISH_DEST)
	$(RSYNC) --links $(ZIPBN_NEWEST).zip $(PUBLISH_DEST)


