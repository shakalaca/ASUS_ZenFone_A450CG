# This file is generated by gyp; do not edit.

include $(CLEAR_VARS)

LOCAL_MODULE_CLASS := GYP
LOCAL_MODULE := webkit_common_user_agent_webkit_version_gyp
LOCAL_MODULE_STEM := webkit_version
LOCAL_MODULE_SUFFIX := .stamp
LOCAL_MODULE_TAGS := optional
gyp_intermediate_dir := $(call local-intermediates-dir)
gyp_shared_intermediate_dir := $(call intermediates-dir-for,GYP,shared)

# Make sure our deps are built first.
GYP_TARGET_DEPENDENCIES :=

### Rules for action "webkit_version":
$(gyp_shared_intermediate_dir)/webkit_version.h: gyp_local_path := $(LOCAL_PATH)
$(gyp_shared_intermediate_dir)/webkit_version.h: gyp_intermediate_dir := $(abspath $(gyp_intermediate_dir))
$(gyp_shared_intermediate_dir)/webkit_version.h: gyp_shared_intermediate_dir := $(abspath $(gyp_shared_intermediate_dir))
$(gyp_shared_intermediate_dir)/webkit_version.h: export PATH := $(subst $(ANDROID_BUILD_PATHS),,$(PATH))
$(gyp_shared_intermediate_dir)/webkit_version.h: $(LOCAL_PATH)/chrome/tools/build/version.py $(LOCAL_PATH)/build/util/LASTCHANGE.blink $(LOCAL_PATH)/webkit/build/webkit_version.h.in $(GYP_TARGET_DEPENDENCIES)
	@echo "Gyp action: webkit_common_user_agent_webkit_user_agent_gyp_webkit_version_target_webkit_version ($@)"
	$(hide)cd $(gyp_local_path)/webkit/common/user_agent; mkdir -p $(gyp_shared_intermediate_dir); python ../../../chrome/tools/build/version.py -f ../../../build/util/LASTCHANGE.blink ../../../webkit/build/webkit_version.h.in "$(gyp_shared_intermediate_dir)/webkit_version.h"



GYP_GENERATED_OUTPUTS := \
	$(gyp_shared_intermediate_dir)/webkit_version.h

# Make sure our deps and generated files are built first.
LOCAL_ADDITIONAL_DEPENDENCIES := $(GYP_TARGET_DEPENDENCIES) $(GYP_GENERATED_OUTPUTS)

### Rules for final target.
# Add target alias to "gyp_all_modules" target.
.PHONY: gyp_all_modules
gyp_all_modules: webkit_common_user_agent_webkit_version_gyp

# Alias gyp target name.
.PHONY: webkit_version
webkit_version: webkit_common_user_agent_webkit_version_gyp

LOCAL_MODULE_PATH := $(PRODUCT_OUT)/gyp_stamp
LOCAL_UNINSTALLABLE_MODULE := true

include $(BUILD_SYSTEM)/base_rules.mk

$(LOCAL_BUILT_MODULE): $(LOCAL_ADDITIONAL_DEPENDENCIES)
	$(hide) echo "Gyp timestamp: $@"
	$(hide) mkdir -p $(dir $@)
	$(hide) touch $@
