LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE    := diag_qcdm
LOCAL_SRC_FILES := diag_qcdm.c com.c utils.c commands.c errors.c result.c
LOCAL_LDLIBS    := -L$(SYSROOT)/usr/lib -llog

include $(BUILD_EXECUTABLE)
