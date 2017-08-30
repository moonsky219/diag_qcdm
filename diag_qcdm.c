/* diag_qcdm.c
 * Author: Yuanjie Li, Jiayao Li, Haotian Deng
 * Try to communicate with /dev/diag device using libqcdm.
 */

#include <assert.h>
#include <endian.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>

#include "com.h"
#include "commands.h"
#include "errors.h"
#include "result.h"
#include "utils.h"

#define _GNU_SOURCE
#define F_SETPIPE_SZ (F_LINUX_SPECIFIC_BASE + 7)
#define F_GETPIPE_SZ (F_LINUX_SPECIFIC_BASE + 8)

#include <android/log.h>
#define  LOG_TAG    "diag_qcdm"

#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#define  LOGW(...)  __android_log_print(ANDROID_LOG_WARN,LOG_TAG,__VA_ARGS__)
#define  LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG,LOG_TAG,__VA_ARGS__)
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)

// #define BUFFER_SIZE	8192
#define BUFFER_SIZE	32768

#define FIFO_MSG_TYPE_LOG 1
#define FIFO_MSG_TYPE_START_LOG_FILE 2
#define FIFO_MSG_TYPE_END_LOG_FILE 3

/*
 * MDM VS. MSM
 * Reference: https://android.googlesource.com/kernel/msm.git/+/android-6.0.0_r0.9/include/linux/diagchar.h
 */
enum remote_procs {
	MSM = 0,
	MDM = 1,
	MDM2 = 2,
	QSC = 5,
};

/* Raw binary data type
 * Reference: https://android.googlesource.com/kernel/msm.git/+/android-6.0.0_r0.9/include/linux/diagchar.h
 */
#define MSG_MASKS_TYPE		0x00000001
#define LOG_MASKS_TYPE		0x00000002
#define EVENT_MASKS_TYPE	0x00000004
#define PKT_TYPE		0x00000008
#define DEINIT_TYPE		0x00000010
#define USER_SPACE_DATA_TYPE	0x00000020
#define DCI_DATA_TYPE		0x00000040
#define CALLBACK_DATA_TYPE	0x00000080
#define DCI_LOG_MASKS_TYPE	0x00000100
#define DCI_EVENT_MASKS_TYPE	0x00000200
#define DCI_PKT_TYPE		0x00000400

/* IOCTL commands for diagnostic port
 * Reference: https://android.googlesource.com/kernel/msm.git/+/android-6.0.0_r0.9/include/linux/diagchar.h
 */ 
#define DIAG_IOCTL_SWITCH_LOGGING	7
#define DIAG_IOCTL_LSM_DEINIT		9
#define DIAG_IOCTL_DCI_REG		23
#define DIAG_IOCTL_DCI_INIT		20
#define DIAG_IOCTL_DCI_DEINIT		21
#define DIAG_IOCTL_DCI_CLEAR_LOGS	28
#define DIAG_IOCTL_DCI_CLEAR_EVENTS	29
#define DIAG_IOCTL_REMOTE_DEV		32
#define DIAG_IOCTL_VOTE_REAL_TIME	33
#define DIAG_IOCTL_GET_REAL_TIME	34
#define DIAG_IOCTL_PERIPHERAL_BUF_CONFIG	35
#define DIAG_IOCTL_PERIPHERAL_BUF_DRAIN		36


#define MEMORY_DEVICE_MODE		2
#define CALLBACK_MODE		6
#define TTY_MODE			8

/*
 * NEXUS-6-ONLY IOCTL
 * Reference: https://github.com/MotorolaMobilityLLC/kernel-msm/blob/kitkat-4.4.4-release-victara/include/linux/diagchar.h
 */
#define DIAG_IOCTL_OPTIMIZED_LOGGING	35
#define DIAG_IOCTL_OPTIMIZED_LOGGING_FLUSH	36


/*
 * Buffering mode
 * Reference: https://android.googlesource.com/kernel/msm.git/+/android-6.0.0_r0.9/include/linux/diagchar.h
 */
#define DIAG_BUFFERING_MODE_STREAMING	0
#define DIAG_BUFFERING_MODE_THRESHOLD	1
#define DIAG_BUFFERING_MODE_CIRCULAR	2
#define DEFAULT_LOW_WM_VAL	15
#define DEFAULT_HIGH_WM_VAL	85
#define NUM_SMD_DATA_CHANNELS 4
#define NUM_SMD_CONTROL_CHANNELS NUM_SMD_DATA_CHANNELS

#define MODEM_DATA		0
#define LAST_PERIPHERAL 3

// size of FIFO pipe between diag_revealer and AndroidDiagMonitor
#define DIAG_FIFO_PIPE_SIZE 128*1024*1024 // 128MB


/*
 * Structures for DCI client registration
 * Reference: https://android.googlesource.com/kernel/msm.git/+/android-6.0.0_r0.9/drivers/char/diag/diag_dci.h
 */
#define DCI_LOG_MASK_SIZE		(16*514)
#define DCI_EVENT_MASK_SIZE		512
struct diag_dci_reg_tbl_t {
	int client_id;
	uint16_t notification_list;
	int signal_type;
	int token;
} __packed;


/*
 * Android 7.0: switch_logging_mode structure
 * Reference: https://android.googlesource.com/kernel/msm.git/+/android-7.1.0_r0.3/drivers/char/diag/diagchar.h
 */
struct diag_logging_mode_param_t {
	uint32_t req_mode;
	uint32_t peripheral_mask;
	uint8_t mode_param;
} __packed;
#define DIAG_CON_APSS		(0x0001)	/* Bit mask for APSS */
#define DIAG_CON_MPSS		(0x0002)	/* Bit mask for MPSS */
#define DIAG_CON_LPASS		(0x0004)	/* Bit mask for LPASS */
#define DIAG_CON_WCNSS		(0x0008)	/* Bit mask for WCNSS */
#define DIAG_CON_SENSORS	(0x0010)	/* Bit mask for Sensors */
#define DIAG_CON_NONE		(0x0000)	/* Bit mask for No SS*/
#define DIAG_CON_ALL		(DIAG_CON_APSS | DIAG_CON_MPSS \
				| DIAG_CON_LPASS | DIAG_CON_WCNSS \
				| DIAG_CON_SENSORS)

/*
 * Structures for ioctl
 * Reference: https://android.googlesource.com/kernel/msm.git/+/android-6.0.0_r0.9/drivers/char/diag/diagchar_core.c
 */

typedef struct {
	char *p;
	size_t len;
} BinaryBuffer;

struct diag_buffering_mode_t {
	uint8_t peripheral;
	uint8_t mode;
	uint8_t high_wm_val;
	uint8_t low_wm_val;
} __packed;

#define DIAG_PROC_DCI			1
#define DIAG_PROC_MEMORY_DEVICE		2

typedef struct {
    char *port;
    int fd;
    struct termios old_t;
} ComData;

struct real_time_vote_t {
	uint16_t proc;
	uint8_t real_time_vote;
};

struct real_time_query_t {
	int real_time;
	int proc;
} __packed;


/*
 * DCI structures
 */
struct diag_dci_client_tbl {
	struct task_struct *client;
	uint16_t list; /* bit mask */
	int signal_type;
	unsigned char dci_log_mask[DCI_LOG_MASK_SIZE];
	unsigned char dci_event_mask[DCI_EVENT_MASK_SIZE];
	unsigned char *dci_data;
	int data_len;
	int total_capacity;
	int dropped_logs;
	int dropped_events;
	int received_logs;
	int received_events;
};

/*
 * Default logging mode and buffer
 * Reference: https://android.googlesource.com/kernel/msm.git/+/android-6.0.0_r0.9/drivers/char/diag/diag_dci.h
 */

char buf_read[BUFFER_SIZE] = {};	// From Haotian: improve reliability
// int mode = CALLBACK_MODE;	// Logging mode
static int mode = MEMORY_DEVICE_MODE;	// logging mode
static uint16_t remote_dev = 0; // MSM (0) or not
int client_id;	// DCI client ID (allocated by diag driver)
int fd; //file descriptor to /dev/diag


// Handle SIGPIPE ERROR
void sigpipe_handler(int signo)
{
  if (signo == SIGPIPE){
  	  // LOGD("received SIGPIPE. Exit elegantly...\n");

    /*
     * Deregister the DCI client
     */

    /*
    int ret;
    ret = ioctl(fd, DIAG_IOCTL_DCI_DEINIT, (char *) &client_id);
    if (ret < 0) {
		LOGD("ioctl DIAG_IOCTL_DCI_DEINIT fails, with ret val = %d\n", ret);
		perror("ioctl DIAG_IOCTL_DCI_DEINIT");
	}
	else
	{
		printf("ioctl DIAG_IOCTL_DCI_DEINIT: ret=%d\n", ret);
	}
	*/

	close(fd);
  }
}

static void print_hex (const char *buf, int len) {
	int i = 0;
	for (i = 0; i < len; i++) {
		printf("%02x ", buf[i]);
		if (((i + 1) % 16) == 0)
			printf("\n");
	}
	if ((i % 16) != 0)
		printf("\n");
}


// Write commands to /dev/diag device.
static int write_commands (int fd, BinaryBuffer *pbuf_write, uint32_t data_type) {
    // data_type: indicate whether CALLBACK_DATA_TYPE or USER_SPACE_DATA_TYPE
	size_t i = 0;
	char *p = pbuf_write->p;

	//Set fd to non-blocking mode
	int flags = fcntl(fd, F_GETFL, 0);
	fcntl(fd, F_SETFL, flags | O_NONBLOCK);

	//Buffer to mask command
	char *send_buf = (char *) malloc(pbuf_write->len + 10);
	if (send_buf == NULL) {
		perror("Error");
		return -1;
	}

    // Metadata for each mask command
	size_t offset = remote_dev ? 8 : 4; //offset of the metadata (4 bytes for MSM, 8 bytes for MDM)
	printf("write_commands: offset=%d remote_dev=%d\n",offset,remote_dev);
	*((int *)send_buf) = htole32(data_type);
	if(remote_dev){
		/*
	 	 * MDM device: should let diag driver know it
	 	 * Reference: diag_get_remote and diagchar_write
	 	 * in https://android.googlesource.com/kernel/msm.git/+/android-6.0.0_r0.9/drivers/char/diag/diagchar_core.c
	 	 */
		*((int *)send_buf+1) =  - MDM;
	}

	while (i < pbuf_write->len) {
		size_t len = 0;
		while (i + len < pbuf_write->len && p[i + len] != 0x7e) len++;
		if (i + len >= pbuf_write->len)
			break;
		len++;
		if (len >= 3) {
			// memcpy(send_buf + 4, p + i, len);
			memcpy(send_buf + offset, p + i, len);
			// LOGD("Writing %d bytes of data\n", len + 4);
			print_hex(send_buf, len + offset);
			// fflush(stdout);
			// int ret = write(fd, (const void *) send_buf, len + 4);
            errno = 0;
            int ret = write(fd, (const void *) send_buf, len + offset);
			printf("write_commands: ret = %d\n",ret);
			if (ret < 0) {
				printf("write_commands error (len=%d, offset=%d, errno=%d): %s\n", len, offset, errno, strerror(errno));
			}
			/*
			 * Read responses after writting each command.
			 * NOTE: This step MUST EXIST. Without it, some phones cannot collect logs for two reasons:
			 *  (1) Ensure every config commands succeeds (otherwise read() will be blocked)
			 *  (2) Clean up the buffer, thus avoiding pollution of later real cellular logs
			 */
			// LOGD("Before read\n");


            // Haotian: don't read response immediately
			int read_len = read(fd, buf_read, sizeof(buf_read));
			if (read_len < 0) {
				printf("write_commands read error: %s\n", strerror(errno));
				return -1;
			} else {
				printf("Reading %d bytes of resp\n", read_len);
				// LOGD("write_commands responses\n");
				// print_hex(buf_read, read_len);
                printf("\n------------------------------------------------------\n");
			}
			// LOGD("After read\n");
		}
		i += len;
	}

	return 0;
}

// Manage the output of logs.
struct LogManagerState {
	const char *dir;
	int log_id;		// ID of the current log.
	FILE *log_fp;	// Point to the current log.
	size_t log_size;	// Number of bytes in the current log.
	size_t log_cut_size;	// Max number of bytes for each log.
};

static void
manager_init_state (struct LogManagerState *pstate, const char *dir, size_t log_cut_size) {
	pstate->dir = dir;
	pstate->log_id = -1;
	pstate->log_fp = NULL;
	pstate->log_size = -1;
	pstate->log_cut_size = log_cut_size;
}

static void
manager_get_log_name (struct LogManagerState *pstate, char *out_buf, size_t out_buf_size) {
	assert(out_buf_size > 0);
	size_t dir_len = strlen(pstate->dir);
	// Remove trailing slashes
	while (dir_len > 0 && pstate->dir[dir_len - 1] == '/') {
		dir_len--;
	}
	assert(dir_len > 0);
	assert(out_buf_size > dir_len + 100);
	strncpy(out_buf, pstate->dir, dir_len);
	snprintf(out_buf + dir_len, out_buf_size - dir_len - 1, "/%d.mi2log", pstate->log_id);
	out_buf[out_buf_size - 1] = '\0';
	return;
}

static int
manager_start_new_log (struct LogManagerState *pstate, int fifo_fd) {
	static char filename[1024] = {};
	int ret;
	if (pstate->log_fp != NULL) {	// end the last log
		assert(pstate->log_id >= 0);
		manager_get_log_name(pstate, filename, sizeof(filename));
		short fifo_msg_type = FIFO_MSG_TYPE_END_LOG_FILE;
		short msg_len = strlen(filename);

		// Wirte msg type to pipe
		ret = write(fifo_fd, &fifo_msg_type, sizeof(short));
		if(ret<0){
			return -1;
		}

		// Write len of filename
		ret = write(fifo_fd, &msg_len, sizeof(short));
		if(ret<0){
			return -1;
		}

		// Write filename of ended log to pipe
		ret = write(fifo_fd, filename, msg_len);
		if(ret<0){
			return -1;
		}

		fclose(pstate->log_fp);
		pstate->log_fp = NULL;
	}
	pstate->log_id = (pstate->log_id < 0? 0: pstate->log_id + 1);
	manager_get_log_name(pstate, filename, sizeof(filename));
	pstate->log_fp = fopen(filename, "wb");
	LOGD("creating %s ...\n", filename);
	if (pstate->log_fp != NULL) {
		// printf("success\n");
		pstate->log_size = 0;
		short fifo_msg_type = FIFO_MSG_TYPE_START_LOG_FILE;
		short msg_len = strlen(filename);
		// Wirte msg type to pipe
		ret = write(fifo_fd, &fifo_msg_type, sizeof(short));
		if(ret<0){
			return -1;
		}

		// Write len of filename
		ret = write(fifo_fd, &msg_len, sizeof(short));
		if(ret<0){
			return -1;
		}

		// Write filename of ended log to pipe
		ret = write(fifo_fd, filename, msg_len);
		if(ret<0){
			return -1;
		}
        // char tmp[4096];
        // sprintf(tmp,"su -c chmod 644 %s\n",filename);
        // system(tmp);
        char tmp[4096];
        sprintf(tmp,"chmod 777 %s\n",filename);
        system(tmp);

	} else {
		return -1;
	}
	return 0;
}

// When appending new data to logs, call this function to maintain states.
// If the size of the current log exceeds log_cut_size, a new log file is created.
static int
manager_append_log (struct LogManagerState *pstate, int fifo_fd, size_t msg_len) {

	if (pstate->log_size + msg_len > pstate->log_cut_size) {
		int ret = manager_start_new_log(pstate, fifo_fd);
		if (ret < 0) {
			return -1;
		}
	}
	pstate->log_size += msg_len;
	return 0;
}

unsigned long wait_reply (ComData *d, char *buf, unsigned long len) {
    int result;
    ssize_t bytes_read;
    size_t decap_len = 0;

    errno = 0;
    // bytes_read = read (d->fd, &buf_read[total], 1);
    printf("wait_reply: call read...\n");
    bytes_read = read (d->fd, &buf_read, sizeof(buf_read));
    printf("wait_reply: bytes_read = %zd\n", bytes_read);
    if (bytes_read < 256) {
        print_hex(buf_read, bytes_read);
    }
    if (bytes_read > 0) {
        printf("first int: %d\n", *((int *)buf_read));
        printf("second int: %d\n", *((int *)(buf_read + 4)));
        if ((*((int *)buf_read) == USER_SPACE_DATA_TYPE) &&
                *((int *)(buf_read + 4)) == MSG_MASKS_TYPE) {
            int num_data = *((int *)(buf_read + 8));
            printf("wait_reply: find bytes beginning with USER_SPACE_DATA_TYPE + MSG_MASKS_TYPE, num_data=%d\n",num_data);

            qcdmbool more = FALSE;
            qcdmbool success;
            size_t used = 0;

            decap_len = 0;
            success = dm_decapsulate_buffer(buf_read + 12,
                    num_data, buf, len, &decap_len, &used, &more);

            if (success && !more) {
                /* Success; we have a packet */
                printf("wait_reply: dm_decapsulate_buffer successfully.\n");
                return decap_len;
            } else {
                if (!success) {
                    printf("wait_reply: dm_decapsulate_buffer failed.\n");
                }
                if (more) {
                    printf("wait_reply: dm_decapsulate_buffer need more.\n");
                }
            }
        }
    } else {
        /* Some error occurred */
        printf("wait_reply: some error occurred: (%d) %s\n", errno, strerror(errno));
        return 0;
    }
    printf("wait_reply: failed to get meaningful response.\n");
    return 0;
}

void com_version_info (ComData *data) {
    int fd = data->fd;
    ComData *d = data;
    char buf[512];
    const char *str;
    int len;
    QcdmResult *result;
    unsigned long reply_len;

    len = qcdm_cmd_version_info_new (buf, sizeof (buf));
    // len = qcdm_cmd_nv_get_mode_pref_new (buf, sizeof (buf), 0);
    assert (len == 4);

    /* Send the command */
    printf("com_version_info: sending command...\n");
    print_hex(buf, len);
    BinaryBuffer buff;
    buff.p = buf;
    buff.len = len;
    printf("write to fd:%d\n", fd);
    write_commands(fd, &buff, USER_SPACE_DATA_TYPE);
    printf("com_version_info: send_command successfully.\n");

    while (1+1 == 2) {
        /* Get a response */
        printf("Try to get a response:\n");
        reply_len = wait_reply (d, buf, sizeof (buf));

        printf("com_version_info reply_len: %lu\n", reply_len);
        /* Parse the response into a result structure */
        result = qcdm_cmd_version_info_result (buf, reply_len, NULL);
        assert (result);

        str = NULL;
        qcdm_result_get_string (result, QCDM_CMD_VERSION_INFO_ITEM_COMP_DATE, &str);
        printf("%s: Compiled Date: %s\n", __func__, str);

        str = NULL;
        qcdm_result_get_string (result, QCDM_CMD_VERSION_INFO_ITEM_COMP_TIME, &str);
        printf ("%s: Compiled Time: %s\n", __func__, str);

        str = NULL;
        qcdm_result_get_string (result, QCDM_CMD_VERSION_INFO_ITEM_RELEASE_DATE, &str);
        printf ("%s: Release Date: %s\n", __func__, str);

        str = NULL;
        qcdm_result_get_string (result, QCDM_CMD_VERSION_INFO_ITEM_RELEASE_TIME, &str);
        printf ("%s: Release Time: %s\n", __func__, str);

        str = NULL;
        qcdm_result_get_string (result, QCDM_CMD_VERSION_INFO_ITEM_MODEL, &str);
        printf ("%s: Model: %s\n", __func__, str);

        qcdm_result_unref (result);
    }
}

void com_nv_cellular_mode(ComData *data) {
    /* do nv related task */
    ComData *d = data;
    int err = QCDM_SUCCESS;
    char buf[512];
    uint8_t pref;
    const char *msg;
    int len;
    QcdmResult *result;
    unsigned long reply_len;

    len = qcdm_cmd_nv_get_mode_pref_new (buf, sizeof (buf), 0);
    assert (len > 0);

    /* Send the command */
    printf("com_nv_task: sending command...\n");
    print_hex(buf, len);
    BinaryBuffer buff;
    buff.p = buf;
    buff.len = len;
    printf("write to fd:%d\n", fd);
    write_commands(d->fd, &buff, CALLBACK_DATA_TYPE);
    printf("com_nv_task: send_command done.\n");

    while (1+1 == 2) {
        usleep(1000000);
        printf("Try to get response.\n");
        /* Get a response */
        reply_len = wait_reply (d, buf, sizeof (buf));

        /* Parse the response into a result structure */
        result = qcdm_cmd_nv_get_mode_pref_result (buf, reply_len, &err);
        if (!result) {
            if (   err == -QCDM_ERROR_NVCMD_FAILED
                || err == -QCDM_ERROR_RESPONSE_BAD_PARAMETER
                || err == -QCDM_ERROR_NV_ERROR_INACTIVE
                || err == -QCDM_ERROR_NV_ERROR_BAD_PARAMETER)
                return;
            if(err != QCDM_SUCCESS) {
                printf("qcdm_cmd_nv_get_mode_pref_result failed.\n");
            }
        }

        err = qcdm_result_get_u8 (result, QCDM_CMD_NV_GET_MODE_PREF_ITEM_MODE_PREF, &pref);
        // g_assert_cmpint (err, ==, QCDM_SUCCESS);
        if(err != QCDM_SUCCESS) {
            printf("qcdm_result_get_u8 failed.\n");
        }

        switch (pref) {
        case QCDM_CMD_NV_MODE_PREF_ITEM_MODE_PREF_DIGITAL:
            msg = "digital";
            break;
        case QCDM_CMD_NV_MODE_PREF_ITEM_MODE_PREF_DIGITAL_ONLY:
            msg = "digital only";
            break;
        case QCDM_CMD_NV_MODE_PREF_ITEM_MODE_PREF_AUTO:
            msg = "automatic";
            break;
        case QCDM_CMD_NV_MODE_PREF_ITEM_MODE_PREF_1X_ONLY:
            msg = "CDMA 1x only";
            break;
        case QCDM_CMD_NV_MODE_PREF_ITEM_MODE_PREF_HDR_ONLY:
            msg = "HDR only";
            break;
        case QCDM_CMD_NV_MODE_PREF_ITEM_MODE_PREF_GPRS_ONLY:
            msg = "GPRS only";
            break;
        case QCDM_CMD_NV_MODE_PREF_ITEM_MODE_PREF_UMTS_ONLY:
            msg = "UMTS only";
            break;
        case QCDM_CMD_NV_MODE_PREF_ITEM_MODE_PREF_GSM_UMTS_ONLY:
            msg = "GSM and UMTS only";
            break;
        case QCDM_CMD_NV_MODE_PREF_ITEM_MODE_PREF_1X_HDR_ONLY:
            msg = "CDMA 1x and HDR only";
            break;
        case QCDM_CMD_NV_MODE_PREF_ITEM_MODE_PREF_LTE_ONLY:
            msg = "LTE only";
            break;
        case QCDM_CMD_NV_MODE_PREF_ITEM_MODE_PREF_GSM_UMTS_LTE_ONLY:
            msg = "GSM/UMTS/LTE only";
            break;
        case QCDM_CMD_NV_MODE_PREF_ITEM_MODE_PREF_1X_HDR_LTE_ONLY:
            msg = "CDMA 1x, HDR, and LTE only";
            break;
        default:
            msg = "unknown";
            break;
        }
        printf("%s: Mode preference: 0x%02X (%s)\n", __func__, pref, msg);

        qcdm_result_unref (result);
    }
}

void com_nv_read_lte_band(ComData *data) {
    /* do nv related task */
    ComData *d = data;
    int err = QCDM_SUCCESS;
    char buf[512];
    uint32_t set1;
    uint8_t set2;
    const char *msg;
    int len;
    QcdmResult *result;
    unsigned long reply_len;

    // len = qcdm_cmd_nv_get_mode_pref_new (buf, sizeof (buf), 0);
    len = qcdm_cmd_nv_get_lte_band_available_new (buf, sizeof (buf));
    assert (len > 0);

    /* Send the command */
    printf("com_nv_lte_band: sending command...\n");
    print_hex(buf, len);
    BinaryBuffer buff;
    buff.p = buf;
    buff.len = len;
    printf("write to fd:%d\n", fd);
    write_commands(d->fd, &buff, CALLBACK_DATA_TYPE);
    printf("com_nv_lte_band: send_command done.\n");

    while (1+1 == 2) {
        usleep(1000000);
        printf("Try to get response.\n");
        /* Get a response */
        reply_len = wait_reply (d, buf, sizeof (buf));

        /* Parse the response into a result structure */
        result = qcdm_cmd_nv_get_lte_band_available_result (buf, reply_len, &err);
        if (!result) {
            if (err == -QCDM_ERROR_NVCMD_FAILED) {
                printf("QCDM_ERROR_NVCMD_FAILED\n");
                return;
            }
            if (err == -QCDM_ERROR_RESPONSE_BAD_PARAMETER) {
                printf("QCDM_ERROR_RESPONSE_BAD_PARAMETER\n");
                return;
            }
            if (err == -QCDM_ERROR_NV_ERROR_INACTIVE) {
                printf("QCDM_ERROR_NV_ERROR_INACTIVE: NV location is not active\n");
                return;
            }
            if (err == -QCDM_ERROR_NV_ERROR_BAD_PARAMETER) {
                printf("QCDM_ERROR_NV_ERROR_BAD_PARAMETER\n");
                return;
            }
            if(err != QCDM_SUCCESS) {
                printf("qcdm_cmd_nv_get_mode_pref_result failed.\n");
            }
        }

        err = qcdm_result_get_u32 (result, QCDM_CMD_NV_GET_LTE_BAND_AVAILABLE_SET1, &set1);
        // g_assert_cmpint (err, ==, QCDM_SUCCESS);
        if(err != QCDM_SUCCESS) {
            printf("qcdm_result_get_u32 failed.\n");
        }
        print_hex((char*)&set1, sizeof(set1));

        qcdm_result_unref (result);
    }
}

void com_nv_write_lte_band(ComData *data) {
    ComData *d = data;
    int err = QCDM_SUCCESS;
    char buf[512];
    uint32_t set1;
    uint8_t set2;
    const char *msg;
    int len;
    QcdmResult *result;
    unsigned long reply_len;

    // len = qcdm_cmd_nv_get_mode_pref_new (buf, sizeof (buf), 0);
    len = qcdm_cmd_nv_set_lte_band_available_new (buf, sizeof (buf), 65759, 0);
    assert (len > 0);

    /* Send the command */
    printf("com_nv_lte_band: sending command...\n");
    print_hex(buf, len);
    BinaryBuffer buff;
    buff.p = buf;
    buff.len = len;
    printf("write to fd:%d\n", fd);
    write_commands(d->fd, &buff, CALLBACK_DATA_TYPE);
    printf("com_nv_lte_band: send_command done.\n");

    while (1+1 == 2) {
        usleep(1000000);
        printf("Try to get response.\n");
        /* Get a response */
        reply_len = wait_reply (d, buf, sizeof (buf));

        /* Parse the response into a result structure */
        result = qcdm_cmd_nv_get_lte_band_available_result (buf, reply_len, &err);
        if (!result) {
            if (err == -QCDM_ERROR_NVCMD_FAILED) {
                printf("QCDM_ERROR_NVCMD_FAILED\n");
                return;
            }
            if (err == -QCDM_ERROR_RESPONSE_BAD_PARAMETER) {
                printf("QCDM_ERROR_RESPONSE_BAD_PARAMETER\n");
                return;
            }
            if (err == -QCDM_ERROR_NV_ERROR_INACTIVE) {
                printf("QCDM_ERROR_NV_ERROR_INACTIVE: NV location is not active\n");
                return;
            }
            if (err == -QCDM_ERROR_NV_ERROR_BAD_PARAMETER) {
                printf("QCDM_ERROR_NV_ERROR_BAD_PARAMETER\n");
                return;
            }
            if(err != QCDM_SUCCESS) {
                printf("qcdm_cmd_nv_get_mode_pref_result failed.\n");
            }
        }

        err = qcdm_result_get_u32 (result, QCDM_CMD_NV_GET_LTE_BAND_AVAILABLE_SET1, &set1);
        // g_assert_cmpint (err, ==, QCDM_SUCCESS);
        if(err != QCDM_SUCCESS) {
            printf("qcdm_result_get_u32 failed.\n");
        }
        print_hex((char*)&set1, sizeof(set1));

        qcdm_result_unref (result);
        break;
    }
}


int main (int argc, char **argv) {

	if (signal(SIGPIPE, sigpipe_handler) == SIG_ERR) {
		LOGW("WARNING: diag_revealer cannot capture SIGPIPE\n");
	}

	fd = open("/dev/diag", O_RDWR|O_LARGEFILE|O_NONBLOCK);
	if (fd < 0) {
		perror("open diag dev");
		return -8002;
	}

	int ret;


	/*
     * EXPERIMENTAL (NEXUS 6 ONLY): 
     * 1. check remote_dev
     * 2. Register a DCI client
     * 3. Send DCI control command
     */
    ret = ioctl(fd, DIAG_IOCTL_REMOTE_DEV, (char *) &remote_dev); 
    if (ret < 0){
	        printf("ioctl DIAG_IOCTL_REMOTE_DEV fails, with ret val = %d\n", ret);
	    	perror("ioctl DIAG_IOCTL_REMOTE_DEV");
	}
	else{
		LOGD("DIAG_IOCTL_REMOTE_DEV remote_dev=%d\n",remote_dev);
	}

	// Register a DCI client
	struct diag_dci_reg_tbl_t dci_client;
	dci_client.client_id = 0;
	dci_client.notification_list = 0;
	dci_client.signal_type = SIGPIPE;
	// dci_client.token = remote_dev;
	dci_client.token = 0;
	ret = ioctl(fd, DIAG_IOCTL_DCI_REG, (char *) &dci_client);
    if (ret < 0){
	        printf("ioctl DIAG_IOCTL_DCI_REG fails, with ret val = %d\n", ret);
	    	perror("ioctl DIAG_IOCTL_DCI_REG");
	}
	else{
		client_id = ret;
		printf("DIAG_IOCTL_DCI_REG client_id=%d\n", client_id);
	}

	// Nexus-6-only logging optimizations
	unsigned int b_optimize = 1;
	ret = ioctl(fd, DIAG_IOCTL_OPTIMIZED_LOGGING, (char *) &b_optimize);
	if (ret < 0){
	        printf("ioctl DIAG_IOCTL_OPTIMIZED_LOGGING fails, with ret val = %d\n", ret);
	    	perror("ioctl DIAG_IOCTL_OPTIMIZED_LOGGING");
	}

    /*
     * EXPERIMENTAL (NEXUS 6 ONLY): configure the buffering mode to circular
     */
    struct diag_buffering_mode_t buffering_mode;
    // buffering_mode.peripheral = remote_dev;
    buffering_mode.peripheral = 0;
    buffering_mode.mode = DIAG_BUFFERING_MODE_STREAMING;
    buffering_mode.high_wm_val = DEFAULT_HIGH_WM_VAL;
    buffering_mode.low_wm_val = DEFAULT_LOW_WM_VAL;

    ret = ioctl(fd, DIAG_IOCTL_PERIPHERAL_BUF_CONFIG, (char *) &buffering_mode);
    if (ret < 0){
        printf("ioctl DIAG_IOCTL_PERIPHERAL_BUF_CONFIG fails, with ret val = %d\n", ret);
    	perror("ioctl DIAG_IOCTL_PERIPHERAL_BUF_CONFIG");
    }

    ///////////////////////////////////////////////////////////////////////////

	/*
	 * Enable logging mode
	 * Reference: https://android.googlesource.com/kernel/msm.git/+/android-6.0.0_r0.9/drivers/char/diag/diagchar_core.c
	 */
	ret = ioctl(fd, DIAG_IOCTL_SWITCH_LOGGING, (char *) &mode);
	if (ret < 0) {
		LOGD("ioctl SWITCH_LOGGING fails, with ret val = %d\n", ret);
		perror("ioctl SWITCH_LOGGING");
		// Yuanjie: the following works for Samsung S5
		ret = ioctl(fd, DIAG_IOCTL_SWITCH_LOGGING, (char *) mode);
		if (ret < 0) {
			LOGD("Alternative ioctl SWITCH_LOGGING fails, with ret val = %d\n", ret);
			perror("Alternative ioctl SWITCH_LOGGING");

			/* Android 7.0 mode
			 * Reference: https://android.googlesource.com/kernel/msm.git/+/android-7.1.0_r0.3/drivers/char/diag/diagchar_core.c
			 */

			struct diag_logging_mode_param_t new_mode;
			new_mode.req_mode = mode;
			new_mode.peripheral_mask = DIAG_CON_ALL;
			new_mode.mode_param = 0;

			ret = ioctl(fd, DIAG_IOCTL_SWITCH_LOGGING, (char *)& new_mode);
			if (ret < 0) {
				LOGD("Android-7.0 ioctl SWITCH_LOGGING fails, with ret val = %d\n", ret);
				perror("Alternative ioctl SWITCH_LOGGING");
			} else {
                printf("Android-7.0 ioctl SWITCH_LOGGING success.\n");
            }

		} else {
            printf("Samsung S5 ioctl SWITCH_LOGGING success.\n");
        }

	}
	else{
		printf("Older way of ioctl succeeds.\n");
	}

    ComData *data = malloc(sizeof(ComData));
    data->fd = fd;
    // com_version_info(data);
    com_nv_cellular_mode(data);
    // com_nv_write_lte_band(data);
    // com_nv_read_lte_band(data);

    free(data);
    close(fd);
    return 0;
}
