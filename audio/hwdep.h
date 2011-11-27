/*
** Copyright 2010, The Android Open-Source Project
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
*/

#define HWDEP_DEVICE            "/dev/snd/hwC0D0"


/* hwdep: ioctl */
#define MC1N2_MAGIC             'N'
#define MC1N2_IOCTL_NR_GET      1
#define MC1N2_IOCTL_NR_SET      2
#define MC1N2_IOCTL_NR_BOTH     3
#define MC1N2_IOCTL_NR_NOTIFY   4

/* haupt ioctls */
#define MC1N2_IOCTL_GET_CTRL    _IOR(MC1N2_MAGIC, MC1N2_IOCTL_NR_GET, struct mc1n2_ctrl_args)
#define MC1N2_IOCTL_SET_CTRL    _IOW(MC1N2_MAGIC, MC1N2_IOCTL_NR_SET, struct mc1n2_ctrl_args)
#define MC1N2_IOCTL_READ_REG    _IOWR(MC1N2_MAGIC, MC1N2_IOCTL_NR_BOTH, struct mc1n2_ctrl_args)
#define MC1N2_IOCTL_NOTIFY      _IOW(MC1N2_MAGIC, MC1N2_IOCTL_NR_NOTIFY, struct mc1n2_ctrl_args)

struct mc1n2_ctrl_args {
	unsigned long dCmd;
	void *pvPrm;
	unsigned long dPrm;
};


/* MC1N2_IOCTL_NOTIFY dCmd definitions */
#define MCDRV_NOTIFY_CALL_START		0x00000000
#define MCDRV_NOTIFY_CALL_STOP		0x00000001
#define MCDRV_NOTIFY_MEDIA_PLAY_START	0x00000002
#define MCDRV_NOTIFY_MEDIA_PLAY_STOP	0x00000003
#define MCDRV_NOTIFY_FM_PLAY_START	0x00000004
#define MCDRV_NOTIFY_FM_PLAY_STOP	0x00000005
#define MCDRV_NOTIFY_BT_SCO_ENABLE	0x00000006
#define MCDRV_NOTIFY_BT_SCO_DISABLE	0x00000007
#define MCDRV_NOTIFY_VOICE_REC_START	0x00000008
#define MCDRV_NOTIFY_VOICE_REC_STOP	0x00000009
#define MCDRV_NOTIFY_HDMI_START		0x0000000A
#define MCDRV_NOTIFY_HDMI_STOP		0x0000000B
#define MCDRV_NOTIFY_2MIC_CALL_START	0x0000000C

#define MC1N2_MODE_IDLE			(0x00)
#define MC1N2_MODE_CALL_ON		(0x1<<0)
#define MC1N2_MODE_FM_ON		(0x1<<1)

int hwdep_open();
int hwdep_close(int fd);
int hwdep_ioctl(int fd, int request, struct mc1n2_ctrl_args *args);

