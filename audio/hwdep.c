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

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/errno.h>
#include <stdio.h>
#include <hwdep.h>

#define LOG_NDEBUG 0
#define LOG_TAG "HardwareDependant"
#include <utils/Log.h>

int hwdep_open()
{
    int fd, ret;

	LOGD("hwdep_open() called\n");
    fd = open(HWDEP_DEVICE, O_RDWR);

    if(fd < 0) 
    {
        LOGE("hwdep_open(): failed on opening /dev/snd/hwC0D0\n");
        return -EBUSY;
    } 
    else 
    {
        LOGV("hwdep_open(): successfully opened /dev/snd/hwC0D0\n");
        return fd;
    }

    return -1;
}

int hwdep_close(int fd)
{
    close(fd);
    return 0;
}

int hwdep_ioctl(int fd, int request, struct mc1n2_ctrl_args *args)
{
    int ret;

    ret = ioctl(fd, request, args);

    if(ret < 0) 
    {
        LOGE("hwdep_ioctl(): ioctl %d failed\n", request);
    } 
    else 
    {
        LOGV("hwdep_ioctl(): ioctl %d success\n", request);
    }

    return ret;
}


