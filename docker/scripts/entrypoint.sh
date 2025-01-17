#!/bin/bash
# Copyright (C) 2025, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its 
#    contributors may be used to endorse or promote products derived 
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Check if HOST_USER_UID and HOST_USER_GID are set
if [ -n "$HOST_USER_UID" ] && [ -n "$HOST_USER_GID" ]; then

    if [ ! $(getent group ${HOST_USER_GID}) ]; then
    echo "Creating non-root container '${USER}' for host user uid=${HOST_USER_UID}:gid=${HOST_USER_GID}"
    groupadd --gid ${HOST_USER_GID} ${USER} &>/dev/null
    fi

    if [ ! $(getent passwd ${HOST_USER_UID}) ]; then
    echo "User with UID ${HOST_USER_UID} does not exist. Creating user '${USER}'"
    useradd --no-log-init --uid ${HOST_USER_UID} --gid ${HOST_USER_GID} -m ${USER} &>/dev/null
    fi

    source /opt/ros/${ROS_DISTRO}/setup.bash

    # Source ROS workspace if exists
    if [[ ! -z "${ROS_WS}" ]]; then
    source ${ROS_WS}/install/setup.bash
    fi

    # Execute command
    if [ $# -eq 0 ]; then
        exec gosu ${USER} ros2 launch nanosaur_${SIMULATOR_PACKAGE} ${SIMULATOR_LAUNCH_FILE:-nanosaur_bridge.launch.py}
    else
        exec gosu ${USER} "$@"
    fi
fi

# Execute command as root if HOST_USER_UID and HOST_USER_GID are not set
echo "Running as root user since HOST_USER_UID and HOST_USER_GID are not set"
exec "$@"
