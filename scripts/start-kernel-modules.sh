if lsmod | grep -q remoteproc
then
    echo "Kernel modules required for DSP are currently runnig. Stopping them...";
    rmmod rpmsg_proto;
    rmmod virtio_rpmsg_bus;
    rmmod da8xx_remoteproc;
    rmmod remoteproc
fi

echo "Starting modules...";
depmod -a
modprobe remoteproc
modprobe da8xx_remoteproc da8xx_fw_name=${1:-"server_dsp.xe674"}
modprobe virtio_rpmsg_bus
modprobe rpmsg_proto
