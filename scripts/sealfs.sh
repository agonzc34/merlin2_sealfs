#!/bin/bash
mkdir $SEALFS_BACK $ROSSEALFS_MOUNTPOINT $SEALFS_KEY_PATH
sudo insmod $SEALFS_PATH/module/sealfs.ko

$SEALFS_PATH/tools/prep $SEALFS_BACK/.SEALFS.LOG $SEALFS_KEY_PATH/k1 $SEALFS_KEY_PATH/k2 $SEALFS_SIZE
sudo mount -o kpath=$SEALFS_KEY_PATH/k1,nratchet=$SEALFS_NRATCHET -t sealfs $SEALFS_BACK $ROSSEALFS_MOUNTPOINT

echo 'Module loaded and mounted'
exit 0