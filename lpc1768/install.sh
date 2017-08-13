#!/bin/bash

MOUNTPOINT_DIR="/media/${USER}/MBED"
IMAGE_FILENAME="$(grep 'PROJECT := ' Makefile | awk '{print $3;}').bin"
IMAGE_SOURCE_PATH="./BUILD/${IMAGE_FILENAME}"
IMAGE_DESTINATION_PATH="${MOUNTPOINT_DIR}/${IMAGE_FILENAME}"

echo "MOUNTPOINT_DIR=${MOUNTPOINT_DIR}"
echo "IMAGE_SOURCE_PATH=${IMAGE_SOURCE_PATH}"
echo "IMAGE_DESTINATION_PATH=${IMAGE_DESTINATION_PATH}"
echo ""

if [ -f "${IMAGE_SOURCE_PATH}" ]; then
   if [ -d "${MOUNTPOINT_DIR}" ]; then
      if [ -f "${IMAGE_DESTINATION_PATH}" ]; then
         echo "Deleting ${IMAGE_DESTINATION_PATH}"
         rm "${IMAGE_DESTINATION_PATH}"
      fi
      echo "Copying ${IMAGE_SOURCE_PATH} --> ${IMAGE_DESTINATION_PATH}"
      cp "${IMAGE_SOURCE_PATH}" "${IMAGE_DESTINATION_PATH}"

      echo "Unmounting ${MOUNTPOINT_DIR}"
      sync
      umount "${MOUNTPOINT_DIR}"
   else
      echo "${MOUNTPOINT_DIR} does not exist"
   fi
else
   echo "${IMAGE_SOURCE_PATH} does not exist"
fi
