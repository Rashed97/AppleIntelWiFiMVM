//
//  FirmwareLoader.cpp
//
//  Copyright © 2016 Aaron Mulder. All rights reserved.
//
#include "AppleIntelWiFiMVM.h"

#define DEBUG_SLEEP 1000
#define AlwaysLog(args...) do {IOLog(MYNAME": " args); IOSleep(DEBUG_SLEEP);}while(0)

// static
void AppleIntelWiFiMVM::firmwareLoadComplete( OSKextRequestTag requestTag, OSReturn result, const void *resourceData, uint32_t resourceDataLength, void *context) {
    AlwaysLog("Received firmware load callback...\n");
    FirmwareLoadProgress *progress = (FirmwareLoadProgress *)context;
    AlwaysLog("Attempting firmware lock to prepare data\n");
    IOLockLock(progress->driver->firmwareLoadLock);
    if(result == kOSReturnSuccess) {
        progress->firmwareData = OSData::withBytes(resourceData, resourceDataLength);
    } else
        IOLog("%s firmwareLoadComplete FAILURE: %08x.\n", MYNAME, result);
    AlwaysLog("Releasing firmware lock\n");
    IOLockUnlock(progress->driver->firmwareLoadLock);
    AlwaysLog("Waking sleepers...\n");
    IOLockWakeup(progress->driver->firmwareLoadLock, progress->driver, true);
    AlwaysLog("Callback complete\n");
}

OSData* AppleIntelWiFiMVM::loadFirmwareSync(struct iwl_drv *drv) {
    AlwaysLog("loadFirmwareSync");
    const char *name_pre = drv->cfg->fw_name_pre;
    char tag[8];
    
    IOLockLock(firmwareLoadLock);
    FirmwareLoadProgress progress = {.driver = this, .firmwareData = NULL};
    drv->fw_index = drv->cfg->ucode_api_max;
    while(true) {
        snprintf(tag, 8, "%d", drv->fw_index);
        /*
         * Starting 8000B - FW name format has changed. This overwrites the
         * previous name and uses the new format.
         */
//        if (drv->trans->cfg->device_family == IWL_DEVICE_FAMILY_8000) {  TODO:FIXME
        if (name_pre[8] == '8') {
            char rev_step = 'A' + 2;// CSR_HW_REV_STEP(drv->trans->hw_rev);  TODO:FIXME replace 2
            
            snprintf(drv->firmware_name, sizeof(drv->firmware_name),
                     "%s%c-%s.ucode", name_pre, rev_step, tag);
        } else {
            snprintf(drv->firmware_name, sizeof(drv->firmware_name), "%s%s.ucode",
                     name_pre, tag);
        }
        
        AlwaysLog("%s::Requesting firmware load for [%s]...\n", MYNAME, drv->firmware_name);
        OSReturn ret = OSKextRequestResource(OSKextGetCurrentIdentifier(),
                                             drv->firmware_name,
                                             firmwareLoadComplete,
                                             &progress,
                                             NULL);
        if(ret != kOSReturnSuccess) {
            AlwaysLog("%s Unable to load firmware file %08x\n", MYNAME, ret);
            IOLockUnlock(firmwareLoadLock);
            return NULL;
        }
        AlwaysLog("Waiting for firmware load...\n");
        IOLockSleep(firmwareLoadLock, this, THREAD_INTERRUPTIBLE);
        AlwaysLog("Woke up after waiting for firmware lock...\n");
        if(progress.firmwareData) break;
        if(--drv->fw_index < drv->cfg->ucode_api_min) break;
        if(drv->fw_index < 0) break;
    }
    IOLockUnlock(firmwareLoadLock);

    AlwaysLog("%s::%s firmware file %s\n", MYNAME, progress.firmwareData ? "LOADED" : "FAILED", drv->firmware_name);
    
    return progress.firmwareData;
}

bool AppleIntelWiFiMVM::startupLoadFirmware() {
    AlwaysLog("Attempting to load Firmware...\n");

    OSData *fwData = loadFirmwareSync(driver);
    if(!fwData)
        return false;

    parser = new FirmwareParser();
    AlwaysLog("Attempting to parse Firmware...\n");
    bool result = parser->processFirmwareData(fwData, driver);
    if(result) {
        AlwaysLog("%s Parsed firmware!\n", MYNAME);
        IOLockLock(firmwareLoadLock);
        firmwareLoaded = true;
        IOLockUnlock(firmwareLoadLock);
    }
    else AlwaysLog("%s Firmware parse failed :(\n", MYNAME);
    RELEASE(fwData)

    return result;
    
//err_fw:
//#ifdef CONFIG_IWLWIFI_DEBUGFS
//err_free_dbgfs:
//    debugfs_remove_recursive(drv->dbgfs_drv);
//err_free_drv:
//#endif
//    IOFree(drv, sizeof(*drv));
//err:
//    return NULL;
}

void AppleIntelWiFiMVM::shutdownStopFirmware() {
    if(!driver) return;
    //    wait_for_completion(&drv->request_firmware_complete);
    IOLockLock(firmwareLoadLock);
    if(!firmwareLoaded) {
        AlwaysLog("%s Stopping before firmware is loaded: waiting\n", MYNAME);
        IOLockSleep(firmwareLoadLock, this, THREAD_INTERRUPTIBLE);
        AlwaysLog("%s Load complete; now stopping\n", MYNAME);
    }
    IOLockUnlock(firmwareLoadLock);
    
//    _iwl_op_mode_stop(drv);
    
    if(parser) {
        parser->releaseFirmware(driver);
        RELEASE(parser)
    }

//    mutex_lock(&iwlwifi_opmode_table_mtx);
//    /*
//     * List is empty (this item wasn't added)
//     * when firmware loading failed -- in that
//     * case we can't remove it from any list.
//     */
//    if (!list_empty(&drv->list))
//        list_del(&drv->list);
//    mutex_unlock(&iwlwifi_opmode_table_mtx);
    
//#ifdef CONFIG_IWLWIFI_DEBUGFS
//    debugfs_remove_recursive(drv->dbgfs_drv);
//#endif
}

