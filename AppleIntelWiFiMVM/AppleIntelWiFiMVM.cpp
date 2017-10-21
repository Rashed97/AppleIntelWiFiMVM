#include "AppleIntelWiFiMVM.h"

extern "C" {
//#include "linux/linux-porting.h"
#include "device-list.h"
#include "iwl-csr.h"
#include "iwl-trans.h"
#include "internal.h"
}

#define AlwaysLog(args...) do {IOLog(MYNAME": " args); IOSleep(1000);}while(0)

#define super IOService
OSDefineMetaClassAndStructors(AppleIntelWiFiMVM, IOService);

// ------------------------ IOService Methods ----------------------------

bool AppleIntelWiFiMVM::init(OSDictionary *dict) {
    bool res = super::init(dict);
    AlwaysLog("init\n");
    kprintf("%s::init\n",MYNAME);
    OSCollectionIterator *it = OSCollectionIterator::withCollection(dict);
    OSObject *key, *value;
    OSString *ks, *vs;
    //va_list foo;
    kprintf("%s::DICT total count %d\n",MYNAME, dict->getCount());
    AlwaysLog("DICT total count %d\n", dict->getCount());
    while ((key = it->getNextObject())) {
        ks = OSDynamicCast(OSString, key);
        if(!ks) {
            AlwaysLog("DICT Unrecognized key class\n");}
        else {
            value = dict->getObject(ks);
            vs = OSDynamicCast(OSString, value);
            if (!vs) AlwaysLog("DICT %s = (not a string)\n", ks->getCStringNoCopy());
            else AlwaysLog("DICT %s = %s\n", ks->getCStringNoCopy(), vs->getCStringNoCopy());
        }
    }
    RELEASE(it)
    return res;
}

bool AppleIntelWiFiMVM::start(IOService* provider) {
    struct iwl_trans *pcieTransport;
    const struct iwl_cfg *card;
    kprintf("%s::start\n",MYNAME);
    AlwaysLog("start\n");
    if(!super::start(provider)) {
        AlwaysLog("Super start failed\n");
        return false;
    }
    IOSleep(2500);
    // Ensure we have a PCI device provider
    pciDevice = OSDynamicCast(IOPCIDevice, provider);
    if(!pciDevice) {
        kprintf("%s::Provider not a PCIDevice\n",MYNAME);
        IOLog("Provider not a PCIDevice\n");
        shutdownFreePCIeTransport(pcieTransport);
        return false;
    }
    pciDevice->retain();

    // Startup Process: 1
    card = startupIdentifyWiFiCard();
    if(!card) {
        kprintf("%s::Unable to find or configure WiFi hardware.\n",MYNAME);
        AlwaysLog("Unable to find or configure WiFi hardware.\n");
        return false;
    }
    kprintf("%s::Loading for device %s\n",MYNAME, card->name);
    AlwaysLog("loading for device %s\n", card->name);
    IOSleep(2500);
    // Startup Process: 2
    AlwaysLog("Starting up process 2");
    pcieTransport = startupCreatePCIeTransport(card);
    AlwaysLog("Initialized PCIe Transport");
    IOSleep(2500);
    if(!pcieTransport) {
        kprintf("%s::Unable to initialize PCIe transport.\n",MYNAME);
        AlwaysLog("Unable to initialize PCIe transport.\n");
        return false;
    }
    IOSleep(2500);
    // Startup Process: 3
    AlwaysLog("Starting up process 3");
    if(!startupCreateDriver(card, pcieTransport)) {
        kprintf("%s::Unable to initialize driver.\n",MYNAME);
        AlwaysLog("Unable to initialize driver.\n");
        shutdownFreeDriver();
        return false;
    }
    IOSleep(2500);
    AlwaysLog("Creating IOLockAllocs");
    // Create locks for synchronization TODO: move me
    firmwareLoadLock = IOLockAlloc();
    if (!firmwareLoadLock) {
        kprintf("%s::Unable to allocate firmware load lock.\n",MYNAME);
        AlwaysLog("Unable to allocate firmware load lock\n");
        shutdownFreeDriver();
        return false;
    }
    IOSleep(2500);
    AlwaysLog("Startup process 4");
    // Startup Process: 4
    if(!startupLoadFirmware()) {
        kprintf("%s::Unable to load firmware.\n",MYNAME);
        AlwaysLog("Unable to load firmware.\n");
        shutdownFreeDriver();
        return false;
    }

    if(driver){
        kprintf("%s::Loading Intel Firmware named %s", MYNAME, driver->firmware_name);
        
        AlwaysLog("Loading Intel Firmware named %s", driver->firmware_name);
        
        setProperty("WiFi card name", OSString::withCString(card->name));
        setProperty("Intel Firmware version", driver->fw.fw_version);
    }else{
        AlwaysLog("WARNING! if(driver) equals false!");
    }
    
    
    IOSleep(2500);
    
    registerService();

    // Startup Process: 5
    AlwaysLog("Startup process 5");
    if(!startupHardware(pciDevice, pcieTransport)){
        AlwaysLog("Error while enabling hardware!");
        shutdownFreeDriver();
        return false;
    }
    
    IOSleep(2500);
    AlwaysLog("Done starting up!");
    
    return true;

/*failAfterDriver:
    shutdownFreeDriver();
failAfterPCIe:
    shutdownFreePCIeTransport(pcieTransport);
failBeforePCIe:
    return false;
 */
}

void AppleIntelWiFiMVM::stop(IOService* provider) {
    kprintf("%s::stop\n",MYNAME);
    AlwaysLog("stop\n");
    shutdownStopFirmware();
    if (firmwareLoadLock)
    {
        IOLockFree(firmwareLoadLock);
        firmwareLoadLock = NULL;
    }
    struct iwl_trans *trans = shutdownFreeDriver();
    shutdownFreePCIeTransport(trans);

    super::stop(provider);
}

void AppleIntelWiFiMVM::free() {
    kprintf("%s::free\n",MYNAME);
    AlwaysLog("free\n");
    RELEASE(pciDevice);
    struct iwl_trans *trans = shutdownFreeDriver();
    shutdownFreePCIeTransport(trans);
    super::free();
}

const struct iwl_cfg *AppleIntelWiFiMVM::startupIdentifyWiFiCard() {
    UInt32 i;
    const struct iwl_cfg *result = NULL;

    UInt16 vendor = pciDevice->configRead16(kIOPCIConfigVendorID);
    UInt16 device = pciDevice->configRead16(kIOPCIConfigDeviceID);
    UInt16 subsystem_vendor = pciDevice->configRead16(kIOPCIConfigSubSystemVendorID);
    UInt16 subsystem_device = pciDevice->configRead16(kIOPCIConfigSubSystemID);
    UInt8 revision = pciDevice->configRead8(kIOPCIConfigRevisionID);
//    vendor = 0x8086;
//    subsystem_vendor = 0x8086;
    // Broadwell NUC 7265
//    device = 0x095a;
//    subsystem_device = 0x9510;
    // Skylake NUC 8260
//    device = 0x24F3;
//    subsystem_device = 0x9010;
    // 7260
//    device = 0x08B1;
//    subsystem_device = 0x4070;
    // 3160
//    device = 0x08B3;
//    subsystem_device = 0x0070;
    // 3165 uses 7165D firmware
//    device = 0x3165;
//    subsystem_device = 0x4010;
    // 4165 uses 8260 firmware above, not retested here

    if(vendor != 0x8086 || subsystem_vendor != 0x8086) {
        kprintf("%s::Unrecognized vendor/sub-vendor ID %#06x/%#06x; expecting 0x8086 for both; cannot load driver.\n", MYNAME, vendor, subsystem_vendor);
        AlwaysLog("Unrecognized vendor/sub-vendor ID %#06x/%#06x; expecting 0x8086 for both; cannot load driver.\n", vendor, subsystem_vendor);
        return NULL;
    }

    kprintf("%s::Vendor %#06x Device %#06x SubVendor %#06x SubDevice %#06x Revision %#04x\n", MYNAME,vendor, device, subsystem_vendor, subsystem_device, revision);
    AlwaysLog("Vendor %#06x Device %#06x SubVendor %#06x SubDevice %#06x Revision %#04x\n", vendor, device, subsystem_vendor, subsystem_device, revision);

    // Investigation: memory regions
    pciDevice->setMemoryEnable(true);
    kprintf("%s::LISTING %u PCI MEMORY REGION(S):\n", MYNAME, pciDevice->getDeviceMemoryCount());
    AlwaysLog("LISTING %u PCI MEMORY REGION(S):\n", pciDevice->getDeviceMemoryCount());
    for (i = 0; i < pciDevice->getDeviceMemoryCount(); i++) {
        IODeviceMemory* memoryDesc = pciDevice->getDeviceMemoryWithIndex(i);
        if (!memoryDesc) continue;
        kprintf("%s::%u: length=%llu bytes\n", MYNAME,i, memoryDesc->getLength());
        AlwaysLog("%u: length=%llu bytes\n", i, memoryDesc->getLength());
    }

    for(i=0; i<sizeof(iwl_hw_card_ids) / sizeof(pci_device_id); i++) {
        if(iwl_hw_card_ids[i].device == device && iwl_hw_card_ids[i].subdevice == subsystem_device) {
            result = (iwl_cfg *) iwl_hw_card_ids[i].driver_data;
            break;
        }
    }
    if(!result) {
        kprintf("%s::Card has the right device ID %#06x but unmatched sub-device ID %#06x; cannot load driver.\n", MYNAME,device, subsystem_device);
        AlwaysLog("Card has the right device ID %#06x but unmatched sub-device ID %#06x; cannot load driver.\n", device, subsystem_device);
        return NULL;
    }
    
    return result;
}

struct iwl_trans *allocatePCIeTransport(const struct iwl_cfg *cfg, IOPCIDevice *pciDevice);

struct iwl_trans *AppleIntelWiFiMVM::startupCreatePCIeTransport(const struct iwl_cfg *cfg) {
    const struct iwl_cfg *cfg_7265d __maybe_unused = NULL;
    struct iwl_trans *iwl_trans;

    // STEP 1: configure the PCIe Transport
    iwl_trans = allocatePCIeTransport(cfg, pciDevice);
    if(!iwl_trans) return NULL;

    // STEP 2: double-check the configuration data based on the hardware revision in the PCIe Transport
    /*
     * special-case 7265D, it has the same PCI IDs.
     *
     * Note that because we already pass the cfg to the transport above,
     * all the parameters that the transport uses must, until that is
     * changed, be identical to the ones in the 7265D configuration.
     */
    if (cfg == &iwl7265_2ac_cfg)
        cfg_7265d = &iwl7265d_2ac_cfg;
    else if (cfg == &iwl7265_2n_cfg)
        cfg_7265d = &iwl7265d_2n_cfg;
    else if (cfg == &iwl7265_n_cfg)
        cfg_7265d = &iwl7265d_n_cfg;
    if (cfg_7265d &&
            (iwl_trans->hw_rev & CSR_HW_REV_TYPE_MSK) == CSR_HW_REV_TYPE_7265D) {
        iwl_trans->cfg = cfg_7265d;
    }
    
    return iwl_trans;
}

struct iwl_trans *allocatePCIeTransport(const struct iwl_cfg *cfg, IOPCIDevice *pciDevice) {
    // From pcie/trans.c iwl_pcie_trans_alloc
    struct iwl_trans_pcie *trans_pcie = NULL;
    struct iwl_trans *trans = NULL;
    // u16 pci_cmd;
    // int ret;
    trans = iwl_trans_alloc(sizeof(struct iwl_trans_pcie),
            NULL, cfg, NULL, 0);  // TODO: passing "device" of NULL, "ops" of NULL vs &trans_ops_pcie
    if (!trans)
        return (struct iwl_trans *) ERR_PTR(-ENOMEM);

    trans->max_skb_frags = IWL_PCIE_MAX_FRAGS;
    trans->hw_id = pciDevice->configRead16(kIOPCIConfigDeviceID);
    trans->hw_rev = pciDevice->configRead16(kIOPCIConfigRevisionID);
    
    trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);

    trans_pcie->trans = trans;
    
#if DISABLED_CODE
    spin_lock_init(&trans_pcie->irq_lock);
    spin_lock_init(&trans_pcie->reg_lock);
    spin_lock_init(&trans_pcie->ref_lock);
    mutex_init(&trans_pcie->mutex);
    init_waitqueue_head(&trans_pcie->ucode_write_waitq);

    ret = pci_enable_device(pdev);
    if (ret)
        goto out_no_pci;

    if (!cfg->base_params->pcie_l1_allowed) {
        /*
         * W/A - seems to solve weird behavior. We need to remove this
         * if we don't want to stay in L1 all the time. This wastes a
         * lot of power.
         */
        pci_disable_link_state(pdev, PCIE_LINK_STATE_L0S |
                PCIE_LINK_STATE_L1 |
                PCIE_LINK_STATE_CLKPM);
    }

    pci_set_master(pdev);

    ret = pci_set_dma_mask(pdev, DMA_BIT_MASK(36));
    if (!ret)
        ret = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(36));
    if (ret) {
        ret = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
        if (!ret)
            ret = pci_set_consistent_dma_mask(pdev,
                    DMA_BIT_MASK(32));
        /* both attempts failed: */
        if (ret) {
            dev_err(&pdev->dev, "No suitable DMA available\n");
            goto out_pci_disable_device;
        }
    }

    ret = pci_request_regions(pdev, DRV_NAME);
    if (ret) {
        dev_err(&pdev->dev, "pci_request_regions failed\n");
        goto out_pci_disable_device;
    }

    trans_pcie->hw_base = pci_ioremap_bar(pdev, 0);
    if (!trans_pcie->hw_base) {
        dev_err(&pdev->dev, "pci_ioremap_bar failed\n");
        ret = -ENODEV;
        goto out_pci_release_regions;
    }

    /* We disable the RETRY_TIMEOUT register (0x41) to keep
     * PCI Tx retries from interfering with C3 CPU state */
    pci_write_config_byte(pdev, PCI_CFG_RETRY_TIMEOUT, 0x00);

    trans->dev = &pdev->dev;
    trans_pcie->pci_dev = pdev;
    iwl_disable_interrupts(trans);

    ret = pci_enable_msi(pdev);
    if (ret) {
        dev_err(&pdev->dev, "pci_enable_msi failed(0X%x)\n", ret);
        /* enable rfkill interrupt: hw bug w/a */
        pci_read_config_word(pdev, PCI_COMMAND, &pci_cmd);
        if (pci_cmd & PCI_COMMAND_INTX_DISABLE) {
            pci_cmd &= ~PCI_COMMAND_INTX_DISABLE;
            pci_write_config_word(pdev, PCI_COMMAND, pci_cmd);
        }
    }

    trans->hw_rev = iwl_read32(trans, CSR_HW_REV);
    /*
     * In the 8000 HW family the format of the 4 bytes of CSR_HW_REV have
     * changed, and now the revision step also includes bit 0-1 (no more
     * "dash" value). To keep hw_rev backwards compatible - we'll store it
     * in the old format.
     */
    if (trans->cfg->device_family == IWL_DEVICE_FAMILY_8000) {
        unsigned long flags;

        trans->hw_rev = (trans->hw_rev & 0xfff0) |
                (CSR_HW_REV_STEP(trans->hw_rev << 2) << 2);

        ret = iwl_pcie_prepare_card_hw(trans);
        if (ret) {
            IWL_WARN(trans, "Exit HW not ready\n");
            goto out_pci_disable_msi;
        }

        /*
         * in-order to recognize C step driver should read chip version
         * id located at the AUX bus MISC address space.
         */
        iwl_set_bit(trans, CSR_GP_CNTRL,
                CSR_GP_CNTRL_REG_FLAG_INIT_DONE);
        udelay(2);

        ret = iwl_poll_bit(trans, CSR_GP_CNTRL,
                CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY,
                CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY,
                25000);
        if (ret < 0) {
            IWL_DEBUG_INFO(trans, "Failed to wake up the nic\n");
            goto out_pci_disable_msi;
        }

        if (iwl_trans_grab_nic_access(trans, false, &flags)) {
            u32 hw_step;

            hw_step = __iwl_read_prph(trans, WFPM_CTRL_REG);
            hw_step |= ENABLE_WFPM;
            __iwl_write_prph(trans, WFPM_CTRL_REG, hw_step);
            hw_step = __iwl_read_prph(trans, AUX_MISC_REG);
            hw_step = (hw_step >> HW_STEP_LOCATION_BITS) & 0xF;
            if (hw_step == 0x3)
                trans->hw_rev = (trans->hw_rev & 0xFFFFFFF3) |
                        (SILICON_C_STEP << 2);
            iwl_trans_release_nic_access(trans, &flags);
        }
    }

    trans->hw_id = (pdev->device << 16) + pdev->subsystem_device;
    snprintf(trans->hw_id_str, sizeof(trans->hw_id_str),
            "PCI ID: 0x%04X:0x%04X", pdev->device, pdev->subsystem_device);

    /* Initialize the wait queue for commands */
    init_waitqueue_head(&trans_pcie->wait_command_queue);

    ret = iwl_pcie_alloc_ict(trans);
    if (ret)
        goto out_pci_disable_msi;

    ret = request_threaded_irq(pdev->irq, iwl_pcie_isr,
            iwl_pcie_irq_handler,
            IRQF_SHARED, DRV_NAME, trans);
    if (ret) {
        IWL_ERR(trans, "Error allocating IRQ %d\n", pdev->irq);
        goto out_free_ict;
    }

    trans_pcie->inta_mask = CSR_INI_SET_MASK;
    trans->d0i3_mode = IWL_D0I3_MODE_ON_SUSPEND;
#endif //DISABLED_CODE
    return trans;
#if DISABLED_CODE
    out_free_ict:
    iwl_pcie_free_ict(trans);
    out_pci_disable_msi:
    pci_disable_msi(pdev);
    out_pci_release_regions:
    pci_release_regions(pdev);
    out_pci_disable_device:
    pci_disable_device(pdev);
    out_no_pci:
    iwl_trans_free(trans);
    return ERR_PTR(ret);
#endif // DISABLED_CODE
}

bool AppleIntelWiFiMVM::startupCreateDriver(const struct iwl_cfg *cfg, struct iwl_trans *trans) {
    driver = (struct iwl_drv*)IOMalloc(sizeof(*driver));
    if (!driver) {
        kprintf("%s::Could not allocate memory for the driver!", MYNAME);
        AlwaysLog("Could not allocate memory for the driver!");
        return false;
    }
    AlwaysLog("Allocated memory for driver");
    driver->trans = trans;
    if(trans)
        driver->dev = trans->dev;
    driver->cfg = cfg;

    AlwaysLog("Allocated struct properties");
#if DISABLED_CODE // TODO: need to initialize the request_firmware_complete
    AlwaysLog("Initialize the request_firmware_complete (???)");
    init_completion(&drv->request_firmware_complete);
    INIT_LIST_HEAD(&drv->list);
#endif
    
#ifdef CONFIG_IWLWIFI_DEBUGFS
    AlwaysLog("Creating debug entries");
    /* Create the device debugfs entries. */
    driver->dbgfs_drv = debugfs_create_dir(dev_name(trans->dev),
                                        iwl_dbgfs_root);

    if (!driver->dbgfs_drv) {
        IWL_ERR(driver, "failed to create debugfs directory\n");
        ret = -ENOMEM;
        goto err_free_drv;
    }

    /* Create transport layer debugfs dir */
    driver->trans->dbgfs_dir = debugfs_create_dir("trans", driver->dbgfs_drv);

    if (!driver->trans->dbgfs_dir) {
        IWL_ERR(driver, "failed to create transport debugfs directory\n");
        ret = -ENOMEM;
        goto err_free_dbgfs;
    }
#endif
    
    return true;
}


#define PCI_COMMAND 0x04
#define PCI_CFG_RETRY_TIMEOUT 0x41
#define PCI_COMMAND_INTX_DISABLE 0x400

bool AppleIntelWiFiMVM::startupHardware(IOPCIDevice *pciDevice, iwl_trans *trans){
    
    pciDevice->setMemoryEnable(true);
    
    // Construct a 36-Bit DMA
    
    IOBufferMemoryDescriptor *bmd = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, kIOMemoryPhysicallyContiguous,256*1024, 0x00000000FFFFF000ULL);
    
    if(bmd != NULL){
        generateDMAAddresses(bmd);
    }else{
        AlwaysLog("IOBufferMemoryDescriptor::inTaskWithPhysicalMask failed\n");
    }
    
    fLowMemory = bmd;
    
    if(pciDevice->setBusMasterEnable(true)){
        AlwaysLog("Enabled Bus Master!");
    }else{
        return false;
    }
    if(pciDevice->hasPCIPowerManagement()){
        AlwaysLog("Found PCIPower management, enabling");
        if(pciDevice->enablePCIPowerManagement() == kIOReturnSuccess){
            AlwaysLog("Enabled pci power management");
        }
    }
    if(pciDevice->setIOEnable((true))){
        AlwaysLog("Was previously enabled");
    }
    
    /* We disable the RETRY_TIMEOUT register (0x41) to keep
     * PCI Tx retries from interfering with C3 CPU state */
    //pci_write_config_byte(pdev, PCI_CFG_RETRY_TIMEOUT, 0x00);
    pciDevice->configWrite8(PCI_CFG_RETRY_TIMEOUT, 0x00);
    
    /* enable rfkill interrupt: hw bug w/a */
    //pci_read_config_word(pdev, PCI_COMMAND, &pci_cmd);
    // PCI_COMMAND (0x04)
    u16 pci_cmd;
    pci_cmd = pciDevice->configRead16(PCI_COMMAND);
    if (pci_cmd & PCI_COMMAND_INTX_DISABLE) {
        pci_cmd &= ~PCI_COMMAND_INTX_DISABLE;
        pciDevice->configWrite16(PCI_COMMAND, pci_cmd);
    }
    
    return true;
}

struct iwl_trans *AppleIntelWiFiMVM::shutdownFreeDriver() {
    if(!driver) return NULL;
    struct iwl_trans *trans = driver->trans;
    IOFree(driver, sizeof(*driver));
    driver = NULL;
    return trans;
}

void AppleIntelWiFiMVM::shutdownFreePCIeTransport(struct iwl_trans *trans) {
    if(!trans) return;
    // From pcie/trans.c iwl_trans_pcie_free
    struct iwl_trans_pcie *trans_pcie = IWL_TRANS_GET_PCIE_TRANS(trans);
    
    if(!trans_pcie){
        kprintf("%s::Unable to obtain PCIe Transport object",MYNAME);
    }
#if DISABLED_CODE // TODO: later
    synchronize_irq(trans_pcie->pci_dev->irq);

    iwl_pcie_tx_free(trans);
    iwl_pcie_rx_free(trans);

    free_irq(trans_pcie->pci_dev->irq, trans);
    iwl_pcie_free_ict(trans);

    pci_disable_msi(trans_pcie->pci_dev);
    iounmap(trans_pcie->hw_base);
    pci_release_regions(trans_pcie->pci_dev);
    pci_disable_device(trans_pcie->pci_dev);

    if (trans_pcie->napi.poll)
        netif_napi_del(&trans_pcie->napi);

    iwl_pcie_free_fw_monitor(trans);
#endif
    iwl_trans_free(trans);
}

IOReturn AppleIntelWiFiMVM::generateDMAAddresses(IOMemoryDescriptor* memDesc)

{
    
    // Get the physical segment list. These could be used to generate a scatter gather
    
    // list for hardware.
    
    
    
    IODMACommand*       cmd;
    
    IOReturn            err = kIOReturnSuccess;
    
    
    
    // 64 bit physical address generation using IODMACommand
    
    do
        
    {
        
        cmd = IODMACommand::withSpecification(
                                              
                                              // outSegFunc - Host endian since we read the address data with the cpu
                                              
                                              // and 64 bit wide quantities
                                              
                                              kIODMACommandOutputHost64,
                                              
                                              // numAddressBits
                                              
                                              64,
                                              
                                              // maxSegmentSize - zero for unrestricted physically contiguous chunks
                                              
                                              0,
                                              
                                              // mappingOptions - kMapped for DMA addresses
                                              
                                              IODMACommand::kMapped,
                                              
                                              // maxTransferSize - no restriction
                                              
                                              0,
                                              
                                              // alignment - no restriction
                                              
                                              1 );
        
        if (cmd == NULL) {
            
            IOLog("IODMACommand::withSpecification failed\n");
            
            break;
            
        }
        
        
        
        // Point at the memory descriptor and use the auto prepare option
        
        // to prepare the entire range
        
        err = cmd->setMemoryDescriptor(memDesc);
        
        if (kIOReturnSuccess != err) {
            
            IOLog("setMemoryDescriptor failed (0x%08x)\n", err);
            
            break;
            
        }
        
        
        
        UInt64 offset = 0;
        
        while ((kIOReturnSuccess == err) && (offset < memDesc->getLength())) {
            
            // Use the 64 bit variant to match outSegFunc
            
            IODMACommand::Segment64 segments[1];
            
            UInt32 numSeg = 1;
            
            
            
            // Use the 64 bit variant to match outSegFunc
            
            err = cmd->gen64IOVMSegments(&offset, &segments[0], &numSeg);
            
            IOLog("gen64IOVMSegments(%x) addr 0x%016llx, len %llu, nsegs %u \n", err, segments[0].fIOVMAddr, segments[0].fLength, numSeg);
        }
        
        
        
        // if we had a DMA controller, kick off the DMA here
        
        
        
        // when the DMA has completed,
        
        
        
        // clear the memory descriptor and use the auto complete option
        
        // to complete the transaction
        
        err = cmd->clearMemoryDescriptor();
        
        if (kIOReturnSuccess != err) {
            
            IOLog("clearMemoryDescriptor failed (0x%08x)\n", err);
            
        }
        
    } while (false);
    
    
    
    if (cmd != NULL) {
        
        cmd->release();
        
    }
    
    // end 64 bit loop
    
    
    
    
    
    // 32 bit physical address generation using IODMACommand
    
    // any memory above 4GiB in the memory descriptor will be bounce-buffered
    
    // to memory below the 4GiB line on machines without remapping HW support
    
    do
        
    {
        
        cmd = IODMACommand::withSpecification(
                                              
                                              // outSegFunc - Host endian since we read the address data with the cpu
                                              
                                              // and 32 bit wide quantities
                                              
                                              kIODMACommandOutputHost32,
                                              
                                              // numAddressBits
                                              
                                              32,
                                              
                                              // maxSegmentSize - zero for unrestricted physically contiguous chunks
                                              
                                              0,
                                              
                                              // mappingOptions - kMapped for DMA addresses
                                              
                                              IODMACommand::kMapped,
                                              
                                              // maxTransferSize - no restriction
                                              
                                              0,
                                              
                                              // alignment - no restriction
                                              
                                              1 );
        
        if (cmd == NULL) {
            
            IOLog("IODMACommand::withSpecification failed\n");
            
            break;
            
        }
        
        
        
        // point at the memory descriptor and use the auto prepare option
        
        // to prepare the entire range
        
        err = cmd->setMemoryDescriptor(memDesc);
        
        if (kIOReturnSuccess != err) {
            
            IOLog("setMemoryDescriptor failed (0x%08x)\n", err);
            
            break;
            
        }
        
        
        
        UInt64 offset = 0;
        
        while ((kIOReturnSuccess == err) && (offset < memDesc->getLength())) {
            
            // use the 32 bit variant to match outSegFunc
            
            IODMACommand::Segment32 segments[1];
            
            UInt32 numSeg = 1;
            
            
            
            // use the 32 bit variant to match outSegFunc
            
            err = cmd->gen32IOVMSegments(&offset, &segments[0], &numSeg);
            
            IOLog("gen32IOVMSegments(%x) addr 0x%08x, len %u, nsegs %u\n",
                  
                  err, segments[0].fIOVMAddr, segments[0].fLength, numSeg);
            
        }
        
        
        
        // if we had a DMA controller, kick off the DMA here
        
        
        
        // when the DMA has completed,
        
        
        
        // clear the memory descriptor and use the auto complete option
        
        // to complete the transaction
        
        err = cmd->clearMemoryDescriptor();
        
        if (kIOReturnSuccess != err) {
            
            IOLog("clearMemoryDescriptor failed (0x%08x)\n", err);
            
        }
        
    } while (false);
    
    
    
    if (cmd != NULL) {
        
        cmd->release();
        
    }
    
    // end 32 bit loop
    
    
    
    return err;
    
}
