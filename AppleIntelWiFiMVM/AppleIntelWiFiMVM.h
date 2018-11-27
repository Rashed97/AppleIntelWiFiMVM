#include <IOKit/IOLib.h>
#include <IOKit/IODMACommand.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <libkern/OSKextLib.h>
#include <IOKit/assert.h>
#include "FirmwareParser.h"

class AppleIntelWiFiMVM : public IOService {
    OSDeclareDefaultStructors(AppleIntelWiFiMVM);
    
public:
    // --------------- IOService methods ---------------
    virtual bool init(OSDictionary *properties) override;
    virtual bool start(IOService *provider) override;
    virtual void stop(IOService *provider) override;
    //virtual IOService* probe(IOService *provider, SInt32 *score) override;
    virtual void free() override;

    //virtual IOPCIDevice* getPCIDevice(){ return pciDevice;};
    virtual struct iwl_trans *allocatePCIeTransport(const struct iwl_cfg *cfg, IOPCIDevice *pciDevice);
    
    IOReturn generateDMAAddresses(IOMemoryDescriptor* memDesc);
private:
    // --------------- Methods ---------------
    const struct iwl_cfg *startupIdentifyWiFiCard();
    struct iwl_trans *startupCreatePCIeTransport(const struct iwl_cfg *cfg);
    bool startupCreateDriver(const struct iwl_cfg *cfg, struct iwl_trans *trans);
    bool startupLoadFirmware();
    bool startupHardware(IOPCIDevice *pciDevice, iwl_trans *trans);
    
    void shutdownStopFirmware();
    struct iwl_trans* shutdownFreeDriver();
    void shutdownFreePCIeTransport(struct iwl_trans *trans);

    OSData* loadFirmwareSync(struct iwl_drv *drv);
    static void firmwareLoadComplete( OSKextRequestTag requestTag, OSReturn result, const void *resourceData, uint32_t resourceDataLength, void *context);
    
    
    
    // --------------- Structs ---------------
    struct FirmwareLoadProgress {
        AppleIntelWiFiMVM *driver;
        OSData *firmwareData;
    };

    // --------------- Variables ---------------
    IOLock *firmwareLoadLock;
    IOPCIDevice *pciDevice;
    iwl_drv *driver;
    bool firmwareLoaded = false;
    FirmwareParser *parser;
    
    IOMemoryDescriptor* fLowMemory;
};

#define MYNAME "AppleIntelWiFiMVM"
#define DEBUGLOG(args...) IOLog("%s::%s",MYNAME, args); kprintf("%s::%s" ,MYNAME, args)
#define	RELEASE(x)	if(x){(x)->release();(x)=NULL;}
