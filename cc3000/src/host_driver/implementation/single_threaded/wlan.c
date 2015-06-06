/*****************************************************************************
*
*  wlan.c  - CC3000 Host Driver Implementation.
*  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*****************************************************************************/

//*****************************************************************************
//
//! \addtogroup wlan_api
//! @{
//
//*****************************************************************************
#include <string.h>

#include "wlan.h"
#include "socket.h"

#include "core_driver/inc/wlan.h"

#include "osal.h"

//*****************************************************************************
//
//!  wlan_init
//!
//!  @param  sWlanCB   Asynchronous events callback.
//!                    0 no event call back.
//!                  -call back parameters:
//!                   1) event_type: HCI_EVNT_WLAN_UNSOL_CONNECT connect event,
//!                     HCI_EVNT_WLAN_UNSOL_DISCONNECT disconnect event,
//!                     HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE config done,
//!                     HCI_EVNT_WLAN_UNSOL_DHCP dhcp report,
//!                     HCI_EVNT_WLAN_ASYNC_PING_REPORT ping report OR
//!                     HCI_EVNT_WLAN_KEEPALIVE keepalive.
//!                   2) data: pointer to extra data that received by the event
//!                     (NULL no data).
//!                   3) length: data length.
//!                  -Events with extra data:
//!                     HCI_EVNT_WLAN_UNSOL_DHCP: 4 bytes IP, 4 bytes Mask,
//!                     4 bytes default gateway, 4 bytes DHCP server and 4 bytes
//!                     for DNS server.
//!                     HCI_EVNT_WLAN_ASYNC_PING_REPORT: 4 bytes Packets sent,
//!                     4 bytes Packets received, 4 bytes Min round time,
//!                     4 bytes Max round time and 4 bytes for Avg round time.
//!
//!  @param    sFWPatches  0 no patch or pointer to FW patches
//!  @param    sDriverPatches  0 no patch or pointer to driver patches
//!  @param    sBootLoaderPatches  0 no patch or pointer to bootloader patches
//!  @param    sReadWlanInterruptPin    init callback. the callback read wlan
//!            interrupt status.
//!  @param    sWlanInterruptEnable   init callback. the callback enable wlan
//!            interrupt.
//!  @param    sWlanInterruptDisable   init callback. the callback disable wlan
//!            interrupt.
//!  @param    sWriteWlanPin      init callback. the callback write value
//!            to device pin.
//!
//!  @return   none
//!
//!  @sa       wlan_set_event_mask , wlan_start , wlan_stop
//!
//!  @brief    Initialize wlan driver
//!
//!  @warning This function must be called before ANY other wlan driver function
//
//*****************************************************************************

void wlan_init(tWlanCB     sWlanCB,
               tFWPatches sFWPatches,
               tDriverPatches sDriverPatches,
               tBootLoaderPatches sBootLoaderPatches,
               tWlanReadInteruptPin  sReadWlanInterruptPin,
               tWlanInterruptEnable  sWlanInterruptEnable,
               tWlanInterruptDisable sWlanInterruptDisable,
               tWriteWlanPin         sWriteWlanPin)
{
    c_wlan_init(sWlanCB, sFWPatches, sDriverPatches,\
                sBootLoaderPatches, sReadWlanInterruptPin,\
                sWlanInterruptEnable, sWlanInterruptDisable, sWriteWlanPin);
}

//*****************************************************************************
//
//!  wlan_start
//!
//!  @param   usPatchesAvailableAtHost -  flag to indicate if patches available
//!                                    from host or from EEPROM. Due to the
//!                                    fact the patches are burn to the EEPROM
//!                                    using the patch programmer utility, the
//!                                    patches will be available from the EEPROM
//!                                    and not from the host.
//!
//!  @return        none
//!
//!  @brief        Start WLAN device. This function asserts the enable pin of
//!                the device (WLAN_EN), starting the HW initialization process.
//!                The function blocked until device Initialization is completed.
//!                Function also configure patches (FW, driver or bootloader)
//!                and calls appropriate device callbacks.
//!
//!  @Note          Prior calling the function wlan_init shall be called.
//!  @Warning       This function must be called after wlan_init and before any
//!                 other wlan API
//!  @sa            wlan_init , wlan_stop
//!
//
//*****************************************************************************

void wlan_start(unsigned short usPatchesAvailableAtHost)
{
    c_wlan_start(usPatchesAvailableAtHost);
}


//*****************************************************************************
//
//!  wlan_stop
//!
//!  @param         none
//!
//!  @return        none
//!
//!  @brief         Stop WLAN device by putting it into reset state.
//!
//!  @sa            wlan_start
//
//*****************************************************************************

void wlan_stop(void)
{
    c_wlan_stop();
}

//*****************************************************************************
//
//!  wlan_connect
//!
//!  @param    sec_type   security options:
//!               WLAN_SEC_UNSEC,
//!               WLAN_SEC_WEP (ASCII support only),
//!               WLAN_SEC_WPA or WLAN_SEC_WPA2
//!  @param    ssid       up to 32 bytes and is ASCII SSID of the AP
//!  @param    ssid_len   length of the SSID
//!  @param    bssid      6 bytes specified the AP bssid
//!  @param    key        up to 16 bytes specified the AP security key
//!  @param    key_len    key length
//!
//!  @return     On success, zero is returned. On error, negative is returned.
//!              Note that even though a zero is returned on success to trigger
//!              connection operation, it does not mean that CCC3000 is already
//!              connected. An asynchronous "Connected" event is generated when
//!              actual association process finishes and CC3000 is connected to
//!              the AP. If DHCP is set, An asynchronous "DHCP" event is
//!              generated when DHCP process is finish.
//!
//!
//!  @brief      Connect to AP
//!  @warning    Please Note that when connection to AP configured with security
//!              type WEP, please confirm that the key is set as ASCII and not
//!              as HEX.
//!  @sa         wlan_disconnect
//
//*****************************************************************************

#ifndef CC3000_TINY_DRIVER
long wlan_connect(unsigned long ulSecType, char *ssid, long ssid_len,
             unsigned char *bssid, unsigned char *key, long key_len)
{
    long ret;
    ret = c_wlan_connect(ulSecType, ssid, ssid_len, bssid, key, key_len);
    return(ret);
}
#else
long wlan_connect(char *ssid, long ssid_len)
{
    long ret;
    ret = c_wlan_connect(ssid, ssid_len);
    return(ret);
}
#endif

/**
 * \brief wlan disconnect
 *
 * Disconnect connection from AP.
 *
 *
 * \return  0 disconnected done, other already disconnected
 *
 * \sa   wlan_connect
 * \note
 * \warning
 */
long wlan_disconnect()
{
    long ret;
    ret = c_wlan_disconnect();
    return(ret);
}

/**
 * \brief set connection policy
 *
 *
 * When auto is enabled, the device tries to connect according
 * the following policy:\n
 * 1) If fast connect is enabled and last connection is valid, the device will try to connect it without the
 * scanning procedure (fast). The last connection marked as
 * invalid, due to adding/removing profile.\n 2) If profile
 * exists, the device will try to connect it (Up to seven
 * profiles)\n 3) If fast and profiles are not found, and open
 * mode is enabled, the device will try to connect to any AP.
 * Note that the policy settings are stored in the CC3000 NVMEM.
 *
 *
 *
 * \param[in]   should_connect_to_open_ap  enable(1), disable(0)
 *       connect to any available AP. This parameter corresponds to the
 *     configuration of item # 3 in the above description.
 * \param[in]   should_use_fast_connect enable(1), disable(0).
 *       if enabled, tries to connect to the last connected
 *       AP. This parameter describes a fast connect option above.
 * \param[in]   auto_start enable(1), disable(0) auto connect
 *       after reset and periodically reconnect if needed.This configuration
 *     configures option 2 in the above description.\n
 *
 *
 * \return  On success, zero is returned. On error, -1 is
 *            returned
 * \sa wlan_add_profile     wlan_ioctl_del_profile
 * \note       The default policy settings are: Use fast connect, ussage of profiles is not enabled and ussage of open AP is not set.
 *          Note also that in case fast connection option is used, the profile is automatically generated.
 * \warning
 */

long
wlan_ioctl_set_connection_policy(unsigned long should_connect_to_open_ap,
                                 unsigned long ulShouldUseFastConnect,
                                 unsigned long ulUseProfiles)
{
    long ret;
    ret = c_wlan_ioctl_set_connection_policy(should_connect_to_open_ap,\
                                             ulShouldUseFastConnect,\
                                             ulUseProfiles);
    return(ret);
}

/**
 * \brief add profile
 *
 *  When auto start is enabled, the device connects to
 *  station from the profiles table. Up to 7 profiles are
 *  supported. If several profiles configured the device chose
 *  the highest priority profile, within each priority group,
 *  device will chose profile based on security policy, signal
 *  strength, etc parameters. All the profiles are stored in CC3000
 *  NVMEM.\n
 *
 *
 * \param[in]   tSecType:\n WLAN_SEC_UNSEC, WLAN_SEC_WEP,
 *       WLAN_SEC_WPA or WLAN_SEC_WPA2
 * \param[in]   ucSsid  ssid, up to 32 bytes
 * \param[in]   ulSsidLen ssid length
 * \param[in]   ucBssid  bssid, 6 bytes
 * \param[in]   ulPriority profile priority. Lowest priority:
 *       0
 * \param[in]   ulPairwiseCipher_Or_Key
 * \param[in]   ulGroupCipher_TxKeyLen
 * \param[in]   ulKeyMgmt
 * \param[in]   ucPf_OrKey
 * \param[in]   ulPassPhraseLen
 *
 *
 * \return  On success, index is returned. On error, -1 is
 *            returned
 *
 * \sa   wlan_ioctl_del_profile
 * \note
 * \warning
 */
long wlan_add_profile(unsigned long ulSecType,
                      unsigned char* ucSsid,
                      unsigned long ulSsidLen,
                      unsigned char *ucBssid,
                      unsigned long ulPriority,
                      unsigned long ulPairwiseCipher_Or_TxKeyLen,
                      unsigned long ulGroupCipher_TxKeyIndex,
                      unsigned long ulKeyMgmt,
                      unsigned char* ucPf_OrKey,
                      unsigned long ulPassPhraseLen)
{
    long ret;
    ret = c_wlan_add_profile(ulSecType, ucSsid, ulSsidLen, ucBssid, ulPriority,
                             ulPairwiseCipher_Or_TxKeyLen, ulGroupCipher_TxKeyIndex,
                             ulKeyMgmt, ucPf_OrKey, ulPassPhraseLen);
    return(ret);
}

/**
 * \brief Delete WLAN profile
 *
 * Delete WLAN profile
 *
 * \param[in]   index  number of profile to delete
 *
 * \return  On success, zero is returned. On error, -1 is
 *            returned
 *
 * \sa   wlan_add_profile
 * \note
 * \warning
 */
long wlan_ioctl_del_profile(unsigned long ulIndex)
{
    long ret;
    ret = c_wlan_ioctl_del_profile(ulIndex);
    return(ret);
}

/**
 * \brief Gets the WLAN scan operation results
 *
 * Gets entry from scan result table.
 * The scan results are returned one by one, and each entry
 * represents a single AP found in the area. The following is a
 * format of the scan result:
 *  - 4 Bytes: number of networks found
 *  - 4 Bytes: The status of the scan: 0 - agged results, 1 - results valid, 2 - no results
 *  - 56 bytes: Result entry, where the bytes are arranged as
 *    follows:
 *              - 1 bit isValid - is result valid or not
 *              - 7 bits rssi           - RSSI value;
 *              - 2 bits: securityMode - security mode of the AP: 0 - Open, 1 - WEP, 2 WPA, 3 WPA2
 *              - 6 bits: SSID name length
 *              - 2 bytes: the time at which the entry has entered into scans result table
 *              - 32 bytes: SSID name
 *              - 6 bytes:  BSSID
 *
 * \param[in] scan_timeout
 * \param[out] ucResults  scan resault
 *       (_wlan_full_scan_results_args_t)
 *
 *
 * \return  On success, zero is returned. On error, -1 is
 *            returned
 *
 * \sa  wlan_ioctl_set_scan_params
 * \note scan_timeout, is not supported on this version.
 * \warning
 */
#ifndef CC3000_TINY_DRIVER
long wlan_ioctl_get_scan_results(unsigned long ulScanTimeout,
                                 unsigned char *ucResults)
{
    long ret;
    ret = c_wlan_ioctl_get_scan_results(ulScanTimeout, ucResults);
    return(ret);
}
#endif

/**
 * \brief Sets the WLAN scan configuration
 *
 * start and stop scan procedure.
 * Set scan parameters
 *
 * \param[in] uiEnable - start/stop application scan
 *       (1=start scan with default interval value of 10 min.
 *       in order to set a diffrent scan interval value apply the value in
 *       miliseconds. minimum 1 second. 0=stop). Wlan reset
 *       (wlan_stop() wlan_start()) is needed when changing scan
 *       interval value. Saved: No
 * \param[in] uiMinDwellTime   minimum dwell time value to be
 *       used for each channel, in millisconds. Saved: yes
 *       Recommended Value: 100 (Default: 20)
 * \param[in] uiMaxDwellTime    maximum dwell time value to be
 *       used for each channel, in millisconds. Saved: yes
 *       Recommended Value: 100 (Default: 30)
 * \param[in] uiNumOfProbeRequests  max probe request between
 *       dwell time. Saved: yes. Recommended Value: 5
 *       (Default:2)
 *
 * \param[in] uiChannelMask  bitwise, up to 13 channels
 *       (0x1fff). Saved: yes. Default: 0x7ff
 * \param[in] uiRSSIThreshold   RSSI threshold. Saved: yes
 *       Default -80
 * \param[in] uiSNRThreshold    NSR thereshold. Saved: yes.
 *       Default: 0
 * \param[in] uiDefaultTxPower  probe Tx power. Saved: yes
 *       Default: 205
 * \param[in] aiIntervalList    pointer to array with 16 entries
 *       (16 channels) each entry (unsigned long) holds timeout
 *       between periodic scan (connection scan) - in
 *       millisconds. Saved: yes. Default 2000ms.
 *
 * \return  On success, zero is returned. On error, -1 is
 *            returned
 * \sa   wlan_ioctl_get_scan_results
 * \note uiDefaultTxPower, is not supported on this version.
 * \warning
 */
#ifndef CC3000_TINY_DRIVER
long wlan_ioctl_set_scan_params(unsigned long uiEnable, unsigned long uiMinDwellTime,
                                unsigned long uiMaxDwellTime, unsigned long uiNumOfProbeRequests,
                                unsigned long uiChannelMask, long iRSSIThreshold,
                                unsigned long uiSNRThreshold, unsigned long uiDefaultTxPower,
                                unsigned long *aiIntervalList)
{
    unsigned long  uiRes;
    uiRes = c_wlan_ioctl_set_scan_params(uiEnable, uiMinDwellTime, uiMaxDwellTime,\
                                         uiNumOfProbeRequests, uiChannelMask, iRSSIThreshold,\
                                         uiSNRThreshold, uiDefaultTxPower, aiIntervalList);
    return (uiRes);
}
#endif

/**
 * \brief set event mask
 *
 * Mask event according to bit mask. In case that event is
 * masked (1), the device will not send the masked event.
 * \param[in] mask  Saved: no. mask option:\n
 *       HCI_EVNT_WLAN_UNSOL_CONNECT connect event\n
 *       HCI_EVNT_WLAN_UNSOL_DISCONNECT disconnect event\n
 *       HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE config done\n
 *       HCI_EVNT_WLAN_UNSOL_INIT init done\n
 *       HCI_EVNT_WLAN_UNSOL_DHCP dhcp report\n
 *       HCI_EVNT_WLAN_ASYNC_PING_REPORT ping report\n
 *       HCI_EVNT_WLAN_KEEPALIVE keepalive\n
 *     HCI_EVNT_WLAN_TX_COMPLETE - disable information on end of transmission
 *
 * \On success, zero is returned. On error, -1 is
 *            returned
 * \sa
 * \note
 * \warning
 */
long wlan_set_event_mask(unsigned long ulMask)
{
    long ret;
    ret = c_wlan_set_event_mask(ulMask);
    return(ret);
}

/**
 * \brief get wlan status
 *
 * get wlan status: disconnected, scaning, connecting or
 * connected
 *
 * \return WLAN_STATUS_DISCONNECTED, WLAN_STATUS_SCANING,
 *         STATUS_CONNECTING or WLAN_STATUS_CONNECTED
 *
 *
 * \sa
 * \note
 * \warning
 */
#ifndef CC3000_TINY_DRIVER
long wlan_ioctl_statusget(void)
{
    long ret;
    ret = c_wlan_ioctl_statusget();
    return(ret);
}
#endif

/**
 * \brief Start acquire profile
 *
 * Start to acquire device profile. The device acquire its own
 * profile, if profile message is found. The acquired AP information is
 * stored in CC3000 EEPROM only in case AES128 encryption is used.
 * In case AES128 encryption is not used, a profile is created by CC3000 internally.\n
 *
 * \param[in] algoEncryptedFlag indicates whether the information is encrypted
 *
 * \return  On success, zero is returned. On error, -1 is
 *            returned
 * \sa   wlan_smart_config_set_prefix  wlan_smart_config_stop
 * \note    An asynchnous event - Smart Config Done will be generated as soon as the process finishes successfully
 * \warning
 */
long wlan_smart_config_start(unsigned long algoEncryptedFlag)
{
    long ret;
    ret = c_wlan_smart_config_start(algoEncryptedFlag);
    return(ret);
}

/**
 * \brief stop acquire profile
 *
 * Stop the acquire profile procedure
 *
 * \return  On success, zero is returned. On error, -1 is
 *            returned
 *
 * \sa   wlan_smart_config_start  wlan_smart_config_set_prefix
 * \note
 * \warning
 */
long wlan_smart_config_stop(void)
{
    long ret;
    ret = c_wlan_smart_config_stop();
    return(ret);
}

/**
 * \brief config set prefix
 *
 * Configure station ssid prefix.
 * The prefix is used internally in CC3000.
 * It should always be TTT.
 *
 * \param[in] newPrefix  3 bytes identify the SSID prefix for
 *       the Simple Config.
 *
 * \return  On success, zero is returned. On error, -1 is
 *            returned
 *
 * \sa   wlan_smart_config_start  wlan_smart_config_stop
 * \note        The prefix is stored in CC3000 NVMEM.\n
 * \warning
 */
long wlan_smart_config_set_prefix(char* cNewPrefix)
{
    long ret;
    ret = c_wlan_smart_config_set_prefix(cNewPrefix);
    return(ret);
}

/**
 * \brief process the acquired data and store it as a profile
 *
 * The acquired AP information is stored in CC3000 EEPROM encrypted.
 * The encrypted data is decrypted and storred as a profile.
 * behavior is as defined by policy. \n
 *
 * \return  On success, zero is returned. On error, -1 is
 *            returned
 * \sa
 * \note
 * \warning
 */

#ifndef CC3000_UNENCRYPTED_SMART_CONFIG
long wlan_smart_config_process()
{
    long ret;
    ret = c_wlan_smart_config_process();
    return(ret);
}
#endif //CC3000_UNENCRYPTED_SMART_CONFIG


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
