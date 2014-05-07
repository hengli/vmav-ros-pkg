// ==============================================================================================
/// @file vrmusbcam2.h                                                   VRmUsbCam C API v3.1.0.1
//  Copyright VRmagic 2004-2013
//
//  vrmusbcam2.dll
//  vrmusbcam2.lib
// ==============================================================================================

#ifndef VRMUSBCAM2_H
#define VRMUSBCAM2_H

/// version of this API header. you can compare it with the value returned from
/// VRmUsbCamGetVersion()
#define VRMUSBCAM_VERSION 3101

#if defined(__cplusplus) && !defined(VRM_NO_EXTERN_C)
extern "C" {
#endif

/// VRmUsbCam is C calling convention (cdecl). define this appropriate if your compiler
/// has a different default
#ifndef VRM_API
#define VRM_API
#endif

// don't define this. for internal use only
#ifndef VRM_EXTERN
#define VRM_EXTERN extern
#endif

// don't define this. for internal use only
#ifndef VRM_STRUCT
#define VRM_STRUCT struct
#endif

// don't define this. for internal use only
#ifndef VRM_ENUM
#define VRM_ENUM enum
#endif

// ------------------------------------------------------------------------
/// @defgroup Types                                             Basic Types
// ------------------------------------------------------------------------
/// @{

typedef unsigned char VRmBYTE;
typedef unsigned short int VRmWORD;
typedef unsigned int VRmDWORD;
typedef unsigned int VRmBOOL;
typedef const char* VRmSTRING;

/// "size" type (int)
typedef VRM_STRUCT _VRmSizeI {
	int m_width;
	int m_height;
} VRmSizeI;

/// "point" type (int)
typedef VRM_STRUCT _VRmPointI {
	int m_x;
	int m_y;
} VRmPointI;

/// "rect" type (int)
typedef VRM_STRUCT _VRmRectI {
	int m_left;
	int m_top;
	int m_width;
	int m_height;
} VRmRectI;

/// @}
// ------------------------------------------------------------------------
/// @defgroup General                   Error Handling / General Management
// ------------------------------------------------------------------------
/// @{

/// return value values of API function calls
typedef VRM_ENUM _VRmRetVal
{
	/// return value for failed function call
	VRM_FAILED=0,
	/// return value for successful function call
	VRM_SUCCESS=1
} VRmRetVal;

/// error codes used by get last error code
typedef VRM_ENUM _VRmErrorCode
{
	//error code for info, these don't create log file entries
	VRM_ERROR_CODE_SUCCESS 					=	0x00000000,

	VRM_ERROR_CODE_FUNCTION_CALL_TIMEOUT	=	0x00040003,

	//error code for failures, these create log file entries
	VRM_ERROR_CODE_GENERIC_ERROR			=	0x80004005,

	VRM_ERROR_CODE_TRIGGER_TIMEOUT			=	0x80040001,
	VRM_ERROR_CODE_TRIGGER_STALL			=	0x80040002,
	VRM_ERROR_CODE_TRANSFER_TIMEOUT			=	0x80040004

} VRmErrorCode;


/// retrieve the error string of the last function call.
/// if an API function fails (return value is VRM_FAILED) use this function
/// to retrieve an error description as C string.
/// NOTE: the returned string is only guaranteed to be valid until the next API call
VRM_EXTERN VRmSTRING VRM_API VRmUsbCamGetLastError(void);

/// retrieve the error code of the last function call.
/// if an API function fails (return is value VRM_FAILED) use this function
/// to retrieve an error error code as int.
/// NOTE: the returned code is only guaranteed to be valid until the next API call
VRM_EXTERN int VRM_API VRmUsbCamGetLastErrorCode(void);

/// reset the error code and error string to success.
VRM_EXTERN void VRM_API VRmUsbCamClearLastError(void);

/// last error was trigger timeout.
/// if an API function fails (return value VRM_FAILED) use this function
/// to check if error was a trigger timeout
VRM_EXTERN VRmBOOL VRM_API VRmUsbCamLastErrorWasTriggerTimeout(void);

/// last error was trigger stall.
/// if an API function fails (return value VRM_FAILED) use this function
/// to check if error was a trigger stall
VRM_EXTERN VRmBOOL VRM_API VRmUsbCamLastErrorWasTriggerStall(void);

/// enable logging.
/// for customer support, enable the logging facilities of the VRmUsbCam
/// library.
/// you may want to add following lines to your source code:
/// @code
///   ...
///   #ifdef _DEBUG
///      VRmUsbCamEnableLogging();
///   #endif
///   ...
/// @endcode
/// NOTE: Only the first successfull call to VRmUsbCamEnableLogging or
/// VRmUsbCamEnableLoggingEx will have an effect. Subsequent calls will
/// be ignored.
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamEnableLogging(void);

/// enable logging with selectable file name
/// for customer support, enable the logging facilities of the VRmUsbCam
/// library.
/// The file "f_log_file_name" will be newly created.
/// you may want to add following lines to your source code:
/// @code
///   ...
///   #ifdef _DEBUG
///      VRmUsbCamEnableLoggingEx( LOG_FILE_NAME );
///   #endif
///   ...
/// @endcode
/// NOTE: Only the first successful call to VRmUsbCamEnableLogging or
/// VRmUsbCamEnableLoggingEx will have an effect. Subsequent calls will
/// be ignored.
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamEnableLoggingEx( VRmSTRING f_log_file_name );

/// get the version of the API.
/// the version number is represented as decimal integer with 4 digits,
/// ie. API version v2.3.0.0 is represented as decimal 2300.
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetVersion(VRmDWORD* fp_version);

/// you should call this at application exit in order to cleanup
/// all resources left over from VRmUsbCam API.
/// AFTER THAT, NO OTHER API FUNCTIONS MUST BE CALLED ANY LONGER!
///
/// easiest way is to use the following call at application init:
/// @code
///   atexit(VRmUsbCamCleanup);
/// @endcode
VRM_EXTERN void VRM_API VRmUsbCamCleanup(void);

/// @}
// ------------------------------------------------------------------------
/// @defgroup DeviceMng                                   Device Management
// ------------------------------------------------------------------------
/// @{

/// device handle.
/// this handle represents a device, create using VRmUsbCamOpenDevice(),
/// release using VRmUsbCamCloseDevice()
#ifndef VRM_VRMUSBCAMDEVICE_DEFINED
#define VRM_VRMUSBCAMDEVICE_DEFINED
typedef struct VRmUsbCamDeviceInternal* VRmUsbCamDevice;
#endif//VRM_VRMUSBCAMDEVICE_DEFINED

/// struct to identify devices.
/// device key is a unique combination of serial, manufacturer
/// and product string
/// NOTE: the strings within this struct are only guaranteed to be valid
/// until the next call to VRmUsbCamUpdateDeviceKeyList()!
typedef VRM_STRUCT _VRmDeviceKey
{
	/// serial number
	VRmDWORD m_serial;
	/// manufacturer name string
	VRmSTRING mp_manufacturer_str;
	/// product name string
	VRmSTRING mp_product_str;
	/// busy means the device is already used by another application
	VRmBOOL m_busy;
	/// private = additional internal data
	void* mp_private;
}VRmDeviceKey;

/// search for compatible devices.
/// if your application wants to support PnP, you should call this function
/// periodically, at least once every 5 seconds, and handle the PnP events
/// using static callback handlers, see VRmUsbCamRegisterStaticCallback()
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamUpdateDeviceKeyList(void);

/// get number of attached devices
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetDeviceKeyListSize(VRmDWORD* fp_size);

/// get device key by index.
/// returns the device key by index = [0...number of attached devices-1],
/// use VRmUsbCamFreeDeviceKey() to free the key when you're done
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetDeviceKeyListEntry(VRmDWORD f_index, VRmDeviceKey** fpp_device_key);

/// get vendor id (16bit) of device key
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetVendorId(const VRmDeviceKey* fcp_device_key, VRmWORD* fp_vendor_id);

/// get product id (16bit) of device key
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetProductId(const VRmDeviceKey* fcp_device_key, VRmWORD* fp_product_id);

/// get group id (16bit) of device key
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetGroupId(const VRmDeviceKey* fcp_device_key, VRmWORD* fp_group_id);

/// get serial string of device key.
/// note: the returned string is only guaranteed to be valid until the next call to VRmUsbCamUpdateDeviceKeyList()!
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetSerialString(const VRmDeviceKey* fcp_device_key, VRmSTRING* fp_serial_str);

/// get IP address string of device key (for non-ethernet devices, this is an empty string).
/// note: the returned string is only guaranteed to be valid until the next API call!
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetIpAddress(const VRmDeviceKey* fcp_device_key, VRmSTRING* fp_value);

/// get IP address string of local interface that is to be used for communication with
/// device of device key (for non-ethernet devices, this is an empty string).
/// note: the returned string is only guaranteed to be valid until the next API call!
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetLocalIpAddress(const VRmDeviceKey* fcp_device_key, VRmSTRING* fp_value);

/// compare two device keys.
/// sets target of fp_result to 0 if keys are equal, sets it to 1 otherwise
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamCompareDeviceKeys(const VRmDeviceKey* fcp_device_key1, const VRmDeviceKey* fcp_device_key2, VRmBOOL* fp_result);

/// free device key received by VRmUsbCamGetDeviceKeyListEntry()
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamFreeDeviceKey(VRmDeviceKey** fpp_device_key);

/// open device.
/// open device using device key returned by VRmUsbCamGetDeviceKeyListEntry(),
/// to release fp_device use VRmUsbCamCloseDevice()
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamOpenDevice(const VRmDeviceKey* fcp_device_key, VRmUsbCamDevice* fp_device);

/// get device key for device in use.
/// use VRmUsbCamFreeDeviceKey() to free the key when you're done
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetDeviceKey(VRmUsbCamDevice f_device, VRmDeviceKey** fpp_device_key);

/// close device using device handle
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamCloseDevice(VRmUsbCamDevice f_device);


/// @}
// ------------------------------------------------------------------------
/// @defgroup UDS                   User Data Storage / Non-Volatile Memory
// ------------------------------------------------------------------------
/// @{

/// struct for user data storage
typedef VRM_STRUCT _VRmUserData
{
	/// length of user data in bytes
	VRmDWORD m_length;
	/// pointer to user data
	VRmBYTE* mp_data;
	/// private = additional internal data
	void* mp_private;
}VRmUserData;

/// load user data from eeprom.
/// use VRmUsbCamFreeUserData() to delete afterwards. if no data was saved before this function
/// returns without error but returns a VRmUserData with length = 0
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamLoadUserData(VRmUsbCamDevice f_device, VRmUserData** fpp_user_data);

/// save user data in eeprom.
/// NOTE: you can use the VRM_PROPID_DEVICE_NV_MEM_FREE_I property to determine the number of
/// bytes free in the non-volatile memory of the device
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamSaveUserData(VRmUsbCamDevice f_device, const VRmUserData* fcp_user_data);

/// allocate new user data.
/// length in bytes, use VRmUsbCamFreeUserData() to delete
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamNewUserData(VRmUserData** fpp_user_data, VRmDWORD f_length);

/// free VRmUserData created by VRmUsbCamNewUserData() and VRmUsbCamLoadUserData()
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamFreeUserData(VRmUserData** fpp_user_data);

/// @}
// ------------------------------------------------------------------------
/// @defgroup Timer                                                   Timer
// ------------------------------------------------------------------------
/// @{

/// restart timer for timestamps, use for all open devices
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamRestartTimer(void);

/// get current timestamp from timer
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetCurrentTime(double* fp_current_time);

/// @}
// ------------------------------------------------------------------------
/// @defgroup Images                                         Image Handling
// ------------------------------------------------------------------------
/// @{

/// enum for color formats
typedef VRM_ENUM _VRmColorFormat
{
	/// 32bit Pixel, Order A R G B using 32bit (DWORD) access
	VRM_ARGB_4X8,
	/// 24bit Pixel, Order B G R using 8bit (BYTE) access
	VRM_BGR_3X8,
	/// 16bit Pixel, Order R(5) G(6) B(5) using 16bit (WORD) access
	VRM_RGB_565,
	/// (effective) 16bit Pixel, Order Y U Y V, U+V = color for 2 pixels
	VRM_YUYV_4X8,
	/// 8bit Pixel
	VRM_GRAY_8,
	/// 8bit Pixel, Order G B (even lines) R G (odd lines)
	VRM_BAYER_GBRG_8,
	/// 8bit Pixel, Order B G (even lines) G R (odd lines)
	VRM_BAYER_BGGR_8,
	/// 8bit Pixel, Order R G (even lines) G B (odd lines)
	VRM_BAYER_RGGB_8,
	/// 8bit Pixel, Order G R (even lines) B G (odd lines)
	VRM_BAYER_GRBG_8,
	/// 10bit Pixel, storing upper 8bit in first byte, storing lower 2bit in second byte (lowest bits)
	VRM_GRAY_10,
	/// 10bit Pixel, Order G B (even lines) R G (odd lines)
	VRM_BAYER_GBRG_10,
	/// 10bit Pixel, Order B G (even lines) G R (odd lines)
	VRM_BAYER_BGGR_10,
	/// 10bit Pixel, Order R G (even lines) G B (odd lines)
	VRM_BAYER_RGGB_10,
	/// 10bit Pixel, Order G R (even lines) B G (odd lines)
	VRM_BAYER_GRBG_10,
	/// 16bit Pixel, using 16bit (WORD) access
	VRM_GRAY_16,
	/// 48bit Pixel, using 16bit (WORD) access
	VRM_BGR_3X16,
	/// 16bit Pixel, Order G B (even lines) R G (odd lines)
	VRM_BAYER_GBRG_16,
	/// 16bit Pixel, Order B G (even lines) G R (odd lines)
	VRM_BAYER_BGGR_16,
	/// 16bit Pixel, Order G R (even lines) B G (odd lines)
	VRM_BAYER_RGGB_16,
	/// 16bit Pixel, Order R G (even lines) G B (odd lines)
	VRM_BAYER_GRBG_16,
	/// (effective) 16bit Pixel, Order U Y V Y, U+V = color for 2 pixels
	VRM_UYVY_4X8
}VRmColorFormat;

/// enum for image modifiers
typedef VRM_ENUM _VRmImageModifier
{
	/// standard image, top->down, left->right orientation
	VRM_STANDARD=0,
	/// vertical mirrored
	VRM_VERTICAL_MIRRORED=1<<0,
	/// horizontal mirrored
	VRM_HORIZONTAL_MIRRORED=1<<1,
	/// first field of interlaced image
	VRM_INTERLACED_FIELD0=1<<2,
	/// second field of interlaced image
	VRM_INTERLACED_FIELD1=1<<3,
	/// any field of interlaced image (this is only used for format selection)
	VRM_INTERLACED_FIELD01=1<<4,
	///  both fields of interlaced image (one field below the other)
	/// is always combined with either VRM_ORDER_TFF or VRM_ORDER_BFF
	/// this can be deinterlaced to one frame (see VRmUsbCamConvertImage)
	VRM_INTERLACED_FRAME=1<<5,
	/// associated LUT has 1 channel, 8bit indices
	VRM_CORRECTION_LUT_1CHANNEL_8=1<<6,
	/// associated LUT has 1 channel, 10bit indices
	VRM_CORRECTION_LUT_1CHANNEL_10=1<<7,
	/// associated LUT has 4 channels, 8bit indices
	VRM_CORRECTION_LUT_4CHANNEL_8=1<<8,
	/// associated LUT has 4 channels, 10bit indices
	VRM_CORRECTION_LUT_4CHANNEL_10=1<<9,
	/// this format has a changeable ROI (User ROI)
	VRM_USER_ROI=1<<10,
	/// subsampled image (skipped pixels or binning)
	VRM_SUBSAMPLED=1<<11,
	/// Run Length Encoded (supports VRM_GRAY_8, VRM_BGR_3X8).
	/// byte pairs (value, length), EOF marked by length=0.
	VRM_RUN_LENGTH_ENCODED=1<<12,
	/// the data in this image is A-LAW compressed
	/// (currently Davinci DSP images only)
	VRM_ALAW_COMPRESSED=1<<13,
	/// TFF (top field first)
	/// upper half image is the first field, lower half image is the second field
	VRM_ORDER_TFF = 1<<14,
	/// BFF (bottom field first)
	/// upper half image is the second field, lower half image is the first field
	VRM_ORDER_BFF = 1<<15,
	/// associated LUT has 1 channel, 16 bit indices, 16 bit data
	VRM_CORRECTION_LUT_1CHANNEL_16=1<<16

}VRmImageModifier;

/// struct for image format
typedef VRM_STRUCT _VRmImageFormat
{
	/// width in pixels
	VRmDWORD m_width;
	/// height in lines
	VRmDWORD m_height;
	/// color format enum
	VRmColorFormat m_color_format;
	/// bit combination of enum VRmImageModifier values
	int m_image_modifier;
}VRmImageFormat;

/// struct for image container
typedef VRM_STRUCT _VRmImage
{
	/// image format struct
	VRmImageFormat m_image_format;
	/// pointer to image buffer
	VRmBYTE* mp_buffer;
	/// pitch = number of bytes from (x, y) to (x, y+1)
	VRmDWORD m_pitch;
	/// timestamp of image in ms since last VRmUsbCamRestartTimer()
	double m_time_stamp;
	/// private = additional internal data
	void* mp_private;
}VRmImage;

/// create new image with given image format.
/// NOTE: free image using VRmUsbCamFreeImage().
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamNewImage(VRmImage** fpp_image, VRmImageFormat f_image_format);

/// create new image as copy of src_image.
/// NOTE: free image using VRmUsbCamFreeImage().
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamCopyImage(VRmImage** fpp_image, const VRmImage* fcp_src_image);

/// get cropped Part of an Image without copying of image data.
/// NOTE: image data is shared with given image.
/// NOTE: free image using VRmUsbCamFreeImage().
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamCropImage(VRmImage** fpp_image, const VRmImage* fcp_src_image, const VRmRectI* fcp_roi);

/// create image container with given format, buffer and pitch.
/// NOTE: free image using VRmUsbCamFreeImage() but you still have to deallocate fp_buffer YOURSELF!
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamSetImage(VRmImage** fpp_image, VRmImageFormat f_image_format, VRmBYTE* fp_buffer, VRmDWORD f_pitch);

/// get Frame Counter of Image
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetFrameCounter(const VRmImage* fcp_image, VRmDWORD* fp_frame_counter);

/// get Footer Data of Image.
/// NOTE: this is only valid as long as the given image is valid.
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetImageFooter(const VRmImage* fcp_image, const void** fpp_data, VRmDWORD* fp_size);

/// get Size of Image Buffer (mp_buffer)
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetImageBufferSize(const VRmImage* fcp_image, VRmDWORD* fp_size);

/// special value for VRmUsbCamSetImageBufferSize()
#define VRM_IMAGE_BUFFER_SIZE_MAX (0xFFFFFFFF)

/// set Size of Image Buffer (mp_buffer).
/// NOTE: in general, this is only applicable for RLE image buffers.
/// resetting the size to VRM_IMAGE_BUFFER_SIZE_MAX resets the size to the maximum value
/// allowable for the format of the image.
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamSetImageBufferSize(const VRmImage* fcp_image, VRmDWORD f_size);

/// get the Image Sensor Port this image originates from.
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetImageSensorPort(const VRmImage* fcp_image, VRmDWORD* fp_port);

/// free VRmImage created by VRmUsbCamNewImage(), VRmUsbCamCopyImage() or VRmUsbCamSetImage()
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamFreeImage(VRmImage** fpp_image);

/// get number of available target format list entries for a given source format
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetTargetFormatListSize(const VRmImageFormat* fcp_source_format, VRmDWORD* fp_size);

/// query target format list entry with index = [0...size-1]
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetTargetFormatListEntry(const VRmImageFormat* fcp_source_format, VRmDWORD f_index, VRmImageFormat* fp_target_format);

/// convert source image to target image.
/// use VRmUsbCamGetTargetFormatListSize() and VRmUsbCamGetTargetFormatListEntry() to generally find out about
/// possible conversions for an image with format fcp_source->m_image_format.
/// use VRmUsbCamGetTargetFormatListSizeEx2() and VRmUsbCamGetTargetFormatListEntryEx2() instead,
/// when you like to convert a source image acquired from the device. this takes additional converter settings
/// of the device into account.
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamConvertImage(const VRmImage* fcp_source, VRmImage* fp_target);

/// get string representation from color format
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetStringFromColorFormat(VRmColorFormat f_color_format, VRmSTRING* fp_string);

/// get pixel depth (in bytes) from color format
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetPixelDepthFromColorFormat(VRmColorFormat f_color_format, VRmDWORD* fp_pixel_depth);

/// compare two image formats.
/// sets target of fp_result to 0 if keys are equal, sets it to 1 otherwise
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamCompareImageFormats(const VRmImageFormat* fcp_format1, const VRmImageFormat* fcp_format2, VRmBOOL* fp_result);

/// get the LUT that is associated with the given VRmImage
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetImageLut(const VRmImage* fcp_image, const VRmBYTE** fpp_lut, VRmDWORD* fp_size);

/// @}
// ------------------------------------------------------------------------
/// @defgroup Grabber                                         Frame Grabber
// ------------------------------------------------------------------------
/// @{

// formats:
// source format = images with this format will be produced by the frame grabber hardware (VRmUsbCamLockNextImageEx())
// target format = images with this format can be produced by the software converter (VRmUsbCamConvertImage())

/// get number of image sensor ports
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetSensorPortListSize(VRmUsbCamDevice f_device, VRmDWORD* fp_size);

/// query image sensor port number entry with index [0...size-1]
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetSensorPortListEntry(VRmUsbCamDevice f_device, VRmDWORD f_index, VRmDWORD* fp_port);

/// get current source format of given sensor port
/// NOTE: single-sensor devices do only have port #1
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetSourceFormatEx(VRmUsbCamDevice f_device, VRmDWORD f_port, VRmImageFormat* fp_source_format);

/// get string description of source format
/// NOTE: single-sensor devices do only have port #1
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetSourceFormatDescription(VRmUsbCamDevice f_device, VRmDWORD f_port, VRmSTRING* fp_string);

/// utility function: query index in sensor port list for specified sensor port number
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamFindSensorPortListIndex(VRmUsbCamDevice f_device, VRmDWORD f_port, VRmDWORD* fp_index);

/// get number of available target format list entries for current source format at given sensor port
/// NOTE: single-sensor devices do only have port #1
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetTargetFormatListSizeEx2(VRmUsbCamDevice f_device, VRmDWORD f_port, VRmDWORD* fp_size);

/// query target format list entry at given sensor port with index = [0...size-1]
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetTargetFormatListEntryEx2(VRmUsbCamDevice f_device, VRmDWORD f_port, VRmDWORD f_index, VRmImageFormat* fp_target_format);

/// start frame grabber.
/// allocates image buffers (queue size) and starts the first image transfers
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamStart(VRmUsbCamDevice f_device);

/// terminates frame grabber, frees unlocked buffers,
/// locked buffers are kept until released with VRmUsbCamUnlockNextImage
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamStop(VRmUsbCamDevice f_device);

/// check if frame grabber is running
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetRunning(VRmUsbCamDevice f_device, VRmBOOL* fp_running);

/// reset frame counter
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamResetFrameCounter(VRmUsbCamDevice f_device);

/// is next image ready.
/// check if the next image from specified sensor port can be immediately accessed
/// via VRmUsbCamLockNextImage(). if not, you can do something else and check again.
/// specify a port of #0 if the port number doesn't matter.
/// NOTE: single-sensor devices do only have port #1
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamIsNextImageReadyEx(VRmUsbCamDevice f_device, VRmDWORD f_port, VRmBOOL* fp_ready);

/// get a locked source image from a specified sensor port.
/// this functions returns with value VRM_FAIL, if no image is locked within approximately f_timeout_ms milliseconds.
/// use f_timeout_ms=0 to wait indefinitely for a locked image (latency optimal, but dangerous, and only applicable for freerunning/constantly triggered applications)
/// use fp_frames_dropped to see how many frames have been dropped before this image (optional, pass NULL if drops don't matter to you).
/// specify a port of #0 if the port number doesn't matter.
/// use VRmUsbCamConvertImage() to convert source image into a traget image.
/// NOTE: single-sensor devices do only have port #1
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamLockNextImageEx2(VRmUsbCamDevice f_device, VRmDWORD f_port, VRmImage** fpp_image, VRmDWORD* fp_frames_dropped, int f_timeout_ms);

/// obsolete API function to get a locked source image.
/// this function is equivalent to VRmUsbCamLockNextImageEx2() with f_timeout_ms=0.
/// WARNING: this function will never return, if no image becomes ready and will potentially stall your application indefinitely.
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamLockNextImageEx(VRmUsbCamDevice f_device, VRmDWORD f_port, VRmImage** fpp_image, VRmDWORD* fp_frames_dropped);

/// unlock source image.
/// you have to Unlock the image which was locked via VRmUsbCamLockNextImage(),
/// do NOT use VRmUsbCamFreeImage() to free this image.
/// NOTE: if the image has successfully been unlocked, *fpp_image will be set to NULL.
/// nevertheless, the function may return an error when re-queuing the buffer into the
/// grabber ring buffer failed.
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamUnlockNextImage(VRmUsbCamDevice f_device, VRmImage** fpp_image);

/// soft trigger.
/// initiate a soft trigger if the current device supports this, otherwise the
/// function fails
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamSoftTrigger(VRmUsbCamDevice f_device);

/// get the LUT that is associated with a specific sensor port.
/// the format modifier returned is a bit combination of VRM_CORRECTION_LUT_xxx constants.
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetDeviceLut(VRmUsbCamDevice f_device, VRmDWORD f_port, const VRmBYTE** fpp_lut, VRmDWORD* fp_size, VRmDWORD* fp_format_modifier);

/// @}
// ------------------------------------------------------------------------
/// @defgroup Config                                 Configuration Settings
// ------------------------------------------------------------------------
/// @{

/// config id.
/// there are several possible camera configs stored in hardware:
/// VRmUsbCamConfigID = 0 is the factory default config (read-only)
/// VRmUsbCamConfigID = 1 is the user default config (first user config, automatically loaded at VRmUsbCamOpenDevice())
/// VRmUsbCamConfigID = 2 is the second user config
/// ...
/// VRmUsbCamConfigID = 9 is the last user config
typedef unsigned int VRmUsbCamConfigID;

/// load config. valid values for f_id = 0 to 9
/// NOTE: the grabber must be stopped when you call this function.
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamLoadConfig(VRmUsbCamDevice f_device, VRmUsbCamConfigID f_id);

/// saves the current config in hardware. valid values for f_id = 1 to 9
/// NOTE: this might take some seconds (blocking) in case of an necessary firmware compression,
/// use VRmUsbCamSaveConfigRequiresFirmwareCompression to check if this will happen.
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamSaveConfig(VRmUsbCamDevice f_device, VRmUsbCamConfigID f_id);

/// check if next VRmUsbCamSaveConfig() requires a firmware compression. valid values for f_id = 1 to 9.
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamSaveConfigRequiresFirmwareCompression(VRmUsbCamDevice f_device, VRmUsbCamConfigID f_id, VRmBOOL* fp_required);

/// deletes the given config from the device. valid values for f_id = 2 to 9
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamDeleteConfig(VRmUsbCamDevice f_device, VRmUsbCamConfigID f_id);

/// gets current device config en bloque as user data.
/// use VRmUsbCamFreeUserData() to free the returned buffer after usage
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetConfigData(VRmUsbCamDevice f_device, VRmUserData** fpp_data);

/// sets current device config en bloque from user data.
/// activate config data that was previously returned by VRmUsbCamGetConfig
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamSetConfigData(VRmUsbCamDevice f_device, const VRmUserData* fcp_data);

/// returns true if VRmUsbCamSaveConfig() / VRmUsbCamGetConfigData() may drop some values.
/// this happens if the config was initially created by a newer application and contains
/// settings that are not supported by this software version and that therefore will be discarded
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamConfigIncludesUnsupportedValues(VRmUsbCamDevice f_device, VRmBOOL* fp_value);

/// @}
// ------------------------------------------------------------------------
// Property Identifiers
// ------------------------------------------------------------------------

#include "vrmusbcam2props.h"

// ------------------------------------------------------------------------
/// @defgroup Properties                                Property Management
// ------------------------------------------------------------------------
/// @{

/// enumeration of supported property value types.
/// see struct returned from VRmUsbCamGetPropertyInfo()
typedef VRM_ENUM _VRmPropType {
	VRM_PROP_TYPE_BOOL = 1,
	VRM_PROP_TYPE_INT,
	VRM_PROP_TYPE_FLOAT,
	VRM_PROP_TYPE_STRING,
	VRM_PROP_TYPE_ENUM,
	VRM_PROP_TYPE_SIZE_I,
	VRM_PROP_TYPE_POINT_I,
	VRM_PROP_TYPE_RECT_I,
	VRM_PROP_TYPE_DOUBLE
} VRmPropType;

/// general information about a property. see VRmUsbCamGetPropertyInfo()
/// NOTE: the strings within this struct are only guaranteed to be valid until the next API call!
typedef VRM_STRUCT _VRmPropInfo {
	VRmPropId   m_id;
	VRmPropType m_type;
	VRmSTRING   m_id_string;
	VRmSTRING   m_description;
	VRmBOOL     m_writeable;
} VRmPropInfo;

/// @addtogroup BoolProps Properties of type VRM_PROP_TYPE_BOOL
/// @{
typedef VRM_STRUCT _VRmPropAttribsB {
	VRmBOOL m_default;
	VRmBOOL m_min;
	VRmBOOL m_max;
	VRmBOOL m_step;
} VRmPropAttribsB;
/// @}

/// @addtogroup IntProps Properties of type VRM_PROP_TYPE_INT
/// @{
typedef VRM_STRUCT _VRmPropAttribsI {
	int m_default;
	int m_min;
	int m_max;
	int m_step;
} VRmPropAttribsI;
/// @}

/// @addtogroup FloatProps Properties of type VRM_PROP_TYPE_FLOAT
/// @{
typedef VRM_STRUCT _VRmPropAttribsF {
	float m_default;
	float m_min;
	float m_max;
	float m_step;
} VRmPropAttribsF;
/// @}

/// @addtogroup DoubleProps Properties of type VRM_PROP_TYPE_DOUBLE
/// @{
typedef VRM_STRUCT _VRmPropAttribsD {
	double m_default;
	double m_min;
	double m_max;
	double m_step;
} VRmPropAttribsD;
/// @}

/// @addtogroup StringProps Properties of type VRM_PROP_TYPE_STRING.
/// NOTE: the strings within the VRmPropAttribsS struct are only guaranteed to be valid until the next API call!
/// @{
typedef VRM_STRUCT _VRmPropAttribsS {
	VRmSTRING m_default;
	VRmSTRING m_min;
	VRmSTRING m_max;
	VRmSTRING m_step;
} VRmPropAttribsS;
/// @}

/// @addtogroup EnumProps Properties of type VRM_PROP_TYPE_ENUM
/// @{
typedef VRM_STRUCT _VRmPropAttribsE {
	VRmPropId m_default;
	VRmPropId m_min;
	VRmPropId m_max;
	VRmPropId m_step;
} VRmPropAttribsE;
/// @}

/// @addtogroup SizeIProps Properties of type VRM_PROP_TYPE_SIZE_I
/// @{
typedef VRM_STRUCT _VRmPropAttribsSizeI {
	VRmSizeI m_default;
	VRmSizeI m_min;
	VRmSizeI m_max;
	VRmSizeI m_step;
} VRmPropAttribsSizeI;
/// @}

/// @addtogroup PointIProps Properties of type VRM_PROP_TYPE_POINT_I
/// @{
typedef VRM_STRUCT _VRmPropAttribsPointI {
	VRmPointI m_default;
	VRmPointI m_min;
	VRmPointI m_max;
	VRmPointI m_step;
} VRmPropAttribsPointI;
/// @}

/// @addtogroup RectIProps Properties of type VRM_PROP_TYPE_RECT_I
/// @{
typedef VRM_STRUCT _VRmPropAttribsRectI {
	VRmRectI m_default;
	VRmRectI m_min;
	VRmRectI m_max;
	VRmRectI m_step;
} VRmPropAttribsRectI;
/// @}

/// get number of available properties
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetPropertyListSize(VRmUsbCamDevice f_device, VRmDWORD* fp_size);

/// get identifier of property with index [0...size-1]
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetPropertyListEntry(VRmUsbCamDevice f_device, VRmDWORD f_index, VRmPropId* fp_id);

/// get info struct of property
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetPropertyInfo(VRmUsbCamDevice f_device, VRmPropId f_id, VRmPropInfo* fp_info);

/// check whether some specific property is supported
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetPropertySupported(VRmUsbCamDevice f_device, VRmPropId f_id, VRmBOOL* fp_supported);

/// @addtogroup BoolProps Properties of type VRM_PROP_TYPE_BOOL
/// @{
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetPropertyValueB(VRmUsbCamDevice f_device, VRmPropId f_id, VRmBOOL* fp_value);
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamSetPropertyValueB(VRmUsbCamDevice f_device, VRmPropId f_id, const VRmBOOL* fcp_value);
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetPropertyAttribsB(VRmUsbCamDevice f_device, VRmPropId f_id, VRmPropAttribsB* fp_attribs);
/// @}

/// @addtogroup IntProps Properties of type VRM_PROP_TYPE_INT
/// @{
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetPropertyValueI(VRmUsbCamDevice f_device, VRmPropId f_id, int* fp_value);
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamSetPropertyValueI(VRmUsbCamDevice f_device, VRmPropId f_id, const int* fcp_value);
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetPropertyAttribsI(VRmUsbCamDevice f_device, VRmPropId f_id, VRmPropAttribsI* fp_attribs);
/// @}

/// @addtogroup FloatProps Properties of type VRM_PROP_TYPE_FLOAT
/// @{
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetPropertyValueF(VRmUsbCamDevice f_device, VRmPropId f_id, float* fp_value);
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamSetPropertyValueF(VRmUsbCamDevice f_device, VRmPropId f_id, const float* fcp_value);
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetPropertyAttribsF(VRmUsbCamDevice f_device, VRmPropId f_id, VRmPropAttribsF* fp_attribs);
/// @}

/// @addtogroup DoubleProps Properties of type VRM_PROP_TYPE_DOUBLE
/// @{
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetPropertyValueD(VRmUsbCamDevice f_device, VRmPropId f_id, double* fp_value);
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamSetPropertyValueD(VRmUsbCamDevice f_device, VRmPropId f_id, const double* fcp_value);
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetPropertyAttribsD(VRmUsbCamDevice f_device, VRmPropId f_id, VRmPropAttribsD* fp_attribs);
/// @}

/// @addtogroup StringProps Properties of type VRM_PROP_TYPE_STRING.
/// NOTE: the string returned by VRmUsbCamGetPropertyValueS() is only guaranteed to be valid until the next API call!
/// @{
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetPropertyValueS(VRmUsbCamDevice f_device, VRmPropId f_id, VRmSTRING* fp_value);
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamSetPropertyValueS(VRmUsbCamDevice f_device, VRmPropId f_id, const VRmSTRING* fcp_value);
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetPropertyAttribsS(VRmUsbCamDevice f_device, VRmPropId f_id, VRmPropAttribsS* fp_attribs);
/// @}

/// @addtogroup EnumProps Properties of type VRM_PROP_TYPE_ENUM
/// @{
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetPropertyValueE(VRmUsbCamDevice f_device, VRmPropId f_id, VRmPropId* fp_value);
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamSetPropertyValueE(VRmUsbCamDevice f_device, VRmPropId f_id, const VRmPropId* fcp_value);
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetPropertyAttribsE(VRmUsbCamDevice f_device, VRmPropId f_id, VRmPropAttribsE* fp_attribs);
/// @}

/// @addtogroup SizeIProps Properties of type VRM_PROP_TYPE_SIZE_I
/// @{
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetPropertyValueSizeI(VRmUsbCamDevice f_device, VRmPropId f_id, VRmSizeI* fp_value);
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamSetPropertyValueSizeI(VRmUsbCamDevice f_device, VRmPropId f_id, const VRmSizeI* fcp_value);
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetPropertyAttribsSizeI(VRmUsbCamDevice f_device, VRmPropId f_id, VRmPropAttribsSizeI* fp_attribs);
/// @}

/// @addtogroup PointIProps Properties of type VRM_PROP_TYPE_POINT_I
/// @{
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetPropertyValuePointI(VRmUsbCamDevice f_device, VRmPropId f_id, VRmPointI* fp_value);
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamSetPropertyValuePointI(VRmUsbCamDevice f_device, VRmPropId f_id, const VRmPointI* fcp_value);
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetPropertyAttribsPointI(VRmUsbCamDevice f_device, VRmPropId f_id, VRmPropAttribsPointI* fp_attribs);
/// @}

/// @addtogroup RectIProps Properties of type VRM_PROP_TYPE_RECT_I
/// @{
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetPropertyValueRectI(VRmUsbCamDevice f_device, VRmPropId f_id, VRmRectI* fp_value);
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamSetPropertyValueRectI(VRmUsbCamDevice f_device, VRmPropId f_id, const VRmRectI* fcp_value);
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamGetPropertyAttribsRectI(VRmUsbCamDevice f_device, VRmPropId f_id, VRmPropAttribsRectI* fp_attribs);
/// @}

/// @}
// ------------------------------------------------------------------------
/// @defgroup Misc                                    Misc Functions
// ------------------------------------------------------------------------
/// @{
/// Save a VRmImage to a PNG file. Compression level 0 is uncompressed, max. is 9, use default with -1.
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamSavePNG(VRmSTRING fcp_file_name, const VRmImage* fcp_image, int f_z_compression_level);
/// Load a VRmImage from a PNG file.
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamLoadPNG(VRmSTRING fcp_file_name, VRmImage** fpp_image);
/// @}

// ------------------------------------------------------------------------
/// @defgroup Callbacks                                    Callbacks/Events
// ------------------------------------------------------------------------
/// @{

/// VRmUsbCam (Static) Callback Types.
/// see VRmUsbCamRegisterStaticCallback().
typedef VRM_ENUM _VRmStaticCallbackType {
	/// A device change event was signaled by the system.
	/// params: a pointer to a VRmDeviceChangeType.
	/// note: this callback is only emitted from within
	/// VRmUsbCamUpdateDeviceKeyList()
	VRM_STATIC_CALLBACK_TYPE_DEVICE_CHANGE = 1,
	/// a CMem allocation or deallocation was performed. this is only
	/// relevant for Davinci based cameras.
	/// params: a pointer to a VRmStaticCallbackCMemAllocationChangeParams struct
	VRM_STATIC_CALLBACK_TYPE_CMEM_ALLOCATION_CHANGE = 2
} VRmStaticCallbackType;

/// Parameter definition for VRM_STATIC_CALLBACK_TYPE_DEVICE_CHANGE callback
typedef VRM_ENUM _VRmDeviceChangeType {
	/// indicates a device arrival
	VRM_DEVICE_CHANGE_TYPE_ARRIVAL = 1,
	/// indicates the completion of a device removal
	VRM_DEVICE_CHANGE_TYPE_REMOVECOMPLETE = 2,
	/// indicates that a device changed its busy state
	VRM_DEVICE_CHANGE_TYPE_BUSY = 3
} VRmDeviceChangeType;

/// parameter definition for VRM_STATIC_CALLBACK_TYPE_CMEM_ALLOCATION_CHANGE callback
typedef VRM_STRUCT _VRmStaticCallbackCMemAllocationChangeParams {
	/// type of event: true= allocation, false= deallocation
	VRmBOOL  m_allocate;
	/// virtual address of memory area
	void*    mp_virtual;
	/// physical address of memory area
	void*    mp_physical;
	/// size (in bytes) of memory area
	VRmDWORD m_size;
} VRmStaticCallbackCMemAllocationChangeParams;

/// Callback function signature definition for VRmUsbCamRegisterStaticCallback()
typedef void (VRM_API *VRmStaticCallback)(
	/// the type of callback
	VRmStaticCallbackType f_type,
	/// user data pointer as passed into VRmUsbCamRegisterStaticCallback()
	void* fp_user_data,
	/// optional additional callback parameters
	const void* fcp_callback_params);

/// register a static callback function.
/// NOTE: a specific callback function pointer can only be registered once per device.
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamRegisterStaticCallback(VRmStaticCallback fp_callback, void* fp_user_data);

/// unregister a static callback function
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamUnregisterStaticCallback(VRmStaticCallback fp_callback);

/// register a static callback function.
/// NOTE: in contrast to VRmUsbCamRegisterStaticCallback(), this functions allows
/// registering a specific callback function pointer multiple times as
/// long as fp_user_data is different each time.
/// you should use VRmUsbCamUnregisterStaticCallbackEx() for later unregistration
/// specifying the same function pointer and fp_user_data pair as used for
/// registration.
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamRegisterStaticCallbackEx(VRmStaticCallback fp_callback, void* fp_user_data);

/// unregister a static callback function.
/// specify the same function pointer and fp_user_data as used at registration.
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamUnregisterStaticCallbackEx(VRmStaticCallback fp_callback, void* fp_user_data);

/// Device Callback Types.
/// see VRmUsbCamRegisterDeviceCallbackEx().
/// these callbacks may be called synchronously from within any VRmUsbCamXXX() functions!
typedef VRM_ENUM _VRmDeviceCallbackType {
	// NOTE: type 0 is reserved
	/// The device LUT changed.
	/// params: a pointer to a VRmDWORD with the associated sensor port id
	VRM_DEVICE_CALLBACK_TYPE_LUT_CHANGED = 1,
	/// The device Source Format changed.
	/// (no additional callback params)
	VRM_DEVICE_CALLBACK_TYPE_SOURCE_FORMAT_CHANGED = 2,
	/// A device Property Value changed.
	/// params: a pointer to a VRmDWORD with the property id
	VRM_DEVICE_CALLBACK_TYPE_PROPERTY_VALUE_CHANGED = 3,
	/// The list of device properties changed.
	/// (no additional callback params)
	VRM_DEVICE_CALLBACK_TYPE_PROPERTY_LIST_CHANGED = 4,
	/// The list of device Source Formats changed.
	/// (no additional callback params)
	/// **THIS CALLBACK IS LEGACY**
	VRM_DEVICE_CALLBACK_TYPE_SOURCE_FORMAT_LIST_CHANGED = 5,
	/// The list of device Target Formats changed.
	/// (no additional callback params)
	VRM_DEVICE_CALLBACK_TYPE_TARGET_FORMAT_LIST_CHANGED = 6,
	/// Some device Property Info changed.
	/// params: a pointer to a VRmDWORD with the property id
	VRM_DEVICE_CALLBACK_TYPE_PROPERTY_INFO_CHANGED = 7,
	/// Some device Property Attribs changed.
	/// params: a pointer to a VRmDWORD with the property id
	VRM_DEVICE_CALLBACK_TYPE_PROPERTY_ATTRIBS_CHANGED = 8
} VRmDeviceCallbackType;

/// Callback function signature definition for VRmUsbCamRegisterDeviceCallbackEx()
typedef void (VRM_API *VRmDeviceCallback)(
	/// the device handle
	VRmUsbCamDevice f_device,
	/// the type of callback
	VRmDeviceCallbackType f_type,
	/// user data pointer as passed into VRmUsbCamRegisterDeviceCallbackEx()
	void* fp_user_data,
	/// optional additional callback parameters
	const void* fcp_callback_params);

/// register a callback function for the given device.
/// NOTE: in contrast to VRmUsbCamRegisterDeviceCallback(), this functions allows
/// registering a specific callback function pointer multiple times per device as
/// long as fp_user_data is different each time.
/// you should use VRmUsbCamUnregisterDeviceCallbackEx() for later unregistration
/// specifying the same function pointer and fp_user_data pair as used for
/// registration.
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamRegisterDeviceCallbackEx(VRmUsbCamDevice f_device, VRmDeviceCallback fp_callback, void* fp_user_data);

/// unregister a callback function for the given device.
/// specify the same function pointer and fp_user_data as used at registration.
VRM_EXTERN VRmRetVal VRM_API VRmUsbCamUnregisterDeviceCallbackEx(VRmUsbCamDevice f_device, VRmDeviceCallback fp_callback, void* fp_user_data);

/// @}
// ------------------------------------------------------------------------
/// @defgroup VM_LIB                            VM_LIB related functions
// ------------------------------------------------------------------------
/// @{
VRM_EXTERN VRmRetVal VRM_API VRmCreateVMLIBKey(VRmDWORD* fp_vmlib_key);
VRM_EXTERN VRmRetVal VRM_API VRmCreateVMLIBKeyDsp(VRmDWORD* fp_vmlib_key);
/// @}

#if defined(__cplusplus) && !defined(VRM_NO_EXTERN_C)
};// extern "C"
#endif

#endif //VRMUSBCAM2_H
