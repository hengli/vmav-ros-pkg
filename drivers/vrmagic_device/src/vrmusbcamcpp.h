// ==============================================================================================
/// @file vrmusbcamcpp.h                                           VRmUsbCam C++ Wrapper v3.1.0.1
//  Copyright VRmagic 2013
//
//  vrmusbcamcpp.cpp
// ==============================================================================================

#ifndef VRMUSBCAMCPP_H
#define VRMUSBCAMCPP_H

// STL
#include <string>
#include <exception>
#include <vector>
#include <list>

// boost
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/variant.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>

// C API
#include "vrmusbcam2.h"

// System
#ifdef WIN32
#include "vrmusbcam2win32.h"
#endif

// API private functions (only consumed by this wrapper)

extern "C" VRmRetVal VRmUsbCamPrivateLock();
extern "C" VRmRetVal VRmUsbCamPrivateUnlock();


namespace VRmUsbCamCPP {

	// forward declarations -------------------------------------------------------------------------

	class Exception;
	class DeviceKey;
	class ImageFormat;
	class Image;
	class DevicePropertyPage;
	class Device;
	class VRmUsbCam;

	// helper types ---------------------------------------------------------------------------------

	typedef boost::shared_ptr<DeviceKey> DeviceKeyPtr;
	typedef boost::shared_ptr<Image> ImagePtr;
	typedef boost::shared_ptr<DevicePropertyPage> DevicePropertyPagePtr;
	typedef boost::shared_ptr<Device> DevicePtr;

	// enums ----------------------------------------------------------------------------------------


	/// typedef for error codes
	typedef int ErrorNumber;
	/// enum for color formats
	typedef VRmColorFormat ColorFormat;
	/// enum for image modifiers
	typedef VRmImageModifier ImageModifier;
	/// enum for device change event types
	typedef VRmDeviceChangeType DeviceChangeType;
	/// enum for property identifiers
	typedef VRmPropId PropId;

	// value structs --------------------------------------------------------------------------------

	/// "size" type (int)
	struct SizeI : VRmSizeI { SizeI(int width=0, int height=0); };
	/// "point" type (int)
	struct PointI : VRmPointI { PointI(int x=0, int y=0); };
	/// "rect" type (int)
	struct RectI : VRmRectI { RectI(int left=0, int top=0, int width=0, int height=0); };

	/// general information about a property. see Device.get_PropertyInfo()
	struct PropInfo : VRmPropInfo
	{
		std::string ToString() const;
	};

	/// property variant
	typedef boost::variant<bool, int, float, double, std::string, PropId, SizeI, PointI, RectI> Variant;

	/// attributes of a property, see Device.get_PropertyAttribs()
	struct PropAttribs
	{
		Variant m_default;
		Variant m_min;
		Variant m_max;
		Variant m_step;
	};

	/// empty arguments of events
	struct EventArgs
	{
		virtual ~EventArgs();
	};

	// Exception ---------------------------------------------------------------------------------------

	/// Exception that is thrown from within any component of VRmUsbCam.
	/// in addition to the standard exception members, this class provides an
	/// ErrorNumber
	class Exception: public std::exception
	{
	public:
		/// Text message associated with this exception
		std::string get_Description() const;
		/// Error number associated with this exception
		ErrorNumber get_Number() const;

		/// default c-tor
		explicit Exception(const std::string& message, ErrorNumber number = VRM_ERROR_CODE_GENERIC_ERROR);

#ifndef WIN32
		virtual const char* what() const throw ();
		virtual ~Exception() throw ();
#endif

	private:
		ErrorNumber m_number;
#ifndef WIN32
		std::string m_message;
#endif
	};


	// DeviceKey --------------------------------------------------------------------------------------

	/// identifies a device.
	/// a device key is a unique combination of serial, manufacturer
	/// and product string
	class DeviceKey : boost::noncopyable
	{
	public:
		/// get combined product/serial string
		std::string ToString() const;
		/// manufacturer string
		std::string get_Manufacturer() const;
		/// product string
		std::string get_Product() const;
		/// device serial number
		int get_Serial() const;
		/// device serial string
		std::string get_SerialString() const;
		/// is device currently opened (busy)?
		bool get_Busy() const;
		/// vendor id
		int get_VendorId() const;
		/// product id
		int get_ProductId() const;
		/// group id
		int get_GroupId() const;
		/// IP address string (for non-ethernet devices, this is an empty string)
		std::string get_IpAddress() const;
		/// IP address string of local interface that is to be used for communication (for non-ethernet devices, this is an empty string)
		std::string get_LocalIpAddress() const;

		bool operator==(const DeviceKeyPtr& other) const;
		bool operator!=(const DeviceKeyPtr& other) const;

		/// frees all resources occupied by this device key
		~DeviceKey();

		VRmDeviceKey* get_NativeRepresentation() const;

	private:
		friend class Device;
		friend class VRmUsbCam;
		DeviceKey(VRmDeviceKey* fp_key);
		VRmDeviceKey* mp_key;
	};

	// ImageFormat ---------------------------------------------------------------------------------------

	/// represents a VRmUsbCam image format
	class ImageFormat
	{
	public:
		/// create a new (empty) image format
		ImageFormat();
		/// create a new image format from the given size, color format and image modifiers
		ImageFormat(const SizeI& size, ColorFormat colorformat, ImageModifier imagemodifier);

		/// size of image in pixels
		void set_Size(const SizeI& size);
		/// size of image in pixels
		SizeI get_Size() const;

		/// color format
		void set_ColorFormat(ColorFormat colorformat);
		/// color format
		ColorFormat get_ColorFormat() const;

		/// bit mask of image modifiers
		void set_ImageModifier(ImageModifier imagemodifier);
		/// bit mask of image modifiers
		ImageModifier get_ImageModifier() const;

		/// list of formats this format can be converted to by VRmUsbCam::ConvertImage
		std::vector<ImageFormat> get_TargetFormatList() const;

		/// pixel depth (in bytes) of this format
		int get_PixelDepth() const;
		/// description of this format (only used for Device Source Formats)
		std::string get_Description() const;

		/// creates a user-readable string representation of this format
		std::string ToString() const;
		/// toggles the modifier for horizontal flip
		void FlipHorizontal();
		/// toggles the modifier for vertical flip
		void FlipVertical();

		bool operator==(const ImageFormat& other) const;
		bool operator!=(const ImageFormat& other) const;

		const VRmImageFormat& get_NativeRepresentation() const;
		ImageFormat(const VRmImageFormat& format);

	private:
		friend class Device;
		ImageFormat(const VRmImageFormat& format, const std::string& description);

		void updateDescription();

		VRmImageFormat m_vrmformat;
		std::string m_description;
	};

	// Image ---------------------------------------------------------------------------------------------

	/// image container
	class Image : boost::noncopyable, public boost::enable_shared_from_this<Image>
	{
	public:
		/// special value for BufferSize
		static const int BUFFER_SIZE_MAX;

		/// creates a new image as deep copy of this image
		ImagePtr Clone() const;

		/// ImageFormat of image
		ImageFormat get_ImageFormat() const;
		/// timestamp of image as received from grabber
		double get_TimeStamp() const;
		/// pitch (in bytes) of the stored image data
		int get_Pitch() const;
		/// pointer to the contained image buffer
		void* get_Buffer() const;
		/// size (in bytes) of the contained image buffer
		void set_BufferSize(int size);
		/// frame Counter of Image
		int get_BufferSize() const;
		/// size (in bytes) of the contained image buffer
		int get_FrameCounter() const;
		/// Image Sensor Port this image originates from
		int get_SensorPort() const;
		/// Footer Data of Image
		std::vector<unsigned char> get_Footer() const;
		/// get cropped part of this Image without copying of image data.
		/// the image data of the returned image is actually shared with this image.
		ImagePtr Crop(const RectI& rect);

		/// frees all resources occupied by this image
		~Image();

		VRmImage* get_NativeRepresentation() const;

	private:
		friend class Device;
		friend class VRmUsbCam;

		typedef boost::variant< DevicePtr, ImagePtr > tOwnerPtr;

		// constructor used by Device::LockNextImage(), Image::Crop() and Image::Clone()
		Image(VRmImage* fp_vrmimage, tOwnerPtr fp_owner);

		// constructor used by VRmUsbCam::NewImage()
		Image(const ImageFormat& format);

		// constructor used by VRmUsbCam::SetImage()
		Image(const ImageFormat& format, void* buffer, int pitch);

		// pointer to original vrm image
		VRmImage* mp_vrmimage;

		// if received by LockNextImage, this is a pointer to the associated Device;
		// if this is a cropped image, this is a pointer to the associated original image.
		// this pointer is only for means of ref-counting, don't access it!
		tOwnerPtr mp_owner;
	};


	// DevicePropertyPage ------------------------------------------------------------------------------

#ifdef WIN32
	/// device property page (device GUI)
	class DevicePropertyPage : boost::noncopyable
	{
	public:
		/// move and resize the page to the given position and size
		void Move(const RectI& rect);
		/// the size this property page looks best
		SizeI get_SizeHint() const;
		/// show the property page (only valid if hWndParent is null)
		void Show();
		/// hide the property page (only valid if hWndParent is null)
		void Hide();
		/// size of the page
		void set_Size(const SizeI& size);
		/// size of the page
		SizeI get_Size() const;
		/// position of the page
		void set_Location(const PointI& location);
		/// position of the page
		PointI get_Location() const;
		/// visibility state of the page
		bool get_Visible() const;
		/// window handle of the property page
		HWND get_Handle() const;

		/// close the property page and free all occupied resources
		~DevicePropertyPage();

		VRmDevicePropertyPage* get_NativeRepresentation() const;

	private:
		friend class Device;

		explicit DevicePropertyPage(VRmDevicePropertyPage* fp_vrmpage);

		VRmDevicePropertyPage* mp_vrmpage;
	};
#endif//WIN32

	// Device ------------------------------------------------------------------------------------------

	/// represents a physical VRmUsbCam device
	class Device : boost::noncopyable, public boost::enable_shared_from_this<Device>
	{
	public:
		/// start image grabbing
		void Start();
		/// stop image grabbing, resources of unlocked images are freed, locked images are kept until released with UnlockNextImage
		void Stop();
		/// check whether next image is ready at specified sensor port and can immediately be locked by LockNextImage
		bool IsNextImageReady(int port) const;

		/// close the device and free all occupied resources
		void Close();

		/// lock next grabbed image at specified sensor port in order to access it. unlock afterwards by UnlockNextImage.
		ImagePtr LockNextImage(int port, int* framesDropped = 0, int timeoutMs = 0);
		/// unlock an Image that was previously locked by LockNextImage
		void UnlockNextImage(ImagePtr& image);

		/// initiate a soft trigger (device must supports this)
		void SoftTrigger();

		/// load user data from eeprom
		std::vector<unsigned char> LoadUserData();
		/// save user data in eeprom
		void SaveUserData(const std::vector<unsigned char>& data);

		/// load config from the device, valid values for id = 0 to 9. id=0 means factory defaults. the grabber must be stopped when you call this.
		void LoadConfig(int id);
		/// saves the current config in hardware, valid values for f_id = 1 to 9
		void SaveConfig(int id);
		/// check if next SaveConfig requires a firmware compression which takes some seconds (blocking), valid values for id = 1 to 9
		bool SaveConfigRequiresFirmwareCompression(int id);
		/// deletes the given config from the device, valid values for f_id = 2 to 9
		void DeleteConfig(int id);

		/// reset Frame Counter
		void ResetFrameCounter();

#ifdef WIN32
		/// create a property page for the device.
		/// if hWndParent is null, a new independent window is created,
		/// if hWndParent is not null, a child window is created within the given parent.
		/// rect determines the initial window position.
		DevicePropertyPagePtr CreatePropertyPage(HWND hWndParent, const RectI& rect);
		/// create a property page for the device (as independent window)
		DevicePropertyPagePtr CreatePropertyPage();
#endif//WIN32

		/// Utility function: query index in sensor port list for specified sensor port number
		int FindSensorPortListIndex(int port) const;

		// properties

		/// DeviceKey of the associated physical device
		DeviceKeyPtr get_DeviceKey() const;

		/// currently selected source format
		ImageFormat get_SourceFormat(int port) const;

		/// list of image sensor ports
		std::vector<int> get_SensorPortList() const;

		/// list of formats the currently selected source format of the specified port can be converted to by VRmUsbCam.ConvertImage
		std::vector<ImageFormat> get_TargetFormatList(int port) const;

		/// current device config in native byte-array representation
		void set_ConfigData(const std::vector<unsigned char>& data);
		/// current device config in native byte-array representation
		std::vector<unsigned char> get_ConfigData() const;

		/// true if SaveConfig()/ConfigData may drop some values
		bool get_ConfigIncludesUnsupportedValues() const;

		/// determines whether the frame grabber is running
		bool get_Running() const;

		// property interface functions

		/// list of supported properties
		std::vector<PropId> get_PropertyList() const;
		/// PropInfo struct of specified property
		PropInfo get_PropertyInfo(PropId propid) const;
		/// determines whether the specified property is supported
		bool get_PropertySupported(PropId propid) const;

		/// attributes of property with given id
		PropAttribs get_PropertyAttribs(PropId propid) const;

		/// value of property with given id
		void set_PropertyValue(PropId propid, const Variant& value);
		/// value of property with given id
		Variant get_PropertyValue(PropId propid) const;


		// events


		/// event helper class for Device events
		template <typename Delegate>
		class Event : boost::noncopyable
		{
		public:
			/// add a handler of this event
			template <typename T> void add(const T& eh)
			{
				VRmUsbCamPrivateLock();
				try
				{
					m_handlers.push_back(eh);
				}
				catch (const std::exception&)
				{
					VRmUsbCamPrivateUnlock();
					throw;
				}
				VRmUsbCamPrivateUnlock();
			}
			template <typename T> Event& operator+=(const T& eh)
			{
				add(eh);
				return *this;
			}
			/// remove a handler of this event
			template <typename T> void remove(const T& eh)
			{
				VRmUsbCamPrivateLock();
				try
				{
					typename std::list<Delegate>::iterator li= std::find(m_handlers.begin(), m_handlers.end(), eh);
					if (li != m_handlers.end())
						m_handlers.erase(li);
				}
				catch (const std::exception&)
				{
					VRmUsbCamPrivateUnlock();
					throw;
				}
				VRmUsbCamPrivateUnlock();
			}
			template <typename T> Event& operator-=(const T& eh)
			{
				remove(eh);
				return *this;
			}
		private:
			friend class Device;
			std::list<Delegate> m_handlers;
		};

		/// delegate type of Device events with no arguments
		typedef boost::function< void (const DevicePtr& sender, const EventArgs& e) > EventHandler;


		/// arguments of Device.LutChanged event
		struct LutChangedEventArgs : public EventArgs
		{
		public:
			/// creates a new LutChangedEventArgs
			LutChangedEventArgs(int sensorPort) : m_sensor_port(sensorPort) {}
			/// associated sensor port id
			int m_sensor_port;
		};

		/// delegate type of Device.LutChanged event
		typedef boost::function< void (const DevicePtr& sender, const LutChangedEventArgs& e) > LutChangedEventHandler;


		/// arguments of Device.PropertyInfoChanged event
		struct PropertyInfoChangedEventArgs : public EventArgs
		{
		public:
			/// creates a new PropertyInfoChangedEventArgs
			PropertyInfoChangedEventArgs(PropId propid) : m_propid(propid) {}
			/// associated property id
			PropId m_propid;
		};

		/// delegate type of Device.PropertyInfoChanged event
		typedef boost::function< void (const DevicePtr& sender, const PropertyInfoChangedEventArgs& e) > PropertyInfoChangedEventHandler;


		/// arguments of Device.PropertyValueChanged event
		struct PropertyValueChangedEventArgs : public EventArgs
		{
			/// creates a new PropertyValueChangedEventArgs
			PropertyValueChangedEventArgs(PropId propid) : m_propid(propid) {}
			/// associated property id
			PropId m_propid;
		};

		/// delegate type of Device.PropertyValueChanged event
		typedef boost::function< void (const DevicePtr& sender, const PropertyValueChangedEventArgs& e) > PropertyValueChangedEventHandler;


		/// arguments of Device.PropertyAttribsChanged event
		struct PropertyAttribsChangedEventArgs : public EventArgs
		{
		public:
			/// creates a new PropertyAttribsChangedEventArgs
			PropertyAttribsChangedEventArgs(PropId propid) : m_propid(propid) {}
			/// associated property id
			PropId m_propid;
		};

		/// delegate type of Device.PropertyAttribsChanged event
		typedef boost::function< void (const DevicePtr& sender, const PropertyAttribsChangedEventArgs& e) > PropertyAttribsChangedEventHandler;


		/// the Device LUT changed
		Event<LutChangedEventHandler> LutChanged;
		/// the Device Source Format changed
		Event<EventHandler> SourceFormatChanged;
		/// a Device PropertyValue changed
		Event<PropertyValueChangedEventHandler> PropertyValueChanged;
		/// the list of Device Properties changed
		Event<EventHandler> PropertyListChanged;
		/// the list of Device Target Formats changed
		Event<EventHandler> TargetFormatListChanged;
		/// some Device PropertyInfo changed
		Event<PropertyInfoChangedEventHandler> PropertyInfoChanged;
		/// some Device PropertyAttribs changed
		Event<PropertyAttribsChangedEventHandler> PropertyAttribsChanged;


		VRmUsbCamDevice get_NativeRepresentation() const;

		~Device();

	private:
		friend class VRmUsbCam;

		explicit Device(const DeviceKeyPtr& key);

		VRmUsbCamDevice m_device;

		// this pointer stores an exception that was thrown meanwhile a device callback
		mutable boost::thread_specific_ptr<std::string> mp_last_callback_exception;
		void checkCallbackExceptions() const;

		// callback/event handling
		static void DeviceCallbackProxy(VRmUsbCamDevice f_device, VRmDeviceCallbackType f_type, void* fp_user_data, const void* fcp_callback_params);
		void RaiseEvent(VRmDeviceCallbackType f_type, const void* fcp_callback_params);

		bool m_callbacks_registered;
	};



	// VRmUsbCam -------------------------------------------------------------------------------------

	/// VRmUsbCam root class
	class VRmUsbCam
	{
	public:
		/// for customer support, enable the logging facilities
		static void EnableLogging();
		/// for customer support, enable the logging facilities (with selectable file name)
		static void EnableLoggingEx(const std::string& logFileName);
		/// search for compatible devices and update DeviceKeyList property. if your application wants to support PnP, you should call this function periodically, at least once every 5 seconds, and handle the DeviceChange events
		static void UpdateDeviceKeyList();
		/// restart reference timer
		static void RestartTimer();
		/// convert the given source Image into the buffer of the given target image.
		/// the target image must be valid, ie. created by NewImage, SetImage or as a copy of another Image
		static void ConvertImage(const ImagePtr& source, ImagePtr& target);
		/// create a new Device Object and bind to the specified physical device
		static DevicePtr OpenDevice(DeviceKeyPtr key);
		/// creates a new Image object in the specified ImageFormat
		static ImagePtr NewImage(const ImageFormat& format);
		/// create a new Image as container for a given buffer.
		/// the buffer must remain valid for the lifetime of the created object and is not automatically deallocated when the Image object is released.
		static ImagePtr SetImage(const ImageFormat& format, void* buffer, int pitch);
		/// save an image as a PNG file
		static void SavePNG(const std::string& fileName, const ImagePtr& source, int z_compession_level = -1);
		/// load an image from a PNG file
		static ImagePtr LoadPNG(const std::string& fileName);

#ifdef WIN32
		/// activates a VRmagic USB device in the current hardware profile, identified by vendor id, product id and serial number
		static void ActivateDevice(int vendorId, int productId, int serial);
		/// activates all VRmagic USB devices in the current hardware profile that are currently deactivated
		static void ActivateAllDevices();
		/// deactivates a VRmagic USB device in the current hardware profile, identified by vendor id, product id and serial number
		static void DeactivateDevice(int vendorId, int productId, int serial);
#endif

		/// current time in ms since last call to RestartTimer
		static double get_CurrentTime();

		/// list of currently connected VRmagic devices
		static std::vector<DeviceKeyPtr> get_DeviceKeyList();

		/// get the version of the API.
		/// the version number is represented as decimal integer with 4 digits, ie. API version v2.3.0.0 is represented as decimal 2300.
		static int get_Version();


		// events


		/// event helper class for VRmUsbCam events
		template <typename Delegate>
		class Event : boost::noncopyable
		{
		public:
			/// add a handler of this event
			template <typename T> void add(const T& eh)
			{
				VRmUsbCamPrivateLock();
				try
				{
					if (m_handlers.empty())
						VRmUsbCamRegisterStaticCallback(VRmUsbCamCallbackProxy, 0);
					m_handlers.push_back(eh);
				}
				catch (const std::exception&)
				{
					VRmUsbCamPrivateUnlock();
					throw;
				}
				VRmUsbCamPrivateUnlock();
			}
			template <typename T> Event& operator+=(const T& eh)
			{
				add(eh);
				return *this;
			}
			/// remove a handler of this event
			template <typename T> void remove(const T& eh)
			{
				VRmUsbCamPrivateLock();
				try
				{
					typename std::list<Delegate>::iterator li= std::find(m_handlers.begin(), m_handlers.end(), eh);
					if (li != m_handlers.end())
						m_handlers.erase(li);
					if (m_handlers.empty())
						VRmUsbCamUnregisterStaticCallback(VRmUsbCamCallbackProxy);
				}
				catch (const std::exception&)
				{
					VRmUsbCamPrivateUnlock();
					throw;
				}
				VRmUsbCamPrivateUnlock();
			}
			template <typename T> Event& operator-=(const T& eh)
			{
				remove(eh);
				return *this;
			}
			~Event();
		private:
			friend class VRmUsbCam;
			std::list<Delegate> m_handlers;
		};

		/// arguments of DeviceChange events
		struct DeviceChangeEventArgs : public EventArgs
		{
			DeviceChangeEventArgs(DeviceChangeType type) : m_type(type) {}
			/// type of change that caused the event
			DeviceChangeType m_type;
		};

		/// delegate type of DeviceChange event
		typedef boost::function< void (const DeviceChangeEventArgs& e) > DeviceChangeEventHandler;

		/// a device change event was signaled by the system. this event is only emitted from within UpdateDeviceKeyList()
		static Event<DeviceChangeEventHandler> DeviceChange;

	private:
		// callback/event handling
		static void VRmUsbCamCallbackProxy(VRmStaticCallbackType f_type, void* fp_user_data, const void* fcp_callback_params);
		static void RaiseEvent(VRmStaticCallbackType f_type, const void* fcp_callback_params);

		VRmUsbCam(); // we do not want to be created
	};


} // namespace VRmUsbCamCPP

#endif//VRMUSBCAMCPP_H

