
#include "vrmusbcamcpp.h"

// c++ headers

#include <exception>
#include <memory>

// boost headers

#include <boost/format.hpp>
#include <boost/foreach.hpp>

// system-dependent includes

#ifdef WIN32
#include <windows.h>
#endif//WIN32


// API private functions (only consumed by this wrapper)

extern "C" VRmRetVal VRmUsbCamPrivateLockDevice(VRmUsbCamDevice f_device);
extern "C" VRmRetVal VRmUsbCamPrivateUnlockDevice(VRmUsbCamDevice f_device);

namespace VRmUsbCamCPP {

	// return-code to exception wrapper -----------------------------------------------------------------------

#define VRMERR(text) \
	{ throw Exception(text, VRM_ERROR_CODE_GENERIC_ERROR); }

#define VRMERR_WITHCODE(text, code) \
	{ throw Exception(text, code); }

	static std::string makeErrorDescription() {
		return std::string(VRmUsbCamGetLastError());
	}

	static ErrorNumber makeErrorNumber() {
		return static_cast<ErrorNumber>(VRmUsbCamGetLastErrorCode());
	};

#define VRMCALL(function) \
	{ \
	if (VRM_SUCCESS!=VRmUsbCam##function) \
	VRMERR_WITHCODE(makeErrorDescription(), makeErrorNumber());	\
	}

#define VRMDEVCALL(function)    /* VRMCALL() for device functions, checks VRmDeviceCallback exceptions. */ \
	{ \
	if (VRM_SUCCESS!=VRmUsbCam##function) \
	VRMERR_WITHCODE(makeErrorDescription(), makeErrorNumber());	\
	checkCallbackExceptions(); \
	}

	void Device::checkCallbackExceptions() const
	{
		std::string* lp_callback_exc= mp_last_callback_exception.get();
		if (lp_callback_exc)
		{
			std::string l_callback_exc= *lp_callback_exc;
			mp_last_callback_exception.reset();
			VRMERR(l_callback_exc);
		}
	}

#define COMCALL(function) \
	{ HRESULT res=(function); \
	if FAILED(res) \
	VRMERR_WITHCODE("function call \"" #function "\" failed", res); \
	}

#define VRMCHECK(condition) \
	{ if (!(condition)) \
	VRMERR("condition \"" #condition "\" is not met"); \
	}

#define null 0		// definitions for VRMCHECK
#ifdef WIN32
#define not  !		//
#endif
#define is_not !=	//


	// Helper functions / classes ---------------------------------------------------------------------

	/// helper class from auto lock/unlock-ing the VRmUsbCam global lock or some device lock
	template<typename t_handle, VRmRetVal(*t_lockfunc)(t_handle), VRmRetVal(*t_unlockfunc)(t_handle)>
	class VRmLockProxy
	{
	public:
		VRmLockProxy(t_handle fp_obj = 0) : mp_obj(fp_obj) {}
		void lock() { t_lockfunc(mp_obj); }
		void unlock() { t_unlockfunc(mp_obj); }
	private:
		t_handle mp_obj;
	};
	inline VRmRetVal globalLock(int) { return VRmUsbCamPrivateLock(); };
	inline VRmRetVal globalUnlock(int) { return VRmUsbCamPrivateUnlock(); };

	typedef VRmLockProxy<int, globalLock, globalUnlock> tVRmGlobalLockProxy;
	typedef VRmLockProxy<VRmUsbCamDevice, VRmUsbCamPrivateLockDevice, VRmUsbCamPrivateUnlockDevice> tVRmDeviceLockProxy;


	SizeI::SizeI(int width, int height)
	{
		m_width= width;
		m_height= height;
	}

	PointI::PointI(int x, int y)
	{
		m_x= x;
		m_y= y;
	}

	RectI::RectI(int left, int top, int width, int height)
	{
		m_left= left;
		m_top= top;
		m_width= width;
		m_height= height;
	}

	std::string PropInfo::ToString() const {
		return m_id_string;
	}

	// Exception --------------------------------------------------------------------------------------

#ifdef WIN32
	Exception::Exception(const std::string& message, ErrorNumber number)
		: std::exception(message.c_str()), m_number(number)
	{
	}
#else
	Exception::Exception(const std::string& message, ErrorNumber number)
		: std::exception(), m_number(number), m_message(message)
	{
	}

	Exception::~Exception() throw ()
	{}

	const char* Exception::what() const throw ()
	{
		return m_message.c_str();
	}

#endif

	std::string Exception::get_Description() const {
		return what();
	}

	ErrorNumber Exception::get_Number() const {
		return m_number;
	}

	// DeviceKey --------------------------------------------------------------------------------------

	std::string DeviceKey::ToString() const {
		return get_Product() + " #" + get_SerialString();
	}

	std::string DeviceKey::get_Manufacturer() const {
		return std::string(mp_key->mp_manufacturer_str);
	}

	std::string DeviceKey::get_Product() const {
		return std::string(mp_key->mp_product_str);
	}

	int DeviceKey::get_Serial() const {
		return mp_key->m_serial;
	}

	std::string DeviceKey::get_SerialString() const {
		VRmSTRING l_str=0;
		VRMCALL(GetSerialString(mp_key, &l_str));
		return std::string(l_str);
	}

	bool DeviceKey::get_Busy() const {
		return mp_key->m_busy?true:false;
	}

	int DeviceKey::get_VendorId() const {
		VRmWORD retval=0;
		VRMCALL(GetVendorId(mp_key, &retval));
		return retval;
	}

	int DeviceKey::get_ProductId() const {
		VRmWORD retval=0;
		VRMCALL(GetProductId(mp_key, &retval));
		return retval;
	}

	int DeviceKey::get_GroupId() const {
		VRmWORD retval=0;
		VRMCALL(GetGroupId(mp_key, &retval));
		return retval;
	}

	std::string DeviceKey::get_IpAddress() const {
		VRmSTRING l_str=0;
		VRMCALL(GetIpAddress(mp_key, &l_str));
		return std::string(l_str);
	}

	std::string DeviceKey::get_LocalIpAddress() const {
		VRmSTRING l_str=0;
		VRMCALL(GetLocalIpAddress(mp_key, &l_str));
		return std::string(l_str);
	}

	DeviceKey::DeviceKey(VRmDeviceKey* fp_key) { mp_key= fp_key; }

	bool DeviceKey::operator==(const DeviceKeyPtr& other) const
	{
		VRMCHECK(other is_not null);
		VRmBOOL l_tmp=1;
		VRMCALL(CompareDeviceKeys(mp_key, other->mp_key, &l_tmp));
		if (l_tmp)
			return false;
		return true;
	}

	bool DeviceKey::operator!=(const DeviceKeyPtr& other) const
	{
		VRMCHECK(other is_not null);
		return !operator==(other);
	}

	DeviceKey::~DeviceKey()
	{
		if (mp_key)
			VRmUsbCamFreeDeviceKey(&mp_key);
	}

	VRmDeviceKey* DeviceKey::get_NativeRepresentation() const
	{
		return mp_key;
	}

	// ImageFormat ---------------------------------------------------------------------------------------

	std::string ImageFormat::ToString() const {
		VRmSTRING lp_color_format= 0;
		VRMCALL(GetStringFromColorFormat(m_vrmformat.m_color_format, &lp_color_format));
		return (boost::format("%1%x%2% (%3%)") % m_vrmformat.m_width % m_vrmformat.m_height % lp_color_format).str();
	}

	std::string ImageFormat::get_Description() const {
		return m_description;
	}

	bool ImageFormat::operator==(const ImageFormat& other) const
	{
		VRmBOOL l_tmp=1;
		VRMCALL(CompareImageFormats(&m_vrmformat, &other.m_vrmformat, &l_tmp));
		if (l_tmp)
			return false;
		return true;
	}

	bool ImageFormat::operator!=(const ImageFormat& other) const
	{
		VRmBOOL l_tmp=1;
		VRMCALL(CompareImageFormats(&m_vrmformat, &other.m_vrmformat, &l_tmp));
		if (l_tmp)
			return true;
		return false;
	}

	ImageFormat::ImageFormat()
	{
		m_vrmformat.m_width= 0;
		m_vrmformat.m_height= 0;
		m_vrmformat.m_color_format= (VRmColorFormat)0;
		m_vrmformat.m_image_modifier= (VRmImageModifier)0;
		updateDescription();
	}

	ImageFormat::ImageFormat(const SizeI& size, ColorFormat colorformat, ImageModifier imagemodifier)
	{
		if (size.m_width<0 || size.m_height<0)
			VRMERR("Size must be non-negative");
		m_vrmformat.m_width= size.m_width;
		m_vrmformat.m_height= size.m_height;
		m_vrmformat.m_color_format= static_cast<VRmColorFormat>(colorformat);
		m_vrmformat.m_image_modifier= static_cast<VRmImageModifier>(imagemodifier);
		updateDescription();
	}

	ImageFormat::ImageFormat(const VRmImageFormat& format, const std::string& description)
	{
		m_vrmformat= format;
		if (description.empty())
			updateDescription();
		else
			m_description= description;
	}

	ImageFormat::ImageFormat( const VRmImageFormat& format )
	{
		m_vrmformat= format;
		updateDescription();
	}

	void ImageFormat::set_Size(const SizeI& size) {
		if (size.m_width<0 || size.m_height<0)
			VRMERR("Size must be non-negative");
		m_vrmformat.m_width= size.m_width;
		m_vrmformat.m_height= size.m_height;
		updateDescription();
	}

	SizeI ImageFormat::get_Size() const {
		return SizeI(m_vrmformat.m_width, m_vrmformat.m_height);
	}

	ColorFormat ImageFormat::get_ColorFormat() const {
		return m_vrmformat.m_color_format;
	}

	void ImageFormat::set_ColorFormat(ColorFormat colorformat) {
		m_vrmformat.m_color_format= colorformat;
		updateDescription();
	}

	ImageModifier ImageFormat::get_ImageModifier() const {
		return static_cast<ImageModifier>(m_vrmformat.m_image_modifier);
	}

	void ImageFormat::set_ImageModifier(ImageModifier imagemodifier) {
		m_vrmformat.m_image_modifier= static_cast<VRmImageModifier>(imagemodifier);
		updateDescription();
	}

	int ImageFormat::get_PixelDepth() const {
		VRmDWORD l_depth=0;
		VRMCALL(GetPixelDepthFromColorFormat(m_vrmformat.m_color_format, &l_depth));
		return l_depth;
	}

	std::vector<ImageFormat> ImageFormat::get_TargetFormatList() const {
		VRmDWORD size= 0;
		VRMCALL(GetTargetFormatListSize(&m_vrmformat, &size));
		std::vector<ImageFormat> l_target_format_list((size_t)size);
		for (VRmDWORD i=0; i<size; ++i) {
			VRmImageFormat l_tmp;
			VRMCALL(GetTargetFormatListEntry(&m_vrmformat, i, &l_tmp));
			l_target_format_list[i]= ImageFormat(l_tmp, std::string());
		}
		return l_target_format_list;
	}

	void ImageFormat::FlipHorizontal() {
		m_vrmformat.m_image_modifier ^= VRM_HORIZONTAL_MIRRORED;
		updateDescription();
	}

	void ImageFormat::FlipVertical() {
		m_vrmformat.m_image_modifier ^= VRM_VERTICAL_MIRRORED;
		updateDescription();
	}

	void ImageFormat::updateDescription() {
		m_description= std::string();
	}

	const VRmImageFormat& ImageFormat::get_NativeRepresentation() const
	{
		return m_vrmformat;
	}

	// Image ---------------------------------------------------------------------------------------------

	Image::Image(const ImageFormat& format)
		: mp_vrmimage(0), mp_owner(ImagePtr())
	{
		VRMCALL(NewImage(&mp_vrmimage, format.get_NativeRepresentation()));
	}

	Image::Image(const ImageFormat& format, void* buffer, int pitch)
		: mp_vrmimage(0), mp_owner(ImagePtr())
	{
		VRMCHECK(buffer is_not null);
		VRMCHECK(pitch>=0);
		VRMCALL(SetImage(&mp_vrmimage, format.get_NativeRepresentation(), (VRmBYTE*)buffer, pitch));
	}

	// used by Device::LockNextImage() and Image::Crop()
	Image::Image(VRmImage* fp_vrmimage, tOwnerPtr fp_owner)
		: mp_vrmimage(fp_vrmimage), mp_owner(fp_owner)
	{
	}

	ImagePtr Image::Clone() const {
		VRmImage* lp_new_image=0;
		VRMCALL(CopyImage(&lp_new_image, mp_vrmimage));
		return ImagePtr(new Image(lp_new_image, tOwnerPtr()));
	}

	Image::~Image() {
		if (mp_vrmimage) {
			if ( DevicePtr* lpp_device= boost::get<DevicePtr>(&mp_owner))
				VRmUsbCamUnlockNextImage((*lpp_device)->get_NativeRepresentation(), &mp_vrmimage);
			else
				VRmUsbCamFreeImage(&mp_vrmimage);
		}
	}

	ImageFormat Image::get_ImageFormat() const {
		return ImageFormat(mp_vrmimage->m_image_format);
	}

	double Image::get_TimeStamp() const {
		return mp_vrmimage->m_time_stamp;
	}

	int Image::get_Pitch() const {
		return mp_vrmimage->m_pitch;
	}

	void* Image::get_Buffer() const {
		return mp_vrmimage->mp_buffer;
	}

	void Image::set_BufferSize(int size) {
		VRMCALL(SetImageBufferSize(mp_vrmimage, VRmDWORD(size)));
	}

	int Image::get_BufferSize() const {
		VRmDWORD l_tmp=0;
		VRMCALL(GetImageBufferSize(mp_vrmimage, &l_tmp));
		return l_tmp;
	}

	int Image::get_FrameCounter() const {
		VRmDWORD l_tmp=0;
		VRMCALL(GetFrameCounter(mp_vrmimage, &l_tmp));
		return l_tmp;
	}

	int Image::get_SensorPort() const {
		VRmDWORD l_tmp=0;
		VRMCALL(GetImageSensorPort(mp_vrmimage, &l_tmp));
		return l_tmp;
	}

	std::vector<unsigned char> Image::get_Footer() const {
		const void* lp_footer= 0;
		VRmDWORD l_length= 0;
		VRMCALL(GetImageFooter(mp_vrmimage, &lp_footer, &l_length));
		std::vector<unsigned char> lp_data((size_t)l_length);
		if (lp_data.size())
			memcpy(&lp_data[0], lp_footer, l_length);
		return lp_data;
	}

	ImagePtr Image::Crop(const RectI& rect) {
		VRmImage* lp_new_image=0;
		VRMCALL(CropImage(&lp_new_image, mp_vrmimage, &rect));
		return ImagePtr(new Image(lp_new_image, shared_from_this()));
	}

	VRmImage* Image::get_NativeRepresentation() const
	{
		return mp_vrmimage;
	}

	// DevicePropertyPage ---------------------------------------------------------------------------

#ifdef WIN32
	DevicePropertyPage::DevicePropertyPage(VRmDevicePropertyPage* fp_vrmpage)
		: mp_vrmpage(fp_vrmpage)
	{
	}

	void DevicePropertyPage::Move(const RectI& rect) {
		VRMCHECK(mp_vrmpage is_not null);
		if (!MoveWindow(mp_vrmpage->m_handle, rect.m_left, rect.m_top, rect.m_width, rect.m_height, TRUE))
			VRMERR_WITHCODE("MoveWindow() failed", GetLastError());
	}

	SizeI DevicePropertyPage::get_SizeHint() const {
		VRMCHECK(mp_vrmpage is_not null);
		return SizeI(mp_vrmpage->m_size_hint.m_width, mp_vrmpage->m_size_hint.m_height);
	}

	void DevicePropertyPage::Show() {
		VRMCHECK(mp_vrmpage is_not null);
		if (GetParent(mp_vrmpage->m_handle))
			VRMERR("only valid for top level pages (hWndParent is null)");
		ShowWindow(mp_vrmpage->m_handle, SW_SHOW); // NOTE: ShowWindow has no error return value
	}

	void DevicePropertyPage::Hide() {
		VRMCHECK(mp_vrmpage is_not null);
		if (GetParent(mp_vrmpage->m_handle))
			VRMERR("only valid for top level pages (hWndParent is null)");
		ShowWindow(mp_vrmpage->m_handle, SW_HIDE); // NOTE: ShowWindow has no error return value
	}

	void DevicePropertyPage::set_Size(const SizeI& size) {
		VRMCHECK(mp_vrmpage is_not null);
		RECT l_rect;
		if (!GetWindowRect(mp_vrmpage->m_handle, &l_rect))
			VRMERR_WITHCODE("GetWindowRect() failed", GetLastError());
		if (!MoveWindow(mp_vrmpage->m_handle, l_rect.left, l_rect.top, size.m_width, size.m_height, TRUE))
			VRMERR_WITHCODE("MoveWindow() failed", GetLastError());
	}

	SizeI DevicePropertyPage::get_Size() const {
		VRMCHECK(mp_vrmpage is_not null);
		RECT l_rect;
		if (!GetWindowRect(mp_vrmpage->m_handle, &l_rect))
			VRMERR_WITHCODE("GetWindowRect() failed", GetLastError());
		return SizeI(l_rect.right-l_rect.left, l_rect.bottom-l_rect.top);
	}

	void DevicePropertyPage::set_Location(const PointI& location) {
		VRMCHECK(mp_vrmpage is_not null);
		RECT l_rect;
		if (!GetWindowRect(mp_vrmpage->m_handle, &l_rect))
			VRMERR_WITHCODE("GetWindowRect() failed", GetLastError());
		if (!MoveWindow(mp_vrmpage->m_handle, location.m_x, location.m_y, l_rect.right-l_rect.left, l_rect.bottom-l_rect.top, TRUE))
			VRMERR_WITHCODE("MoveWindow() failed", GetLastError());
	}

	PointI DevicePropertyPage::get_Location() const {
		VRMCHECK(mp_vrmpage is_not null);
		HWND l_parent= GetParent(mp_vrmpage->m_handle);
		RECT l_rect;
		if (!l_parent)
		{
			if (!GetWindowRect(mp_vrmpage->m_handle, &l_rect))
				VRMERR_WITHCODE("GetWindowRect() failed", GetLastError());
			return PointI(l_rect.left, l_rect.top);
		}
		else
		{
			POINT l_pos;
			if (!ClientToScreen(mp_vrmpage->m_handle, &l_pos))
				VRMERR_WITHCODE("ClientToScreen() failed", GetLastError());
			POINT l_parentpos;
			if (!ClientToScreen(l_parent, &l_parentpos))
				VRMERR_WITHCODE("ClientToScreen() failed", GetLastError());
			l_pos.x-=l_parentpos.x;
			l_pos.y-=l_parentpos.y;
			return PointI(l_pos.x, l_pos.y);
		}
	}

	bool DevicePropertyPage::get_Visible() const {
		VRMCHECK(mp_vrmpage is_not null);
		return IsWindowVisible(mp_vrmpage->m_handle)?true:false;
	}

	HWND DevicePropertyPage::get_Handle() const {
		if (mp_vrmpage)
			return mp_vrmpage->m_handle;
		return 0;
	}

	DevicePropertyPage::~DevicePropertyPage() {
		if (mp_vrmpage)
		{
			VRmUsbCamDestroyDevicePropertyPage(&mp_vrmpage);
		}
	}

	VRmDevicePropertyPage* DevicePropertyPage::get_NativeRepresentation() const
	{
		return mp_vrmpage;
	}
#endif//WIN32

	// Device ---------------------------------------------------------------------------------------

	// methods

	Device::Device(const DeviceKeyPtr& key)
		: m_device(0), m_callbacks_registered(false)
	{
		VRMCHECK(key is_not null);
		VRMCALL(OpenDevice(key->get_NativeRepresentation(), &m_device));
		VRMCALL(RegisterDeviceCallbackEx(m_device, DeviceCallbackProxy, this));
		m_callbacks_registered= true;
	}

	void Device::Start() {
		VRMCHECK(m_device is_not null);
		VRMDEVCALL(Start(m_device));
	}

	void Device::Stop() {
		VRMCHECK(m_device is_not null);
		VRMDEVCALL(Stop(m_device));
	}

	bool Device::IsNextImageReady(int port) const {
		VRMCHECK(m_device is_not null);
		VRmBOOL tmp= false;
		VRMDEVCALL(IsNextImageReadyEx(m_device, port, &tmp));
		return tmp?true:false;
	}

	void Device::Close() {
		VRMCHECK(m_device is_not null);
		VRMDEVCALL(CloseDevice(m_device));
		m_device= 0;
	}

	ImagePtr Device::LockNextImage(int port, int* framesDropped, int timeoutMs) {
		VRMCHECK(m_device is_not null);
		VRmImage* lp_vrmimage= 0;
		VRmDWORD l_frames_dropped;
		VRMDEVCALL(LockNextImageEx2(m_device, port, &lp_vrmimage, &l_frames_dropped, timeoutMs));
		ImagePtr lp_image(new Image(lp_vrmimage, shared_from_this()/*, mp_com_device*/));
		if (framesDropped)
			*framesDropped= l_frames_dropped;
		return lp_image;
	}

	void Device::UnlockNextImage(ImagePtr& image) {
		VRMCHECK(m_device is_not null);
		VRMCHECK(image is_not null);
		DevicePtr* lpp_owner_device= boost::get<DevicePtr>(&image->mp_owner);
		if (!lpp_owner_device || *lpp_owner_device != shared_from_this())
			VRMERR("Image is not locked by this device");
		image.reset(); // this will unlock it when there are no other references!
	}

	void Device::SoftTrigger() {
		VRMCHECK(m_device is_not null);
		VRMDEVCALL(SoftTrigger(m_device));
	}

	std::vector<unsigned char> Device::LoadUserData() {
		VRMCHECK(m_device is_not null);
		VRmUserData* lp_vrmdata=0;
		VRMDEVCALL(LoadUserData(m_device, &lp_vrmdata));
		std::vector<unsigned char> lp_data((size_t)lp_vrmdata->m_length);
		if (lp_data.size())
			memcpy(&lp_data[0], lp_vrmdata->mp_data, lp_data.size());
		VRMCALL(FreeUserData(&lp_vrmdata));
		return lp_data;
	}

	void Device::SaveUserData(const std::vector<unsigned char>& data) {
		VRMCHECK(m_device is_not null);
		VRmUserData* lp_vrmdata=0;
		VRMCALL(NewUserData(&lp_vrmdata, VRmDWORD(data.size())));
		try
		{
			if (lp_vrmdata->m_length)
				memcpy(lp_vrmdata->mp_data, &data[0], lp_vrmdata->m_length);
			VRMDEVCALL(SaveUserData(m_device, lp_vrmdata));
		}
		catch (const std::exception&)
		{
			VRmUsbCamFreeUserData(&lp_vrmdata);
			throw;
		}
		VRMCALL(FreeUserData(&lp_vrmdata));
	}

	void Device::ResetFrameCounter() {
		VRMCHECK(m_device is_not null);
		VRMDEVCALL(ResetFrameCounter(m_device));
	}

#ifdef WIN32
	DevicePropertyPagePtr Device::CreatePropertyPage(HWND hWndParent, const RectI& rect) {
		VRMCHECK(m_device is_not null);

		VRmDevicePropertyPage* lpp_vrmpage= 0;
		VRMDEVCALL(CreateDevicePropertyPage(m_device, hWndParent, &rect, &lpp_vrmpage));

		return DevicePropertyPagePtr(new DevicePropertyPage(lpp_vrmpage));
	}

	DevicePropertyPagePtr Device::CreatePropertyPage() {
		VRMCHECK(m_device is_not null);

		VRmDevicePropertyPage* lpp_vrmpage= 0;
		VRMDEVCALL(CreateDevicePropertyPage(m_device, 0, 0, &lpp_vrmpage));

		return DevicePropertyPagePtr(new DevicePropertyPage(lpp_vrmpage));
	}
#endif//WIN32

	void Device::LoadConfig(int id) {
		VRMCHECK(m_device is_not null);
		VRMDEVCALL(LoadConfig(m_device, id));
	}

	void Device::SaveConfig(int id) {
		VRMCHECK(m_device is_not null);
		VRMDEVCALL(SaveConfig(m_device, id));
	}

	bool Device::SaveConfigRequiresFirmwareCompression(int id) {
		VRMCHECK(m_device is_not null);
		VRmBOOL tmp= false;
		VRMDEVCALL(SaveConfigRequiresFirmwareCompression(m_device, id, &tmp));
		return tmp?true:false;
	}

	void Device::DeleteConfig(int id) {
		VRMCHECK(m_device is_not null);
		VRMDEVCALL(DeleteConfig(m_device, id));
	}

	int Device::FindSensorPortListIndex(int port) const
	{
		VRMCHECK(m_device is_not null);
		VRmDWORD index;
		VRMDEVCALL(FindSensorPortListIndex(m_device, port, &index));
		return index;
	}

	// properties

	DeviceKeyPtr Device::get_DeviceKey() const {
		VRMCHECK(m_device is_not null);
		VRmDeviceKey* lp_vrmkey=0;
		VRMDEVCALL(GetDeviceKey(m_device, &lp_vrmkey));
		return DeviceKeyPtr(new DeviceKey(lp_vrmkey));
	}

	ImageFormat Device::get_SourceFormat(int port) const {
		VRMCHECK(m_device is_not null);
		tVRmDeviceLockProxy l_device_lock_proxy(m_device);
		boost::lock_guard<tVRmDeviceLockProxy> l_device_lock(l_device_lock_proxy);
		VRmImageFormat l_tmp;
		VRMDEVCALL(GetSourceFormatEx(m_device, port, &l_tmp));
		const char* lcp_description;
		VRMDEVCALL(GetSourceFormatDescription(m_device, port, &lcp_description));
		return ImageFormat(l_tmp, lcp_description?std::string(lcp_description):std::string());
	}

	std::vector<int> Device::get_SensorPortList() const {
		VRMCHECK(m_device is_not null);
		tVRmDeviceLockProxy l_device_lock_proxy(m_device);
		boost::lock_guard<tVRmDeviceLockProxy> l_device_lock(l_device_lock_proxy);
		VRmDWORD size= 0;
		VRMDEVCALL(GetSensorPortListSize(m_device, &size));
		std::vector<int> l_port_list((size_t)size);
		for (VRmDWORD i=0; i<size; ++i) {
			VRmDWORD l_tmp;
			VRMDEVCALL(GetSensorPortListEntry(m_device, i, &l_tmp));
			l_port_list[i]= l_tmp;
		}
		return l_port_list;
	}

	std::vector<ImageFormat> Device::get_TargetFormatList(int port) const {
		VRMCHECK(m_device is_not null);
		tVRmDeviceLockProxy l_device_lock_proxy(m_device);
		boost::lock_guard<tVRmDeviceLockProxy> l_device_lock(l_device_lock_proxy);
		VRmDWORD size= 0;
		VRMDEVCALL(GetTargetFormatListSizeEx2(m_device, port, &size));
		std::vector<ImageFormat> l_target_format_list((size_t)size);
		for (VRmDWORD i=0; i<size; ++i) {
			VRmImageFormat l_tmp;
			VRMDEVCALL(GetTargetFormatListEntryEx2(m_device, port, i, &l_tmp));
			l_target_format_list[i]= ImageFormat(l_tmp, std::string());
		}
		return l_target_format_list;
	}

	void Device::set_ConfigData(const std::vector<unsigned char>& data) {
		VRMCHECK(m_device is_not null);
		VRmUserData* lp_vrmdata=0;
		VRMCALL(NewUserData(&lp_vrmdata, (VRmDWORD)data.size()));
		try {
			if (lp_vrmdata->m_length)
				memcpy(lp_vrmdata->mp_data, &data[0], lp_vrmdata->m_length);
			VRMDEVCALL(SetConfigData(m_device, lp_vrmdata));
		}
		catch (const std::exception&)
		{
			VRmUsbCamFreeUserData(&lp_vrmdata);
			throw;
		}
		VRMCALL(FreeUserData(&lp_vrmdata));
	}

	std::vector<unsigned char> Device::get_ConfigData() const {
		VRMCHECK(m_device is_not null);
		VRmUserData* lp_vrmdata=0;
		VRMDEVCALL(GetConfigData(m_device, &lp_vrmdata));
		std::vector<unsigned char> data((size_t)lp_vrmdata->m_length);
		try {
			if (data.size())
				memcpy(&data[0], lp_vrmdata->mp_data, data.size());
		}
		catch (const std::exception&)
		{
			VRmUsbCamFreeUserData(&lp_vrmdata);
			throw;
		}
		VRMCALL(FreeUserData(&lp_vrmdata));
		return data;
	}

	bool Device::get_ConfigIncludesUnsupportedValues() const {
		VRMCHECK(m_device is_not null);
		VRmBOOL l_val=0;
		VRMDEVCALL(ConfigIncludesUnsupportedValues(m_device, &l_val));
		return l_val!=0;
	}

	bool Device::get_Running() const {
		VRMCHECK(m_device is_not null);
		VRmBOOL tmp= false;
		VRMDEVCALL(GetRunning(m_device, &tmp));
		return tmp?true:false;
	}

	std::vector<PropId> Device::get_PropertyList() const {
		VRMCHECK(m_device is_not null);
		tVRmDeviceLockProxy l_device_lock_proxy(m_device);
		boost::lock_guard<tVRmDeviceLockProxy> l_device_lock(l_device_lock_proxy);

		VRmDWORD size= 0;
		VRMDEVCALL(GetPropertyListSize(m_device, &size));

		std::vector<PropId> l_list((size_t)size);
		for (VRmDWORD i=0; i<size; ++i) {
			VRmPropId l_tmp;
			VRMDEVCALL(GetPropertyListEntry(m_device, i, &l_tmp));
			l_list[i]= static_cast<PropId>(l_tmp);
		}
		return l_list;
	}

	template<typename t> static Variant convert2variant(const t& fcr_val) { return fcr_val; }
	template<> Variant convert2variant(const VRmBOOL& fcr_val) { return bool(fcr_val?true:false); }
	template<> Variant convert2variant(const VRmSTRING& fcr_val) { return std::string(fcr_val); }
	template<> Variant convert2variant(const VRmPropId& fcr_val) { return static_cast<PropId>(fcr_val); }
	template<> Variant convert2variant(const VRmSizeI& fcr_val) { return SizeI(fcr_val.m_width, fcr_val.m_height); }
	template<> Variant convert2variant(const VRmPointI& fcr_val) { return PointI(fcr_val.m_x, fcr_val.m_y); }
	template<> Variant convert2variant(const VRmRectI& fcr_val) { return RectI(fcr_val.m_left, fcr_val.m_top, fcr_val.m_width, fcr_val.m_height); }

	PropInfo Device::get_PropertyInfo(PropId propid) const {
		VRMCHECK(m_device is_not null);
		PropInfo l_vrmpropinfo;
		VRMDEVCALL(GetPropertyInfo(m_device, static_cast<VRmPropId>(propid), &l_vrmpropinfo));
		return l_vrmpropinfo;
	}

	bool Device::get_PropertySupported(PropId propid) const {
		VRMCHECK(m_device is_not null);
		VRmBOOL l_supported;
		VRMDEVCALL(GetPropertySupported(m_device, static_cast<VRmPropId>(propid), &l_supported));
		return l_supported?true:false;
	}

#define CONVERTATTRIBS(vrm_suffix) \
	{ \
	VRmPropAttribs##vrm_suffix vrm_attribs; \
	VRMDEVCALL(GetPropertyAttribs##vrm_suffix(m_device, static_cast<VRmPropId>(propid), &vrm_attribs)); \
	l_attribs.m_default= convert2variant(vrm_attribs.m_default); \
	l_attribs.m_min= convert2variant(vrm_attribs.m_min); \
	l_attribs.m_max= convert2variant(vrm_attribs.m_max); \
	l_attribs.m_step= convert2variant(vrm_attribs.m_step); \
	} \
	break

	PropAttribs Device::get_PropertyAttribs(PropId propid) const {
		VRMCHECK(m_device is_not null);
		VRmPropInfo vrm_propinfo;
		VRMDEVCALL(GetPropertyInfo(m_device, static_cast<VRmPropId>(propid), &vrm_propinfo));
		PropAttribs l_attribs;
		switch (vrm_propinfo.m_type) {
	case VRM_PROP_TYPE_BOOL: CONVERTATTRIBS(B);
	case VRM_PROP_TYPE_INT: CONVERTATTRIBS(I);
	case VRM_PROP_TYPE_FLOAT: CONVERTATTRIBS(F);
	case VRM_PROP_TYPE_STRING: CONVERTATTRIBS(S);
	case VRM_PROP_TYPE_ENUM: CONVERTATTRIBS(E);
	case VRM_PROP_TYPE_SIZE_I: CONVERTATTRIBS(SizeI);
	case VRM_PROP_TYPE_POINT_I: CONVERTATTRIBS(PointI);
	case VRM_PROP_TYPE_RECT_I: CONVERTATTRIBS(RectI);
	case VRM_PROP_TYPE_DOUBLE: CONVERTATTRIBS(D);
	default:
		VRMERR(std::string("unsupported property type for property ") + vrm_propinfo.m_id_string);
		}
		return l_attribs;
	}

#define RETURNGETVALUE(vrm_type, vrm_suffix) \
	{ \
	vrm_type l_vrmval; \
	VRMDEVCALL(GetPropertyValue##vrm_suffix(m_device, static_cast<VRmPropId>(propid), &l_vrmval)); \
	return convert2variant(l_vrmval); \
	}

	Variant Device::get_PropertyValue(PropId propid) const {
		VRMCHECK(m_device is_not null);
		VRmPropInfo vrm_propinfo;
		VRMDEVCALL(GetPropertyInfo(m_device, static_cast<VRmPropId>(propid), &vrm_propinfo));
		switch (vrm_propinfo.m_type) {
	case VRM_PROP_TYPE_BOOL: RETURNGETVALUE(VRmBOOL, B);
	case VRM_PROP_TYPE_INT: RETURNGETVALUE(int, I);
	case VRM_PROP_TYPE_FLOAT: RETURNGETVALUE(float, F);
	case VRM_PROP_TYPE_STRING: RETURNGETVALUE(VRmSTRING, S);
	case VRM_PROP_TYPE_ENUM: RETURNGETVALUE(VRmPropId, E);
	case VRM_PROP_TYPE_SIZE_I: RETURNGETVALUE(VRmSizeI, SizeI);
	case VRM_PROP_TYPE_POINT_I: RETURNGETVALUE(VRmPointI, PointI);
	case VRM_PROP_TYPE_RECT_I: RETURNGETVALUE(VRmRectI, RectI);
	case VRM_PROP_TYPE_DOUBLE: RETURNGETVALUE(double, D);
	default:
		VRMERR(std::string("unsupported property type for property ") + vrm_propinfo.m_id_string);
		}
	}

	void Device::set_PropertyValue(PropId propid, const Variant& value) {
		VRMCHECK(m_device is_not null);
		VRmPropInfo vrm_propinfo;
		VRMDEVCALL(GetPropertyInfo(m_device, static_cast<VRmPropId>(propid), &vrm_propinfo));
		const int* p_val_i = boost::get<int>(&value);
		switch (vrm_propinfo.m_type) {
	case VRM_PROP_TYPE_BOOL:
		{
			VRmBOOL vrm_val;
			const bool* p_val_b = boost::get<bool>(&value);
			if (p_val_b)
				vrm_val= *p_val_b ? 1:0;
			else if (p_val_i && (*p_val_i==0 || *p_val_i==1))
				vrm_val= *p_val_i;
			else VRMERR(std::string("type mismatch. expected boolean type for property ")+vrm_propinfo.m_id_string);
			VRMCALL(SetPropertyValueB(m_device, static_cast<VRmPropId>(propid), &vrm_val));
		}
		break;
	case VRM_PROP_TYPE_INT:
		{
			if (!p_val_i) VRMERR(std::string("type mismatch. expected integer type for property ")+vrm_propinfo.m_id_string);
			VRMCALL(SetPropertyValueI(m_device, static_cast<VRmPropId>(propid), p_val_i));
		}
		break;
	case VRM_PROP_TYPE_FLOAT:
		{
			float vrm_val;
			const float* p_val_f = boost::get<float>(&value);
			if (p_val_f)
				vrm_val= *p_val_f;
			else if (p_val_i)
				vrm_val= static_cast<float>(*p_val_i);
			else VRMERR(std::string("type mismatch. expected floating point type for property ")+vrm_propinfo.m_id_string);
			VRMCALL(SetPropertyValueF(m_device, static_cast<VRmPropId>(propid), &vrm_val));
		}
		break;
	case VRM_PROP_TYPE_DOUBLE:
		{
			double vrm_val;
			const double* p_val_d = boost::get<double>(&value);
			const float* p_val_f = boost::get<float>(&value);
			if (p_val_d)
				vrm_val= *p_val_d;
			else if (p_val_f)
				vrm_val= static_cast<double>(*p_val_f);
			else if (p_val_i)
				vrm_val= static_cast<double>(*p_val_i);
			else VRMERR(std::string("type mismatch. expected floating point type for property ")+vrm_propinfo.m_id_string);
			VRMCALL(SetPropertyValueD(m_device, static_cast<VRmPropId>(propid), &vrm_val));
		}
		break;
	case VRM_PROP_TYPE_STRING:
		{
			const std::string* p_val_s = boost::get<std::string>(&value);
			if (!p_val_s)
				VRMERR(std::string("type mismatch. expected string type for property ")+vrm_propinfo.m_id_string);
			VRmSTRING vrm_val= p_val_s->c_str();
			VRMCALL(SetPropertyValueS(m_device, static_cast<VRmPropId>(propid), &vrm_val));
		}
		break;
	case VRM_PROP_TYPE_ENUM:
		{
			VRmPropId vrm_val;
			const PropId* p_val_e = boost::get<PropId>(&value);
			if (p_val_e)
				vrm_val= static_cast<VRmPropId>(*p_val_e);
			else if (p_val_i)
				vrm_val= static_cast<VRmPropId>(*p_val_i);
			else VRMERR(std::string("type mismatch. expected PropId (integer) type for property ")+vrm_propinfo.m_id_string);
			VRMCALL(SetPropertyValueE(m_device, static_cast<VRmPropId>(propid), &vrm_val));
		}
		break;
	case VRM_PROP_TYPE_SIZE_I:
		{
			const VRmSizeI* p_val_size_i = boost::get<SizeI>(&value);
			if (!p_val_size_i)
				VRMERR(std::string("type mismatch. expected Size type for property ")+vrm_propinfo.m_id_string);
			VRMCALL(SetPropertyValueSizeI(m_device, static_cast<VRmPropId>(propid), p_val_size_i));
		}
		break;
	case VRM_PROP_TYPE_POINT_I:
		{
			const VRmPointI* p_val_point_i = boost::get<PointI>(&value);
			if (!p_val_point_i)
				VRMERR(std::string("type mismatch. expected Point type for property ")+vrm_propinfo.m_id_string);
			VRMCALL(SetPropertyValuePointI(m_device, static_cast<VRmPropId>(propid), p_val_point_i));
		}
		break;
	case VRM_PROP_TYPE_RECT_I:
		{
			const VRmRectI* p_val_rect_i = boost::get<RectI>(&value);
			if (!p_val_rect_i)
				VRMERR(std::string("type mismatch. expected Rectangle type for property ")+vrm_propinfo.m_id_string);
			VRMCALL(SetPropertyValueRectI(m_device, static_cast<VRmPropId>(propid), p_val_rect_i));
		}
		break;
	default:
		VRMERR(std::string("unsupported property type for property ") + vrm_propinfo.m_id_string);
		}
	}

	Device::~Device()
	{
		if (m_callbacks_registered)
			VRmUsbCamUnregisterDeviceCallbackEx(m_device, DeviceCallbackProxy, this);
		m_callbacks_registered= false;

		VRmUsbCamCloseDevice(m_device);
		m_device=0;
	}

	// receives Device* as fp_user_data
	void Device::DeviceCallbackProxy(VRmUsbCamDevice f_device, VRmDeviceCallbackType f_type, void* fp_user_data, const void* fcp_callback_params)
	{
		if (!fp_user_data)
			return;
		Device* lp_device= (Device*)(fp_user_data);
		if (lp_device->m_device == f_device)
			lp_device->RaiseEvent(f_type, fcp_callback_params);
	}

	EventArgs::~EventArgs()
	{
		// NOTE: this d-tor allows for proper derivation and turns EventArgs into a polymorphic type
	}

	template class Device::Event<Device::EventHandler>;
	template class Device::Event<Device::PropertyValueChangedEventHandler>;

	void Device::RaiseEvent(VRmDeviceCallbackType f_type, const void* fcp_callback_params)
	{
		try
		{
			switch (f_type)
			{
			case VRM_DEVICE_CALLBACK_TYPE_LUT_CHANGED:
				if (fcp_callback_params)
				{
					int l_sensor_port= *reinterpret_cast<const int*>(fcp_callback_params);
					BOOST_FOREACH(const LutChangedEventHandler& handler, LutChanged.m_handlers)
						handler(shared_from_this(), l_sensor_port);
				}
				break;
			case VRM_DEVICE_CALLBACK_TYPE_SOURCE_FORMAT_CHANGED:
				BOOST_FOREACH(const EventHandler& handler, SourceFormatChanged.m_handlers)
					handler(shared_from_this(), EventArgs());
				break;
			case VRM_DEVICE_CALLBACK_TYPE_PROPERTY_VALUE_CHANGED:
				if (fcp_callback_params)
				{
					PropId l_id= *reinterpret_cast<const PropId*>(fcp_callback_params);
					BOOST_FOREACH(const PropertyValueChangedEventHandler& handler, PropertyValueChanged.m_handlers)
						handler(shared_from_this(), l_id);
				}
				break;
			case VRM_DEVICE_CALLBACK_TYPE_PROPERTY_LIST_CHANGED:
				BOOST_FOREACH(const EventHandler& handler, PropertyListChanged.m_handlers)
					handler(shared_from_this(), EventArgs());
				break;
			case VRM_DEVICE_CALLBACK_TYPE_TARGET_FORMAT_LIST_CHANGED:
				BOOST_FOREACH(const EventHandler& handler, TargetFormatListChanged.m_handlers)
					handler(shared_from_this(), EventArgs());
				break;
			case VRM_DEVICE_CALLBACK_TYPE_PROPERTY_INFO_CHANGED:
				if (fcp_callback_params)
				{
					PropId l_id= *reinterpret_cast<PropId*>(const_cast<void*>(fcp_callback_params));
					BOOST_FOREACH(const PropertyInfoChangedEventHandler& handler, PropertyInfoChanged.m_handlers)
						handler(shared_from_this(), l_id);
				}
				break;
			case VRM_DEVICE_CALLBACK_TYPE_PROPERTY_ATTRIBS_CHANGED:
				if (fcp_callback_params)
				{
					PropId l_id= *reinterpret_cast<PropId*>(const_cast<void*>(fcp_callback_params));
					BOOST_FOREACH(const PropertyAttribsChangedEventHandler& handler, PropertyAttribsChanged.m_handlers)
						handler(shared_from_this(), l_id);
				}
				break;
			default:
				break;
			}
		}
		catch (const std::exception& exc)
		{
			if (!mp_last_callback_exception.get())
				mp_last_callback_exception.reset(
				new std::string(std::string("An event handler raised an exception: ") + exc.what()));
		}
		catch (...)
		{
			if (!mp_last_callback_exception.get())
				mp_last_callback_exception.reset(
				new std::string("An event handler raised an exception"));
		}
	}

	VRmUsbCamDevice Device::get_NativeRepresentation() const
	{
		return m_device;
	}

	// Global objects -----------------------------------------------------------------------------------

	void VRmUsbCam::EnableLogging() { VRMCALL(EnableLogging()); }

	void VRmUsbCam::EnableLoggingEx( const std::string& logFileName ) {
		VRMCALL(EnableLoggingEx(logFileName.c_str()));
	}
	void VRmUsbCam::UpdateDeviceKeyList() {
		VRMCALL(UpdateDeviceKeyList());
	}

	void VRmUsbCam::RestartTimer() { VRMCALL(RestartTimer()); }

	void VRmUsbCam::ConvertImage(const ImagePtr& source, ImagePtr& target) {
		VRMCHECK(source is_not null);
		VRMCHECK(target is_not null);
		if (source->get_NativeRepresentation()==0)
			VRMERR("Source image is not valid");
		if (target->get_NativeRepresentation()==0)
			VRMERR("Target image is not valid");
		VRMCALL(ConvertImage(source->get_NativeRepresentation(), target->get_NativeRepresentation()));
	}

	DevicePtr VRmUsbCam::OpenDevice(DeviceKeyPtr key) {
		VRMCHECK(key is_not null);
		return DevicePtr(new Device(key));
	}

	ImagePtr VRmUsbCam::NewImage(const ImageFormat& format) {
		return ImagePtr(new Image(format));
	}

	ImagePtr VRmUsbCam::SetImage(const ImageFormat& format, void* buffer, int pitch) {
		return ImagePtr(new Image(format, buffer, pitch));
	}

	void VRmUsbCam::SavePNG(const std::string& fileName, const ImagePtr& source, int z_compession_level /* = -1 */)
	{
		VRMCHECK(source is_not null);
		if (source->get_NativeRepresentation()==0)
			VRMERR("Source image is not valid");
		VRMCALL(SavePNG(fileName.c_str(), source->get_NativeRepresentation(), z_compession_level));
	}

	ImagePtr VRmUsbCam::LoadPNG(const std::string& fileName)
	{
		VRmImage* lp_vrmimage = 0;
		VRMCALL(LoadPNG(fileName.c_str(), &lp_vrmimage));
		ImagePtr lp_image = ImagePtr(new Image(lp_vrmimage, ImagePtr()));
		return lp_image;
	}


#ifdef WIN32
	void VRmUsbCam::ActivateDevice(int vendorId, int productId, int serial) {
		VRMCALL(ActivateDevice(VRmWORD(vendorId), VRmWORD(productId), serial));
	}

	void VRmUsbCam::ActivateAllDevices() {
		VRMCALL(ActivateAllDevices());
	}

	void VRmUsbCam::DeactivateDevice(int vendorId, int productId, int serial) {
		VRMCALL(DeactivateDevice(VRmWORD(vendorId), VRmWORD(productId), serial));
	}
#endif

	template <typename Delegate>
	VRmUsbCamCPP::VRmUsbCam::Event<Delegate>::~Event()
	{
		if (!m_handlers.empty())
			VRmUsbCamUnregisterStaticCallback(VRmUsbCamCallbackProxy);
	}

	template class VRmUsbCam::Event<VRmUsbCam::DeviceChangeEventHandler>;
	VRmUsbCam::Event<VRmUsbCam::DeviceChangeEventHandler> VRmUsbCam::DeviceChange;

	void VRmUsbCam::VRmUsbCamCallbackProxy(VRmStaticCallbackType f_type, void* /*fp_user_data*/, const void* fcp_callback_params)
	{
		VRmUsbCam::RaiseEvent(f_type, fcp_callback_params);
	}

	void VRmUsbCam::RaiseEvent(VRmStaticCallbackType f_type, const void* fcp_callback_params)
	{
		try
		{
			switch (f_type)
			{
			case VRM_STATIC_CALLBACK_TYPE_DEVICE_CHANGE:
				if (fcp_callback_params)
				{
					DeviceChangeType l_type= *reinterpret_cast<const DeviceChangeType*>(fcp_callback_params);
					BOOST_FOREACH(const DeviceChangeEventHandler& handler, DeviceChange.m_handlers)
						handler(l_type);
				}
				break;
			default:
				break;
			}
		}
		catch (...)
		{
			// don't propagate this back through VRmUsbCam. I'm sorry.
		}
	}

#undef GetCurrentTime // stupid windows...

	double VRmUsbCam::get_CurrentTime() {
		double l_time;
		VRMCALL(GetCurrentTime(&l_time));
		return l_time;
	}

	std::vector<DeviceKeyPtr> VRmUsbCam::get_DeviceKeyList() {
		tVRmGlobalLockProxy l_global_lock_proxy;
		boost::lock_guard<tVRmGlobalLockProxy> l_global_lock(l_global_lock_proxy);

		VRmDWORD size= 0;
		VRMCALL(GetDeviceKeyListSize(&size));
		std::vector<DeviceKeyPtr> l_array((size_t)size);
		for (VRmDWORD i=0; i<size; ++i) {
			VRmDeviceKey* lp_key= 0;
			VRMCALL(GetDeviceKeyListEntry(i, &lp_key));
			l_array[i]= DeviceKeyPtr(new DeviceKey(lp_key));
		}
		return l_array;
	}

	int VRmUsbCam::get_Version() {
		VRmDWORD version= 0;
		VRMCALL(GetVersion(&version));
		return (int)version;
	}

} // namespace VRmUsbCamCPP

