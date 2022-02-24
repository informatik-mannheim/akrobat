%begin %{
#include <roypy.h>

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wfloat-conversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#endif

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdynamic-class-memaccess"
#endif
%}

%module(directors="1", threads="1") roypy

%{
#ifdef ROYALE_ACTIVATE_NUMPY
#define SWIG_FILE_WITH_INIT
#endif

#include <royale.hpp>
#include <royale/IReplay.hpp>
using namespace royale;
%}

%include <std_vector.i>
%include <stdint.i>
%include <std_map.i>
%include <std_string.i>
%include <std_pair.i>


#ifdef ROYALE_ACTIVATE_NUMPY
%include "numpy.i"

%init %{
    import_array();
%}
%apply (double* ARGOUT_ARRAY1, int DIM1) {(double* p, int n)};
#endif


namespace std
{
  template<typename T>
  class unique_ptr {
  public:
      T* operator->();
      T* get();
  };
}

%define WRAP_VECTOR(NAME,TYPE)
%typemap(out) royale::Vector<TYPE> %{
    $result = SWIG_NewPointerObj(new std::vector<TYPE>($1.toStdVector()), $descriptor(std::vector<TYPE>*), SWIG_POINTER_OWN);
%}
%template (NAME##Vector) std::vector<TYPE>;
%enddef

%define WRAP_VECTOR_TO_PY(TYPE,CONVERT_FUN)
    %typemap(out) royale::Vector<TYPE> %{
    {
        auto pyList = PyList_New($1.size());
        for (decltype ($1.size()) i = 0; i < $1.size(); i++)
        {
            if (PyList_SetItem(pyList, i, CONVERT_FUN ($1[i])))
            {
                SWIG_Error(SWIG_RuntimeError, "Could not create Python list");
            }
        }
        $result = pyList;
    }
    %}
%enddef

%{
    inline PyObject* royaleStrToPy(const royale::String &str)
    {
        return PyUnicode_DecodeUTF8(str.data(), static_cast< Py_ssize_t >(str.size()), "surrogateescape");
    }

    inline PyObject* royale2Py(bool b)
    {
        return PyBool_FromLong(static_cast<long>(b));
    }

    template<typename T>
    inline PyObject* royale2Py(T i)
    {
        return PyLong_FromLong(static_cast<long>(i));
    }

    inline PyObject* royale2Py(royale::CameraAccessLevel l)
    {
        return PyLong_FromLong(static_cast<long>(l));
    }

%}

// Python String -> royale::String
%typemap(in) const royale::String& (royale::String tmp) %{
    void *ptr;
    if(SWIG_IsOK(SWIG_ConvertPtr($input, &ptr, $1_descriptor, 0)))
    {
        $1 = reinterpret_cast<$1_ltype>(ptr);
    }
    else if(PyUnicode_Check($input)) {
        ssize_t size = 0;
        auto ptr = PyUnicode_AsUTF8AndSize($input, &size);
        tmp = royale::String(ptr, size);
        $1 = &tmp;
    }
    else {
        SWIG_exception(SWIG_RuntimeError, "Could not convert argument to string");
    }
%}
%typemap(typecheck) const royale::String& = char *;
%typemap(out) royale::String %{
    $result = royaleStrToPy ($1);
%}

%typemap(out) royale::String & %{
    $result = royaleStrToPy (*$1);
%}

%typemap(out) royale::String * %{
    $result = royaleStrToPy (*$1);
%}

%typemap(in) uint16_t {
    $1 = static_cast<uint16_t> (PyLong_AsLong($input));
}

// Python microseconds time -> std::chrono::microseconds
// used for Royale timestamps
%typemap(in) const std::chrono::microseconds& (std::chrono::microseconds tmp) %{
    $1 = std::chrono::microseconds ($input);

%}
%typemap(out) std::chrono::microseconds %{
    $result = PyLong_FromUnsignedLongLong ($1.count());
%}

namespace royale
{
    typedef uint16_t StreamId;
}

// Vector
WRAP_VECTOR(String, royale::String);
WRAP_VECTOR(Point, royale::DepthPoint);
WRAP_VECTOR(InterPoint, royale::IntermediatePoint);
WRAP_VECTOR(StreamVec, royale::StreamId);
WRAP_VECTOR(Uint8Vec, uint8_t);
WRAP_VECTOR(Uint32Vec, uint32_t);
WRAP_VECTOR_TO_PY(uint32_t, royale2Py);
WRAP_VECTOR_TO_PY(uint16_t, royale2Py);
WRAP_VECTOR_TO_PY(uint8_t, royale2Py);
WRAP_VECTOR_TO_PY(size_t, royale2Py);

typedef uint16_t royale::StreamId;

// Remove output arguments from the interface -> and return them as result
%define WRAP_SIMPLE_ARG(TYPE)
%typemap(in, numinputs=0) (TYPE&) (TYPE temp) {
    $1 = &temp;
}
%typemap(argout) TYPE& {
    $result = royale2Py(*$1);
}
%typemap(argout) TYPE {

}
%enddef

// Remove output arguments from the interface -> and return them as result and get the correct name
%define WRAP_FILTER_ARG(TYPE)
%typemap(in, numinputs=0) (TYPE&) (TYPE temp) {
    $1 = &temp;
}
%typemap(argout) TYPE& {
    $result = royaleStrToPy(getFilterLevelName(*$1));
}
%typemap(argout) TYPE {

}
%enddef

WRAP_SIMPLE_ARG(bool);
WRAP_SIMPLE_ARG(uint16_t);
WRAP_SIMPLE_ARG(royale::CameraAccessLevel);
WRAP_SIMPLE_ARG(royale::ExposureMode);
WRAP_FILTER_ARG(royale::FilterLevel);

%typemap(in, numinputs=0) (royale::String &) (royale::String temp) {
  $1 = &temp;
}
%typemap(argout) royale::String &  {
    $result = royaleStrToPy(*$1);
}
%typemap(argout) const royale::String & {

}

%define WRAP_VECTOR_ARG(TYPE)
%typemap(in, numinputs=0) (royale::Vector<TYPE> &) (royale::Vector<TYPE> temp) {
  $1 = &temp;
}
%typemap(argout) royale::Vector<TYPE>&  {
    std::vector<TYPE>* vec;
    vec = new std::vector<TYPE>($1->toStdVector());
    $result = SWIG_NewPointerObj(vec, $descriptor(std::vector<TYPE>*), SWIG_POINTER_OWN);
}
%enddef
WRAP_VECTOR_ARG(royale::String);
WRAP_VECTOR_ARG(royale::StreamId);
WRAP_VECTOR_ARG(uint8_t);
WRAP_VECTOR_ARG(uint32_t);

// Raise an exception if a CameraStatus != SUCCESS was encoutered
%typemap(out) royale::CameraStatus {
    if($1 != CameraStatus::SUCCESS)
    {
        SWIG_exception (SWIG_RuntimeError, royale::getErrorString($1).c_str());
    }
    $result = PyLong_FromLong(static_cast<long>($1));
}

// Access to points member in DepthData struct
%extend royale::DepthData {
    std::vector<royale::DepthPoint> points() {
        return $self->points.toStdVector();
    }
}
%ignore royale::DepthData::points;

// Access to points member in IntermediateData struct
%extend royale::IntermediateData {
    std::vector<royale::IntermediatePoint> points() {
        return $self->points.toStdVector();
    }
}
%ignore royale::IntermediateData::points;

// Access to points member in IntermediateData struct
%extend royale::IntermediateData {
    float getDistance(size_t idx) {
        return $self->points[idx].distance;
        }
    float getAmplitude(size_t idx) {
        return $self->points[idx].amplitude;
        }
    float getIntensity(size_t idx) {
        return $self->points[idx].intensity;
        }
    uint32_t getFlags(size_t idx) {
        return $self->points[idx].flags;
        }
    size_t getNumPoints() {
        return $self->points.size ();
        }
}
%ignore royale::IntermediateData::points;



#ifdef ROYALE_ACTIVATE_NUMPY
%extend royale::DepthData {
    void fastPoints (double * p, int n) {
        for (auto i = 0; i < n; i+=6) {
            p[i] = $self->points[i/6].x;
            p[i+1] = $self->points[i/6].y;
            p[i+2] = $self->points[i/6].z;
            p[i+3] = $self->points[i/6].noise;
            p[i+4] = $self->points[i/6].grayValue;
            p[i+5] = $self->points[i/6].depthConfidence;
        }
    }
  
    %pythoncode %{
def npoints (self):
    return self.fastPoints(self.width * self.height * 6).reshape(-1,self.width,6)
%}
}
#else
%extend royale::DepthData {
    void fastPoints (double * p, int n) {
    }
  
    %pythoncode %{
def npoints (self):
    print('Roypy was compiled without numpy support!')
    raise RuntimeError('Roypy was compiled without numpy support!')
%}
}
#endif


// Access to points member in DepthData struct
%extend royale::DepthData {
    float getX(size_t idx) {
        return $self->points[idx].x;
        }
    float getY(size_t idx) {
        return $self->points[idx].y;
        }
    float getZ(size_t idx) {
        return $self->points[idx].z;
        }
    float getNoise(size_t idx) {
        return $self->points[idx].noise;
        }
    int getGrayValue(size_t idx) {
        return $self->points[idx].grayValue;
        }
    int getDepthConfidence(size_t idx) {
        return $self->points[idx].depthConfidence;
        }
    size_t getNumPoints() {
        return $self->points.size ();
        }
}
%ignore royale::DepthData::points;

// Access to points member in DepthImage struct
%extend royale::DepthImage {
    std::vector<uint16_t> cdData() {
        return $self->cdData.toStdVector();
    }
}
%ignore royale::DepthImage::cdData;

// Access to points member in DepthImage struct
%extend royale::DepthImage {
    float getCDData(size_t idx) {
        return $self->cdData[idx];
        }

    size_t getNumPoints() {
        return $self->cdData.size ();
        }
}
%ignore royale::DepthImage::cdData;

// Make processing parameters more pythonic
%typemap(in,numinputs=0) (royale::ProcessingParameterVector &) (royale::ProcessingParameterVector temp) {
    $1 = &temp;
}
%typemap(argout) royale::ProcessingParameterVector & {
    auto dict = PyDict_New();
    for (const auto &e : *$1)
    {
        // gets the flag name in C
        auto cKeyName = royale::getProcessingFlagName (static_cast<royale::ProcessingFlag> (e.first));
        // converts to python readable format
        auto keyName = PyUnicode_DecodeLatin1 (cKeyName.data(), cKeyName.length(), nullptr);

        // determine the value
        PyObject *val = nullptr;
        switch (e.second.variantType())
        {
            case royale::VariantType::Int:
                val = PyLong_FromLong (e.second.getInt());
                break;
            case royale::VariantType::Float:
                val = PyFloat_FromDouble (e.second.getFloat());
                break;
            case royale::VariantType::Bool:
                val = PyBool_FromLong (e.second.getBool() ? 1 : 0);
                break;
            default:
                SWIG_exception (SWIG_RuntimeError, "Invalid Variant found");
        }

        PyDict_SetItem (dict, keyName, val);
        Py_DECREF (keyName);
        Py_DECREF (val);
    }

    $result = dict;
}
%typemap(in) (const royale::ProcessingParameterVector &) (royale::ProcessingParameterVector vec)
{
    bool err = true;
    if (PyMapping_Check ($input))
    {
        err = false;
        auto size = PyMapping_Length ($input);
        auto keys = PyMapping_Keys ($input);
        auto values = PyMapping_Values ($input);
        for (auto i = 0; i < size; i++)
        {
            auto namePyFlag = PySequence_GetItem (keys, i);

            // current flag in python readable format
            PyObject *temp_bytes = PyUnicode_AsEncodedString (namePyFlag, "UTF-8", "strict");

            // convert it back so we can parse and get the correct key in p
            const char *s = PyString_AsString (temp_bytes);
            ProcessingFlag p;
            royale::parseProcessingFlagName (s, p);

            auto pyFlag = PyLong_FromLong (static_cast<long> (p));

            auto val = PySequence_GetItem (values, i);

            if (p < royale::ProcessingFlag::NUM_FLAGS)
            {
                royale::Variant var;
                if (PyBool_Check (val))
                {
                    var.setBool (PyLong_AsLong (val) > 0);
                }
                else if (PyLong_Check (val))
                {
                    var.setInt (static_cast<int> (PyLong_AsLong (val)));
                }
                else if (PyFloat_Check (val))
                {
                    var.setFloat (static_cast<float> (PyFloat_AsDouble (val)));
                }
                else
                {
                    err = true;
                }
                vec.push_back (royale::Pair<royale::ProcessingFlag, royale::Variant> (p, std::move (var)));
            }

            Py_DECREF (pyFlag);
            Py_DECREF (val);
        }
        Py_DECREF (keys);
        Py_DECREF (values);
    }

    if (err)
    {
        SWIG_exception (SWIG_RuntimeError, "Could not map value to a valid ProcessingFlag");
    }

    $1 = &vec;
}
%typemap(argout) (const royale::ProcessingParameterVector &) {

}
// TODO: Unsure why this does not work with the overload / typecheck map combination
%ignore royale::ICameraDevice::setProcessingParameters(const royale::ProcessingParameterVector&);

%ignore royale::getVersion(unsigned int &,unsigned int &,unsigned int &,unsigned int &,royale::String &,royale::String &);
%ignore royale::getVersion(unsigned int &,unsigned int &,unsigned int &,unsigned int &,royale::String &);
%ignore royale::getVersion(unsigned int &,unsigned int &,unsigned int &);

%apply unsigned int *OUTPUT { unsigned &major, unsigned &minor, unsigned &patch, unsigned &build };
void royale::getVersion(unsigned &major, unsigned &minor, unsigned &patch, unsigned &build);

%ignore royale::ICameraDevice::setExposureMode (royale::ExposureMode);
%ignore royale::ICameraDevice::getExposureMode (royale::ExposureMode&) const;

%ignore royale::ICameraDevice::setFilterLevel (const royale::FilterLevel);
%ignore royale::ICameraDevice::getFilterLevel (royale::FilterLevel&) const;

%pythoncode %{
def getVersionString ():
    a,b,c,d = getVersion()
    return str(a)+'.'+str(b)+'.'+str(c)+'.'+str(d)
%}

// createCamera Function
%ignore royale::CameraManager::createCamera(const royale::String&, const royale::TriggerMode);
%template(ICameraDevicePtr) std::unique_ptr<royale::ICameraDevice>;
%typemap(out) std::unique_ptr<royale::ICameraDevice> %{
    $result = SWIG_NewPointerObj(new $1_ltype(std::move($1)), $&1_descriptor, SWIG_POINTER_OWN);
%}

%template() std::pair<const std::string, float>;
%template(LensParamMap) std::map<std::string, float>;
%ignore royale::ICameraDevice::getLensParameters (royale::LensParameters &param);
%extend royale::ICameraDevice {
    std::map<std::string,float> getLensParameters()
    {
        std::map<std::string,float> retParams;

        royale::LensParameters lensParam;
        auto status = $self->getLensParameters(lensParam);

        if (status != CameraStatus::SUCCESS)
        {
            SWIG_Error(SWIG_RuntimeError, "Problem reading out lens parameters");
            return retParams;
        }

        retParams.insert(std::make_pair("cx", lensParam.principalPoint.first));
        retParams.insert(std::make_pair("cy", lensParam.principalPoint.second));

        retParams.insert(std::make_pair("fx", lensParam.focalLength.first));
        retParams.insert(std::make_pair("fy", lensParam.focalLength.second));

        retParams.insert(std::make_pair("p1", lensParam.distortionTangential.first));
        retParams.insert(std::make_pair("p2", lensParam.distortionTangential.second));

        retParams.insert(std::make_pair("k1", lensParam.distortionRadial.at(0)));
        retParams.insert(std::make_pair("k2", lensParam.distortionRadial.at(1)));
        retParams.insert(std::make_pair("k3", lensParam.distortionRadial.at(2)));

        return retParams;
    }
}

%template() std::pair<std::string,std::string>;
%template(CamInfoMap) std::vector<std::pair<std::string, std::string>>;

%ignore royale::ICameraDevice::getCameraInfo (royale::Vector<royale::Pair<royale::String, royale::String>> &camInfo);
%extend royale::ICameraDevice {
    std::vector<std::pair<std::string, std::string>> getCameraInfo ()
    {
        royale::Vector<royale::Pair<royale::String, royale::String>> cameraInfo;
        $self->getCameraInfo (cameraInfo);

        std::vector<std::pair<std::string, std::string>> retCamInfo;

        for (auto camInfo : cameraInfo)
        {
            retCamInfo.push_back (std::pair<std::string, std::string>(camInfo.first.toStdString(), camInfo.second.toStdString()));
        }

        return retCamInfo;
    }
}

%ignore royale::ICameraDevice::getNumberOfStreams (const royale::String &name, uint32_t &nrStreams);
%extend royale::ICameraDevice {
    uint32_t getNumberOfStreams (const royale::String &name)
    {
        uint32_t nrStreams;
        $self->getNumberOfStreams (name, nrStreams);

        return nrStreams;
    }
}

%template(exposurePair) std::pair<uint32_t, uint32_t>;
%ignore royale::ICameraDevice::getExposureLimits (royale::Pair<uint32_t, uint32_t> &exposureLimits, royale::StreamId streamId = 0);
%extend royale::ICameraDevice {
    std::pair<uint32_t, uint32_t> getExposureLimits (royale::StreamId streamId = 0)
    {
        royale::Pair<uint32_t, uint32_t> exposureLimits;
        auto status = $self->getExposureLimits (exposureLimits, streamId);

        if (status != royale::CameraStatus::SUCCESS)
        {
           SWIG_Error(SWIG_RuntimeError, "Problem getting exposure limits");
           return std::pair<uint32_t, uint32_t> (0u, 0u);
        }

        return std::pair<uint32_t, uint32_t> (exposureLimits.first, exposureLimits.second);
    }
}
%ignore royale::ICameraDevice::getExposureLimits (const String &exposureGroup, royale::Pair<uint32_t, uint32_t> &exposureLimits);
%extend royale::ICameraDevice {
    std::pair<uint32_t, uint32_t> getExposureLimits (const royale::String &exposureGroup)
    {
        royale::Pair<uint32_t, uint32_t> exposureLimits;
        auto status = $self->getExposureLimits (exposureGroup, exposureLimits);

        if (status != royale::CameraStatus::SUCCESS)
        {
            SWIG_Error(SWIG_RuntimeError, "Problem getting exposure limits");
            return std::pair<uint32_t, uint32_t> (0u, 0u);
        }

        return std::pair<uint32_t, uint32_t> (exposureLimits.first, exposureLimits.second);
    }
}

%ignore royale::ICameraDevice::getLensCenter (uint16_t &x, uint16_t &y);
%extend royale::ICameraDevice {
    uint16_t getLensCenterX ()
    {
        uint16_t x, y;
        $self->getLensCenter (x, y);

        return x;
    }
    uint16_t getLensCenterY ()
    {
        uint16_t x, y;
        $self->getLensCenter (x, y);

        return y;
    }
}

%pythoncode %{
def getLensCenter (self):
    x = self.getLensCenterX ()
    y = self.getLensCenterY ()
    return x,y

ICameraDevicePtr.getLensCenter = getLensCenter
%}


%ignore royale::ICameraDevice::readRegisters (royale::Vector<royale::Pair<royale::String, uint64_t>> &registers);
%extend royale::ICameraDevice {
    uint16_t readRegister (const uint16_t registerAddress)
    {
        std::stringstream ss;

        ss << registerAddress;
        royale::String address(ss.str());
        royale::Vector<royale::Pair<royale::String, uint64_t>> rdRegister = { { address, 0x0 } };
        auto status = $self->readRegisters (rdRegister);

        if (status != CameraStatus::SUCCESS)
        {
            SWIG_Error(SWIG_RuntimeError, "Problem reading register");
            return 0u;
        }

        return static_cast<uint16_t> (rdRegister.at (0).second);
    }
}

%ignore royale::ICameraDevice::writeRegisters (const royale::Vector<royale::Pair<royale::String, uint64_t>> &registers);
%extend royale::ICameraDevice {
    void writeRegister (const uint16_t registerAddress, const uint16_t registerValue)
    {
        std::stringstream ss;

        ss << registerAddress;
        royale::String address(ss.str());
        royale::Vector<royale::Pair<royale::String, uint64_t>> rdRegister = { { address, registerValue } };
        auto status = $self->writeRegisters (rdRegister);

        if (status != CameraStatus::SUCCESS)
        {
            SWIG_Error(SWIG_RuntimeError, "Problem writing register");
        }
    }
}

%ignore royale::ICameraDevice::setExposureTimes (const royale::Vector<uint32_t> &exposureTimes, royale::StreamId streamId = 0);
%extend royale::ICameraDevice {
    void setExposureTimes (const std::vector<uint32_t> exposureTimes)
    {
        $self->setExposureTimes (exposureTimes);
    }
}

%extend royale::ICameraDevice {
    royale::IReplay* asReplay ()
    {
        auto replay = dynamic_cast<IReplay *> (self);
        
        if (replay == nullptr)
        {
            SWIG_Error(SWIG_RuntimeError, "Error retrieving IReplay interface");
        }
        return replay;
    }
}

%ignore royale::ICameraDevice::setCalibrationData (const royale::String &filename);
%extend royale::ICameraDevice {
    void setCalibrationDataFromFile (const royale::String &filename)
    {
        $self->setCalibrationData (filename);
    }
}

%ignore royale::ICameraDevice::setCalibrationData (const royale::Vector<uint8_t> &data);
%extend royale::ICameraDevice {
    void setCalibrationDataFromBuffer (const std::vector<uint8_t> &data)
    {
        $self->setCalibrationData (royale::Vector<uint8_t>::fromStdVector (data));
    }
}

// Wrapped Royale headers
%include <royale/Definitions.hpp>
%include <royale.hpp>
%include <royale/CameraManager.hpp>
%include <royale/ICameraDevice.hpp>
%include <royale/Status.hpp>
%include <royale/CameraAccessLevel.hpp>
%feature("director") royale::IDepthDataListener;
%include <royale/IDepthDataListener.hpp>
%feature("director") royale::IDepthImageListener;
%include <royale/IDepthImageListener.hpp>
%include <royale/IExtendedData.hpp>
%ignore royale::RawData::rawData;
%include <royale/RawData.hpp>
%extend royale::RawData {
    royale::Vector<uint16_t> getRawPhase(unsigned idx)
    {
        if ($self->rawData.size() > idx)
        {
            auto rawPtr = $self->rawData[idx];
            return royale::Vector<uint16_t>(rawPtr, rawPtr + $self->width * $self->height);
        }

        SWIG_Error(SWIG_IndexError, "Requested rawData does not exist");
        return royale::Vector<uint16_t>{};
    }
}

%ignore royale::IntermediateData::operator= (const IntermediateData &dd);
%ignore royale::DepthData::operator= (const DepthData &dd);

%rename(_None) None;
%include <royale/CallbackData.hpp>
%include <royale/IntermediateData.hpp>
%feature("director") royale::IExtendedDataListener;
%include <royale/IExtendedDataListener.hpp>
%feature("director") royale::IRecordStopListener;
%include <royale/IRecordStopListener.hpp>
%include <royale/DepthData.hpp>
%include <royale/DepthImage.hpp>
%include <royale/RawData.hpp>
//%include <processing/ExtendedData.hpp>
%include <royale/ProcessingFlag.hpp>
%include <royale/ExposureMode.hpp>
%include <royale/FilterLevel.hpp>
%include <royale/LensParameters.hpp>
%include <royale/IReplay.hpp>
