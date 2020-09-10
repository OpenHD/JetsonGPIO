/*
Copyright (c) 2012-2017 Ben Croston ben@croston.org.
Copyright (c) 2019, NVIDIA CORPORATION.
Copyright (c) 2019 Jueon Park(pjueon) bluegbg@gmail.com.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/


#include <unistd.h>
#include <dirent.h>
#include <iostream>
#include <string>
#include <stdexcept>
#include <vector>
#include <map>
#include <set>
#include <fstream>
#include <sstream>
#include <cctype>
#include <algorithm>

#include <chrono>
#include <thread>

#include "JetsonGPIO.h"
#include "gpio_pin_data.h"
#include "PythonFunctions.h"

using namespace GPIO;


// The user CAN'T use GPIO::UNKNOW, GPIO::HARD_PWM 
// These are only for implementation
constexpr Directions UNKNOWN = Directions::UNKNOWN;
constexpr Directions HARD_PWM = Directions::HARD_PWM;

// Jetson Models
enum class Model{ JETSON_XAVIER, JETSON_TX2, JETSON_TX1, JETSON_NANO };

extern const Model JETSON_XAVIER = Model::JETSON_XAVIER;
extern const Model JETSON_TX2 = Model::JETSON_TX2;
extern const Model JETSON_TX1 = Model::JETSON_TX1;
extern const Model JETSON_NANO = Model::JETSON_NANO;



constexpr auto _SYSFS_ROOT = "/sys/class/gpio";


//================================================================================
// All global variables are wrapped in a singleton class except for public APIs, 
// in order to avoid initialization order problem among global variables in different compilation units.

class GlobalVariableWrapper{
private:
    GlobalVariableWrapper()
    : _gpio_data(get_data()), // Get GPIO pin data
      _model(_gpio_data.model),
      _JETSON_INFO(_gpio_data.pin_info),
      _channel_data_by_mode(_gpio_data.channel_data),
    _gpio_warnings(true), _gpio_mode(NumberingModes::None)
    {
        _CheckPermission();
    }

    void _CheckPermission(){
        std::string path1 = std::string(_SYSFS_ROOT) + "/export";
        std::string path2 = std::string(_SYSFS_ROOT) + "/unexport";
        if(!os_access(path1, W_OK) || !os_access(path2, W_OK) ){
            std::cerr << "[ERROR] The current user does not have permissions set to "
                    "access the library functionalites. Please configure "
                    "permissions or use the root user to run this." << std::endl;
            throw std::runtime_error("Permission Denied.");
        }
    }

public:
    GlobalVariableWrapper(const GlobalVariableWrapper&) = delete;
    GlobalVariableWrapper& operator=(const GlobalVariableWrapper&) = delete;

    ~GlobalVariableWrapper() = default;

    static GlobalVariableWrapper& get_instance(){
        static GlobalVariableWrapper singleton;
        return singleton;
    }
    
    // -----Global Variables----
    // NOTE: DON'T change the declaration order of fields.
    // declaration order == initialization order

    GPIO_data _gpio_data;
    const Model _model;
    const _GPIO_PIN_INFO _JETSON_INFO;
    const std::map<GPIO::NumberingModes, std::map<std::string, ChannelInfo>> _channel_data_by_mode;

    // A map used as lookup tables for pin to linux gpio mapping
    std::map<std::string, ChannelInfo> _channel_data;

    bool _gpio_warnings;
    NumberingModes _gpio_mode;
    std::map<std::string, Directions> _channel_configuration; 
};

//================================================================================
GlobalVariableWrapper& GlobalVariables = GlobalVariableWrapper::get_instance();

//================================================================================


// This function will be called when the global variable named "JETSON_INFO" is initialized.
const std::string _get_JETSON_INFO(){
    // GlobalVariableWrapper singleton object must be initialized before JETSON_INFO.
    // The reason that JETSON_INFO is not wrapped by GlobalVariableWrapper
    // is because it belongs to API
    GlobalVariableWrapper& GlobalVariables = GlobalVariableWrapper::get_instance();

    std::stringstream ss;
    ss << "[JETSON_INFO]\n";
    ss << "P1_REVISION: " << GlobalVariables._JETSON_INFO.P1_REVISION << std::endl;
    ss << "RAM: " << GlobalVariables._JETSON_INFO.RAM << std::endl;
    ss << "REVISION: " << GlobalVariables._JETSON_INFO.REVISION << std::endl;
    ss << "TYPE: " << GlobalVariables._JETSON_INFO.TYPE << std::endl;
    ss << "MANUFACTURER: " << GlobalVariables._JETSON_INFO.MANUFACTURER << std::endl;
    ss << "PROCESSOR: " << GlobalVariables._JETSON_INFO.PROCESSOR << std::endl;
    return ss.str();
}

// This function will be called when the global variable named "model" is initialized.
const char* _get_model(){
    // GlobalVariableWrapper singleton object must be initialized before model.
    // The reason that model is not wrapped by GlobalVariableWrapper
    // is because it belongs to API
    GlobalVariableWrapper& GlobalVariables = GlobalVariableWrapper::get_instance();

    if(GlobalVariables._model == Model::JETSON_XAVIER) return "JETSON_XAVIER";
    else if(GlobalVariables._model == Model::JETSON_TX1) return "JETSON_TX1";
    else if(GlobalVariables._model == Model::JETSON_TX2) return "JETSON_TX2";
    else if(GlobalVariables._model == Model::JETSON_NANO) return "JETSON_NANO";
    else{
        throw std::runtime_error("get_model error");
    };
}



void _validate_mode_set(){
    if(GlobalVariables._gpio_mode == NumberingModes::None)
        throw std::runtime_error("Please set pin numbering mode using "
                           "GPIO::setmode(GPIO::BOARD), GPIO::setmode(GPIO::BCM), "
                           "GPIO::setmode(GPIO::TEGRA_SOC) or "
                           "GPIO::setmode(GPIO::CVM)");
}


ChannelInfo _channel_to_info_lookup(const std::string& channel, bool need_gpio, bool need_pwm){
    if(GlobalVariables._channel_data.find(channel) == GlobalVariables._channel_data.end())
        throw std::runtime_error("Channel " + channel + " is invalid");
    ChannelInfo ch_info = GlobalVariables._channel_data.at(channel);
    if (need_gpio && ch_info.gpio_chip_dir == "None")
        throw std::runtime_error("Channel " + channel + " is not a GPIO");
    if (need_pwm && ch_info.pwm_chip_dir == "None")
        throw std::runtime_error("Channel " + channel + " is not a PWM");
    return ch_info;
}


ChannelInfo _channel_to_info(const std::string& channel, bool need_gpio = false, bool need_pwm = false){
    _validate_mode_set();
    return _channel_to_info_lookup(channel, need_gpio, need_pwm);
}


std::vector<ChannelInfo> _channels_to_infos(const std::vector<std::string>& channels, bool need_gpio = false, bool need_pwm = false){
    _validate_mode_set();
    std::vector<ChannelInfo> ch_infos;
    for (const auto& c : channels){
        ch_infos.push_back(_channel_to_info_lookup(c, need_gpio, need_pwm));
    }
    return ch_infos;
}


/* Return the current configuration of a channel as reported by sysfs. 
   Any of IN, OUT, HARD_PWM, or UNKNOWN may be returned. */
Directions _sysfs_channel_configuration(const ChannelInfo& ch_info){
    if (ch_info.pwm_chip_dir != "None") {
        std::string pwm_dir = ch_info.pwm_chip_dir + "/pwm" + std::to_string(ch_info.pwm_id);
        if (os_path_exists(pwm_dir))
            return HARD_PWM;
    }

    std::string gpio_dir = std::string(_SYSFS_ROOT) + "/gpio" + std::to_string(ch_info.gpio);
    if (!os_path_exists(gpio_dir))
        return UNKNOWN;  // Originally returns None in NVIDIA's GPIO Python Library

    std::string gpio_direction;
    { // scope for f
        std::ifstream f_direction(gpio_dir + "/direction");
        std::stringstream buffer;
        buffer << f_direction.rdbuf();
        gpio_direction = buffer.str();
        gpio_direction = strip(gpio_direction);
        // lower()
        std::transform(gpio_direction.begin(), gpio_direction.end(), 
                gpio_direction.begin(), [](unsigned char c){ return tolower(c); });
    } // scope ends

    if (gpio_direction == "in")
        return IN;
    else if (gpio_direction == "out")
        return OUT;
    else
        return UNKNOWN;  // Originally returns None in NVIDIA's GPIO Python Library
}


/* Return the current configuration of a channel as requested by this
   module in this process. Any of IN, OUT, or UNKNOWN may be returned. */
Directions _app_channel_configuration(const ChannelInfo& ch_info){
    if (GlobalVariables._channel_configuration.find(ch_info.channel) == GlobalVariables._channel_configuration.end())
        return UNKNOWN;   // Originally returns None in NVIDIA's GPIO Python Library
    return GlobalVariables._channel_configuration[ch_info.channel];
}


void _export_gpio(const int gpio){
    if(os_path_exists(std::string(_SYSFS_ROOT) + "/gpio" + std::to_string(gpio) ))
        return;
    { // scope for f_export
        std::ofstream f_export(std::string(_SYSFS_ROOT) + "/export");
        f_export << gpio;
    } // scope ends

    std::string value_path = std::string(_SYSFS_ROOT) + "/gpio" + std::to_string(gpio) + "/value";
    
    int time_count = 0;
    while (!os_access(value_path, R_OK | W_OK)){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
	if(time_count++ > 100) throw std::runtime_error("Permission denied: path: " + value_path + "\n Please configure permissions or use the root user to run this.");
    }
}


void _unexport_gpio(const int gpio){
    if (!os_path_exists(std::string(_SYSFS_ROOT) + "/gpio" + std::to_string(gpio)))
        return;

    std::ofstream f_unexport(std::string(_SYSFS_ROOT) + "/unexport");
    f_unexport << gpio;
}


void _output_one(const std::string gpio, const int value){
    std::ofstream value_file(std::string(_SYSFS_ROOT) + "/gpio" + gpio + "/value");
    value_file << int(bool(value));
}

void _output_one(const int gpio, const int value){
    _output_one(std::to_string(gpio), value);
}


void _setup_single_out(const ChannelInfo& ch_info, int initial = -1){
    _export_gpio(ch_info.gpio);

    std::string gpio_dir_path = std::string(_SYSFS_ROOT) + "/gpio" + std::to_string(ch_info.gpio) + "/direction";
    { // scope for direction_file
        std::ofstream direction_file(gpio_dir_path);
        direction_file << "out";
    } // scope ends

    if(initial != -1)
        _output_one(ch_info.gpio, initial);
    
    GlobalVariables._channel_configuration[ch_info.channel] = OUT;
}


void _setup_single_in(const ChannelInfo& ch_info){
    _export_gpio(ch_info.gpio);

    std::string gpio_dir_path = std::string(_SYSFS_ROOT) + "/gpio" + std::to_string(ch_info.gpio) + "/direction";
    { // scope for direction_file
        std::ofstream direction_file(gpio_dir_path);
        direction_file << "in";
    } // scope ends

    GlobalVariables._channel_configuration[ch_info.channel] = IN;
}

std::string _pwm_path(const ChannelInfo& ch_info){
    return ch_info.pwm_chip_dir + "/pwm" + std::to_string(ch_info.pwm_id);
}


std::string _pwm_export_path(const ChannelInfo& ch_info){
    return ch_info.pwm_chip_dir + "/export";
}


std::string _pwm_unexport_path(const ChannelInfo& ch_info){
    return ch_info.pwm_chip_dir + "/unexport";
}


std::string _pwm_period_path(const ChannelInfo& ch_info){
    return _pwm_path(ch_info) + "/period";
}


std::string _pwm_duty_cycle_path(const ChannelInfo& ch_info){
    return _pwm_path(ch_info) + "/duty_cycle";
}


std::string _pwm_enable_path(const ChannelInfo& ch_info){
    return _pwm_path(ch_info) + "/enable";
}


void _export_pwm(const ChannelInfo& ch_info){
    if(os_path_exists(_pwm_path(ch_info)))
        return;
    
    { // scope for f
        std::string path = _pwm_export_path(ch_info);
	    std::ofstream f(path);
	
	if(!f.is_open()) throw std::runtime_error("Can't open " + path);

        f << ch_info.pwm_id;
    } // scope ends

    std::string enable_path = _pwm_enable_path(ch_info);
	
    int time_count = 0;
    while (!os_access(enable_path, R_OK | W_OK)){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
	if(time_count++ > 100) throw std::runtime_error("Permission denied: path: " + enable_path + "\n Please configure permissions or use the root user to run this.");
    }
}


void _unexport_pwm(const ChannelInfo& ch_info){
    std::ofstream f(_pwm_unexport_path(ch_info));
    f << ch_info.pwm_id;
}


void _set_pwm_period(const ChannelInfo& ch_info, const int period_ns){
    std::ofstream f(_pwm_period_path(ch_info));
    f << period_ns;
}


void _set_pwm_duty_cycle(const ChannelInfo& ch_info, const int duty_cycle_ns){
    std::ofstream f(_pwm_duty_cycle_path(ch_info));
    f << duty_cycle_ns;
}


void _enable_pwm(const ChannelInfo& ch_info){
    std::ofstream f(_pwm_enable_path(ch_info));
    f << 1;
}


void _disable_pwm(const ChannelInfo& ch_info){
    std::ofstream f(_pwm_enable_path(ch_info));
    f << 0;
}


void  _cleanup_one(const ChannelInfo& ch_info){
    Directions app_cfg = GlobalVariables._channel_configuration[ch_info.channel];
    if (app_cfg == HARD_PWM){
        _disable_pwm(ch_info);
        _unexport_pwm(ch_info);
    }
    else{
        // event.event_cleanup(ch_info.gpio)  // not implemented yet
        _unexport_gpio(ch_info.gpio);
    }
    GlobalVariables._channel_configuration.erase(ch_info.channel);
}


void _cleanup_all(){
    for (const auto& _pair : GlobalVariables._channel_configuration){
        const auto& channel = _pair.first;
        ChannelInfo ch_info = _channel_to_info(channel);
        _cleanup_one(ch_info);
    }
     GlobalVariables._gpio_mode = NumberingModes::None;
}




//==================================================================================
// APIs

extern const std::string GPIO::model = _get_model();
extern const std::string GPIO::JETSON_INFO = _get_JETSON_INFO();


/* Function used to enable/disable warnings during setup and cleanup. */
void GPIO::setwarnings(bool state){
    GlobalVariables._gpio_warnings = state;
}


// Function used to set the pin mumbering mode. 
// Possible mode values are BOARD, BCM, TEGRA_SOC and CVM
void GPIO::setmode(NumberingModes mode){
    try{
        // check if mode is valid
        if(mode == NumberingModes::None)
            throw std::runtime_error("Pin numbering mode must be "
                                "GPIO::BOARD, GPIO::BCM, "
                                "GPIO::TEGRA_SOC or "
                                "GPIO::CVM");
        // check if a different mode has been set                        
        if(GlobalVariables._gpio_mode != NumberingModes::None && mode != GlobalVariables._gpio_mode)
            throw std::runtime_error("A different mode has already been set!");
        
        GlobalVariables._channel_data = GlobalVariables._channel_data_by_mode.at(mode);
        GlobalVariables._gpio_mode = mode;
    }
    catch(std::exception& e){
        std::cerr << "[Exception] " << e.what() << " (catched from: setmode())" << std::endl;
        std::terminate();
    }
}


// Function used to get the currently set pin numbering mode
NumberingModes GPIO::getmode(){
    return GlobalVariables._gpio_mode;
}


/* Function used to setup individual pins or lists/tuples of pins as
   Input or Output. direction must be IN or OUT, initial must be 
   HIGH or LOW and is only valid when direction is OUT  */
void GPIO::setup(const std::string& channel, Directions direction, int initial){
    try{
        ChannelInfo ch_info = _channel_to_info(channel, true);

        if (GlobalVariables._gpio_warnings){
            Directions sysfs_cfg = _sysfs_channel_configuration(ch_info);
            Directions app_cfg = _app_channel_configuration(ch_info);

            if (app_cfg == UNKNOWN && sysfs_cfg != UNKNOWN){
                std::cerr << "[WARNING] This channel is already in use, continuing anyway. Use GPIO::setwarnings(false) to disable warnings.\n";
            }
            
        }

        if(direction != OUT && direction != IN)
            throw std::runtime_error("GPIO direction must be GPIO::IN or GPIO::OUT");

        if(direction == OUT){
            _setup_single_out(ch_info, initial);
        }
        else{ // IN
            if(initial != -1) throw std::runtime_error("initial parameter is not valid for inputs");
            _setup_single_in(ch_info);
        }
    }
    catch(std::exception& e){
        std::cerr << "[Exception] " << e.what() << " (catched from: setup())" << std::endl;
        std::terminate();
    }
}


void GPIO::setup(int channel, Directions direction, int initial){
    setup(std::to_string(channel), direction, initial);
}


/* Function used to cleanup channels at the end of the program.
   If no channel is provided, all channels are cleaned */
void GPIO::cleanup(const std::string& channel){
    try{
        // warn if no channel is setup
        if (GlobalVariables._gpio_mode == NumberingModes::None && GlobalVariables._gpio_warnings){
            std::cerr << "[WARNING] No channels have been set up yet - nothing to clean up! "
                    "Try cleaning up at the end of your program instead!";
            return;
        }

        // clean all channels if no channel param provided
        if (channel == "None"){
            _cleanup_all();
            return;
        }

        ChannelInfo ch_info = _channel_to_info(channel);
        if (GlobalVariables._channel_configuration.find(ch_info.channel) != GlobalVariables._channel_configuration.end()){
            _cleanup_one(ch_info);
        }
    }
    catch (std::exception& e){
        std::cerr << "[Exception] " << e.what() << " (catched from: cleanup())" << std::endl;
        std::terminate();
    }
}

void GPIO::cleanup(int channel){
    std::string str_channel = std::to_string(channel);
    cleanup(str_channel);
}

/* Function used to return the current value of the specified channel.
   Function returns either HIGH or LOW */
int GPIO::input(const std::string& channel){
    try{
        ChannelInfo ch_info = _channel_to_info(channel, true);

        Directions app_cfg = _app_channel_configuration(ch_info);

        if (app_cfg != IN && app_cfg != OUT)
            throw std::runtime_error("You must setup() the GPIO channel first");

        { // scope for value
            std::ifstream value(std::string(_SYSFS_ROOT) + "/gpio" + std::to_string(ch_info.gpio) + "/value");
            int value_read;
            value >> value_read;
            return value_read;
        } // scope ends
    }
    catch (std::exception& e){
        std::cerr << "[Exception] " << e.what() << " (catched from: input())" << std::endl;
        std::terminate();
    }
}

int GPIO::input(int channel){
    return input(std::to_string(channel));
}


/* Function used to set a value to a channel.
   Values must be either HIGH or LOW */
void GPIO::output(const std::string& channel, int value){
    try{
        ChannelInfo ch_info = _channel_to_info(channel, true);
        // check that the channel has been set as output
        if(_app_channel_configuration(ch_info) != OUT)
            throw std::runtime_error("The GPIO channel has not been set up as an OUTPUT");
        _output_one(ch_info.gpio, value);
    }
    catch (std::exception& e){
        std::cerr << "[Exception] " << e.what() << " (catched from: output())" << std::endl;
        std::terminate();
    }
}

void GPIO::output(int channel, int value){
    output(std::to_string(channel), value);
}


/* Function used to check the currently set function of the channel specified. */
Directions GPIO::gpio_function(const std::string& channel){
    try{
        ChannelInfo ch_info = _channel_to_info(channel);
        return _sysfs_channel_configuration(ch_info);
    }
    catch (std::exception& e){
        std::cerr << "[Exception] " << e.what() << " (catched from: gpio_function())" << std::endl;
        std::terminate();
    }
}

Directions GPIO::gpio_function(int channel){
    gpio_function(std::to_string(channel));
}



//PWM class ==========================================================
struct GPIO::PWM::Impl {
    ChannelInfo _ch_info;
    bool _started;
    int _frequency_hz;
    int _period_ns;
    double _duty_cycle_percent;
    int _duty_cycle_ns;
    ~Impl() = default;

    void _reconfigure(int frequency_hz, double duty_cycle_percent, bool start = false);
};

void GPIO::PWM::Impl::_reconfigure(int frequency_hz, double duty_cycle_percent, bool start){

    if (duty_cycle_percent < 0.0 || duty_cycle_percent > 100.0)
            throw std::runtime_error("invalid duty_cycle_percent");
    bool restart = start || _started;

    if (_started){
        _started = false;
        _disable_pwm(_ch_info);
    }

    _frequency_hz = frequency_hz;
    _period_ns = int(1000000000.0 / frequency_hz);
    _set_pwm_period(_ch_info, _period_ns);

    _duty_cycle_percent = duty_cycle_percent;
    _duty_cycle_ns = int(_period_ns * (duty_cycle_percent / 100.0));
    _set_pwm_duty_cycle(_ch_info, _duty_cycle_ns);

    if (restart){
        _enable_pwm(_ch_info);
        _started = true;
    }

}


GPIO::PWM::PWM(int channel, int frequency_hz)
    : pImpl( std::make_unique<Impl>(Impl{_channel_to_info(std::to_string(channel), false, true), 
            false, 0, 0, 0.0, 0}))  //temporary values
{
    try{
        Directions app_cfg = _app_channel_configuration(pImpl->_ch_info);
        if(app_cfg == HARD_PWM)
            throw std::runtime_error("Can't create duplicate PWM objects");
        /*
        Apps typically set up channels as GPIO before making them be PWM,
        because RPi.GPIO does soft-PWM. We must undo the GPIO export to
        allow HW PWM to run on the pin.
        */
        if(app_cfg == IN || app_cfg == OUT) {cleanup(channel);}

        if (GlobalVariables._gpio_warnings){
            auto sysfs_cfg = _sysfs_channel_configuration(pImpl->_ch_info);
            app_cfg = _app_channel_configuration(pImpl->_ch_info);

            // warn if channel has been setup external to current program
            if (app_cfg == UNKNOWN && sysfs_cfg != UNKNOWN){
                std::cerr <<  "[WARNING] This channel is already in use, continuing anyway. "
                            "Use GPIO::setwarnings(false) to disable warnings" << std::endl;
            }
        }

        _export_pwm(pImpl->_ch_info);
        pImpl->_reconfigure(frequency_hz, 50.0);
        GlobalVariables._channel_configuration[std::to_string(channel)] = HARD_PWM;

    }
    catch (std::exception& e){
        std::cerr << "[Exception] " << e.what() << " (catched from: PWM::PWM())" << std::endl;
        _cleanup_all();
        std::terminate();
    }

}

GPIO::PWM::~PWM(){
    auto itr = GlobalVariables._channel_configuration.find(pImpl->_ch_info.channel);
    if (itr == GlobalVariables._channel_configuration.end() || itr->second != HARD_PWM){
        /* The user probably ran cleanup() on the channel already, so avoid
           attempts to repeat the cleanup operations. */
       return;
    }
    try{
        stop();
        _unexport_pwm(pImpl->_ch_info);
        GlobalVariables._channel_configuration.erase(pImpl->_ch_info.channel);
        }
    catch(...){
        std::cerr << "[Exception] ~PWM Exception! shut down the program." << std::endl;
        _cleanup_all();
        std::terminate();
    }
}

void GPIO::PWM::start(double  duty_cycle_percent){
    try{
        pImpl->_reconfigure(pImpl->_frequency_hz, duty_cycle_percent, true);
    }
    catch(std::exception& e){
        std::cerr << "[Exception] " << e.what() << " (catched from: PWM::start())" << std::endl;
        _cleanup_all();
        std::terminate();
    }
}

void GPIO::PWM::ChangeFrequency(int frequency_hz){
    try{
        pImpl->_reconfigure(frequency_hz, pImpl->_duty_cycle_percent);
    }
    catch(std::exception& e){
        std::cerr << "[Exception] " << e.what() << " (catched from: PWM::ChangeFrequency())" << std::endl;
        std::terminate();
    }
}

void GPIO::PWM::ChangeDutyCycle(double duty_cycle_percent){
    try{
        pImpl->_reconfigure(pImpl->_frequency_hz, duty_cycle_percent);
    }
    catch(std::exception& e){
        std::cerr << "[Exception] " << e.what() << " (catched from: PWM::ChangeDutyCycle())" << std::endl;
        std::terminate();
    }
}

void GPIO::PWM::stop(){
    try{
        if (!pImpl->_started)
            return;
        
        _disable_pwm(pImpl->_ch_info);
    }
    catch(std::exception& e){
        std::cerr << "[Exception] " << e.what() << " (catched from: PWM::stop())" << std::endl;
        throw std::runtime_error("Exeception from GPIO::PWM::stop");
    }
}

//====================================================================


//=========================== Originally added ===========================
struct _cleaner{
private:
    _cleaner() = default;

public:
    _cleaner(const _cleaner&) = delete;
    _cleaner& operator=(const _cleaner&) = delete;

    static _cleaner& get_instance(){
        static _cleaner singleton;
        return singleton;
    }

    ~_cleaner(){
        try{  
            // When the user forgot to call cleanup() at the end of the program,
            // _cleaner object will call it.
            _cleanup_all(); 
        }
        catch(std::exception& e){
            std::cerr << "Exception: " << e.what() << std::endl;
            std::cerr << "Exception from destructor of _cleaner class." << std::endl;
        }
    }
};

// AutoCleaner will be destructed at the end of the program, and call _cleanup_all().
// It COULD cause a problem because at the end of the program,
// GlobalVariables._channel_configuration and GlobalVariables._gpio_mode MUST NOT be destructed before AutoCleaner.
// But the static objects are destructed in the reverse order of construction,
// and objects defined in the same compilation unit will be constructed in the order of definition.
// So it's supposed to work properly. 
_cleaner& AutoCleaner = _cleaner::get_instance(); 

//=========================================================================
