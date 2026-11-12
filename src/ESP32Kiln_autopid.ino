
#include <PID_v1.h>
#include <PID_AutoTune_v0.h> // https://github.com/t0mpr1c3/Arduino-PID-AutoTune-Library

#define DEFAULT_TEMP_RISE_AFTER_OFF 30.0
#define CONTROL_HYSTERISIS .01

double _CALIBRATE_max_temperature;
int measureInterval = 500;
int tuner_id = 5;
double tuner_noise_band = 1;
double tuner_output_step;

float _calP = .5 / DEFAULT_TEMP_RISE_AFTER_OFF;
float _calI = 4 / DEFAULT_TEMP_RISE_AFTER_OFF;
float _calD = 5.0 / DEFAULT_TEMP_RISE_AFTER_OFF;

PID_ATune aTune(&kiln_temp, &pid_out);

// Attempt to start PID auto-calibration.
// Returns true if calibration started, false if blocked (e.g. a program is already running).
bool CalibrateInit()
{
    // Guard: do not hijack an active program or an already-running calibration
    if (Program_run_state == PR_RUNNING || Program_run_state == PR_PAUSED ||
        Program_run_state == PR_THRESHOLD || Program_run_state == PR_CALIBRATE) {
        DBG dbgLog(LOG_ERR, "[PID] CalibrateInit BLOCKED - system is in state %d (%s)",
                   (int)Program_run_state, Prog_Run_Names[(int)Program_run_state]);
        return false;
    }

    Program_run_state = PR_CALIBRATE;
    measureInterval = Prefs[PRF_PID_MEASURE_INTERVAL].value.uint16;
    tuner_id = Prefs[PRF_PID_ALGORITHM].value.uint8;
    set_temp = Prefs[PRF_PID_CAL_TEMP].value.uint16;
    tuner_output_step = Prefs[PRF_PID_WINDOW].value.uint16 / (2.0 * PID_WINDOW_DIVIDER);
    _CALIBRATE_max_temperature = 0;
    Enable_EMR();
    aTune.Cancel();                         // just in case
    aTune.SetNoiseBand(tuner_noise_band);   // noise band +-1*C
    aTune.SetOutputStep(tuner_output_step); // half of PID window range
    aTune.SetControlType(tuner_id);
    aTune.SetLookbackSec(30);              // 30 seconds lookback for oscillation detection
    windowStartTime = millis();
    aTune.Runtime(); // initialize autotuner here, as later we give it actual readings

    DBG dbgLog(LOG_INFO, "[PID] AutoTune STARTED - target %.0f°C, output_step %.0f, window %d ms, noise_band %.1f°C",
               set_temp, tuner_output_step, Prefs[PRF_PID_WINDOW].value.uint16, tuner_noise_band);

    // Open CSV log directly -- Init_log_file() would crash because
    // Program_run_name/desc are NULL when no program is loaded
    if (Prefs[PRF_LOG_WINDOW].value.uint16) {
        char str[33];
        struct tm timeinfo;
        if(CSVFile) CSVFile.close();
        if(getLocalTime(&timeinfo)) strftime(str, 32, "/logs/%y%m%d_%H%M%S.csv", &timeinfo);
        else sprintf(str, "/logs/%d.csv", millis());
        if(CSVFile = SPIFFS.open(str, "w")){
#ifdef ENERGY_MON_PIN
            CSVFile.print(String("Date,Temperature,Housing,Energy"));
#else
            CSVFile.print(String("Date,Temperature,Housing"));
#endif
        }
        Add_log_line();
    }
    return true;
}

void HandleCalibration(unsigned long now)
{
    static uint16_t cal_log_cnt = 0;
    static uint16_t cal_progress_cnt = 0;

    if (aTune.Runtime())
    {
        Disable_EMR();
        Disable_SSR();
        _calP = aTune.GetKp();
        _calI = aTune.GetKi();
        _calD = aTune.GetKd();

        DBG dbgLog(LOG_INFO, "[PID] AutoTune FINISHED - raw results: Kp=%.4f  Ki=%.4f  Kd=%.4f", _calP, _calI, _calD);

        // Sanity check: if Kp is zero or negative, the tuner likely failed
        if (_calP <= 0.0 || _calI < 0.0 || _calD < 0.0) {
            DBG dbgLog(LOG_ERR, "[PID] Calibration FAILED - invalid results: PID = [%f, %f, %f]", _calP, _calI, _calD);
            Program_run_state = PR_ABORTED;
        } else {
            // Cap Kd for slow thermal processes -- relay tuning produces Kd proportional
            // to oscillation period, which is huge for kilns.  Kd > Kp causes the derivative
            // term to overpower proportional/integral and stall heating.
            double maxKd = _calP * 0.5;
            if (_calD > maxKd) {
                DBG dbgLog(LOG_INFO, "[PID] Capping Kd from %.4f to %.4f (0.5 * Kp)", _calD, maxKd);
                _calD = maxKd;
            }
            DBG dbgLog(LOG_INFO, "[PID] Calibration ACCEPTED: Kp=%.4f  Ki=%.4f  Kd=%.4f  (max temp reached: %.1f°C)",
                       _calP, _calI, _calD, _CALIBRATE_max_temperature);
            Prefs[PRF_PID_KP].value.vfloat = _calP;
            Prefs[PRF_PID_KI].value.vfloat = _calI;
            Prefs[PRF_PID_KD].value.vfloat = _calD;
            KilnPID.SetTunings(_calP, _calI, _calD);
            Save_prefs();
            Program_run_state = PR_NONE;
        }
        cal_log_cnt = 0;
        cal_progress_cnt = 0;
        Close_log_file();
        return;
    }

    // Periodic progress logging (every ~30 seconds)
    cal_progress_cnt++;
    if (cal_progress_cnt >= 30) {
        cal_progress_cnt = 0;
        DBG dbgLog(LOG_INFO, "[PID] AutoTune in progress - kiln_temp=%.1f°C  set_temp=%.0f°C  pid_out=%.1f  max_temp=%.1f°C",
                   kiln_temp, set_temp, pid_out, _CALIBRATE_max_temperature);
    }

    if (Prefs[PRF_LOG_WINDOW].value.uint16) {
        cal_log_cnt++;
        if (cal_log_cnt > Prefs[PRF_LOG_WINDOW].value.uint16) {
            cal_log_cnt = 1;
            Add_log_line();
        }
    }

    handle_pid(now);
    _CALIBRATE_max_temperature = max(_CALIBRATE_max_temperature, kiln_temp);
}

void CalibrateAbort()
{
    aTune.Cancel();
    Program_run_state = PR_ABORTED;
    Disable_EMR();
    Disable_SSR();
    Close_log_file();
    DBG dbgLog(LOG_INFO, "[PID] Calibration aborted by user");
}

void handle_pid(unsigned long now)
{
    uint16_t pidWindow = Prefs[PRF_PID_WINDOW].value.uint16;
    if (now - windowStartTime > pidWindow) {
        windowStartTime += pidWindow;
    }
    // SSR on when pid_out (scaled by PID_WINDOW_DIVIDER) exceeds elapsed time in current window
    bool heater = (pid_out * PID_WINDOW_DIVIDER > now - windowStartTime);
    heater ? Enable_SSR() : Disable_SSR();
}
