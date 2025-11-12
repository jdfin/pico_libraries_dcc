#include "dcc_adc.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "hardware/adc.h"
#include "xassert.h"


DccAdc::DccAdc(int gpio) :
    _gpio(gpio),
    _err_cnt(0),
    _log_max(0),
    _log_idx(0),
    _log(nullptr),
    _dbg_loop_gpio(-1)
{
    if (_gpio < 0)
        return;

    adc_init();
    adc_gpio_init(_gpio);         // e.g. 26
    adc_select_input(_gpio - 26); // e.g. 0; rp2040 GPIO 26 is ADC 0
    adc_fifo_setup(true, false, 0, true, false); // err_in_fifo true
    adc_set_clkdiv(clock_rate / sample_rate - 1);
}


DccAdc::~DccAdc()
{
    stop();
}


void DccAdc::start()
{
    if (_gpio < 0)
        return;

    adc_run(true);
}


void DccAdc::stop()
{
    if (_gpio < 0)
        return;

    adc_run(false);
}


// When the ADC FIFO is empty, this function takes about 250 ns; when there is
// data, about 600 ns (by one particular measurement). Note that at 10 KHz, a
// new samples is available every 100 usec. With the rp2040 4-sample fifo,
// that means this must be called at least every 400 usec. Calling it once per
// DCC bit time should be fine (zeros are 200 usec). Sometimes one call will
// get two samples, so make sure that works.
// Return true if there was at least one sample, false if none.
bool DccAdc::loop()
{
    DbgGpio d(_dbg_loop_gpio);

    if (_gpio < 0)
        return false;

    bool any = false;

    while (!adc_fifo_is_empty()) {

        any = true;

        uint16_t adc_val = adc_fifo_get();

        if (adc_val & 0x8000)
            _err_cnt++;

        adc_val &= 0x0fff;

        if (logging() && _log_idx < _log_max)
            _log[_log_idx++] = adc_val;

        _avg[_avg_idx] = adc_val;
        _avg_idx++;
        if (_avg_idx >= avg_max)
            _avg_idx = 0;
    }

    return any;
}


uint16_t DccAdc::short_avg_ma() const
{
    uint16_t raw = short_avg_raw();
    uint16_t mv = raw_to_mv(raw);
    return mv_to_ma(mv);
}


uint16_t DccAdc::long_avg_ma() const
{
    uint16_t raw = long_avg_raw();
    uint16_t mv = raw_to_mv(raw);
    return mv_to_ma(mv);
}


void DccAdc::log_init(int samples)
{
    if (_log != nullptr) {
        delete[] _log;
        _log = nullptr;
        _log_max = 0;
        _log_idx = 0;
    }
    if (samples > 0) {
        _log = new uint16_t[samples];
        xassert(_log != nullptr);
        _log_max = samples;
        _log_idx = 0;
    }
}


void DccAdc::log_reset()
{
    if (logging()) {
        memset(_log, 0, _log_max * sizeof(_log[0]));
        _log_idx = 0;
    }
}


void DccAdc::log_show() const
{
    if (logging()) {
        printf("\n");
        printf("adc log: %d entries\n", _log_idx);
        printf("\n");
        printf("err_cnt = %d\n", _err_cnt);
        printf("\n");
        printf(" idx  raw\n");
        //      ---- ----
        for (int i = 0; i < _log_idx; i++) printf("%4d %4u\n", i, _log[i]);
        printf("\n");
    }
}


uint16_t DccAdc::avg_raw(int cnt) const
{
    // XXX lock _avg[]
    uint32_t sum = 0;
    int i = _avg_idx;
    for (int j = 0; j < cnt; j++) {
        i--;
        if (i < 0)
            i = avg_max - 1;
        sum += _avg[i];
    }
    return (sum + cnt / 2) / cnt;
}
