#include <cstdint>
#include "hardware/adc.h"
#include "dcc_adc.h"

#ifdef INCLUDE_ADC_LOG
#include <cstring>
#include <cstdio>
#endif


DccAdc::DccAdc(int gpio) :
    _gpio(gpio),
    _err_cnt(0)
{
    if (_gpio < 0)
        return;

    adc_init();
    adc_gpio_init(_gpio);            // e.g. 26
    adc_select_input(_gpio - 26);    // e.g. 0; rp2040 GPIO 26 is ADC 0
    adc_fifo_setup(true, false, 0, true, false); // err_in_fifo true
    adc_set_clkdiv(clock_rate / sample_rate - 1);
    log_reset();
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


void DccAdc::loop()
{
    if (_gpio < 0)
        return;

    // We get readings in the adc fifo at sample_rate

    if (adc_fifo_is_empty())
        return;

    uint16_t adc_val = adc_fifo_get();

    if (adc_val & 0x8000)
        _err_cnt++;

    adc_val &= 0x0fff;

#ifdef INCLUDE_ADC_LOG
    if (_log_idx < log_max)
        _log[_log_idx++] = adc_val;
#endif

    _avg[_avg_idx] = adc_val;
    _avg_idx++;
    if (_avg_idx >= avg_max)
        _avg_idx = 0;
}


uint16_t DccAdc::short_ma() const
{
    uint16_t raw = short_raw();
    uint16_t mv = raw_to_mv(raw);
    return mv_to_ma(mv);
}


uint16_t DccAdc::long_ma() const
{
    uint16_t raw = long_raw();
    uint16_t mv = raw_to_mv(raw);
    return mv_to_ma(mv);
}


void DccAdc::log_reset()
{
#ifdef INCLUDE_ADC_LOG
    _log_idx = 0;
    memset(_log, 0, sizeof(_log));
#endif
}


void DccAdc::log_show()
{
#ifdef INCLUDE_ADC_LOG
    printf("\n");
    printf("adc log: %d entries\n", _log_idx);
    printf("\n");
    printf("err_cnt = %d\n", _err_cnt);
    printf("\n");
    printf(" idx  raw\n");
  //               ---- ----
    for (int i = 0; i < _log_idx; i++)
        printf("%4d %4u\n", i, _log[i]);
    printf("\n");
#endif
}


uint16_t DccAdc::avg_raw(int cnt) const
{
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