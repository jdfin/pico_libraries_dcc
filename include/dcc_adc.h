#pragma once

#include <cstdint>

#undef INCLUDE_ADC_LOG

class DccAdc
{
public:
    DccAdc(int gpio);
    ~DccAdc();

    void start();
    void stop();

    void loop();

    uint16_t short_ma() const;
    uint16_t long_ma() const;

    static constexpr bool logging()
    {
#ifdef INCLUDE_ADC_LOG
        return true;
#else
        return false;
#endif
    }

    void log_reset();
    void log_show();

private:
    int _gpio;

    uint16_t avg_raw(int cnt) const;

    uint16_t short_raw() const { return avg_raw(short_cnt); }

    uint16_t long_raw() const { return avg_raw(long_cnt); }

    static uint16_t raw_to_mv(uint16_t raw)
    {
        // With 12 bits, 3.3V ref, mv = (raw / 4096) * 3300 = raw * 0.80
        // Basically, [0...4096] -> [0...3300]
        const uint32_t ref_mv = 3300;
        const uint16_t raw_max = 4096;
        return (raw * ref_mv + raw_max / 2) / raw_max;
    }

    static uint16_t mv_to_ma(uint16_t mv)
    {
        // Pololu DRV8874: 1.1 mv/ma
        //                 0.9091 ma/mv
        // 1.1 = 8192 / 7447
        const uint32_t mul = 8192 / 1.1;  // 7447
        const uint32_t div = 8192;
        // ma = mv * 7447 / 8192 = mv / 1.10 = raw * 0.73
        return (mv * mul + div / 2) / div;
    }

    static const uint32_t clock_rate = 48000000;
    static const uint32_t sample_rate = 10000;  // 10 KHz = 100 usec per sample

    static const int avg_max =
        sample_rate / 60;  // 1 cycle of 60 Hz noise (166 for 10 KHz)
    uint16_t _avg[avg_max];
    int _avg_idx;

    static const int short_cnt = 16;

    static const int long_cnt = avg_max;

    int _err_cnt;

#ifdef INCLUDE_ADC_LOG
    static const int log_max = 1 * sample_rate;  // 1 sec
    uint16_t _log[log_max];
    int _log_idx;
#endif

};  // class DccAdc
