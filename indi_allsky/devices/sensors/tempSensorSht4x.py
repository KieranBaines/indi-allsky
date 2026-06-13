import time
import logging

from .sensorBase import SensorBase
from ... import constants
from ..exceptions import SensorReadException
from ..exceptions import DeviceControlException


logger = logging.getLogger('indi_allsky')


# Sensirion SHT4x heater command table.
# Source: Sensirion Application Note "Using the Integrated Heater of SHT4x in
# High-Humidity Environments" (v1, April 2022), Table 1.
# avg_dt_c = typical average temperature increase for the described test setup;
# treat as a starting point, not a guarantee (depends on thermal coupling).
SHT4X_HEATER_COMMANDS = {
    # adafruit Mode name : (power_mW, duration_s, avg_dt_c)
    'HIGHHEAT_1S'   : (200, 1.0, 50),   # 0x39
    'HIGHHEAT_100MS': (200, 0.1, 29),   # 0x32
    'MEDHEAT_1S'    : (110, 1.0, 31),   # 0x2F
    'MEDHEAT_100MS' : (110, 0.1, 16),   # 0x24
    'LOWHEAT_1S'    : (20,  1.0, 6),    # 0x1E
    'LOWHEAT_100MS' : (20,  0.1, 3),    # 0x15
}

NOHEAT_MODE = 'NOHEAT_HIGHPRECISION'


class TempSensorSht4x(SensorBase):

    def update(self):
        # handle day/night precision (no-heat) mode change
        if self.night != bool(self.night_av[constants.NIGHT_NIGHT]):
            self.night = bool(self.night_av[constants.NIGHT_NIGHT])
            self.update_sensor_settings()


        # run the heater state machine before reading. it decides whether a
        # heat pulse is due, performs it, and signals whether the RH reading
        # that follows is trustworthy (i.e. the sensor has equilibrated).
        rh_trustworthy = self._service_heater()


        try:
            temp_c, rel_h = self.sht4x.measurements
            temp_c = float(temp_c)
            rel_h = float(rel_h)
        except RuntimeError as e:
            raise SensorReadException(str(e)) from e


        if not rh_trustworthy:
            # sensor is still equilibrating after a heat pulse. RH (and temp)
            # are corrupted by residual heat. fall back to the last good RH so
            # we don't publish a creep-corrupted-the-other-way value.
            logger.info('[%s] SHT4x - within equilibration window, reusing last RH', self.name)
            if self._last_good_rh is not None:
                rel_h = self._last_good_rh
        else:
            self._last_good_rh = rel_h


        logger.info('[%s] SHT4x - temp: %0.1fc, humidity: %0.1f%% (heater_on=%s)',
                    self.name, temp_c, rel_h, self.heater_on)


        try:
            dew_point_c = self.get_dew_point_c(temp_c, rel_h)
            frost_point_c = self.get_frost_point_c(temp_c, dew_point_c)
        except ValueError as e:
            logger.error('Dew Point calculation error - ValueError: %s', str(e))
            dew_point_c = 0.0
            frost_point_c = 0.0


        heat_index_c = self.get_heat_index_c(temp_c, rel_h)


        if self.config.get('TEMP_DISPLAY') == 'f':
            current_temp = self.c2f(temp_c)
            current_dp = self.c2f(dew_point_c)
            current_fp = self.c2f(frost_point_c)
            current_hi = self.c2f(heat_index_c)
        elif self.config.get('TEMP_DISPLAY') == 'k':
            current_temp = self.c2k(temp_c)
            current_dp = self.c2k(dew_point_c)
            current_fp = self.c2k(frost_point_c)
            current_hi = self.c2k(heat_index_c)
        else:
            current_temp = temp_c
            current_dp = dew_point_c
            current_fp = frost_point_c
            current_hi = heat_index_c


        data = {
            'dew_point' : current_dp,
            'frost_point' : current_fp,
            'heat_index' : current_hi,
            'data' : (
                current_temp,
                rel_h,
                current_dp,
            ),
        }

        return data


    def _service_heater(self):
        """
        Heater state machine. Returns True if the RH reading taken immediately
        after this call can be trusted, False if the sensor is still
        equilibrating from a recent heat pulse.

        Honors the manufacturer constraints:
          - max 5% duty cycle (configurable, capped)
          - equilibration wait after a pulse before reading RH
          - heating only when the configured policy calls for it
        """
        if not self.heater_available:
            return True

        now = time.time()

        # --- are we currently equilibrating from a prior pulse? ---
        if self._equilibrate_until and now < self._equilibrate_until:
            self.heater_on = True
            return False

        if self._equilibrate_until and now >= self._equilibrate_until:
            # equilibration just finished
            self._equilibrate_until = 0.0
            self.heater_on = False

        # --- decide whether a new heat cycle is due ---
        if self.heater_mode == 'OFF':
            return True

        # interval gate: never heat more often than the configured interval,
        # which also keeps us under the duty-cycle cap.
        if self._next_heat_allowed and now < self._next_heat_allowed:
            return True

        want_heat = False

        if self.heater_mode in ('CONTINUOUS', 'SINGLE_SHOT'):
            want_heat = True
        elif self.heater_mode == 'THRESHOLD':
            # read RH cheaply to evaluate the threshold without committing to
            # a heat cycle. uses hysteresis from the base class / config.
            try:
                _, rel_h = self.sht4x.measurements
                rel_h = float(rel_h)
            except RuntimeError:
                return True

            if not self.heater_engaged and rel_h >= self.rh_heater_on_level:
                self.heater_engaged = True
            elif self.heater_engaged and rel_h <= self.rh_heater_off_level:
                self.heater_engaged = False

            want_heat = self.heater_engaged

        if not want_heat:
            return True

        # --- perform the heat pulse(s) ---
        return self._do_heat_pulse(now)


    def _do_heat_pulse(self, now):
        import adafruit_sht4x

        power_mw, duration_s, _ = SHT4X_HEATER_COMMANDS.get(
            self.heater_command, SHT4X_HEATER_COMMANDS['LOWHEAT_100MS'])

        # number of consecutive pulses (single-shot uses several; continuous
        # uses one). adafruit's measurements property fires the pulse + reads.
        pulses = self.heater_pulses if self.heater_mode == 'SINGLE_SHOT' else 1

        logger.warning('[%s] SHT4x heater pulse: mode=%s cmd=%s %dmW %0.1fs x%d',
                       self.name, self.heater_mode, self.heater_command,
                       power_mw, duration_s, pulses)

        try:
            heat_mode = getattr(adafruit_sht4x.Mode, self.heater_command)
            self.sht4x.mode = heat_mode

            for _ in range(pulses):
                # reading in a heat mode triggers the heater then measures
                self.sht4x.measurements
                time.sleep(duration_s + 0.05)

            # return to no-heat high precision for normal reads
            self.sht4x.mode = getattr(adafruit_sht4x.Mode, NOHEAT_MODE)
        except Exception as e:
            logger.error('[%s] SHT4x heater error: %s', self.name, str(e))
            self.heater_on = False
            return True

        # schedule equilibration window and next allowed heat, enforcing the
        # configured interval and the duty-cycle cap.
        total_heat_s = duration_s * pulses
        equilibration_s = self.heater_equilibration_s

        # duty-cycle enforcement: heat_time / cycle_time <= max_duty
        min_cycle_for_duty = total_heat_s / max(self.heater_max_duty, 0.001)
        cycle_s = max(self.heater_interval_s, min_cycle_for_duty, equilibration_s + total_heat_s)

        self._equilibrate_until = now + equilibration_s
        self._next_heat_allowed = now + cycle_s
        self.heater_on = True

        logger.info('[%s] SHT4x heater: equilibrating %0.0fs, next heat in %0.0fs (duty<=%0.1f%%)',
                    self.name, equilibration_s, cycle_s, self.heater_max_duty * 100)

        # RH is not trustworthy until equilibration completes
        return False


    def update_sensor_settings(self):
        if self.night:
            logger.info('[%s] Switching SHT4X to night mode - Mode %s', self.name, hex(self.mode_night))
            self.sht4x.mode = self.mode_night
        else:
            logger.info('[%s] Switching SHT4X to day mode - Mode %s', self.name, hex(self.mode_day))
            self.sht4x.mode = self.mode_day

        time.sleep(1.0)


class TempSensorSht4x_I2C(TempSensorSht4x):

    METADATA = {
        'name' : 'SHT4x (i2c)',
        'description' : 'SHT4x i2c Temperature Sensor',
        'count' : 3,
        'labels' : (
            'Temperature',
            'Relative Humidity',
            'Dew Point',
        ),
        'types' : (
            constants.SENSOR_TEMPERATURE,
            constants.SENSOR_RELATIVE_HUMIDITY,
            constants.SENSOR_TEMPERATURE,
        ),
    }


    def __init__(self, *args, **kwargs):
        super(TempSensorSht4x_I2C, self).__init__(*args, **kwargs)

        i2c_address_str = kwargs['i2c_address']

        import board
        #import busio
        import adafruit_sht4x

        i2c_address = int(i2c_address_str, 16)  # string in config

        logger.warning('Initializing [%s] SHT4x I2C temperature device @ %s', self.name, hex(i2c_address))

        try:
            i2c = board.I2C()
            self.sht4x = adafruit_sht4x.SHT4x(i2c, address=i2c_address)
        except Exception as e:
            logger.error('Device init exception: %s', str(e))
            raise DeviceControlException from e


        temp_sensor_config = self.config.get('TEMP_SENSOR', {})

        # day/night no-heat precision modes (unchanged behavior)
        self.mode_night = getattr(adafruit_sht4x.Mode, temp_sensor_config.get('SHT4X_MODE_NIGHT', NOHEAT_MODE))
        self.mode_day = getattr(adafruit_sht4x.Mode, temp_sensor_config.get('SHT4X_MODE_DAY', NOHEAT_MODE))

        # --- heater control configuration ---
        heater_enable = bool(temp_sensor_config.get('SHT4X_HEATER_ENABLE', False))
        self.heater_available = heater_enable

        # OFF / CONTINUOUS / SINGLE_SHOT / THRESHOLD
        self.heater_mode = temp_sensor_config.get('SHT4X_HEATER_MODE', 'OFF')

        # which heater command (power/duration) from the Sensirion table
        self.heater_command = temp_sensor_config.get('SHT4X_HEATER_COMMAND', 'LOWHEAT_100MS')
        if self.heater_command not in SHT4X_HEATER_COMMANDS:
            logger.error('[%s] Unknown SHT4x heater command %s, using LOWHEAT_100MS',
                         self.name, self.heater_command)
            self.heater_command = 'LOWHEAT_100MS'

        # minimum seconds between heat cycles (Sensirion continuous example: 60)
        self.heater_interval_s = float(temp_sensor_config.get('SHT4X_HEATER_INTERVAL_S', 60.0))

        # equilibration wait after a pulse before RH is trustworthy
        # (Sensirion continuous example: 60s; single-shot example: 120s)
        self.heater_equilibration_s = float(temp_sensor_config.get('SHT4X_HEATER_EQUILIBRATION_S', 60.0))

        # consecutive pulses for single-shot mode
        self.heater_pulses = int(temp_sensor_config.get('SHT4X_HEATER_PULSES', 1))

        # hard duty-cycle safety cap (datasheet max 5%)
        self.heater_max_duty = min(float(temp_sensor_config.get('SHT4X_HEATER_MAX_DUTY', 0.05)), 0.05)

        # RH thresholds for THRESHOLD mode (override base-class defaults)
        self.rh_heater_on_level = float(temp_sensor_config.get('SHT4X_HEATER_RH_ON', self.rh_heater_on_level))
        self.rh_heater_off_level = float(temp_sensor_config.get('SHT4X_HEATER_RH_OFF', self.rh_heater_off_level))

        # state machine internals
        self._next_heat_allowed = 0.0
        self._equilibrate_until = 0.0
        self._last_good_rh = None
        self.heater_engaged = False  # for THRESHOLD hysteresis

        if heater_enable:
            logger.warning('[%s] SHT4x heater enabled: mode=%s cmd=%s interval=%0.0fs equil=%0.0fs maxduty=%0.1f%%',
                           self.name, self.heater_mode, self.heater_command,
                           self.heater_interval_s, self.heater_equilibration_s,
                           self.heater_max_duty * 100)
