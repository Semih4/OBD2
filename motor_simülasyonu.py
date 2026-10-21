"""
Dizel ECU Simülatörü - Fiziksel Model V1
Python standart kütüphanesi yeterlidir.

Çalıştırma:
    python ecu_dizel_v1.py
    python ecu_dizel_v1.py --test

Not:
Bu, parametreleri açık bir yaklaşık mühendislik modelidir; üretici ECU yazılımı
veya gerçek motor dinamometresiyle doğrulanmış bir motor modeli değildir.
"""
import argparse
import math
import os
import random
import sys
import time


# terminal renkleri oluşturma
RESET = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"
KIRMIZI = "\033[91m"
YESIL = "\033[92m"
SARI = "\033[93m"
MAVI = "\033[94m"
CYAN = "\033[96m"
BEYAZ = "\033[97m"
EKRANI_TEMIZLE = "\033[2J"
IMLEC_BASA = "\033[H"
IMLEC_GIZLE = "\033[?25l"
IMLEC_GOSTER = "\033[?25h"


def renklendir(metin: str, *kodlar: str) -> str:
    return "".join(kodlar) + metin + RESET


def clamp(value, low, high):
    return max(low, min(high, value))


class DieselVehicleModel:
    """
    Basitleştirilmiş 2.0 L turbo dizel + manuel oranlı otomatik şanzıman + araç.

    Birimler:
      devir: rpm, tork: N.m, hız: m/s (dış arayüzde km/h),
      kuvvet: N, kütle: kg, sıcaklık: °C, yakıt debisi: L/s.

    Tork eğrisi ve araç parametreleri örnek kalibrasyon değerleridir.
    Belirli bir araç markası/modeli için fabrika verisi olarak değerlendirilmemeli.
    """
    # başlangıç durumunu tanımlama 
    IDLE_RPM = 800.0
    MAX_RPM = 4500.0

    # (rpm, tam-yük torku N.m) - temsili 2.0 L turbo dizel eğrisi
    TORQUE_CURVE = (
        (800, 160), (1000, 220), (1500, 280), (2000, 300),
        (2500, 290), (3000, 260), (3500, 220), (4000, 170),
        (4300, 100), (4500, 0),
    )

    # Vites oranları 1-6; diferansiyel ayrı tanımlıdır.
    GEAR_RATIOS = [0.0, 3.60, 2.10, 1.40, 1.00, 0.78, 0.65]
    FINAL_DRIVE = 3.70
    DRIVETRAIN_EFFICIENCY = 0.90

    VEHICLE_MASS_KG = 1550.0
    WHEEL_RADIUS_M = 0.31
    CD = 0.30
    FRONTAL_AREA_M2 = 2.20
    AIR_DENSITY_KG_M3 = 1.225
    ROLLING_RESISTANCE = 0.012
    GRAVITY = 9.80665

    FUEL_TANK_L = 55.0
    DIESEL_DENSITY_KG_L = 0.835
    DIESEL_STOICH_AFR = 14.5  # Yaklaşık kimyasal referans; dizel lambda değişkendir.

    def __init__(self, seed=7):
        self.rng = random.Random(seed)
        self.rpm = self.IDLE_RPM
        self.speed_mps = 0.0
        self.throttle_pct = 0.0
        self.gear = 1
        self.engine_torque_nm = 0.0
        self.available_torque_nm = self.torque_at(self.rpm)
        self.load_pct = 0.0
        self.maf_g_s = 3.0
        self.fuel_level_pct = 80.0
        self.fuel_flow_l_s = 0.5 / 3600.0
        self.coolant_c = 20.0
        self.intake_c = 20.0
        self.battery_v = 14.1
        self.ambient_c = 20.0
        self.road_grade = 0.0  # eğim: 0.05 yaklaşık %5 yokuş
        self._target_throttle = 0.0
        self._next_throttle_change = 0.0
        self._shift_lock_s = 0.0
        self._elapsed_s = 0.0

    @property
    def speed_kmh(self):
        return self.speed_mps * 3.6

    @property
    def throttle(self):
        return self.throttle_pct

    @property
    def coolant(self):
        return self.coolant_c

    @property
    def intake_temp(self):
        return self.intake_c

    @property
    def fuel_level(self):
        return self.fuel_level_pct

    @property
    def battery(self):
        return self.battery_v

    @property
    def maf(self):
        return self.maf_g_s

    @property
    def load(self):
        return self.load_pct

    def torque_at(self, rpm):
        """Tam-yük tork eğrisini doğrusal ara değerleme ile hesapla."""
        rpm = clamp(rpm, self.TORQUE_CURVE[0][0], self.TORQUE_CURVE[-1][0])
        for i in range(1, len(self.TORQUE_CURVE)):
            r0, t0 = self.TORQUE_CURVE[i - 1]
            r1, t1 = self.TORQUE_CURVE[i]
            if rpm <= r1:
                fraction = (rpm - r0) / (r1 - r0)
                return t0 + fraction * (t1 - t0)
        return self.TORQUE_CURVE[-1][1]

    def _driver_input(self, dt):
        """Terminal demosu için sürücü talebi; fiziksel modelden ayrı tutulur."""
        if self._elapsed_s >= self._next_throttle_change:
            self._target_throttle = max(
                0.0,
                self.rng.choice([0, 0, 5, 10, 15, 25, 40, 60, 80])
                + self.rng.uniform(-1.0, 1.0),
            )
            self._next_throttle_change = self._elapsed_s + self.rng.uniform(2.0, 5.0)
        # Gaz pedalı sinyali için basit bir aktüatör/filtre gecikmesi
        alpha = 1.0 - math.exp(-dt / 0.25)
        self.throttle_pct += (self._target_throttle - self.throttle_pct) * alpha
        self.throttle_pct = clamp(self.throttle_pct, 0.0, 100.0)

    def update(self, dt):
        """Modeli dt saniye ilerlet."""
        if not math.isfinite(dt) or dt <= 0:
            return
        # Büyük zaman sıçramaları integratörü bozmasın.
        remaining = min(dt, 0.25)
        while remaining > 1e-9:
            step = min(0.02, remaining)
            self._step(step)
            remaining -= step

    def _step(self, dt):
        self._elapsed_s += dt
        self._driver_input(dt)
        self._shift_lock_s = max(0.0, self._shift_lock_s - dt)

        # Şanzıman geçişi: devir eşiği + kilitleme süresi; dururken 1. vites.
        if self.speed_kmh < 2.0:
            self.gear = 1
        elif self._shift_lock_s <= 0.0:
            if self.rpm > 3200 and self.gear < 6:
                self.gear += 1
                self._shift_lock_s = 0.8
            elif self.rpm < 1350 and self.gear > 1 and self.throttle_pct < 45:
                self.gear -= 1
                self._shift_lock_s = 0.8

        # Motor torku: pedal isteği * çalışma noktasındaki tam-yük torku.
        self.available_torque_nm = max(0.0, self.torque_at(self.rpm))
        requested_fraction = self.throttle_pct / 100.0
        self.engine_torque_nm = self.available_torque_nm * requested_fraction

        # Kavrama yaklaşık modeli: kalkışta kısmi aktarım, hızlanınca tam aktarım.
        clutch = 0.25 + 0.75 * clamp(self.speed_kmh / 8.0, 0.0, 1.0)
        ratio = self.GEAR_RATIOS[self.gear] * self.FINAL_DRIVE
        wheel_force = (
            self.engine_torque_nm * ratio * self.DRIVETRAIN_EFFICIENCY
            * clutch / self.WHEEL_RADIUS_M
        )

        # Yol yükleri: aerodinamik sürükleme v², yuvarlanma ve eğim kuvveti.
        air_force = (
            0.5 * self.AIR_DENSITY_KG_M3 * self.CD * self.FRONTAL_AREA_M2
            * self.speed_mps * self.speed_mps
        )
        rolling_force = (
            self.ROLLING_RESISTANCE * self.VEHICLE_MASS_KG * self.GRAVITY
            if self.speed_mps > 0.01 else 0.0
        )
        grade_force = (
            self.VEHICLE_MASS_KG * self.GRAVITY
            * math.sin(math.atan(self.road_grade))
        )
        net_force = wheel_force - air_force - rolling_force - grade_force
        acceleration = net_force / self.VEHICLE_MASS_KG
        self.speed_mps = max(0.0, min(61.11, self.speed_mps + acceleration * dt))

        # Motor devri, tekerlek devri ve toplam aktarma oranıyla ilişkilidir.
        wheel_rpm = self.speed_mps / (2.0 * math.pi * self.WHEEL_RADIUS_M) * 60.0
        coupled_rpm = wheel_rpm * ratio
        # Kavrama kayması düşük hızda motoru rölantiye yakın tutar.
        target_rpm = max(self.IDLE_RPM, self.IDLE_RPM * (1.0 - clutch) + coupled_rpm * clutch)
        target_rpm = min(self.MAX_RPM, target_rpm)
        rpm_alpha = 1.0 - math.exp(-dt / 0.30)
        self.rpm += (target_rpm - self.rpm) * rpm_alpha
        self.rpm = clamp(self.rpm, self.IDLE_RPM - 30.0, self.MAX_RPM)

        # Tahmini motor yükü: istenen torkun mevcut tam-yük torkuna oranı.
        self.load_pct = clamp(
            100.0 * self.engine_torque_nm / max(1.0, self.available_torque_nm), 0.0, 100.0
        )

        # Yakıt: yaklaşık BSFC güç modeli + rölanti için taban tüketim.
        power_kw = max(0.0, self.engine_torque_nm * self.rpm * 2.0 * math.pi / 60.0 / 1000.0)
        bsfc_g_kwh = 230.0  # Temsili değer; gerçek BSFC devir/yükle değişir.
        fuel_g_s = power_kw * bsfc_g_kwh / 3600.0
        idle_l_s = 0.5 / 3600.0
        power_fuel_l_s = (fuel_g_s / 1000.0) / self.DIESEL_DENSITY_KG_L
        self.fuel_flow_l_s = idle_l_s + power_fuel_l_s
        self.fuel_level_pct = max(
            0.0,
            self.fuel_level_pct - (self.fuel_flow_l_s * dt / self.FUEL_TANK_L) * 100.0,
        )

        # Hava debisi: yakıt debisi ve fazla hava katsayısına dayalı yaklaşık model.
        # Yük arttıkça lambda azalır; hava debisi fiziksel olarak negatif olamaz.
        lambda_air = 3.0 - 1.2 * (self.load_pct / 100.0)
        fuel_g_s_actual = self.fuel_flow_l_s * self.DIESEL_DENSITY_KG_L * 1000.0
        self.maf_g_s = max(0.0, fuel_g_s_actual * self.DIESEL_STOICH_AFR * lambda_air)

        # Isıl model: uzun zaman sabiti ile ısınma; soğutma sistemi için hedef sıcaklık.
        target_coolant = 88.0 + 0.10 * self.load_pct
        thermal_tau_s = 240.0 if self.coolant_c < target_coolant else 420.0
        thermal_alpha = 1.0 - math.exp(-dt / thermal_tau_s)
        self.coolant_c += (target_coolant - self.coolant_c) * thermal_alpha

        # Emme havası: ortam + motor bölmesi/şarj havası için basit yaklaşık ilişki.
        target_intake = self.ambient_c + 0.12 * max(0.0, self.coolant_c - self.ambient_c)
        target_intake += 0.08 * self.load_pct
        intake_alpha = 1.0 - math.exp(-dt / 35.0)
        self.intake_c += (target_intake - self.intake_c) * intake_alpha

        # Şarj sistemi çalışırken nominal 14 V civarı; yük etkisi küçükçe modellenir.
        self.battery_v = 14.1 - 0.15 * (self.load_pct / 100.0)


# Terminal uyumluluğu için eski ECU alan adlarını koruyan adaptör.
class ECU(DieselVehicleModel):
    RPM_ROLANTI = DieselVehicleModel.IDLE_RPM
    RPM_MAKS = DieselVehicleModel.MAX_RPM

    @property
    def hiz(self):
        return self.speed_kmh

    @property
    def gaz(self):
        return self.throttle_pct

    @property
    def yuk(self):
        return self.load_pct

    @property
    def su_sicakligi(self):
        return self.coolant_c

    @property
    def emme_sicakligi(self):
        return self.intake_c

    @property
    def yakit(self):
        return self.fuel_level_pct

    @property
    def aku(self):
        return self.battery_v

    @property
    def maf(self):
        return self.maf_g_s

    @property
    def vites(self):
        return self.gear

    def guncelle(self, dt):
        self.update(dt)

    def obd_rpm_cercevesi(self):
        """Mode 01 PID 0C: RPM = (A*256 + B)/4; 8 baytlık klasik CAN yanıtı."""
        raw = int(clamp(round(self.rpm * 4), 0, 65535))
        a, b = raw >> 8, raw & 0xFF
        return bytes([0x04, 0x41, 0x0C, a, b, 0x55, 0x55, 0x55])


GENISLIK = 56


def cubuk(deger, min_d, max_d, renk=None, uzunluk=24):
    oran = clamp((deger - min_d) / (max_d - min_d), 0.0, 1.0)
    dolu = int(oran * uzunluk)
    if renk is None:
        renk = YESIL if oran < 0.7 else (SARI if oran < 0.9 else KIRMIZI)
    return renk + "█" * dolu + RESET + DIM + "░" * (uzunluk - dolu) + RESET


def satir_olc(etiket, deger, min_d, max_d, birim, format_="{:8.1f}", renk=None):
    govde = (" " + etiket.ljust(12) + cubuk(deger, min_d, max_d, renk)
             + " " + format_.format(deger) + " " + birim.ljust(8) + " ")
    return renklendir("║", CYAN) + govde + renklendir("║", CYAN)


def satir_metin(metin, renk=BEYAZ):
    metin = metin[:GENISLIK].ljust(GENISLIK)
    return renklendir("║", CYAN) + renklendir(metin, renk) + renklendir("║", CYAN)


def sparkline(gecmis, min_d, max_d, uzunluk=40):
    blocks = "▁▂▃▄▅▆▇█"
    result = ""
    for value in gecmis[-uzunluk:]:
        ratio = clamp((value - min_d) / (max_d - min_d), 0.0, 1.0)
        result += blocks[int(ratio * (len(blocks) - 1))]
    return result.ljust(uzunluk)


def su_renk(temp):
    if temp < 60:
        return MAVI
    if temp < 100:
        return YESIL
    return KIRMIZI


def ekrani_ciz(ecu, rpm_gecmisi, gecen_sure):
    lines = [
        renklendir("╔" + "═" * GENISLIK + "╗", CYAN),
        satir_metin("DİZEL ECU SİMÜLATÖRÜ - FİZİKSEL MODEL V1".center(GENISLIK), BOLD + BEYAZ),
        renklendir("╠" + "═" * GENISLIK + "╣", CYAN),
        satir_olc("Motor Devri", ecu.rpm, 0, ecu.RPM_MAKS, "rpm", "{:8.0f}"),
        satir_olc("Araç Hızı", ecu.hiz, 0, 220, "km/h", "{:8.1f}"),
        satir_olc("Gaz Talebi", ecu.gaz, 0, 100, "%"),
        satir_olc("Motor Yükü", ecu.yuk, 0, 100, "%"),
        satir_olc("Hava Akışı", ecu.maf, 0, 120, "g/s", "{:8.2f}"),
        renklendir("╟" + "─" * GENISLIK + "╢", CYAN),
        satir_olc("Soğutma Suyu", ecu.su_sicakligi, 0, 130, "°C", renk=su_renk(ecu.su_sicakligi)),
        satir_olc("Emme Havası", ecu.emme_sicakligi, -10, 100, "°C", renk=CYAN),
        renklendir("╟" + "─" * GENISLIK + "╢", CYAN),
        satir_olc("Yakıt Deposu", ecu.yakit, 0, 100, "%", renk=KIRMIZI if ecu.yakit < 15 else YESIL),
        satir_olc("Şarj Voltajı", ecu.aku, 10, 16, "V", "{:8.2f}", renk=YESIL),
        renklendir("╟" + "─" * GENISLIK + "╢", CYAN),
        satir_metin(" " + "RPM Grafiği".ljust(12) + sparkline(rpm_gecmisi, 0, ecu.RPM_MAKS), YESIL),
        renklendir("╟" + "─" * GENISLIK + "╢", CYAN),
        satir_metin(f" Vites: {ecu.vites}    Süre: {gecen_sure:6.1f} sn    Model: 2.0L turbo dizel (temsili)", BEYAZ),
        satir_metin(" OBD-II RPM yanıtı: " + ecu.obd_rpm_cercevesi().hex(" ").upper(), DIM + BEYAZ),
        renklendir("╚" + "═" * GENISLIK + "╝", CYAN),
        renklendir("  Çıkmak için Ctrl+C", DIM),
    ]
    return "\n".join(lines)


def run_tests():
    """Standart kütüphane ile temel regresyon testleri."""
    # 1) OBD RPM encoding / decoding
    ecu = ECU(seed=1)
    frame = ecu.obd_rpm_cercevesi()
    decoded_rpm = (frame[3] * 256 + frame[4]) / 4.0
    assert abs(decoded_rpm - ecu.rpm) <= 0.25, (decoded_rpm, ecu.rpm)
    assert len(frame) == 8 and frame[1] == 0x41 and frame[2] == 0x0C

    # 2) Dururken rölantiye yakın kalma ve hız sınırları
    for _ in range(500):
        ecu.throttle_pct = 0.0
        ecu._target_throttle = 0.0
        ecu.update(0.02)
    assert 750 <= ecu.rpm <= 900, ecu.rpm
    assert 0 <= ecu.speed_kmh <= 220, ecu.speed_kmh

    # 3) Gaz talebiyle araç hızlanmalı; hız ve RPM sınırları korunmalı
    ecu = ECU(seed=2)
    ecu._target_throttle = 65.0
    for _ in range(1500):
        ecu._target_throttle = 65.0
        ecu.update(0.02)
    assert ecu.speed_kmh > 5.0, ecu.speed_kmh
    assert 0 <= ecu.speed_kmh <= 220, ecu.speed_kmh
    assert ecu.IDLE_RPM - 30 <= ecu.rpm <= ecu.MAX_RPM, ecu.rpm
    assert 0 <= ecu.fuel_level_pct <= 100
    assert ecu.maf_g_s >= 0
    assert 0 <= ecu.load_pct <= 100
    print("TEST OK: OBD RPM kodlama, rölanti, hız sınırları, hızlanma ve sensör aralıkları.")


def main():
    parser = argparse.ArgumentParser(description="Dizel ECU simülatörü")
    parser.add_argument("--test", action="store_true", help="Temel regresyon testlerini çalıştır")
    args = parser.parse_args()
    if args.test:
        run_tests()
        return

    os.system("")
    ecu = ECU()
    rpm_history = []
    start = time.monotonic()
    previous = start
    sys.stdout.write(EKRANI_TEMIZLE + IMLEC_GIZLE)
    try:
        while True:
            now = time.monotonic()
            dt = now - previous
            previous = now
            ecu.guncelle(dt)
            rpm_history.append(ecu.rpm)
            rpm_history = rpm_history[-200:]
            sys.stdout.write(IMLEC_BASA + ekrani_ciz(ecu, rpm_history, now - start))
            sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write(IMLEC_GOSTER + "\n")
        print("Dizel ECU simülatörü durduruldu.")


if __name__ == "__main__":
    main()
