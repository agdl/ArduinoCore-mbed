#include "Arduino.h"
#include "pinDefinitions.h"
#include "mbed.h"

RTC_HandleTypeDef RTCHandle;

AnalogPinDescription g_AAnalogPinDescription[] = {
  { PA_0C,        NULL },    // A0    ADC2_INP0
  { PA_1C,        NULL },    // A1    ADC2_INP1
  { PC_2C,        NULL },    // A2    ADC3_INP0
  { PC_3C,        NULL },    // A3    ADC3_INP1
  { PC_2_ALT0,    NULL },    // A4    ADC1_INP12
  { PC_3_ALT2,    NULL },    // A5    ADC2_INP13
  { PA_4,         NULL },    // A6    ADC1_INP18
  { PA_6,         NULL }     // A7    ADC1_INP7
};

PinDescription g_APinDescription[] = {
  // D0 - D7
  { PH_15,        NULL, NULL, NULL },    // D0
  { PK_1,         NULL, NULL, NULL },    // D1
  { PJ_11,        NULL, NULL, NULL },    // D2
  { PG_7,         NULL, NULL, NULL },    // D3
  { PC_7,         NULL, NULL, NULL },    // D4
  { PC_6,         NULL, NULL, NULL },    // D5
  { PA_8,         NULL, NULL, NULL },    // D6
  { PI_0,         NULL, NULL, NULL },    // D7

  // D8 - D14
  { PC_3,         NULL, NULL, NULL },    // D8
  { PI_1,         NULL, NULL, NULL },    // D9
  { PC_2,         NULL, NULL, NULL },    // D10
  { PH_8,         NULL, NULL, NULL },    // D11
  { PH_7,         NULL, NULL, NULL },    // D12
  { PA_10,        NULL, NULL, NULL },    // D13
  { PA_9,         NULL, NULL, NULL },    // D14

  // A0 - A6
  { PA_0C,        NULL, NULL, NULL },    // A0    ADC2_INP0 D15
  { PA_1C,        NULL, NULL, NULL },    // A1    ADC2_INP1 D16
  { PC_2C,        NULL, NULL, NULL },    // A2    ADC3_INP0 D17
  { PC_3C,        NULL, NULL, NULL },    // A3    ADC3_INP1 D18
  { PC_2_ALT0,    NULL, NULL, NULL },    // A4    ADC1_INP12 D19
  { PC_3_ALT0,    NULL, NULL, NULL },    // A5    ADC1_INP13 D20
  { PA_4,         NULL, NULL, NULL },    // A6    ADC1_INP18 D21
  { PA_6,         NULL, NULL, NULL },    // A7    ADC1_INP7 D22

  // LEDS
  { PK_5,         NULL, NULL, NULL },    // LEDR
  { PK_6,         NULL, NULL, NULL },    // LEDG
  { PK_7,         NULL, NULL, NULL },    // LEDB

  { PA_0,         NULL, NULL, NULL }, //D26
  { PA_1,         NULL, NULL, NULL }, //D27
  { PA_2,         NULL, NULL, NULL }, //D28
  { PA_3,         NULL, NULL, NULL }, //D29
  { PA_4,         NULL, NULL, NULL }, //D30
  { PA_5,         NULL, NULL, NULL }, //D31
  { PA_6,         NULL, NULL, NULL }, //D32
  { PA_7,         NULL, NULL, NULL }, //D33
  { PA_8,         NULL, NULL, NULL }, //D34
  { PA_9,         NULL, NULL, NULL }, //D35
  { PA_10,        NULL, NULL, NULL }, //D36
  { PA_11,        NULL, NULL, NULL }, //D37
  { PA_12,        NULL, NULL, NULL }, //D38
  { PA_13,        NULL, NULL, NULL }, //D39
  { PA_14,        NULL, NULL, NULL }, //D40
  { PA_15,        NULL, NULL, NULL }, //D41
  { PB_0,         NULL, NULL, NULL }, //D42
  { PB_1,         NULL, NULL, NULL }, //D43
  { PB_2,         NULL, NULL, NULL }, //D44
  { PB_3,         NULL, NULL, NULL }, //D45
  { PB_4,         NULL, NULL, NULL }, //D46
  { PB_5,         NULL, NULL, NULL }, //D47
  { PB_6,         NULL, NULL, NULL }, //D48
  { PB_7,         NULL, NULL, NULL }, //D49
  { PB_8,         NULL, NULL, NULL }, //D50
  { PB_9,         NULL, NULL, NULL }, //D51
  { PB_10,        NULL, NULL, NULL }, //D52
  { PB_11,        NULL, NULL, NULL }, //D53
  { PB_12,        NULL, NULL, NULL }, //D54
  { PB_13,        NULL, NULL, NULL }, //D55
  { PB_14,        NULL, NULL, NULL }, //D56
  { PB_15,        NULL, NULL, NULL }, //D57
  { PC_0,         NULL, NULL, NULL }, //D58
  { PC_1,         NULL, NULL, NULL }, //D59
  { PC_2,         NULL, NULL, NULL }, //D60
  { PC_3,         NULL, NULL, NULL }, //D61
  { PC_4,         NULL, NULL, NULL }, //D62
  { PC_5,         NULL, NULL, NULL }, //D63
  { PC_6,         NULL, NULL, NULL }, //D64
  { PC_7,         NULL, NULL, NULL }, //D65
  { PC_8,         NULL, NULL, NULL }, //D66
  { PC_9,         NULL, NULL, NULL }, //D67
  { PC_10,        NULL, NULL, NULL }, //D68
  { PC_11,        NULL, NULL, NULL }, //D69
  { PC_12,        NULL, NULL, NULL }, //D70
  { PC_13,        NULL, NULL, NULL }, //D71
  { PC_14,        NULL, NULL, NULL }, //D72
  { PC_15,        NULL, NULL, NULL }, //D73
  { PD_0,         NULL, NULL, NULL }, //D74
  { PD_1,         NULL, NULL, NULL }, //D75
  { PD_2,         NULL, NULL, NULL }, //D76
  { PD_3,         NULL, NULL, NULL }, //D77
  { PD_4,         NULL, NULL, NULL }, //D78
  { PD_5,         NULL, NULL, NULL }, //D79
  { PD_6,         NULL, NULL, NULL }, //D80
  { PD_7,         NULL, NULL, NULL }, //D81
  { PD_8,         NULL, NULL, NULL }, //D82
  { PD_9,         NULL, NULL, NULL }, //D83
  { PD_10,        NULL, NULL, NULL }, //D84
  { PD_11,        NULL, NULL, NULL }, //D85
  { PD_12,        NULL, NULL, NULL }, //D86
  { PD_13,        NULL, NULL, NULL }, //D87
  { PD_14,        NULL, NULL, NULL }, //D88
  { PD_15,        NULL, NULL, NULL }, //D89
  { PE_0,         NULL, NULL, NULL }, //D90
  { PE_1,         NULL, NULL, NULL }, //D91
  { PE_2,         NULL, NULL, NULL }, //D92
  { PE_3,         NULL, NULL, NULL }, //D93
  { PE_4,         NULL, NULL, NULL }, //D94
  { PE_5,         NULL, NULL, NULL }, //D95
  { PE_6,         NULL, NULL, NULL }, //D96
  { PE_7,         NULL, NULL, NULL }, //D97
  { PE_8,         NULL, NULL, NULL }, //D98
  { PE_9,         NULL, NULL, NULL }, //D99
  { PE_10,        NULL, NULL, NULL }, //D100
  { PE_11,        NULL, NULL, NULL }, //D101
  { PE_12,        NULL, NULL, NULL }, //D102
  { PE_13,        NULL, NULL, NULL }, //D103
  { PE_14,        NULL, NULL, NULL }, //D104
  { PE_15,        NULL, NULL, NULL }, //D105
  { PF_0,         NULL, NULL, NULL }, //D106
  { PF_1,         NULL, NULL, NULL }, //D107
  { PF_2,         NULL, NULL, NULL }, //D108
  { PF_3,         NULL, NULL, NULL }, //D109
  { PF_4,         NULL, NULL, NULL }, //D110
  { PF_5,         NULL, NULL, NULL }, //D111
  { PF_6,         NULL, NULL, NULL }, //D112
  { PF_7,         NULL, NULL, NULL }, //D113
  { PF_8,         NULL, NULL, NULL }, //D114
  { PF_9,         NULL, NULL, NULL }, //D115
  { PF_10,        NULL, NULL, NULL }, //D116
  { PF_11,        NULL, NULL, NULL }, //D117
  { PF_12,        NULL, NULL, NULL }, //D118
  { PF_13,        NULL, NULL, NULL }, //D119
  { PF_14,        NULL, NULL, NULL }, //D120
  { PF_15,        NULL, NULL, NULL }, //D121
  { PG_0,         NULL, NULL, NULL }, //D122
  { PG_1,         NULL, NULL, NULL }, //D123
  { PG_2,         NULL, NULL, NULL }, //D124
  { PG_3,         NULL, NULL, NULL }, //D125
  { PG_4,         NULL, NULL, NULL }, //D126
  { PG_5,         NULL, NULL, NULL }, //D127
  { PG_6,         NULL, NULL, NULL }, //D128
  { PG_7,         NULL, NULL, NULL }, //D129
  { PG_8,         NULL, NULL, NULL }, //D130
  { PG_9,         NULL, NULL, NULL }, //D131
  { PG_10,        NULL, NULL, NULL }, //D132
  { PG_11,        NULL, NULL, NULL }, //D133
  { PG_12,        NULL, NULL, NULL }, //D134
  { PG_13,        NULL, NULL, NULL }, //D135
  { PG_14,        NULL, NULL, NULL }, //D136
  { PG_15,        NULL, NULL, NULL }, //D137
  { PH_0,         NULL, NULL, NULL }, //D138
  { PH_1,         NULL, NULL, NULL }, //D139
  { PH_2,         NULL, NULL, NULL }, //D140
  { PH_3,         NULL, NULL, NULL }, //D141
  { PH_4,         NULL, NULL, NULL }, //D142
  { PH_5,         NULL, NULL, NULL }, //D143
  { PH_6,         NULL, NULL, NULL }, //D144
  { PH_7,         NULL, NULL, NULL }, //D145
  { PH_8,         NULL, NULL, NULL }, //D146
  { PH_9,         NULL, NULL, NULL }, //D147
  { PH_10,        NULL, NULL, NULL }, //D148
  { PH_11,        NULL, NULL, NULL }, //D149
  { PH_12,        NULL, NULL, NULL }, //D150
  { PH_13,        NULL, NULL, NULL }, //D151
  { PH_14,        NULL, NULL, NULL }, //D152
  { PH_15,        NULL, NULL, NULL }, //D153
  { PI_0,         NULL, NULL, NULL }, //D154
  { PI_1,         NULL, NULL, NULL }, //D155
  { PI_2,         NULL, NULL, NULL }, //D156
  { PI_3,         NULL, NULL, NULL }, //D157
  { PI_4,         NULL, NULL, NULL }, //D158
  { PI_5,         NULL, NULL, NULL }, //D159
  { PI_6,         NULL, NULL, NULL }, //D160
  { PI_7,         NULL, NULL, NULL }, //D161
  { PI_8,         NULL, NULL, NULL }, //D162
  { PI_9,         NULL, NULL, NULL }, //D163
  { PI_10,        NULL, NULL, NULL }, //D164
  { PI_11,        NULL, NULL, NULL }, //D165
  { PI_12,        NULL, NULL, NULL }, //D166
  { PI_13,        NULL, NULL, NULL }, //D167
  { PI_14,        NULL, NULL, NULL }, //D168
  { PI_15,        NULL, NULL, NULL }, //D169
  { PJ_0,         NULL, NULL, NULL }, //D170
  { PJ_1,         NULL, NULL, NULL }, //D171
  { PJ_2,         NULL, NULL, NULL }, //D172
  { PJ_3,         NULL, NULL, NULL }, //D173
  { PJ_4,         NULL, NULL, NULL }, //D174
  { PJ_5,         NULL, NULL, NULL }, //D175
  { PJ_6,         NULL, NULL, NULL }, //D176
  { PJ_7,         NULL, NULL, NULL }, //D177
  { PJ_8,         NULL, NULL, NULL }, //D178
  { PJ_9,         NULL, NULL, NULL }, //D179
  { PJ_10,        NULL, NULL, NULL }, //D180
  { PJ_11,        NULL, NULL, NULL }, //D181
  { PJ_12,        NULL, NULL, NULL }, //D182
  { PJ_13,        NULL, NULL, NULL }, //D183
  { PJ_14,        NULL, NULL, NULL }, //D184
  { PJ_15,        NULL, NULL, NULL }, //D185
  { PK_0,         NULL, NULL, NULL }, //D186
  { PK_1,         NULL, NULL, NULL }, //D187
  { PK_2,         NULL, NULL, NULL }, //D188
  { PK_3,         NULL, NULL, NULL }, //D189
  { PK_4,         NULL, NULL, NULL }, //D190
  { PK_5,         NULL, NULL, NULL }, //D191
  { PK_6,         NULL, NULL, NULL }, //D192
  { PK_7,         NULL, NULL, NULL }, //D193
};

extern "C" {
  unsigned int PINCOUNT_fn() {
    return (sizeof(g_APinDescription) / sizeof(g_APinDescription[0]));
  }
}

#include "drivers/I2C.h"

void fixup3V1Rail() {
  mbed::I2C i2c(PB_7, PB_6);
  char data[2];
  data[0]=0x42;
  data[1]=(1);
  i2c.write(8 << 1, data, sizeof(data));
}

#include "QSPIFBlockDevice.h"

class SecureQSPIFBlockDevice: public QSPIFBlockDevice {
  public:
    virtual int readSecure(void *buffer, mbed::bd_addr_t addr, mbed::bd_size_t size) {
      int ret = 0;
      ret &= _qspi.command_transfer(0xB1, -1, nullptr, 0, nullptr, 0);
      ret &= read(buffer, addr, size);
      ret &= _qspi.command_transfer(0xC1, -1, nullptr, 0, nullptr, 0);
      return ret;
    }
};

#include "portenta_info.h"

static uint8_t *_boardInfo = (uint8_t*)(0x801F000);
static bool has_otp_info = false;

static SecureQSPIFBlockDevice secure_root;

// 8Kbit secure OTP area (on MX25L12833F)
bool getSecureFlashData() {
    static PortentaBoardInfo* info = new PortentaBoardInfo();
    secure_root.init();
    auto ret = secure_root.readSecure(info, 0, sizeof(PortentaBoardInfo));
    if (info->magic == OTP_QSPI_MAGIC) {
      _boardInfo = (uint8_t*)info;
      has_otp_info = true;
    } else {
      delete info;
    }
    secure_root.deinit();
    return ret == 0;
}

uint8_t* boardInfo() {
    return _boardInfo;
}

uint16_t boardRevision() {
    return (((PortentaBoardInfo*)_boardInfo)->revision);
}

uint8_t bootloaderVersion() {
    return _boardInfo[1];
}

uint32_t lowSpeedClockInUse() {
    return __HAL_RCC_GET_LPTIM4_SOURCE();
}

#define BOARD_REVISION(x,y)   (x << 8 | y)

extern "C" bool isLSEAvailableAndPrecise() {
  if (has_otp_info && (boardRevision() >= BOARD_REVISION(4,3))) {
    return true;
  }
  if (__HAL_RCC_GET_LPTIM4_SOURCE() == RCC_LPTIM4CLKSOURCE_LSI || bootloaderVersion() < 24) {
    // LSE is either not mounted, imprecise or the BL already configures RTC clock with LSI (and we are doomed)
    return false;
  }
  return true;
}

extern "C" void lp_ticker_reconfigure_with_lsi();

void initVariant() {
  RTCHandle.Instance = RTC;
  // Turn off LED from bootloader
  pinMode(PK_6, OUTPUT);
  digitalWrite(PK_6, HIGH);
  fixup3V1Rail();
  // Disable the FMC bank1 (enabled after reset)
  // See https://github.com/STMicroelectronics/STM32CubeH7/blob/beced99ac090fece04d1e0eb6648b8075e156c6c/Projects/STM32H747I-DISCO/Applications/OpenAMP/OpenAMP_RTOS_PingPong/Common/Src/system_stm32h7xx.c#L215
  FMC_Bank1_R->BTCR[0] = 0x000030D2;

  getSecureFlashData();
  if (has_otp_info && (boardRevision() >= BOARD_REVISION(4,10))) {
    // LSE works and also keeps counting in VBAT mode
    return;
  }

  // Check that the selected lsi clock is ok
  if (__HAL_RCC_GET_LPTIM4_SOURCE() == RCC_LPTIM4CLKSOURCE_LSI) {
    // rtc is not mounted, no need to do other actions
    return;
  }

  // Use micros() to check the lptim precision
  // if the error is > 1% , reconfigure the clock using lsi
  uint32_t start_ms = millis();
  uint32_t start_us = micros();
  while (micros() - start_us < 100000);
  if (millis() - start_ms != 100) {
    lp_ticker_reconfigure_with_lsi();
    // reconfiguring RTC clock would trigger a backup subsystem reset;
    // keep the clock configured in the BL
  }
}

#ifdef SERIAL_CDC

static void utox8(uint32_t val, uint8_t* s) {
  for (int i = 0; i < 16; i=i+2) {
    int d = val & 0XF;
    val = (val >> 4);

    s[15 - i -1] = d > 9 ? 'A' + d - 10 : '0' + d;
    s[15 - i] = '\0';
  }
}

uint8_t getUniqueSerialNumber(uint8_t* name) {
  utox8(HAL_GetUIDw0(), &name[0]);
  utox8(HAL_GetUIDw1(), &name[16]);
  utox8(HAL_GetUIDw2(), &name[32]);
  return 48;
}

void _ontouch1200bps_() {
  HAL_RTCEx_BKUPWrite(&RTCHandle, RTC_BKP_DR0, 0xDF59);
  NVIC_SystemReset();
}

#include "stm32h7xx_ll_system.h"

void bootM4() {

#if 0
  // This address need to be in range 0x10000000-0x3FFF0000 to be usable by the M4 as a trampoline
  uint32_t  __attribute__((aligned(0x10000))) trampoline[2];
  static const uint32_t RAM_BASE_FOR_TRAMPOLINE = (uint32_t)&trampoline[0];

#if 0

  // This snippet MUST be executed BEFORE calling bootM4()
  // The only purpose it to fread() a file into CM4_BINARY_START location

  SDRAM.begin(0);

  // Copy M4 firmware to SDRAM
  FILE* fw = fopen("/fs/fw.bin", "r");
  if (fw == NULL) {
    while (1) {
      Serial.println("Please copy a firmware for M4 core in the PORTENTA mass storage");
      delay(100);
    }
  }
  fread((uint8_t*)CM4_BINARY_START, getFileSize(fw), 1, fw);
  fclose(fw);
#endif

  // We need to call this in case we want to use BACKUP_SRAM as trampoline
  HAL_PWR_EnableBkUpAccess();

  // Copy first 2 words of the firmware in trampoline location
  memcpy((void*)RAM_BASE_FOR_TRAMPOLINE, (void*)CM4_BINARY_START, 8);

  SCB_CleanDCache();

  // Set CM4_BOOT0 address
  // This actually writes a flash register and thus is persistent across reboots
  // RAM_BASE_FOR_TRAMPOLINE must be aligned to 0x10000 barrier
  LL_SYSCFG_SetCM4BootAddress0(RAM_BASE_FOR_TRAMPOLINE >> 16);
#else
  // Classic boot, just set the address and we are ready to go
  LL_SYSCFG_SetCM4BootAddress0(CM4_BINARY_START >> 16);
  LL_RCC_ForceCM4Boot();
#endif
}

#endif
