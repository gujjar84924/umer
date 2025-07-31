#ifndef __PDM2PCM_H
#define __PDM2PCM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Bit order */
#define PDM2PCM_BIT_ORDER_LSB           (0U)
#define PDM2PCM_BIT_ORDER_MSB           (1U)

/* Endianness */
#define PDM2PCM_ENDIANNESS_LE           (0U)
#define PDM2PCM_ENDIANNESS_BE           (1U)

/* Decimation factors */
#define PDM2PCM_DEC_FACTOR_16           (16U)
#define PDM2PCM_DEC_FACTOR_24           (24U)
#define PDM2PCM_DEC_FACTOR_32           (32U)
#define PDM2PCM_DEC_FACTOR_48           (48U)
#define PDM2PCM_DEC_FACTOR_64           (64U)
#define PDM2PCM_DEC_FACTOR_80           (80U)
#define PDM2PCM_DEC_FACTOR_96           (96U)
#define PDM2PCM_DEC_FACTOR_128          (128U)
#define PDM2PCM_DEC_FACTOR_64_HI_PERF   (0xA0U)
#define PDM2PCM_DEC_FACTOR_96_HI_PERF   (0xA1U)

/* Bit depth */
#define PDM2PCM_BITDEPTH_16             (16U)
#define PDM2PCM_BITDEPTH_24             (24U)
#define PDM2PCM_BITDEPTH_24IN32         (32U)

/* Return error codes */
#define PDM2PCM_OK                      (0U)
#define PDM2PCM_INIT_ERROR              (1U)
#define PDM2PCM_CONFIG_ERROR            (2U)
#define PDM2PCM_ENDIANNESS_ERROR        (3U)
#define PDM2PCM_BIT_ORDER_ERROR         (4U)
#define PDM2PCM_CRC_LOCK_ERROR          (5U)
#define PDM2PCM_DECIMATION_ERROR        (6U)
#define PDM2PCM_GAIN_ERROR              (7U)
#define PDM2PCM_SAMPLES_NUMBER_ERROR    (8U)
#define PDM2PCM_BITDEPTH_ERROR          (9U)
#define PDM2PCM_NOT_SUPPORTED_ERROR     (10U)

/* Default internal memory size */
#define PDM2PCM_MEMORY_SIZE             (512U)

typedef struct {
  uint8_t bit_order;           // LSB or MSB
  uint8_t endianness;          // LE or BE
  uint16_t high_pass_tap;      // Default: 2104533974
  uint8_t in_ptr_channels;
  uint8_t out_ptr_channels;
  void *internal_memory;       // Internal buffer
} PDM2PCM_Handler_t;

typedef struct {
  uint32_t decimation_factor;
  uint16_t output_samples_number;
  uint8_t mic_gain;            // e.g., 24
  uint8_t bit_depth;           // Usually 16
} PDM2PCM_Config_t;

/* API */
uint32_t PDM_Filter_Init(PDM2PCM_Handler_t *handler);
uint32_t PDM_Filter_setConfig(PDM2PCM_Handler_t *handler, PDM2PCM_Config_t *config);
uint32_t PDM_Filter(void *pdm_data, void *pcm_data, PDM2PCM_Handler_t *handler);

#ifdef __cplusplus
}
#endif

#endif /* __PDM2PCM_H */
